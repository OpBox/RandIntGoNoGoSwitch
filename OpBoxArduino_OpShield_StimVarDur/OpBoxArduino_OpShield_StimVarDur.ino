/**
 * Operant Conditioning - Go Nogo Task mixed with Random Interval Schedule, Possible Switch
 * Runs task nearly on own, but can pull data from Serial port (processing), which displays & saves data, so waits for a start signal externally
 * Combines OpBox Operant Task & Position Tracker
 * Eyal Kimchi, 2014
 *
 * CC Creative Commons Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
 * https://creativecommons.org/licenses/by-sa/4.0/
 *
 * To start from Arduino Serial Monitor: P<Start@Start>
 *
 * Modify \Arduino\hardware\arduino\avr\cores\arduino\HardwareSerial.h so that
 #define SERIAL_TX_BUFFER_SIZE 256
 #define SERIAL_RX_BUFFER_SIZE 256
 * In order to prevent packet collisions
 *
 **/

// Timing Results:
// Loop: Takes ~28 us to go through loop without any actions
// Actions: If need to take action, timing is dominated by serial packet transmission
// Serial transmission: Essentially takes a ms to send each packet
// E.g. PosTracker loop takes 1352us or just over 1ms w/transmission
// Timing resolution is dependent ultimately on ceramic resonator accuracy, which in practice appears to be consistently ~0.1% off

// OpBox Serial Library for communication of Arduino with Processing/Python/Serial Monitor
#include "OpBoxSerialLibrary.h"

// *** DIFFERENT CODE FOR SHIELD R2R: START GLOBAL VARIABLES
// HARDWARE LINES INTO AND OUT OF ARDUINO BOARD/OpBox Operant Shield
const int npEmitPin = 18;
const int npDetectPin = 17;
const int lickEmitPin = 16;
const int lickDetectPin = 15;
const int npLightPin = 14;
const int houseLightPin = 2;
const int speakerPin = 3;
const int trigStimPin = 13;
const int trigNpPin = 12; // Can't just hardware copy/split inputs TTLs since signal is actually analog in practice, but thresholded by Arduino
const int trigLickPin = 11; // Can't just hardware copy/split inputs TTLs since signal is actually analog in practice, but thresholded by Arduino
const int trigFluidPin = 10; // Ideally would just hardware copy/split outputs TTLs
const int trigStartedPin = 9; // Ideally would just hardware copy/split outputs TTLs
const int backgroundPin = 4; // For background devices such as fan, IR beams, etc
const int fluid1BwdPin = 8;
const int fluid1FwdPin = 5;
const int fluid2BwdPin = 7;
const int fluid2FwdPin = 6;
// LED_BUILTIN = const int ledArduinoPin = 13; // Internal Arduino LED, used at times for debugging purposes

// Reward Parameters
long reward_ms_pulse = 35; // long so as not to convert/wrap ms_next into 16 bit int
long reward_ms_inter = 190; // For 225ms inter pulse interval from starts (as per first protocol)
int reward_num_pulse = 3;
// For OpBox Shield Rev1, before R2R, etc
//const long reward_ms_pulse = 25; // long so as not to convert/wrap ms_next into 16 bit int
//const long reward_ms_inter = 200; // long so as not to convert/wrap ms_next into 16 bit int
//// Reward Parameters, can be modified externally in OpBoxShield version
//long reward_ms_pulse = 100; // long so as not to convert/wrap ms_next into 16 bit int
//int reward_speed = 100; //Pumps do not turn consistently <70 pump speed


// Object Structures
typedef struct {
  int state;
  long ms_change; // Using "plain" long rather than unsigned since the protocols include subtractions at times and negative numbers have meaning. Using signed long ints, ms will roll over after 24 days, long enough here. Using 16-bit default ints, ms will roll over after 65535ms = 1 min
}
DetectObj;
DetectObj detectNP;
DetectObj detectLick;

typedef struct {
  long ms_start;
  long ms_end; // Need start and end for Stim & Reward, otherwise just need end/next for other objects
  boolean flag_toactivate;
  boolean flag_activated;
  long sec_iti; // for ITI Object. long so as not to convert/wrap ms_next into 16 bit int
  int i_pulse; // for Reward Object
}
TimeObj;
TimeObj timeStim;
TimeObj timeReward;
TimeObj timeRT;
TimeObj timeMT;
TimeObj timeITI;
TimeObj timePosTracker;

// Other behavioral variables/constants
const int MAX_STIM_IDS = 9; // There should never be more than this number of characters loaded into the stim_ids arrays, which really means 1 less than this due to terminal \0 char. Realistically will likely be 1-2
// Subject Info Object
typedef struct {
  int num_free_hits;
  int mean_iti;
  int max_iti;
  float log_prob_iti;
  int prob_go;
  int max_go_row;
  int max_nogo_row;
  long max_rt;
  long max_mt;
  boolean flag_rep_fa;
  long downtime_free_rwd;
  char go_stim_ids[MAX_STIM_IDS];
  int num_go_stim_ids;
  char nogo_stim_ids[MAX_STIM_IDS];
  int num_nogo_stim_ids;
  char switch_go_stim_ids[MAX_STIM_IDS];
  int num_switch_go_stim_ids;
  char switch_nogo_stim_ids[MAX_STIM_IDS];
  int num_switch_nogo_stim_ids;
  boolean flag_lick_train;
}
SubjectInfo;
SubjectInfo subjectInfo;


// SWITCHING Criteria Variables
// Significance of the switching criteria depends a bit on subject behavior and p(Go)/p(Nogo) stim
// If p(Nogo) = 80%, then if a subject never responds acc will be 80%!
// In CAM-ICU Letter A task: criteria is 80%, but nogo stim are 60%
// However, so far rats are always overresponding, so when they achieve >80% accuracy this is fairly good performance
// Can modify criteria to be more stringent, but should also consider modifying Go/Nogo probs & number of stimuli
// Still have to think about how the relative proportions of the individual stim ids vs. overall classes affect learning performance
const int MAX_SWITCH_WIN_DUR = 200;
typedef struct {
  int idx_stim;
  int win_crit; // ultimately between 80-90%
  int win_dur; // ultimately between 50-100 trials
  int win_sum_acc;
  int acc[MAX_SWITCH_WIN_DUR]; // Allocate for a maximum trial duration, can always scale back using win_dur
  int idx_stim_last_switch;
  int num_switch;
}
SwitchInfo;
SwitchInfo switchInfo;

// Other Global Stim Variables: Only 8k of SRAM on Arduino Mega2560 (http://arduino.cc/en/Tutorial/Memory). Therefore can not store longer arrays on Arduino
char stim_class;
char stim_id;
int num_hits = 0;
int num_go_row = 0;
int num_nogo_row = 0;
long ts_start = 0;
boolean flag_devices_active = false;
boolean flag_started = false;

// PosTracker PINS & VARIABLES: Do a box check from Processing to get COM#/Box ID and then set up necessary pins only?. Or just load/check/all? If did 22-53 (32 pins), some might conflict with emitter trigger for relay and with SD card transmission, not to mention some pins being unnecessary to check in smaller boxes
const int MAX_TRACKER_PINS = 32;
int trackerPins[] = {
  22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53
};
int num_tracker_pins = sizeof(trackerPins) / sizeof(trackerPins[0]);
long ms_tracker_resolution = 1000; // Default period in ms, likely to be modified to 10ms = 100Hz by processing. 1 sec as default makes it easier to debug on Arduino serial monitor when needed. long so as not to convert/wrap ms_next into 16 bit int

// STIM Constants: Can easily change these to be passed in/out
const int STIM_DUR = 1000; // Max duration of tone
const int freqVerylow = 1880;
const int freqLow = 3500;
const int freqMed = 6500;
const int freqHigh = 12000;

// TTL Constants
const int MS_TTL_PULSE = 100;


// BEGINNNING OF FUNCTIONS
void setup() {
  // SETUP Arduino: Output pins
  pinMode(npEmitPin, OUTPUT);
  pinMode(lickEmitPin, OUTPUT);
  pinMode(npLightPin, OUTPUT);
  pinMode(houseLightPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(fluid1FwdPin, OUTPUT);
  pinMode(fluid1BwdPin, OUTPUT);
  pinMode(backgroundPin, OUTPUT);
  pinMode(trigStimPin, OUTPUT);
  pinMode(trigNpPin, OUTPUT);
  pinMode(trigLickPin, OUTPUT);
  pinMode(trigFluidPin, OUTPUT);
  pinMode(trigStartedPin, OUTPUT);
  // Turn OFF Trigger & Fluid Pins (LOW): should be unnecessary as should start low, but seemed to improve stability in early betas
  digitalWrite(speakerPin, LOW);
  digitalWrite(fluid1FwdPin, LOW);
  digitalWrite(fluid1BwdPin, LOW);
  digitalWrite(trigStimPin, LOW);
  digitalWrite(trigNpPin, LOW);
  digitalWrite(trigLickPin, LOW);
  digitalWrite(trigFluidPin, LOW);
  digitalWrite(trigStartedPin, LOW);
  // Turn ON IR Emitters (HIGH)
  digitalWrite(npEmitPin, HIGH);
  digitalWrite(lickEmitPin, HIGH);
  // Turn ON Visible Light (HIGH): HouseLight. NP Light turned on after protocol starts
  digitalWrite(houseLightPin, HIGH); // Check time of day?
  digitalWrite(backgroundPin, HIGH); // Turn on background devices: Fan/IR LEDs/etc
  // Setup Input pins/IR Detectors as Inputs w/Pullup resistors enabled
  pinMode(npDetectPin, INPUT_PULLUP);
  pinMode(lickDetectPin, INPUT_PULLUP);

  // Setup PosTracker Pins: Detectors as Inputs w/Pullup resistors enabled
  for (int i_pin = 0; i_pin < num_tracker_pins; i_pin++) {
    pinMode(trackerPins[i_pin], INPUT_PULLUP);
  }

  // Set random seed by using open analog pin
  randomSeed(analogRead(0));

  // Initialize Serial communication
  Serial.begin(115200);
  // Setup Default Info Values (To display for debugging purposes need to be after Serial.begin();, but will keep before handshake so as not to delay further communication
  SubjectInfoDefaults();
  SwitchInfoDefaults();

  // Send "handshake" until get a response
  SerialHandshake();  // send bytes to establish contact and wait until receiver responds

  //  // Await start signal from Serial: Collect variable data and then Mark time to start session
  //  // To bypass, can type: P<Start@Start>
  //  WaitForText("Start");
  //
  //  // Prep First Trial, but not via PrepNextTrial which would record a response and try to UpdateAcc
  //  RandomITI(); // RandInt: Get this/next ITI
  //  ChooseStim(); // Go/Nogo: Choose the next stimulus class/id
  //  // Start session
  //  flag_devices_active = true; // Activate devices (NP & Lick for this protocol)
  //  ts_start = millis();
  //  digitalWrite(npLightPin, HIGH); // Turn on NP Light to indicate start of session
  //  SerialSendInt("T", ts_start); // Send start time in ms
}


void loop() {
  char buffer_label[MAX_BUFFER];
  char buffer_data[MAX_BUFFER];

  // Check if there is pending input from Serial stream/Processing
  if (Serial.available() > 0) {
    char data_type = SerialReceiveAndParsePacket(buffer_label, buffer_data);
    AssignData(data_type, buffer_label, buffer_data);
  }
  // Loop through possible actions
  CheckNP(); // CheckNP for Transitions: Entry & Exit: In/Out
  CheckLick(); // CheckLick for Transitions: Entry & Exit: In/Out
  CheckStim(); // CheckStim: Time to start or end?
  CheckReward(); // CheckReward: Time to start or end? Also checks for free reward within. And if movement time for a reward has elapsed then need to deactivate?
  CheckPosTracker(); // Check whether time to collect and send PosTracker data via Serial
}


void CheckNP() {
  // Use struct or pointer for objects? http://arduino.cc/en/Reference/Pointer Global struct for now
  const long MS_FOREPERIOD = 100; // Min duration an NP must be to get a stimulus. long so as not to convert/wrap ms_next into 16 bit int

  // If the current state is different from the last state, there has been a transition. HIGH = 1 (beam interrupted, subject inside port), LOW = 0 (beam unbroken, subject not in port)
  if (detectNP.state != digitalRead(npDetectPin)) {
    // TRANSITION: Save the time the transition/change occurred and save the new state
    detectNP.ms_change = millis();
    detectNP.state = !detectNP.state;
    digitalWrite(trigNpPin, detectNP.state); // Send TTL trigger to signal event

    // Process transitions in and out differently
    if (detectNP.state) {
      // NEW Nosepoke In/Entry
      SerialSendInt("N", detectNP.ms_change); // Send event time to Serial port
      // Debounce? May not need to debounce for now since will only place stim for a sustained entry (min duration defined elsewhere). Have to see how noisy this is though... if miss entries & exits while processing
      // If device is "active", then check for possible actions
      if (flag_devices_active) {
        // Check if trial is "ongoing", i.e. if Movement Time is activated
        if (timeMT.flag_activated) {
          // Nosepoke while a movement time was active. Reset Trial (End this trial and prep next)
          PrepNextTrial('N');
        }
        // Check if this NP has occurred after the predetermined ITI end
        if (millis() >= timeITI.ms_end) { // Use millis() instead of detectNP.ms_change as the time to test against, since detectNP.ms_change will be before a trial is terminated in PrepNextTrial and the ITI is reset. Otherwise can not have FR1 like trials at start or after errors
          // NP after ITI: Activate/Queue the Stimulus to play after foreperiod (ms)
          timeStim.flag_toactivate = true;
          timeStim.ms_start = detectNP.ms_change + MS_FOREPERIOD;
        }
      }
    }
    else {
      // NEW Nosepoke Out/Exit
      SerialSendInt("n", detectNP.ms_change); // Send event time to Serial port
      // If device is "active", then check for possible actions
      if (flag_devices_active) {
        // RT on a stimulus trial can be Early/Aborted/Premature OR Punctual OR Long/Delayed
        // Early/Aborted/Premature Trials
        if (timeStim.flag_toactivate) {
          // Early/Aborted/Premature Nosepoke: A stimulus is going to be activated, but has not yet been activated. Stimulus should be aborted
          // Should not just check NP at time of possible stim onset since this would miss in and out head bob nosepokes in the intervening time and still play stim inappropriately
          timeStim.flag_toactivate = false; // NP was too short, did not last min duration/"foreperiod", abort/turn off stimulus flag to activate
        }
        else if (timeRT.flag_activated) {
          // The reaction time window was activated, meaning that a stimulus was played, so check if this response is Punctual or Late
          if (detectNP.ms_change <= timeRT.ms_end) {
            // Punctual response: Activate/Initiate Movement Time Window
            timeMT.flag_activated = true;
            timeMT.ms_end = detectNP.ms_change + subjectInfo.max_mt;
          }
          else {
            // Late response: Close RT window and initiate new trial
            // SerialSendInt("m", detectNP.ms_change); // Send RT/MT time out/done. Redundant
            PrepNextTrial('R'); // Reaction time limit reached
          }
        }
      }
    }
  }
}


void CheckLick() {
  static const long MS_REWARD_DELAY = 100; // Delay onset of reward by # ms. long so as not to convert/wrap ms_next into 16 bit int

  // If the current state is different from the last state, there has been a transition. HIGH = 1 (beam interrupted, subject inside port), LOW = 0 (beam unbroken, subject not in port)
  if (detectLick.state != digitalRead(lickDetectPin)) {
    // TRANSITION: Save the time the transition/change occurred and save the new state
    detectLick.ms_change = millis();
    detectLick.state = !detectLick.state;
    digitalWrite(trigLickPin, detectLick.state); // Send TTL trigger to signal event

    // Process transitions in and out differently
    if (detectLick.state) {
      // NEW Lick In/Entry
      SerialSendInt("L", detectLick.ms_change); // Send event time to Serial port, before reward is given
      if (subjectInfo.flag_lick_train) {
        if (flag_devices_active) { // Check if devices are active (protocol has started/etc)
          // PROTOCOL/TRAINING STAGE: Lick
          // Check if this Lick has occurred when a stimulus is no longer active and after the determined ITI end
          if (millis() >= timeITI.ms_end && !timeStim.flag_activated) { // Use millis() instead of detectNP.ms_change as the time to test against, since detectNP.ms_change will be before a trial is terminated in PrepNextTrial and the ITI is reset. Otherwise can not have FR1 like trials at start or after errors
            // ITI has now ended: Queue the Stimulus to play after MIN_STIM_DELAY of # ms
            timeStim.flag_toactivate = true;
            static const long MS_STIM_DELAY = 50; // Delay from event to stimulus. long so as not to convert/wrap ms_next into 16 bit int
            timeStim.ms_start = detectLick.ms_change + MS_STIM_DELAY;
            // Check whether this was for a Go or Nogo stimulus
            if ('G' == stim_class) {
              // Lick for a Go trial = Correct Hit -> Reward
              timeReward.flag_toactivate = true; // Flag Reward to activate. Stim will be inactivated in PrepNextTrial so no more rewards for same stim
              timeReward.ms_start = detectLick.ms_change + MS_REWARD_DELAY;
            }
            else  {
              // Lick for a Nogo stimulus = Incorrect False Alarm -> No reward, may have to repeat as per PrepNextTrial
            }
            // *** Prep next trial after stimulus delivery, not here, since may interrupt timing of intercurrent events ***
          }
        }
      } else {
        // PROTOCOL/TRAINING STAGE: NP & Beyond
        // Check whether the movement time is "active": Only will be active if other devices are active
        if (timeMT.flag_activated) {
          // Check whether this was a Go or Nogo stimulus
          if ('G' == stim_class) {
            // Lick for a Go trial = Correct Hit -> Reward
            timeReward.flag_toactivate = true; // Flag Reward to activate. Stim will be inactivated in PrepNextTrial so no more rewards for same stim
            timeReward.ms_start = detectLick.ms_change + MS_REWARD_DELAY;
            // Currently can get rewards if still in nosepoke and manages to interrupt lick beam, consider checking that nosepoke is no longer triggered? Though would be unlikely for a rat to be able to do so
          }
          else  {
            // Lick for a Nogo stimulus = Incorrect False Alarm -> No reward, may have to repeat as per PrepNextTrial
            // Possible "punishments" for false alarms?: timeout, delayed next trial, forced repetition of nogo stimulus...
            // If don't reset trial then FAs can >> # nogo stim
            // OK to reset trial given stimulus repetition for errors? Previously was concerned about decreasing time to next Go stim, but now have error repetition built in
            // Record false alarms inside PrepNextTrial
          }
          // Run PrepNextTrial: Do here so that if lick duration is long this is not triggered by the MT duration limit. Otherwise may end up with spurious expired responses
          PrepNextTrial('L'); // Stim and Movement Time window also get turned off within PrepNextTrial
        }
      }
    }
    else {
      // NEW Lick Out/Exit: Not much to do, just send time
      SerialSendInt("l", detectLick.ms_change); // Send event time to Serial port
    }
  }
}

void CheckStim() {
  // Store preprogrammed stims in flash memory? http://arduino.cc/en/Reference/Include http://arduino.cc/en/Reference/PROGMEM
  long ms = millis();
  if (timeStim.flag_toactivate && (ms >= timeStim.ms_start)) {
    PlayStim();
    timeRT.flag_activated = true;
    timeRT.ms_end = ms + subjectInfo.max_rt;
  }
  else if (timeStim.flag_activated && (ms >= timeStim.ms_end)) {
    // Stimulus is activated/playing and time to turn it off has passed
    SilenceStim();
    if (subjectInfo.flag_lick_train) {
      PrepNextTrial('L'); // From lick protocol
    }
  }
}


void PrepNextTrial(char resp) {
  // If stimulus activated, turn off. Which can be frequent potentially after a rapid nosepoke for a Nogo stimulus. Overall this line may only be necessary ~1% of the time, but this line takes only <12-20us to process
  if (timeStim.flag_activated) {
    SilenceStim(); // Turn off stimulus--in case it was playing,
  }
  // Just set following as false rather than check?
  if (timeRT.flag_activated) {
    timeRT.flag_activated = false;
  }
  if (timeMT.flag_activated) {
    timeMT.flag_activated = false;
  }

  // Determine what to do next based on Stim:Go/Nogo and Response:Lick/NP/eXpired
  if ('L' == resp) {
    // Lick response
    if ('G' == stim_class) {
      num_hits++; // Update total number of rewards/hits for PrepNextTrial calculators
      // CORRECT HIT: Lick response for Go stimulus
      SerialSendCharPair("O", resp, 'H'); // Send Response & Result of Trial: Hit
      UpdateAcc(1); // Evaluate Switching criteria
      RandomITI(); // RandInt: Get this/next ITI
      ChooseStim(); // Go/Nogo: Choose the next stimulus class/id
    }
    else {
      // ERROR FALSE ALARM: Lick reponse for Nogo stimulus
      SerialSendCharPair("O", resp, 'F'); // Send Response & Result of Trial: False Alarm
      UpdateAcc(0); // Evaluate Switching criteria
      // ITI & Stim depend on whether using error correction trials for false alarms
      if (subjectInfo.flag_rep_fa) {
        // Force repetition of current stimulus class on an FR1 like schedule, do not update num in a row
        // "RandInt": ITI: FR1 like: ITI delay of 0
        timeITI.sec_iti = 0;
        timeITI.ms_end = millis();
        SerialSendIntPair("I", timeITI.ms_end, timeITI.sec_iti);    // Send ITI event time & val to Serial port

        // "Go/Nogo": Stim: Repeat Stimulus Class: Don't change a thing! Same stim_class = 'N', same stim_id = [selected]
        // Only other thing missing from NextStimNogo() is num_nogo_row++; Fine not to do so since don't want to push to limit for these errors, which would "reward" the subject/advance the next go stim earlier
      }
      else {
        // Not repeating false alarms, choose next ITI & stimulus
        RandomITI(); // RandInt: Get this/next ITI
        ChooseStim(); // Go/Nogo: Choose the next stimulus class/id
      }
    }
  }
  else {
    // Non-lick response: Nosepoke or eXpired Reaction Time or Movement Time window
    if ('G' == stim_class) {
      // ERROR MISS: NP or eXpired response for a Go trial = Error, however, just reset/generate next trial
      SerialSendCharPair("O", resp, 'M'); // Send Response & Result of Trial: Miss
      UpdateAcc(0); // Evaluate Switching criteria
      RandomITI(); // RandInt: Get this/next ITI
      ChooseStim(); // Go/Nogo: Choose the next stimulus class/id
    }
    else {
      // CORRECT REJECTION: NP or eXpired response for a Nogo trial = Correct
      SerialSendCharPair("O", resp, 'R'); // Send Response & Result of Trial: correct Rejection
      UpdateAcc(1); // Evaluate Switching criteria
      RandomITI(); // RandInt: Get this/next ITI
      ChooseStim(); // Go/Nogo: Choose the next stimulus class/id
    }
  }
}


void UpdateAcc(int temp_acc) {
  // Update crit counters
  // Have to track the history of correct responses over some window of duration win_dur
  // With each new response, have to remove oldest result and add newest result
  switchInfo.win_sum_acc -= switchInfo.acc[switchInfo.idx_stim % switchInfo.win_dur];
  switchInfo.win_sum_acc += temp_acc;
  switchInfo.acc[switchInfo.idx_stim % switchInfo.win_dur] = temp_acc;

  // Check if switch is allowed? If switch_go_stim_ids = go_stim_ids, then there will be no switching, so can set this way rather than have a special flag

  // Check if enough (win_dur) stims already to check for criteria since last switch (otherwise could switch after 16 correct trials before full window has elapsed
  // At start last switch = 0, # new stims = idx_stim
  if ((switchInfo.idx_stim - switchInfo.idx_stim_last_switch) >= switchInfo.win_dur) {
    // Check if met crit
    if (switchInfo.win_sum_acc >= switchInfo.win_crit) {
      switchInfo.idx_stim_last_switch = switchInfo.idx_stim; // Indicate last trial pre new switch
      switchInfo.num_switch++;
      SerialSendChar("W", 'S');

      // Swap original & switch stimuli!
      char temp_stim_ids[MAX_STIM_IDS];
      int num_temp_stim_ids;

      num_temp_stim_ids = CopyCharArray(subjectInfo.go_stim_ids, temp_stim_ids, MAX_STIM_IDS);
      subjectInfo.num_go_stim_ids = CopyCharArray(subjectInfo.switch_go_stim_ids, subjectInfo.go_stim_ids, MAX_STIM_IDS);
      subjectInfo.num_switch_go_stim_ids = CopyCharArray(temp_stim_ids, subjectInfo.switch_go_stim_ids, MAX_STIM_IDS);

      num_temp_stim_ids = CopyCharArray(subjectInfo.nogo_stim_ids, temp_stim_ids, MAX_STIM_IDS);
      subjectInfo.num_nogo_stim_ids = CopyCharArray(subjectInfo.switch_nogo_stim_ids, subjectInfo.nogo_stim_ids, MAX_STIM_IDS);
      subjectInfo.num_switch_nogo_stim_ids = CopyCharArray(temp_stim_ids, subjectInfo.switch_nogo_stim_ids, MAX_STIM_IDS);
    }
  }
  // after changing info for current spot update idx. this way can start with idx=0 at start of session
  switchInfo.idx_stim++;
}


void RandomITI() {
  const int MAX_SIGNED_INT = 32767;  // Assume max of 16 bits for signed integer: true for Arduino Uno & Mega, but Due could support 32bit ints
  int rand_int;
  float rand_float;
  float rand_iti_float;

  if ((num_hits < subjectInfo.num_free_hits) || (0 == subjectInfo.mean_iti)) {
    // Give X number of free hits at start of session to get subjects rolling
    // Or automaticallly use ITI of 0 depending on protocol (e.g. FR1 stage of early training)
    timeITI.sec_iti = 0;
    timeITI.ms_end = millis();
  }
  else {
    // Random Interval ITI: Use an exponential distribution. Remove all calculations that are repetitive
    rand_int = random(1, MAX_SIGNED_INT); // random(min(inclusive), max(exclusive)). Using 1 at bottom end since otherwise may end up taking a log of 0 = ~ -Inf
    rand_float = float(rand_int) / float(MAX_SIGNED_INT);
    rand_iti_float = log(rand_float) / subjectInfo.log_prob_iti;
    timeITI.sec_iti = floor(rand_iti_float); // practically means that mean_iti is (mean_iti - 1) since flooring to lower int
    // Cap longest ITIs (calculations elsewhere to eliminate longest ~1% of distribution)
    if (timeITI.sec_iti > subjectInfo.max_iti) {
      timeITI.sec_iti = subjectInfo.max_iti;
    }
    timeITI.ms_end = millis() + (1000 * timeITI.sec_iti);
  }
  SerialSendIntPair("I", timeITI.ms_end, timeITI.sec_iti);    // Send ITI event time & val to Serial port
}


void ChooseStim() {
  // Only called from PrepNextTrial
  // If end up adding Serial transmission to NextStimNogo may need to add it to error checking in PrepNextTrial too vs. pass in class to other function
  int p_stim;

  // Choose Go vs. Nogo stimulus
  if ((num_hits < subjectInfo.num_free_hits) || (100 == subjectInfo.prob_go)) {
    // Give X number of free hits at start of session to get subjects rolling OR if subject is in Go only training (e.g. early FR1)
    NextStimGo();
  }
  else if (num_go_row >= subjectInfo.max_go_row) {
    // Too many Go stim: next is Nogo
    // Consider checking stim_id to see if too many of a specific stimulus in a row?
    NextStimNogo();
  }
  else if (num_nogo_row >= subjectInfo.max_nogo_row) {
    // Too many Nogo stim: next is Go
    NextStimGo();
  }
  else {
    // Choose Go vs. Nogo stim using p(Go), with p(Nogo) = 1 - p(Go) in percent form
    p_stim = random(0, 100);
    if (p_stim < subjectInfo.prob_go) {
      NextStimGo();
    }
    else {
      NextStimNogo();
    }
  }
  SerialSendCharPair("U", stim_class, stim_id); // Send stimulus upcoming/planned information
}


void NextStimGo() {
  stim_class = 'G';
  num_go_row++;
  num_nogo_row = 0;
  stim_id = subjectInfo.go_stim_ids[random(subjectInfo.num_go_stim_ids)]; // Select stim_id from possible stimulus class ids
}


void NextStimNogo() {
  stim_class = 'N';
  num_go_row = 0;
  num_nogo_row++;
  stim_id = subjectInfo.nogo_stim_ids[random(subjectInfo.num_nogo_stim_ids)]; // Select stim_id from possible stimulus class ids
}


void PlayStim() {
  // Store static stimuli in PROGMEM? http://arduino.cc/en/Tutorial/Memory
  // Add broadband noise stimulus?: might require too much processing power of switching sample outputs every cycle-ish?
  long ms = millis();
  // switch based on current stim ID: Verylow, Low, Med, High, Noise?
  switch (toupper(stim_id)) {
    case 'V':
      tone(speakerPin, freqVerylow);
      break;
    case 'L':
      tone(speakerPin, freqLow);
      break;
    case 'M':
      tone(speakerPin, freqMed);
      break;
    case 'H':
      tone(speakerPin, freqHigh);
      break;
    default:
      SerialSendChar("SIDerr", stim_id);
      break;
  }
  digitalWrite(trigStimPin, HIGH); // Send TTL trigger that stim has been played
  timeStim.ms_end = ms + STIM_DUR; // Set time to turn stim off
  SerialSendInt("S", ms); // Send event time to Serial port
  SerialSendCharPair("G", stim_class, stim_id); // Go/Nogo stim class & ID data: Send stimulus info for the actual stimulus played (may be different from planned/upcoming due to free rewards

  timeStim.flag_toactivate = false; // So as not to retrigger
  timeStim.flag_activated = true; // So as not to retrigger but still communicate that a stimulus is active
}


// Keep this function in case ultimately use other types of stimuli (e.g. noise bursts, etc)
// Overhead of calling a function rather than just one line is very small <4us
void SilenceStim() {
  noTone(speakerPin);
  digitalWrite(trigStimPin, LOW); // Send TTL trigger that stim has been silenced
  SerialSendInt("s", millis());
  timeStim.flag_activated = false; // Deactivate stim to indicate end of a stim trial
}


void SubjectInfoDefaults() { // Operates on global var
  // Default values for subject info (pain to write out like this)
  subjectInfo.num_free_hits = 3;
  subjectInfo.mean_iti = 10;
  subjectInfo.max_iti = ceil(subjectInfo.mean_iti / 2 * 9); // Roughly translates to cutting off longest 1% of distribution
  subjectInfo.log_prob_iti = log(1 - (1 / float(subjectInfo.mean_iti))); // Take log for calculations in RandomITI Random Interval ITI function. 1 / mean_iti = Probability of ITI in any given second for exponential distribution calculations
  subjectInfo.max_rt = 1000;
  subjectInfo.max_mt = 5000;
  subjectInfo.prob_go = 20;
  subjectInfo.max_go_row = 3;
  subjectInfo.max_nogo_row = (float((100 - subjectInfo.prob_go)) / float(subjectInfo.prob_go)) * subjectInfo.max_go_row;
  subjectInfo.flag_rep_fa = true;
  subjectInfo.downtime_free_rwd = 15 * 60 * 1000L; // min -> sec -> ms. If doing math with integers, at least one of the numbers must be followed by an L, forcing it to be a long. http://arduino.cc/en/Reference/long
  subjectInfo.num_go_stim_ids = 0;
  subjectInfo.num_nogo_stim_ids = 0;
  subjectInfo.num_switch_go_stim_ids = 0;
  subjectInfo.num_switch_nogo_stim_ids = 0;
  subjectInfo.go_stim_ids[subjectInfo.num_go_stim_ids] = 'L';
  subjectInfo.num_go_stim_ids++;
  subjectInfo.nogo_stim_ids[subjectInfo.num_nogo_stim_ids] = 'M';
  subjectInfo.num_nogo_stim_ids++;
  subjectInfo.switch_go_stim_ids[subjectInfo.num_switch_go_stim_ids] = 'M';
  subjectInfo.num_switch_go_stim_ids++;
  subjectInfo.switch_nogo_stim_ids[subjectInfo.num_switch_nogo_stim_ids] = 'H';
  subjectInfo.num_switch_nogo_stim_ids++;
  subjectInfo.flag_lick_train = false;
}


// SWITCHING Criteria Variables
// Significance of the switching criteria depends a bit on subject behavior and p(Go)/p(Nogo) stim
// If p(Nogo) = 80%, then if a subject never responds acc will be 80%!
// In CAM-ICU Letter A task: criteria is 80%, but nogo stim are 60%
// However, so far rats are always overresponding, so when they achieve >80% accuracy this is fairly good performance
// Can modify criteria to be more stringent, but should also consider modifying Go/Nogo probs & number of stimuli
// Still have to think about how the relative proportions of the individual stim ids vs. overall classes affect learning performance
void SwitchInfoDefaults() { // Operates on global var
  switchInfo.idx_stim = 0;
  switchInfo.win_crit = 85; // ultimately between 80-90%
  switchInfo.win_dur = 100; // ultimately between 50-100 trials
  switchInfo.win_sum_acc = 0;
  // switchInfo.acc[switchInfo.win_dur]; // 0's size of win_dur, but better to initialize for max win
  switchInfo.idx_stim_last_switch = -1; // Last trial before current switching phase, so -1 at start
}


// Limbo Monitors for IR devices
// Change this to change what is checked during the limbo state
void CheckLimbo() {
  CheckNP();
  CheckLick();
  CheckPosTracker();
}


// POSITION TRACKER Functions
void CheckPosTracker() {
  long tracker_bits = 0; // No significant performance benefit to declaring as static, clears tracker bit values (init declare = 0);
  long ms = millis(); // Slightly faster to do this than recheck millis each time and need to store for sending later
  if (ms >= timePosTracker.ms_end) {
    // Check Pins: Best to index according to number/allocate in that way, and then loop through captured indices
    for (int i_pin = 0; i_pin < num_tracker_pins; i_pin++) {
      if (HIGH == digitalRead(trackerPins[i_pin])) {
        // If beam broken, set that i_pin bit as 1, else should be 0
        bitSet(tracker_bits, i_pin);
      }
    }
    // Send current position data over Serial port
    SerialSendIntPair("P", ms, tracker_bits); // PosTracker: Send TIMESTAMP first, BITS second. Breaks convention of initial files
    // Mark time for next loop
    timePosTracker.ms_end = ms + ms_tracker_resolution; // If just add to previous value without updating (=millis()) then will overwhelm with checks at the start of the session. So do this way and see how many loops dropped? Won't be as easy to make up time
  }
}


// Extra Serial related functions dependent on Operant variables
//// Wait for text waits until receiving target text in either label or data
//void WaitForText(char text[]) {
//  char buffer_data[MAX_BUFFER];
//  char buffer_label[MAX_BUFFER];
//
//  // Deactivate devices while waiting
//  flag_devices_active = false;
//
//  // Forever loop until receive target text, then break/return/resume elsewhere
//  while (true) {
//    if (Serial.available () > 0) {
//      char data_type = SerialReceiveAndParsePacket(buffer_label, buffer_data);
//      AssignData(data_type, buffer_label, buffer_data);
//      if (!strcmp(text, buffer_label) || !strcmp(text, buffer_data)) {
//        break;
//      }
//    }
//    CheckLimbo();
//  }
//  // Reactivate devices now that done waiting for info
//  flag_devices_active = true;
//}


// Serial Data Assignment Function
void AssignData(char data_type, char buffer_label[], char buffer_data[]) {
  long num_data = atol(buffer_data); // Returns signed long, not unsigned, but should not be an issue-- passing lower numbers

  if (!strcmp("num_free_hits", buffer_label)) { // A zero value indicates that both strings are equal ?! http://www.cplusplus.com/reference/cstring/strcmp/
    subjectInfo.num_free_hits = num_data;
  }
  else if (!strcmp("mean_iti", buffer_label)) {
    subjectInfo.mean_iti = num_data;
    subjectInfo.max_iti = ceil(subjectInfo.mean_iti / 2 * 9); // Roughly translates to cutting off longest 1% of distribution
    subjectInfo.log_prob_iti = log(1 - (1 / float(subjectInfo.mean_iti))); // Take log for calculations in RandomITI Random Interval ITI function. 1 / mean_iti = Probability of ITI in any given second for exponential distribution calculations
  }
  else if (!strcmp("max_iti", buffer_label)) {
    subjectInfo.max_iti = num_data;
  }
  else if (!strcmp("max_rt", buffer_label)) {
    subjectInfo.max_rt = num_data;
  }
  else if (!strcmp("max_mt", buffer_label)) {
    subjectInfo.max_mt = num_data;
  }
  else if (!strcmp("prob_go", buffer_label)) {
    subjectInfo.prob_go = num_data;
  }
  else if (!strcmp("max_go_row", buffer_label)) {
    subjectInfo.max_go_row = num_data;
    subjectInfo.max_nogo_row = (float((100 - subjectInfo.prob_go)) / float(subjectInfo.prob_go)) * subjectInfo.max_go_row;
  }
  else if (!strcmp("flag_rep_fa", buffer_label)) {
    subjectInfo.flag_rep_fa = (num_data > 0);
  }
  else if (!strcmp("mtr", buffer_label)) {
    ms_tracker_resolution = num_data;
  }
  else if (!strcmp("dtfr", buffer_label)) {
    subjectInfo.downtime_free_rwd = num_data;
  }
  else if (!strcmp("win_crit", buffer_label)) {
    switchInfo.win_crit = num_data;
  }
  else if (!strcmp("win_dur", buffer_label)) {
    switchInfo.win_dur = num_data;
    if (switchInfo.win_dur > MAX_SWITCH_WIN_DUR) {
      switchInfo.win_dur = MAX_SWITCH_WIN_DUR;
      SerialSendErrorText("Switch win too large");
    }
  }
  else if (!strcmp(buffer_data, "PumpOn")) {
    // Turn Pump On
    digitalWrite(fluid1FwdPin, HIGH);
    //    analogWrite(fluid1FwdPin, 100);
    digitalWrite(trigFluidPin, HIGH); // Send TTL trigger to signal event
  }
  else if (!strcmp(buffer_data, "PumpBwd")) {
    // Turn Pump On but going backward
    digitalWrite(fluid1BwdPin, HIGH);
  }
  else if (!strcmp(buffer_data, "PumpOff")) {
    // Turn Pump Off
    digitalWrite(fluid1FwdPin, LOW);
    digitalWrite(fluid1BwdPin, LOW);
    digitalWrite(trigFluidPin, LOW); // Send TTL trigger to signal event
  }
  else if (!strcmp(buffer_data, "FreeRwd")) {
    GiveFreeReward();
  }
  else if (!strcmp(buffer_data, "Start")) {
    SessionStart();
  }
  else if (!strcmp(buffer_data, "Quit")) { // A zero value indicates that both strings are equal ?! http://www.cplusplus.com/reference/cstring/strcmp/
    SessionQuit();
  }
  else if (!strcmp(buffer_label, "go_stim_ids")) {
    subjectInfo.num_go_stim_ids = CopyCharArray(buffer_data, subjectInfo.go_stim_ids, MAX_STIM_IDS);
  }
  else if (!strcmp(buffer_label, "nogo_stim_ids")) {
    subjectInfo.num_nogo_stim_ids = CopyCharArray(buffer_data, subjectInfo.nogo_stim_ids, MAX_STIM_IDS);
  }
  else if (!strcmp(buffer_label, "switch_go_stim_ids")) {
    subjectInfo.num_switch_go_stim_ids = CopyCharArray(buffer_data, subjectInfo.switch_go_stim_ids, MAX_STIM_IDS);
  }
  else if (!strcmp(buffer_label, "switch_nogo_stim_ids")) {
    subjectInfo.num_switch_nogo_stim_ids = CopyCharArray(buffer_data, subjectInfo.switch_nogo_stim_ids, MAX_STIM_IDS);
  }
  else if (!strcmp(buffer_label, "TP")) {
    // <TrackerPins@ !"#$%&'()*+,-./01234>
    for (int i = 0; i < MAX_TRACKER_PINS; i++) {
      if (buffer_data[i]) {
        trackerPins[i] = buffer_data[i];
      }
      else {
        num_tracker_pins = i; // ok not to subtract 1 since num is not zero indexed as a counter
        break;
      }
    }
  }
  else if (!strcmp("Protocol", buffer_label)) {
    if (!strcmp("Lick", buffer_data)) {
      subjectInfo.flag_lick_train = true;
    } else {
      subjectInfo.flag_lick_train = false;
    }
  }
  //  else if (!strcmp("rwd_speed", buffer_label)) {
  //    reward_speed = num_data;
  //  }
  else if (!strcmp("rwd_ms_pulse", buffer_label)) {
    reward_ms_inter = reward_ms_inter + reward_ms_pulse - num_data; // To keep same 'ITI' between pulse starts as before
    reward_ms_pulse = num_data;
  }
  else if (!strcmp("rwd_num_pulse", buffer_label)) {
    reward_num_pulse = num_data;
  }
  else {
    SerialSendErrorText("InputErr");
    SerialSendErrorText(buffer_label);
    SerialSendErrorText(buffer_data);
  }
}


void SessionStart() {
  if (!flag_started) { // Only start if not already started
    // Prep First Trial, but not via PrepNextTrial which would record a response and try to UpdateAcc
    RandomITI(); // RandInt: Get this/next ITI
    ChooseStim(); // Go/Nogo: Choose the next stimulus class/id

    flag_devices_active = true; // Activate devices (NP & Lick for this protocol), can turn off in limbo states (e.g. free reward)
    flag_started = true; // To prevent restarts

    // Start to start session! Turn on some non-Stim TTLs to synch recording system
    digitalWrite(trigNpPin, HIGH);
    digitalWrite(trigLickPin, HIGH);
    delay(MS_TTL_PULSE); // Delay slightly (# ms) and then turn off TTL pulses. Blocking delay
    digitalWrite(trigNpPin, LOW);
    digitalWrite(trigLickPin, LOW);

    digitalWrite(trigStartedPin, HIGH); // Align with ts_start
    ts_start = millis();
    digitalWrite(npLightPin, HIGH); // Turn on NP Light to indicate start of session
    SerialSendInt("T", ts_start); // Send start time in ms
  }
}

void SessionQuit() {
  flag_started = false; // Don't need to check since this self-terminates--can only stop once

  // First turn off any device outputs that are active/force end of actions
  // Force input triggers low (e.g. Lick) so as not to interfere with other TTL triggers for end of session
  // Other TTLs may be active if subject is already engaging with devices, so turn off temporarily to issue a clean start signal
  if (timeStim.flag_activated) {
    SilenceStim(); // If a stim is playing, turn it off. Otherwise might play indefinitely
  }
  // If trial currently active, send End/X-ed out response/outcome
  if (timeRT.flag_activated || timeMT.flag_activated) {
    SerialSendCharPair("O", 'E', 'E'); // Send Response & Result of Trial: False Alarm
  }
  if (detectNP.state) {
    detectNP.state = !detectNP.state; // If currently in NP, end action
    digitalWrite(trigNpPin, detectNP.state); // Send TTL trigger to signal event
    SerialSendInt("n", millis()); // Send event time to Serial port
  }
  if (detectLick.state) {
    detectLick.state = !detectLick.state; // If currently in NP, end action
    digitalWrite(trigLickPin, detectLick.state); // Send TTL trigger to signal event
    SerialSendInt("l", millis()); // Send event time to Serial port
  }

  SerialSendInt("t", millis());
  // End! Send some non-Stim TTLs to synch recording system
  digitalWrite(trigStartedPin, LOW); // Keep w/"t" = ts_end
  // Send multiplexed signal
  digitalWrite(trigNpPin, HIGH);
  digitalWrite(trigFluidPin, HIGH);
  delay(MS_TTL_PULSE); // Delay slightly (# ms) and then turn off TTL pulses. Blocking delay
  digitalWrite(trigNpPin, LOW);
  digitalWrite(trigFluidPin, LOW);

  // Turn off/down all devices
  const int MAX_NUM_DIO_PINS = 53;
  for (int i = 0; i <= MAX_NUM_DIO_PINS; i++) {
    digitalWrite(i, LOW);
  }

  // Endless Loop at end
  while (true) {
  }
  //    WaitForText("Restart"); //? Not sure what to do with restart yet, but theoretically allows input for things like pump activity still
  // Practical restart occurs when the serial port is (re)established
}


// *** PREVIOUSLY DIFFERENT CODE BY SHIELDS
void CheckReward() {
  // Use timers? http://arduino-info.wikispaces.com/Timers-Arduino http://letsmakerobots.com/node/28278 http://arduino.cc/en/Reference/Volatile
  long ms = millis();
  if ((timeReward.flag_toactivate) && (ms >= timeReward.ms_start)) {
    // Time to turn on a pump pulse
    digitalWrite(fluid1FwdPin, HIGH);
    digitalWrite(trigFluidPin, HIGH); // Send TTL trigger to signal event
    timeReward.flag_toactivate = false;
    timeReward.flag_activated = true;
    // Prep end time/action for this pulse
    timeReward.ms_end = ms + reward_ms_pulse;
    timeReward.i_pulse++;
    // Check if this is the first pulse: If so, send the time to the Serial port
    if (1 == timeReward.i_pulse) {
      SerialSendInt("R", ms);
    }
  }
  else if ((timeReward.flag_activated) && (ms >= timeReward.ms_end)) {
    // Pump currently on, turn off pulse
    digitalWrite(fluid1FwdPin, LOW);
    digitalWrite(trigFluidPin, LOW); // Send TTL trigger to signal event
    timeReward.flag_activated = false;
    // Check if need to give more pulses/not yet done
    if (timeReward.i_pulse < reward_num_pulse) {
      timeReward.ms_start = ms + reward_ms_inter;
      timeReward.flag_toactivate = true;
    }
    else {
      // Done: Update end of reward and reset pulse count
      SerialSendInt("r", ms);
      timeReward.i_pulse = 0;
    }
  }
  // Check if movement/reward window was "activated" and if so, if MT/lick window has passed?
  else if ((timeMT.flag_activated) && (ms >= timeMT.ms_end)) {
    // If so, reset Trial (End this trial and prep next)
    SerialSendInt("m", ms); // Send MT time out/done: Not redundant since no other timestamp may have been sent at this time
    PrepNextTrial('M'); // Movement time limit reached
  }
  // Check if time for a free reward, only if protocol actively running
  else if (flag_devices_active) {
    // if ms > time_last_event + downtime_free_rwd, then give free reward
    // ie. if time_last_event < (ms - downtime_free_rwd), then give free reward
    // to make calculations more efficient, will do subtraction outside of ifs => ms - downtime_free_rwd > time_last_event)
    long ms_last_cutoff = ms - subjectInfo.downtime_free_rwd; // has to be signed so that large subtractions yield negative numbers, ok as long as <24 days
    // if ((ts_start < ms_last_cutoff) && (detectNP.ms_change < ms_last_cutoff) && (detectLick.ms_change < ms_last_cutoff) && (timeReward.ms_start < ms_last_cutoff)) {
    // If nothing important has happened in > # ms (min -> sec -> ms), then give a free reward
    if ((ts_start < ms_last_cutoff) && (timeStim.ms_start < ms_last_cutoff) && (timeReward.ms_start < ms_last_cutoff)) {
      // If no stims have been played in > # ms (min -> sec -> ms), then give a free reward
      GiveFreeReward();
    }
  }
}


void GiveFreeReward() {
  // Check if a stim or reward is currently active: If so, turn it off
  if (timeStim.flag_activated) {
    SilenceStim();
  }
  if (timeReward.flag_activated || timeReward.flag_toactivate) {
    // Done: need to turn off completely
    timeReward.flag_activated = false;
    timeReward.flag_toactivate = false;
    SerialSendInt("r", millis()); // Update end of reward
  }
  // De-activate devices during free reward
  flag_devices_active = false;

  // Don't disrupt current planned stims: copy prepped stim to temp, then make current stim "Go"
  char temp_class = stim_class;
  char temp_id = stim_id;
  stim_class = 'G';
  stim_id = subjectInfo.go_stim_ids[random(subjectInfo.num_go_stim_ids)];

  // Start Stimulus, flagged as Free reward
  long ms_stim = millis();
  SerialSendInt("F", ms_stim); // Send event time to Serial port
  PlayStim();
  // SEND RESPONSE DUMMY CODE FOR FREE REWARD
  SerialSendCharPair("O", 'F', 'X'); // Send Response & Result of Trial: Hit

  timeStim.ms_start = ms_stim;
  // Delay until time to start fluid
  int ms_fluid_delay = 50;
  while (millis() < (ms_stim + ms_fluid_delay)) {
    CheckLimbo();
  }

  // Now actually Give Reward
  timeReward.ms_start = millis();
  for (int i = 0; i < reward_num_pulse; i++) {
    digitalWrite(fluid1FwdPin, HIGH);
    digitalWrite(trigFluidPin, HIGH); // Send TTL trigger to signal event
    long ms_pump = millis();
    while (millis() < (ms_pump + reward_ms_pulse)) {
      CheckLimbo();
    }
    digitalWrite(fluid1FwdPin, LOW);
    digitalWrite(trigFluidPin, LOW); // Send TTL trigger to signal event
    ms_pump = millis();
    while (millis() < (ms_pump + reward_ms_inter)) {
      CheckLimbo();
    }
  }

  // Done giving reward, reset stimuli
  stim_class = temp_class;
  stim_id = temp_id;

  // Wait until planned stim time has elapsed and then turn it off. Note: this continues to block into a limbo check
  while (millis() < (ms_stim + STIM_DUR)) {
    CheckLimbo();
  }
  // Use blocking call in case subject is in the nosepoke, which would turn off this stimulus
  SilenceStim(); // Don't need to deactivate since flag was never activated, just stim
  // Re-activate devices
  flag_devices_active = true;
}

