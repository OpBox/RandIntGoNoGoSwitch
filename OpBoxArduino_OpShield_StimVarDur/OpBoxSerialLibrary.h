/**
 * OpBox Serial Library for Operant Conditioning Sketches
 * Used to communicate between Arduion & Processing or Python Serial Monitors
 * Eyal Kimchi, 2014
 * Creative Commons Attribution-ShareAlike 4.0 International License
 * http://CreativeCommons.org/licenses/by-sa/4.0/
 **/

#include <Arduino.h>

// Packet Parsing: Needs to match Processing
static const char PACKET_START = '<'; // Start Of Packet
static const char PACKET_INT = '|'; // Marker Of 1 Integer Packet (1x4 = 4 consecutive bytes = 32 bit integer)
static const char PACKET_INT2 = '~'; // Marker Of 2 Integer Packet (2x4 = 8 consecutive bytes)
static const char PACKET_CHAR = '@'; // Marker Of Character Packet
static const char PACKET_END = '>'; // End Of Packet
static const int MAX_BUFFER = 100; // Serial buffer caps at 64 bytes on Arduino, make max larger for unneeded but low cost cushion


// SERIAL FUNCTIONS
// Open a Serial Connection with a Handshake
// Send a char (here 'A' for Arduino) and 
// look for a specific char as a response (here 'P' for Processing)
void SerialHandshake() {
  while (true) {
    while (Serial.available() <= 0) {
      Serial.write('A');   // send a capital A from Arduino to Serial Processing/Python/etc
      delay(500);
    }
    if ((Serial.available () > 0) && (Serial.read() == 'P')) {
      // Reads through all available buffer to try to find target character
      // If true: received return P from Processing/Python/etc = 
      // Hands shaken: delay & break from loop
      delay(500);
      break;
    }
  }
}


// Serial Long Integer communication functions
// Arduino Mega2560 Rev3 uses 16bit integers, 
// but Processing interprets 32 bit integers, 
// which are equivalent to 32 bit long ints for  Arduino
// Regardless of specific bit counts, if more than 1 byte (8 bits), 
// then need to send any int info as structured successive bytes
void SerialWriteLongInt(unsigned long val) {
  Serial.write((val >> 24) & 0xff);
  Serial.write((val >> 16) & 0xff);
  Serial.write((val >> 8) & 0xff);
  Serial.write((val) & 0xff);
}

void SerialSendInt(char text[], unsigned long ts) {
  Serial.write(PACKET_START);
  Serial.write(text);
  Serial.write(PACKET_INT);
  SerialWriteLongInt(ts);
  // Fixed width packet of 1 long int, no PACKET_END needed but terminate in case characters are misinterpreted, prevents "run-off"
  Serial.write(PACKET_END);
  Serial.flush();
}

void SerialSendIntPair(char text[], unsigned long int1, unsigned long int2) {
  Serial.write(PACKET_START);
  Serial.write(text);
  Serial.write(PACKET_INT2);
  SerialWriteLongInt(int1);
  SerialWriteLongInt(int2);
  // Fixed width packet of 2 long ints, no PACKET_END needed but terminate in case characters are misinterpreted, prevents "run-off"
  Serial.write(PACKET_END);
  Serial.flush(); // For 12 byte packet: takes for 12 byte packet: takes 1.019+ ms per packet without flush. With: 1.0357. May prevent buffer overruns. Added 2015/02/20
}

// Serial Char communication functions
// Send character data, which is delimited at start and end
void SerialSendChar(char text[], char data) {
  Serial.write(PACKET_START);
  Serial.write(text);
  Serial.write(PACKET_CHAR);
  Serial.write(data);
  Serial.write(PACKET_END);
  Serial.flush();
}

// Simplified call for 2 characters, rather than sending an array
void SerialSendCharPair(char text[], char char1, char char2) {
  Serial.write(PACKET_START);
  Serial.write(text);
  Serial.write(PACKET_CHAR);
  Serial.write(char1);
  Serial.write(char2);
  Serial.write(PACKET_END);
  Serial.flush();
}


// Serial Error communication functions
void SerialSendErrorText(char text[]) {
  Serial.write(PACKET_START);
  Serial.write("Err");
  Serial.write(PACKET_CHAR);
  Serial.write(text);
  Serial.write(PACKET_END);
  Serial.flush();
}

void SerialSendErrorNumAsText(long num_data) {
  Serial.write(PACKET_START);
  Serial.write("Err");
  Serial.write(PACKET_CHAR);
  Serial.write(num_data);
  Serial.write(PACKET_END);
  Serial.flush();
}


// MAIN SERIAL PACKET WORKHORSE FUNCTION
// Identify the label and data in an active serial buffer
// Modifies arrays internally, but returns char indicating data_type for external conversion/assignment
char SerialReceiveAndParsePacket(char buffer_label[], char buffer_data[]) {
  boolean flag_ser_label = false;
  boolean flag_ser_data = false;
  boolean flag_ser_end = false;
  char data_type;
  int inByte;
  int length_label = 0;
  int length_data = 0;

  while (!flag_ser_end) { 
    if (Serial.available () > 0) { 
      inByte = Serial.read();

      if (flag_ser_label) {
        // Actively collecting label data from serial port. First check if done with label and need to note whether incoming data should be interpreted as char or num (both text--not binary)
        if ((inByte == PACKET_INT) || (inByte == PACKET_CHAR)) {
          // Switch to collecting data in text form, but note how should be interpreted
          flag_ser_label = false;
          flag_ser_data = true;
          buffer_label[length_label] = '\0'; // Terminate the string
          length_data = 0;
          data_type = inByte;
        } 
        else {
          // Continue to collect char/text label to buffer
          buffer_label[length_label] = char(inByte);
          length_label++;
        }
      }
      else if (flag_ser_data) {
        // See if reached a packet end
        if (inByte == PACKET_END) {
          // Go to the end of the data: Decide what to do with data collected!
          // println("   CharData: " + buffer_char_data);
          flag_ser_data = false;
          flag_ser_end = true;
          buffer_data[length_data] = '\0'; // Terminate the string
          return data_type;
        }
        else {
          // Collect text data
          buffer_data[length_data] = char(inByte);
          length_data++;
        }
      }
      // Else search for flags from some sort of text data that should be interpreted as text  
      else if (inByte == PACKET_START) {
        flag_ser_label = true;
        length_label = 0;
      }
      else {
        SerialSendErrorText("SerInputErr");  
        Serial.write(char(inByte)); // Ideally send as error message, but have to change char to char array?
      }
    }
  }
}


// Utility Char Array/String functions
int CopyCharArray(char source[], char target[], int max_num_data) {
  int i;
  for (i = 0; i < max_num_data; i++) {
    if (source[i]) {
      target[i] = source[i];
    } 
    else {
      break;
    }
  }
  target[i] = '\0';
  return strlen(target);
}    

