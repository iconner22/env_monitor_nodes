// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable
// messaging client with the RH_RF69 class.
// It is designed to work with the other example RadioHead69_AddrDemo_TX.
// Demonstrates the use of AES encryption, setting the frequency and
// modem configuration.
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>  
/************ Radio Setup ***************/
#define RF69_FREQ 915.0
#define MY_ADDRESS   0
//ARDUINO PINS
#define RFM69_CS    4  //
#define RFM69_INT   3  //
#define RFM69_RST   10
#define LED        13
/********** Routing Constants ********/
#define MAX_HOPS 4
#define TYPE_STR_LEN 8
#define ENCODING_BITS 32
#define BUFFERSIZE 100

// Create the radio driver and manager
RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

/******* Transmission Type Enum *********/
enum TransmissionType {
  DISCOVERY_REQUEST,
  GPS_REQUEST,
  ENV_SENSOR_REQUEST,
  DISCOVERY_RESPONSE,
  SOUND_RESPONSE,
  GPS_RESPONSE,
  ENV_SENSOR_RESPONSE,
  DISCOVERY_RESPONSE_FINISHED,
  WEIGHT_CALCS
};

/********* Transmission Struct ********/
struct TransmissionData{
    int dest;
    int curr_num_hops;
    int path[MAX_HOPS+1];
    float dataOne; //temp, latitude, rssi
    float dataTwo; //humidity, longitude, timing
    TransmissionType dataType; //consider using enums here
    uint32_t nodeClosedOpenEncoding; //1 means the node needs to be visited, 0 means it has already been visited
};


TransmissionData sentData; //instance to pass along with new data
TransmissionData recievedData; //instance to use when handling new data
//handle recieved buffer - load into buffer the size of struct
uint8_t buf[sizeof(TransmissionData)]; //ensure that this should be here
uint8_t len = sizeof(buf); //ensure that this should be here
uint16_t timeout = 2000;
uint8_t from;

// variables used for timing calculations
unsigned long startTime; //used to calculate timing for discovery
unsigned long endTime;
unsigned long elapsed_time_us; //unsigned means that it is okay if roll over happens

uint16_t clock_ticks = 0; //pulled from the timer

//estimated using micros()
unsigned long estimated_ticks;
unsigned long overflow_ticks;
unsigned long total_ticks;
float calculated_time;


void clearRecievedData(){
  recievedData.dest = -1;
  recievedData.curr_num_hops = 0;
  for (int index = 0; index <= MAX_HOPS; index++){
    recievedData.path[index] = -1;
  }
  recievedData.dataOne = -1.0;
  recievedData.dataTwo = -1.0;
  recievedData.dataType = DISCOVERY_REQUEST;
  recievedData.nodeClosedOpenEncoding = 0;
}


void setupTimer(){
  // Set Timer1 to overflow at the desired rate
TCCR1A = 0;                  // Normal operation mode
TCCR1B = 0;                  // Stop the timer initially
TCCR1B |= (1 << CS10);       // No prescaler (16 MHz system clock)

}

void clearSentData(){
  sentData.dest = -1;
  sentData.curr_num_hops = 0;
  for (int index = 0; index <= MAX_HOPS; index++){
    sentData.path[index] = -1;
  }
  sentData.dataOne = -1.0;
  sentData.dataTwo = -1.0;
  sentData.dataType = DISCOVERY_REQUEST;
  sentData.nodeClosedOpenEncoding = 0;
}

void nodeExplore(uint32_t& nodeEncoding){

    for (int index = 0; index < ENCODING_BITS; index++){
      if (nodeEncoding & (1 << index)){
        clearRecievedData(); //reset the data 
        recievedData.path[1] = MY_ADDRESS; //hardcoded so that it is is reversed the same way that the other nodes are
        recievedData.dataType = DISCOVERY_RESPONSE;
        Serial.print("Trying to explore node: ");Serial.println(index);
        sentData.dataType = DISCOVERY_REQUEST;
        if(rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,index)){
          recievedData.curr_num_hops++;
          recievedData.path[0] = index; //hardcoded so that it is is reversed the same way that the other nodes are
          int time_counter = 0;
          int rssi_counter = 0;
          sentData.dataType = WEIGHT_CALCS;

          float maxTimingVal = -INFINITY;
          float minTimingVal = INFINITY;
          float maxRSSIVal = -INFINITY;
          float minRSSIVal = INFINITY;

          for (int i = 0; i<5; i++){
            TCCR1B = 0;                  // Stop the timer
            TCNT1 = 0;                   // Reset the counter
            TIFR1 |= (1 << TOV1);        // Clear the overflow bit 
            TCCR1B |= (1 << CS10);       // Start the timer
            startTime = micros();       //start micros (might need to find a way for micros overflow)
            if(rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,index)){
              TCCR1B &= ~((1 << CS11) | (1 << CS10));   //stop the timer
              endTime = micros();
              clock_ticks = TCNT1;
              elapsed_time_us = endTime - startTime;
              estimated_ticks = elapsed_time_us * 16UL; //16 of the 62.5ns per 1 us
              overflow_ticks = (estimated_ticks/65536UL) * 65536UL; //finds the total number of overflow times (int) then multiplies it by the number of ticks
              total_ticks = overflow_ticks + clock_ticks;
              calculated_time = total_ticks * 0.0625;
              if (calculated_time > elapsed_time_us + 2048){
                calculated_time -= 4096; //avoids a case where micros rolled over right before the counter did
              }
              recievedData.dataTwo += (calculated_time)/2; //oneway transmission
              time_counter++;
              recievedData.dataOne += rf69.lastRssi();
              rssi_counter++;

              if ((calculated_time/2) > maxTimingVal){
                maxTimingVal = calculated_time/2;
              }
              if ((calculated_time/2) < minTimingVal){
                minTimingVal = calculated_time/2;
              }

              if (rf69.lastRssi() > maxRSSIVal){
                maxRSSIVal = rf69.lastRssi();
              }
              if (rf69.lastRssi() < minRSSIVal){
                minRSSIVal = rf69.lastRssi();
              }
        }
        delay(10);
      }

      recievedData.dataOne-=minRSSIVal;
      recievedData.dataOne-=maxRSSIVal;

      recievedData.dataTwo -=minTimingVal;
      recievedData.dataTwo -= maxTimingVal;

      recievedData.dataOne/=rssi_counter - 2; //subtract two because of dropping the max and min values
      recievedData.dataTwo/=time_counter -2;
      printDataJson();
    }
  }
  }
  clearRecievedData();
  recievedData.dataType = DISCOVERY_RESPONSE_FINISHED;
  printDataJson();
}
void printDataJson(){
  //rpi calculates the timing offset
  switch (recievedData.dataType){
    case DISCOVERY_RESPONSE_FINISHED:
      Serial.print(F("{ \"Type\": \"DISCOVERY REQUEST FINISHED\" "));
      Serial.println(F("}"));
      break;
    case DISCOVERY_RESPONSE: //add reverse path to the explore for gateway to get a standard order of the discovered in path[0] TODO - Ian!!!
      Serial.print(F("{ \"Type\": \"DISCOVERY\", "));
      Serial.print(F("\"Edge\": ("));Serial.print(recievedData.path[0]);Serial.print(F(","));Serial.print(recievedData.path[1]);Serial.print(F("), "));
      Serial.print(F("\"RSSI\": "));Serial.print(recievedData.dataOne,6);Serial.print(F(","));
      Serial.print(F("\"Time\": "));Serial.print(recievedData.dataTwo,6);Serial.print(F(","));
      Serial.print(F("\"Total Hops\": "));Serial.print(recievedData.curr_num_hops);Serial.print(F(","));

      Serial.print(F("\"Path\": \"["));
      for (int index = 0; index <= recievedData.curr_num_hops; index++){
        Serial.print(recievedData.path[index]);
        if (index != recievedData.curr_num_hops){
          Serial.print(F(","));
        }
      }
      Serial.print(F("]\""));

      Serial.println(F("}"));
      break;
    case GPS_RESPONSE:
      Serial.print(F("{ \"Type\": \"GPS\", "));
      Serial.print(F("\"Address\": "));Serial.print(recievedData.path[0]);Serial.print(F(","));
      Serial.print(F("\"Latitude\": "));Serial.print(recievedData.dataOne,6);Serial.print(F(","));
      Serial.print(F("\"Longitude\": "));Serial.print(recievedData.dataTwo,6);
      Serial.println(F("}"));
      break;
    case ENV_SENSOR_RESPONSE:
      Serial.print(F("{ \"Type\": \"ENV\", "));
      Serial.print(F("\"Address\": "));Serial.print(recievedData.path[0]);Serial.print(F(","));
      Serial.print(F("\"Temperature\": "));Serial.print(recievedData.dataOne,6);Serial.print(F(","));
      Serial.print(F("\"Humidity\": "));Serial.print(recievedData.dataTwo,6);
      Serial.println(F("}"));
      break;

    case SOUND_RESPONSE:
      Serial.print(F("{ \"Type\": \"SOUND\", "));
      Serial.print(F("\"Address\": "));Serial.print(recievedData.path[0]);Serial.print(F(","));
      Serial.print(F("\"Sound Level\": "));Serial.print(recievedData.dataOne);Serial.print(F(","));
      Serial.print(F("\"Path\": \"["));
      for (int index = 0; index <= recievedData.curr_num_hops; index++){
        Serial.print(recievedData.path[index]);
        if (index != recievedData.curr_num_hops){
          Serial.print(F(","));
        }
      }
      Serial.print(F("]\""));
      Serial.println(F("}"));
      break;
    default:
      break;      
    }

}
//**************Globals Used for Reading In Serial *****************//
bool response_recieved = true;
int transmission_type;
int dest,p0,p1,p2,p3,p4,num_hops;
uint32_t node_encoding;
char inputBuffer[100];
int bytesRead;



//setup function
void setup() {
  Serial.begin(9600);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the client
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  rf69.setEncryptionKey(key);
  Serial.print(F("RFM69 radio @"));  Serial.print((int)RF69_FREQ);  Serial.println(F(" MHz"));
  
  setupTimer();
}


void loop() {
  //expect this format: dest transmission_type path0 path1 path2 path3 path4 num_hops (put -1 in the case of no path)
    if (Serial.available()){
    Serial.println(" 'Test': ");
    bytesRead = Serial.readBytesUntil('\n', inputBuffer, BUFFERSIZE-1);
    inputBuffer[bytesRead] = '\0';
    if (sscanf(inputBuffer, "%d %d", &dest, &transmission_type) != 2){
      Serial.println(F("Issue Parsing Data"));
    }
    switch (transmission_type){
      case (0):
        Serial.println("Reading In Discovery Data");
        if (sscanf(inputBuffer, "%d %d %d %d %d %d %d %u", &dest, &transmission_type, &p0, &p1, &p2, &p3, &p4, &node_encoding) != 8){
          Serial.println(F("Issue parsing data"));
        }
        break;
      default:
        Serial.println("Reading In Typical Data");
        if (sscanf(inputBuffer, "%d %d %d %d %d %d %d %d", &dest, &transmission_type, &p0, &p1, &p2, &p3, &p4, &num_hops) != 8){
          Serial.println(F("Issue Parsing Data"));
        }
        break;
    }
    sentData.dest = dest;
    switch (transmission_type){
      case(0)://discovery request
      if (dest == MY_ADDRESS){ //explore this node if it is the destination
        nodeExplore(node_encoding);

      }
      else{ //send message along the path
        sentData.dataType = DISCOVERY_REQUEST;
        sentData.path[0] = MY_ADDRESS;
        sentData.path[1] = p1;
        sentData.path[2] = p2;
        sentData.path[3] = p3;
        sentData.path[4] = p4;
        sentData.curr_num_hops = 0;
        sentData.nodeClosedOpenEncoding = node_encoding;

        rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,sentData.path[1]);
      }
      break; 
      
      case (1): //gps request
        sentData.dataType = GPS_REQUEST;
        sentData.path[0] = MY_ADDRESS;
        sentData.path[1] = p1;
        sentData.path[2] = p2;
        sentData.path[3] = p3;
        sentData.path[4] = p4;
        sentData.curr_num_hops = 0;

        rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,sentData.path[1]);
        break;

      case (2): //env request
        sentData.dataType = ENV_SENSOR_REQUEST;
        sentData.path[0] = p0;
        sentData.path[1] = p1;
        sentData.path[2] = p2;
        sentData.path[3] = p3;
        sentData.path[4] = p4;
        sentData.curr_num_hops = 0;
        rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,sentData.path[1]);

      break;
      default:
        Serial.println(F("Issue with datatype, please enter a valid request"));
        break;
    }
  }
  // Check the envelope input
  if (rf69_manager.available()) {
    // Wait for a message addressed to us from the client
    len = sizeof(buf);
    if (rf69_manager.recvfromAckTimeout(buf, &len, timeout, &from)) {
      buf[len] = 0; // zero out remaining string
      memcpy(&recievedData, buf, len );
      Serial.println(recievedData.dest);
      if (recievedData.dest == MY_ADDRESS){
        //check what the data request is and process it appropriately
        switch (recievedData.dataType){
          case DISCOVERY_RESPONSE:
            recievedData.curr_num_hops ++;
            printDataJson();
            break;
          case GPS_RESPONSE:
            recievedData.curr_num_hops ++;
            printDataJson();
            break;
          case ENV_SENSOR_RESPONSE:
            recievedData.curr_num_hops ++;
            printDataJson();
            break;
          case SOUND_RESPONSE:
            recievedData.curr_num_hops ++;
            printDataJson();
            break;
          case DISCOVERY_RESPONSE_FINISHED:
            recievedData.curr_num_hops ++;
            printDataJson();
            break;
          default:
            Serial.println(F("Invalid Message Type for GATEWAY node"));
            break;
            }
      }
      else{
        return; //should only be interacting if it it is for the GATEWAY
      }
    } 
  }


}




