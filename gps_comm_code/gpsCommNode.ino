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
#include <SoftwareSerial.h>  // make sure this is NewSoftSerial beta 11 - now called SoftwareSerial
#include <TinyGPS++.h>       
#include "LowPower.h"

/************ Radio Setup ***************/
#define RF69_FREQ 915.0
#define MY_ADDRESS   1
#define GATEWAY_ADDRESS 0 //check that this is a valid address
#define ENCODING_BITS 32

//ARDUINO PINS
#define RFM69_CS    4  //
#define RFM69_INT   3  //
#define RFM69_RST   10
/********** Routing Constants ********/
#define MAX_HOPS 4
#define TYPE_STR_LEN 8

// Create the radio driver and manager
RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);
/************ GPS Setup ***************/
SoftwareSerial* GPSSerial = nullptr;
TinyGPSPlus gps;                     // instantiate the tinygps instance and call it 'gps'
volatile unsigned long timeEnteringGPS;
volatile unsigned long currTime;


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
unsigned long startTime; //used to calculate timing for discovery

int tmpPath[MAX_HOPS+1];
//nbr struct: hops, rssi, addr
// determine best nbr by the sum of rssi
struct NeighborData{ // could get rid of path
  int address;
  int numberHops;
  float totalRSSI;
  int path[MAX_HOPS+1];
};

int current_neighbors = 0;
int best_path_index = 0;
const int max_neighbors = 10;
NeighborData neighborList[max_neighbors];

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

void reverse_path(int inputList[], int numHops){
  int counter = 0;
  for (int original_index = numHops; original_index >= 0; original_index--){
    sentData.path[counter] = inputList[original_index];
    counter++;
  }
}
//this honestly might only need to be on the sound node.
void buildNeighbor(){
//information to get back to the gateway node, path needs to get reversed before traversing back to gateway
  NeighborData newNbr;
  newNbr.address = from;
  recievedData.path[recievedData.curr_num_hops] = MY_ADDRESS;
  memcpy(newNbr.path, recievedData.path, sizeof(recievedData.path));
  newNbr.numberHops = recievedData.curr_num_hops;
  newNbr.path[newNbr.numberHops] = MY_ADDRESS;

  newNbr.totalRSSI = rf69.lastRssi();
  if (current_neighbors >= max_neighbors ){
      current_neighbors = current_neighbors%max_neighbors; //TODO - fix this bandaid
  }
  neighborList[current_neighbors] = newNbr;
  if (neighborList[best_path_index].totalRSSI > neighborList[current_neighbors].totalRSSI){
    best_path_index = current_neighbors;
  }
  current_neighbors ++; 

  //add something to delete the worst one.
}

void nodeExplore(uint32_t& nodeEncoding){
    memcpy(tmpPath, recievedData.path, sizeof(recievedData.path)); //path taken to get here
    int next_index = recievedData.curr_num_hops+1;
    for (int index = 0; index < ENCODING_BITS; index++){
      if (nodeEncoding & (1 << index)){
        clearSentData(); //reset the data 
        sentData.dataType = DISCOVERY_REQUEST;
        sentData.curr_num_hops = recievedData.curr_num_hops; //initial hops before something is added
        sentData.dest = MY_ADDRESS; //makes sure that the next exploration node follows this properly
        memcpy(sentData.path, recievedData.path, sizeof(recievedData.path)); //make sure the path is still being transmit
        if(rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,index)){
          tmpPath[next_index] = index; //add discovered node to the list
          int counter = 0;
          sentData.dataType =  WEIGHT_CALCS;
          for (int i = 0; i<5; i++){
            startTime = millis();
            if(rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) ,index)){
              sentData.dataTwo += (millis() - startTime)/2; //oneway transmission
              sentData.dataOne += rf69.lastRssi();
              counter++;
        }
      }
      sentData.dataOne/=counter;
      sentData.dataTwo/=counter;
      reverse_path(tmpPath, next_index);
      sentData.dataType = DISCOVERY_RESPONSE;

      sentData.dest = GATEWAY_ADDRESS;
      sentData.curr_num_hops = 1; //from the found node to current
      rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData),sentData.path[sentData.curr_num_hops+1]); //send back down the path
    }
  }
  
  }

  clearSentData();
  sentData.dataType = DISCOVERY_RESPONSE_FINISHED;
  sentData.dest = GATEWAY_ADDRESS;
  reverse_path(recievedData.path, recievedData.curr_num_hops);
  rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData),sentData.path[sentData.curr_num_hops+1]); //all finished flag down the path
}

//setup function
void setup() {
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    while (1);
  }

  if (!rf69.setFrequency(RF69_FREQ)) {
  }

  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the client
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  rf69.setEncryptionKey(key);
}

void loop() {
  // Check the envelope input
  if (rf69_manager.available()) {
    // Wait for a message addressed to us from the client
    len = sizeof(buf);
    if (rf69_manager.recvfromAckTimeout(buf, &len, timeout, &from)) {
      buf[len] = 0; // zero out remaining string
      memcpy(&recievedData, buf, len );


      if (recievedData.dest == MY_ADDRESS){
        //check what the data request is and process it appropriately
        switch (recievedData.dataType){
          case DISCOVERY_REQUEST:
            //respond with current path and rssi  NEED TO THINK ABOUT THE BEST WAY TO RESPOND. can collect RSSI on the way back, and just reverse the path. just needs to be consistent. Hops = len of the list
            recievedData.curr_num_hops ++;
            recievedData.path[recievedData.curr_num_hops] = MY_ADDRESS;
            nodeExplore(recievedData.nodeClosedOpenEncoding);
            break;
          case GPS_REQUEST:
            recievedData.curr_num_hops ++;
            clearSentData();
            GPSCalculations();

            sentData.dataOne = gps.location.lat();
            sentData.dataTwo = gps.location.lng();
            sentData.dest = GATEWAY_ADDRESS;
            sentData.curr_num_hops = 0;
            sentData.dataType = GPS_RESPONSE;
            reverse_path(recievedData.path, recievedData.curr_num_hops);

            rf69_manager.sendtoWait((uint8_t *)&sentData, sizeof(sentData) , sentData.path[sentData.curr_num_hops+1]); //send to the next address in the path. Assumes that the dest is in it
            break;
          case ENV_SENSOR_REQUEST:
            break;
          default:
            break;
            }
      }
      else{

        recievedData.curr_num_hops ++;

        if (recievedData.curr_num_hops >= MAX_HOPS){
          return;
        }
          
        //check to see the type of message: broadcast or addressed
        switch (recievedData.dataType){
          case DISCOVERY_REQUEST:
            if (from == recievedData.dest){
              buildNeighbor();
            }
            else{
              //continue down the path
              rf69_manager.sendtoWait((uint8_t *)&recievedData, sizeof(recievedData) , recievedData.path[recievedData.curr_num_hops+1]); //send to the next address in the path. Assumes that the dest is in it
            }
            break;
          case WEIGHT_CALCS:
            break;
          case DISCOVERY_RESPONSE: //still want to update the RSSI
            // recievedData.path_weight[recievedData.curr_num_hops-1] = rf69.lastRssi();
            rf69_manager.sendtoWait((uint8_t *)&recievedData, sizeof(recievedData) , recievedData.path[recievedData.curr_num_hops+1]); //send to the next address in the path. Assumes that the dest is in it
            break;
          default:
            //addressed transmission
            rf69_manager.sendtoWait((uint8_t *)&recievedData, sizeof(recievedData) , recievedData.path[recievedData.curr_num_hops+1]); //send to the next address in the path. Assumes that the dest is in it
            break;
        } 
      }
 
    } 
  }
  if (!rf69_manager.available()){ //make sure there isn't a message waiting
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); 
  }

}
void GPSCalculations(){
  GPSSerial = new SoftwareSerial(6, -1);
  GPSSerial->begin(9600);
  delay(10);
  GPSSerial->flush();
  timeEnteringGPS = millis();
  while ((! gps.location.isValid()) || (gps.location.age() > 2)){ // keep reading data until there is a valid GPS

    while (GPSSerial->available())  // if serial data is available then read and encode
      {
        gps.encode(GPSSerial->read());
      }
      if ((millis() - timeEnteringGPS) > 10000){ //break if it has been 10 seconds
        break;
      }

  }
  GPSSerial->end();
  delay(10);
  delete GPSSerial;  // Free memory and stop interrupts
  GPSSerial = nullptr;
}



