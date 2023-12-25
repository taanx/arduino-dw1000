#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>

// data buffer
#define LEN_DATA 32
byte data[LEN_DATA];

#define POLL  0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */
// connection pins
const uint8_t PIN_RST = 25; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 21; // spi select pin /SPICSn
const uint8_t PIN_SCK = 18;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_WAKEUP = 27;
// const uint8_t PIN_EXTON = 26;
const uint8_t PIN_LED = 32;

// DEBUG packet sent status and count
volatile boolean sentAck = false;
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile byte expectedMsgId = POLL_ACK;
volatile boolean Sleep = false;
int16_t sentNum = 0; // todo check int type
// DW1000Time sentTime;
// String message;
// watchdog and reset period
uint32_t lastActivity;
unsigned long last_time; //ประกาศตัวแปรเป็น global เพื่อเก็บค่าไว้ไม่ให้ reset จากการวนloop
uint32_t resetPeriod = 300;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
DW1000Time timePollSent;
DW1000Time timeReceivePoll;
DW1000Time timeSentPollAck;
// DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceive;
DW1000Time timeComputedRange;
//AnchorAddress
DW1000Time timeA0RecvAck;
DW1000Time timeA1RecvAck;
DW1000Time timeA2RecvAck;
DW1000Time timeA3RecvAck;

void handleReceived();
void handleError();
void receiver();
void transmitRange();
void handleSent();
void transmitPoll();
void noteActivity();
// void Sleeps();
// void Wakeups();

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.println(F("### DW1000-arduino-receiver-test ###"));
  // initialize the driver
  SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI);
  DW1000.begin(PIN_IRQ, PIN_RST,PIN_SCK,PIN_MISO,PIN_MOSI);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  // DW1000.setChannel(1);
  DW1000.setAntennaDelay(16463);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  // start reception  
  receiver();
  transmitPoll();
  // transmitRange(); 
  noteActivity();
  pinMode(PIN_LED,OUTPUT);
}

void noteActivity()
{
  lastActivity = millis();
}
void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    transmitPoll();
    noteActivity();
}
void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on reception success
  received = true;
}

void transmitPoll() 
{
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL;
  DW1000.setData(data, LEN_DATA);
  // Serial.print("Tx Poll ... "); Serial.println();
  DW1000.startTransmit();
}
void transmitRange() {
  // transmit some data
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE;
  // String msg = "Hello DW1000, it's #"; msg += sentNum;
  // DW1000.setData(msg);
  // delay sending the message for the given amount
  DW1000Time deltaTime = DW1000Time(3000, DW1000Time::MICROSECONDS);
  timeRangeSent = DW1000.setDelay(deltaTime);
  timePollSent.getTimestamp(data+1);
  timeA0RecvAck.getTimestamp(data+11);
  timeA1RecvAck.getTimestamp(data+16);
  timeA2RecvAck.getTimestamp(data+21);
  timeA3RecvAck.getTimestamp(data+26);
  timeRangeSent.getTimestamp(data + 6);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
  // Serial.print("Transmitting Range ... "); Serial.println();
  // delaySent = millis();
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void Rxs(){
  if (received) 
  {
    received = false;
    // get message and parse
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];
    byte msgIdA = data[1];
    // Serial.print("Receive Message from "); Serial.println(data[1],HEX);
    if (msgId != expectedMsgId) 
    {
      // unexpected message, start over again
      // Serial.print("Received wrong message # "); Serial.println(msgId);
      expectedMsgId = POLL_ACK;
      // error = false;
      transmitPoll();
      return;
    }
    if (msgId == POLL_ACK) 
    {
      if (msgIdA == 0xA0)       //recv Anchor1
      {
        DW1000.getReceiveTimestamp(timeA0RecvAck);
        Serial.print("Receive PollAck Message from "); Serial.println(msgIdA,HEX);
      }
      else if (msgIdA == 0xA1) //recv Anchor2
      {
        DW1000.getReceiveTimestamp(timeA1RecvAck);
        Serial.print("Receive PollAck Message from "); Serial.println(msgIdA,HEX);
      }
      else if (msgIdA == 0xA2)  //recv Anchor3
      {
        DW1000.getReceiveTimestamp(timeA2RecvAck);
        Serial.print("Receive PollAck Message from "); Serial.println(msgIdA,HEX);
      }      
      digitalWrite(PIN_LED,0);
      transmitRange();
      noteActivity();

    }
    else if (msgId == RANGE_FAILED)
    {
      expectedMsgId = POLL_ACK;
      transmitPoll();
      noteActivity();
    }
    // Sleeps();
  }
}
void loop() {

  if (!sentAck && !received) 
  {
    // check if inactive
    if (millis() - lastActivity > resetPeriod) 
    {
      resetInactive();
      // Serial.print("!sent and !receive "); Serial.println();
    }
    return;
  }
  if (millis() - last_time > 2000)
  {
    last_time = millis();
  }
  
  Rxs();
  // continue on any success confirmation
  if (sentAck) 
  {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL) 
    {
      DW1000.getTransmitTimestamp(timePollSent);
      Serial.println("tx Poll");
      digitalWrite(PIN_LED,1);
      //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
    } 
    else if (msgId == RANGE) 
    {
      DW1000.getTransmitTimestamp(timeRangeSent);
      Serial.println("tx final ");
      last_time = millis();
      // noteActivity();
    }
    // return;
  }
  
}