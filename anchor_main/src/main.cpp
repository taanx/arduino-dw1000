#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include <Arduinojson.h>

// #define AnchorAddress 0
// #define AnchorAddress 1 
#define AnchorAddress 2
// #define AnchorAddress 3

#define settimeDelay 5000*(AnchorAddress+1)
// data buffer
#define LEN_DATA 32
byte data[LEN_DATA];
// connection pins
const uint8_t PIN_RST = 25; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 21; // spi select pin
const uint8_t PIN_SCK = 18;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_LED = 32;
// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// DEBUG packet sent status and count
volatile boolean received = false;
volatile boolean sentAck = false;
// protocol error state
boolean protocolFailed = false;
volatile unsigned long delaySent = 0;
volatile unsigned long activityDelay;
volatile unsigned long sendSerialReportDelay;
bool fSendSerialReport;
// message flow state
volatile byte expectedMsgId = POLL;
int16_t sentNum = 0; // todo check int type
int16_t numReceived = 0; // todo check int type
String message;
DW1000Time sentTime;
volatile unsigned long lastAc;
unsigned long lastTime = 0;
uint32_t Period = 300;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

// timestamps to remember
DW1000Time timeSentPoll;
DW1000Time timeReceivePoll;
DW1000Time timeSentPollAck;
DW1000Time timeReceivePollAck;
DW1000Time timeSentRange;
DW1000Time timeRangeReceive;
// last computed range/time
DW1000Time timeComputedRange;
DW1000Time timeReportA0;
DW1000Time timeReportA1;
DW1000Time timeReportA2;

DW1000Time timeA0;
DW1000Time timeA1;
DW1000Time timeA2;
float rangeA0, rangeA1, rangeA2, rangeA3;

void handleReceived();
void handleSent();
void transmitRangeReport();
void receiver();
void ActDelay();
void transmitPollAck();

void setup() {
  pinMode(PIN_LED,OUTPUT);
  // DEBUG monitoring
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("### DW1000-arduino-sender-test ###"));
  // initialize the driver
  SPI.begin(PIN_SCK,PIN_MISO,PIN_MOSI);
  // SPI.setFrequency(10000000);
  DW1000.begin(PIN_IRQ, PIN_RST,PIN_SCK,PIN_MISO,PIN_MOSI);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(5);
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
  // attach callback for (successfully) sent messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // start a transmission
  receiver();
  ActDelay();
  // transmitRangeReport();
  rangingCountPeriod = millis();
}

void resetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    receiver();
    ActDelay();
}

void ActDelay()
{
  lastAc = millis();
}

void handleSent() {
  // status change on sent success
  sentAck = true;
}
void handleReceived() {
  // status change on reception success
  received = true;
}
void transmitPollAck()
{
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL_ACK;
  data[1] = 0xA0 + AnchorAddress;
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(3000, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  // Serial.print("Transmitting Poll Ack "); Serial.println();
  DW1000.startTransmit();
}

void transmitRangeFailed()
{
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_FAILED;
  DW1000.setData(data,LEN_DATA);
  DW1000.startTransmit();
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void loop() 
{ 
  int32_t curMillis = millis();
  if (!sentAck && !received) 
  {
    // check if inactive
    if (curMillis - lastAc > Period) 
    {
      // Serial.println("null");
      resetInactive();
      // lastAc = millis();
    }
    return;
  }
  // continue on any success confirmation
  if (millis() - lastTime > settimeDelay)
  {
    lastTime = millis();
  }
  if (fSendSerialReport)
  {
    if (millis() - sendSerialReportDelay > 50)
    {
      fSendSerialReport = 0;
    }
  }
  if (sentAck) 
  {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL_ACK) 
    {
      // delaySent = millis();
      DW1000.getTransmitTimestamp(timeSentPollAck);
      // Serial.print("Sent Poll Ack"); Serial.println(msgId);
      // transmitPollAck();
      ActDelay();
    }
  }
  if (received)
  {
    // received = false;
    // lastAc = millis();
    DW1000.getData(data,LEN_DATA);
    byte msgId = data[0];
    byte msgA = data[1];
    if (msgId != expectedMsgId)
    {
      protocolFailed = true;
    }
    
    if (msgId == POLL)
    {
      protocolFailed = false;
      DW1000.getReceiveTimestamp(timeReceivePoll);  // on POLL we (re-)start, so no protocol failure
      // Serial.print("receive Poll "); Serial.println();
      expectedMsgId = RANGE; 
      if (AnchorAddress)
      {
        delay(5 * (AnchorAddress));
      }
      transmitPollAck();
      digitalWrite(PIN_LED,HIGH);
      ActDelay();
    }
    else if (msgId == RANGE)
    {
      DW1000.getReceiveTimestamp(timeRangeReceive);
      // Serial.print("Receive Range"); Serial.println(msgId);
      if (!protocolFailed)
      {
        timeSentPoll.setTimestamp(data+1);
        timeReceivePollAck.setTimestamp(data+ (11+(AnchorAddress * 5)));
        timeSentRange.setTimestamp(data+6);
        // (re-)compute range as two-way ranging is done
        // asymmetric two-way ranging (more computation intense, less error prone)
        DW1000Time round1 = (timeReceivePollAck - timeSentPoll).wrap();
        DW1000Time reply1 = (timeSentPollAck - timeReceivePoll).wrap();
        DW1000Time round2 = (timeRangeReceive - timeSentPollAck).wrap();
        DW1000Time reply2 = (timeSentRange - timeReceivePollAck).wrap();
        DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
        // set tof timestamp
        timeComputedRange.setTimestamp(tof);
        // transmitRangeReport(timeComputedRange.getAsMicroSeconds());
        rangeA2 = timeComputedRange.getAsMeters();
        // Serial.print("Round1 ... "); Serial.println(round1.getAsMicroSeconds());
        // Serial.print("Reply1 ... "); Serial.println(reply1.getAsMicroSeconds());
        // Serial.print("Round2 ... "); Serial.println(round2.getAsMicroSeconds());
        // Serial.print("Reply2 ... "); Serial.println(reply2.getAsMicroSeconds());
        // Serial.print("tof ... "); Serial.println(tof.getAsMicroSeconds(),6);
        // Serial.print("A3 Distance ... "); 
        // Serial.println(rangeA2);
        // Serial.print(""); Serial.println(); 
        // expectedMsgId = RANGE_REPORT; 
        if (AnchorAddress > 0)
        {
          delay(10*(AnchorAddress));
        }else{
          sendSerialReportDelay = millis();
          fSendSerialReport = false;
        }
        delay(100);
      }
      // else{
      //   transmitRangeFailed();
      // }
      expectedMsgId = RANGE_REPORT; 
      ActDelay(); 
    }
    else if (msgId == RANGE_REPORT)
    {
      if (msgA == 0xA0)
      {
        // DW1000.getReceiveTimestamp(timeA0);
        timeReportA0.setTimestamp(data+2);
        rangeA0 = timeReportA0.getAsMeters();
        // Serial.print("Anchor Address  :"); 
        // Serial.println(msgA,HEX);
      }
      else if (msgA == 0xA1)
      {
        // DW1000.getReceiveTimestamp(timeA1);
        timeReportA1.setTimestamp(data+2);
        rangeA1 = timeReportA1.getAsMeters();
        // Serial.print("Anchor Address  :"); 
        // Serial.println(msgA,HEX);
      }
      Serial.print("A0 "); 
      Serial.println(rangeA0);
      Serial.print("A1 "); 
      Serial.println(rangeA1);
      Serial.print("A2 "); 
      Serial.println(rangeA2);
      // Serial.println();
      // expectedMsgId = RANGE_REPORT;
      digitalWrite(PIN_LED,LOW);
      ActDelay();
    }
    received = false;
  }
}

