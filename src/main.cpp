#include <Arduino.h>
#include <permaDefs.h>
#include <SPI.h>
#include <Wire.h>
#include <time.h>
#include <TimeLib.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <elapsedMillis.h>
#include <avr/sleep.h>
#include <DFRobot_LIS2DW12.h>
#include <IridiumSBD.h>

#define IridiumSerial Serial2
#define DIAGNOSTICS true // Change this to see diagnostics

//*** Library Declarations ***//
elapsedMillis Btime;
elapsedMillis mTime;
TinyGPSPlus gps;
SPIFlash flash(FCS);
DFRobot_LIS2DW12_I2C acce;
IridiumSBD modem(IridiumSerial);



//#####################################################################################################//

//*** Variables ***//

// General Variables //
uint16_t cnt;                     //** Total data points collected

// Flash Addresses // 
uint32_t wAdd = 0;                //** Last written to flash address
uint32_t rAdd = 0;                //** Last read from flash address

// GPS Control Variables //
int gpsFrequency = 2;            //(Minutes)>>> GPS Frequency in minutes *** USER CONFIG ***
int gpsTimeout = 50;              //(Seconds)>>> GPS Timesout after 'x' seconds *** USER CONFIG ***
int gpsHdop = 5;                  //(N/A)>>> GPS HODP Parameter *** USER CONFIG ***

// GPS Storage Variables //
float lat;                        //** Storing last known Latitude
float lng;                        //** Storing last known Longitude

// Time Variables // 
time_t strtTime = 1667704472;     //** Start Time Acquired by GPS on Start Up

// Booleans and Flags //
bool wipe_memory = true;         //>>> Wipe memory during reset/Activation
bool activation_resp_rcvd = false;//** Flag to mark if activation commnd was rcvd
bool gps_wait_time = false;

// Radio Variables //
int radioFrequency = 1;           //Frequency of Pings in minutes *** USER CONFIG ***
int rcv_duration = 3;             // Receive Window Duration in seconds *** USER CONFIG ***

// Iridium Variables //
int transmissionFrequency = 6;

// Control Variables //
uint16_t pingSecondCounter;
uint16_t gpsSecondCounter;
uint16_t pingCounterTarget;
uint16_t gpsCounterTarget;

//#####################################################################################################//

//*** Functions ***//
void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, resulting in 32.768kHz/16384 = 2Hz */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
  pingSecondCounter = pingSecondCounter + 1;
  gpsSecondCounter = gpsSecondCounter + 1;
}

// Function 1 : Record and Store GPS data
void recGPS(){
  double hdopstrt = 10.00;
  Btime = 0;
  digitalWrite(GPS_PIN, HIGH);
  while ((Btime/1000) <= gpsTimeout)
  {
    // Serial.println(currentTime-xa);
    while (Serial1.available() > 0)
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println(F("Acquiring"));
        }      
      }else{
        // Serial.println(gps.location.isUpdated());
        // Serial.print(F("Location Age:"));
        // Serial.println(gps.location.age());
        // Serial.print(F("Time Age:"));
        // Serial.println(gps.time.age());
        Serial.print(F("HDOP Age:"));
        Serial.println(gps.hdop.age());
        // Serial.print(F("Satellites:"));
        // Serial.println(gps.satellites.value());
        Serial.print(F("HDOP:"));
        Serial.println(gps.hdop.value());
        if (gps.hdop.hdop() != 0.00)
        {
          hdopstrt = gps.hdop.hdop();
          Serial.println(hdopstrt);
        }
      }    
    }
    if (hdopstrt < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000 && gps.hdop.age() < 50)
    {
      break;
    }  
  }   

  digitalWrite(GPS_PIN, LOW);
  
  data dat;

  if (gps.location.age() < 60000)
  {
    //pack data into struct
    lat = gps.location.lat();
    lng = gps.location.lng();
    dat.lat = gps.location.lat();
    dat.lng = gps.location.lng();
  }else{
    // pack data into struct with lat long = 0
    dat.lat = 0;
    dat.lng = 0;
  }
    Serial.print(gps.date.day());Serial.print(gps.date.month());Serial.println(gps.date.year());
    // digitalWrite(RTC_PIN, HIGH);
    setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day (), gps.date.month (),gps.date.year());
    time_t x = now();
    dat.datetime = x;
    // digitalWrite(RTC_PIN, LOW);
    dat.locktime = (Btime/1000);
    dat.hdop = gps.hdop.hdop();
    
    
    Serial.println(dat.datetime);
    Serial.println(dat.lat);
    Serial.println(dat.lng);
    Serial.println(dat.locktime);
    Serial.println(dat.hdop);


  if (flash.powerUp())
  {
    Serial.println(F("Powered Up"));
    delay(500);
    Serial.println((int)sizeof(dat));
    wAdd = flash.getAddress(sizeof(dat));
    Serial.println(wAdd);
    if (flash.writeAnything(wAdd, dat))
    {
      Serial.println(F("Write Successful"));
      cnt = cnt + 1;
    }else
    {
      Serial.println(F("Write Failed"));
      Serial.println(flash.error(VERBOSE));
    }     
  }else
  {
    Serial.println(F("Power Up Failed"));
  }   
  flash.powerDown();

}

// Function 1-a : Secondary GPS Function used only in activation sequence
void acqGPS(){
  digitalWrite(GPS_PIN, HIGH);
      do{ 
        while (Serial1.available() > 0)
        {
          if (gps.encode(Serial1.read()))
          {
            if (!gps.location.isValid())
            {
              Serial.println(F("Not Valid"));
            }else{
              Serial.println(gps.location.isUpdated());
              Serial.print("Location Age:");
              Serial.println(gps.location.age());
              Serial.print("Time Age:");
              Serial.println(gps.time.age());
              Serial.print("Date Age:");
              Serial.println(gps.date.age());
              Serial.print("Satellites:");
              Serial.println(gps.satellites.value());
              Serial.print("HDOP:");
              Serial.println(gps.hdop.hdop());
            }
          }
        }
      }while(!gps.location.isValid());
    if (gps.location.age() < 60000)
    {
      //pack data into struct
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      time_t n = now();
      strtTime = n;
      Serial.print(F("START TIME : ")); Serial.println(strtTime);
    }    
    digitalWrite(GPS_PIN, LOW);
}

// Function 2 : Read data from flash and send it through radio
void read_send(){ 
  data dat;

  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      // dat.id = tag;
      Serial.println(F("Read Successful"));
      Serial.println(dat.datetime);
      Serial.println(dat.hdop);
      Serial.println(dat.lat);
      Serial.println(dat.lng);
      Serial.println(dat.locktime);
    }else
    {
      Serial.println(F("Read Failed"));
    }    
  }
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&dat, sizeof(dat));
      LoRa.endPacket();
      LoRa.sleep();
}

// Function 3 : RTC Timer Interrupt Service Routine
// Function 4 : Check the elctrodes for sumbersion/surfacing events
void irdSend(){
  data dat;
  int err;
  if (flash.powerUp())
  {
    if (flash.readAnything(rAdd, dat))
    {
      Serial.println(F("Reading in"));     
    }else
    {
      Serial.println(F("Read Failed"));
    } 
  }
  Serial.println(F("Sending Data"));
  err = modem.sendSBDBinary((uint8_t*)&dat,sizeof(dat));
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Send Failed - "));
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println(F("Try again!!."));
  }else{
    Serial.println(F("Transmitted"));
  } 
}

// Function 5 : Pinger - Takes several inputs and sends ping
void Ping(float x, float y, uint16_t a, uint16_t c, byte d){

  ping p;

  p.devtyp = d;
  p.ta = a;
  p.la = x;
  p.ln = y;
  p.cnt = c;

  Serial.print(F("Size")); Serial.println((int)sizeof(p));
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.sleep();
  
}

// Function 6 : Look for and process inbound transmission right after ping.
void receive(unsigned int rcv_time){
  Serial.println(F("Receiving"));
  LoRa.idle();
  mTime = 0;
  int x = 0;
  do
  {  
    x = LoRa.parsePacket();
    if (x)
    {
      Serial.println(x);
    }
    
    
    if (x == 3)
    { 
      Serial.print(F("int : ")); Serial.println(x);
      struct request{
      uint16_t tag;
      byte request;
      }r;
      while (LoRa.available())
      {
        Serial.println(F("Reading in"));
        LoRa.readBytes((uint8_t*)&r, sizeof(r));
      }
      Serial.println(r.tag);
      Serial.println(r.request);
      if (r.tag == tag && r.request == (byte)82)
      {
        
        do
        {
          Serial.println("Init Stream");
          read_send();
          delay(50);
          rAdd = rAdd + 15;
          Serial.println(rAdd);
        
        } while (rAdd <= wAdd);

        delay(1000);
        struct resp{
        uint16_t tag;
        byte res;
        }r;
        r.res = (byte) 68;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
      }            
    }

    if (x == 21)
    {
      setttings set;

      while (LoRa.available())
      {
        Serial.println(F("Incoming Settings"));
        LoRa.readBytes((uint8_t*)&set, sizeof(set));
      }

      gpsFrequency = set.gpsFrq;
      gpsTimeout = set.gpsTout;
      gpsHdop = set.hdop;
      radioFrequency = set.radioFrq;
      rcv_duration = set.rcv_dur;
      
      Serial.println(set.gpsFrq);
      Serial.println(set.gpsTout);
      Serial.println(set.hdop);
      Serial.println(set.radioFrq);
      Serial.println(set.rcv_dur);
      delay(100);

      resPing r;
        r.resp = (byte)83;
        r.tag = tag;

        LoRa.idle();
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&r, sizeof(r));
        LoRa.endPacket();
        LoRa.sleep();
    }
  }while(mTime <= rcv_time);
  LoRa.sleep();
  delay(50);
}

// Function 7 : First activation ping - process active/sleep/wipe modes
void activationPing(){

  reqPing px1;
  resPing rs1;

  int x;

  px1.tag = tag;
  px1.request = (byte)73;


  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&px1, sizeof(px1));
  LoRa.endPacket();
  
  mTime = 0;
  while (mTime < 30000)
  {
    x = LoRa.parsePacket();
    if (x)
    {
      activation_resp_rcvd = true;
      Serial.println(F("Incoming"));
      Serial.println(x);
    }else{
      activation_resp_rcvd = false;
    }
    
    if (x == 3)
    {
      while (LoRa.available())
      {
        Serial.println(F("Message"));
        LoRa.readBytes((uint8_t*)&rs1, sizeof(rs1));
      }
      break;      
    }     
  } 
  LoRa.sleep();

  if (rs1.tag == tag && rs1.resp == (byte)70)
  {
    // Serial.print(F("System Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
    Serial.print(F("System Initialising with wipe"));
     /// Begin GPS and Acquire Lock ////
    acqGPS();
    
    wipe_memory = true;

    px1.request = (byte)106;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
  }

  if (rs1.tag == tag && rs1.resp == (byte)71)
  {
    Serial.print(F("System Initialising without wipe"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    // acqGPS();

    wipe_memory = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();

  }

  if (rs1.tag == tag && rs1.resp == (byte)115){
        Serial.print(F("Indefinite Sleep"));
        EEPROM.put(1, false);
        Serial.println(EEPROM.read(1));
        delay(50);  
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();      
        sleep_cpu();
  }

  Serial.println(EEPROM.read(1));
  if (activation_resp_rcvd == false)
  {
    if (EEPROM.read(1) == false){
    Serial.println(F("SLEEP1"));
    delay(50);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    }else{
    Serial.println(F("Reset"));
    Serial.print(F("System Re - Initialising"));
    EEPROM.write(1, true);
    Serial.println(EEPROM.read(1));
     /// Begin GPS and Acquire Lock ////
    acqGPS();

    wipe_memory = false;

    px1.request = (byte)105;
    px1.tag = tag;
    LoRa.idle();
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&px1, sizeof(px1));
    LoRa.endPacket();
    LoRa.sleep();
    }
  }    
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(19200);
  Wire.swapModule(&TWI1);
  Wire.usePullups();
  Wire.begin();
  RTC_init();

  //***********************************************//
  // if(!acce.begin()){
  //    Serial.println(F("Communication failed, check the connection and I2C address setting when using I2C communication."));
  //    delay(1000);
  // }else{
  // Serial.print(F("chip id : "));
  // Serial.println(acce.getID(),HEX);
  // }
  // acce.softReset();
  // acce.setRange(DFRobot_LIS2DW12::e4_g);
  // acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  // acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  // acce.setWakeUpDur(/*dur = */2);
  // acce.setWakeUpThreshold(/*threshold = */0.3);
  // acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
  // acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
  // acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
  // acce.setDataRate(DFRobot_LIS2DW12::eRate_100hz);
  
  //***********************************************//
  LoRa.setPins(LCS, LRST, LDIO0);
  if(!LoRa.begin(867E6)){
    Serial.println(F("LoRa Failed Init"));
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(12);
  // LoRa.setSignalBandwidth(62.5E3);
  LoRa.sleep();
  //***********************************************//
  // activationPing();
  //***********************************************//
  if(flash.powerUp()){
    Serial.println(F("Powered Up1"));
  }
  if(!flash.begin()){
    Serial.println(F("Flash again"));
    Serial.println(flash.error(VERBOSE));
  } 
  Serial.println(flash.getManID());
  if(flash.powerUp()){
    Serial.println(F("Powered Up"));
  }else{
    Serial.println(F("PWR UP Failed!"));
  }
  if (wipe_memory == true)
  {
    Serial.println(F("WIPING FLASH"));
    if(flash.eraseChip()){
    Serial.println(F("Memory Wiped"));  
    }else
    {
      Serial.println(flash.error(VERBOSE));
    }
  }else{
    rAdd = flash.getAddress(15); 
    wAdd = flash.getAddress(15);
  }    
  if(flash.powerDown()){
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

  //***********************************************//
  acqGPS();
  //***************************************************//
  digitalWrite(GSM_PIN,HIGH);
  Serial.println("Warming Iridium");
  delay(15000);
  Serial.println("Starting Iridium");
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
  int err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }else{
    if (err == ISBD_SUCCESS){
      Serial.println("Iridium Ready");      
      Serial.println(modem.isAsleep());
      Serial.println(modem.isConnected());
      modem.adjustSendReceiveTimeout(120);
    }
  }
  digitalWrite(GSM_PIN,LOW);
  delay(5000);
  //***********************************************//

  gpsCounterTarget = gpsFrequency*60;
  pingCounterTarget = transmissionFrequency*60;  
  gpsSecondCounter = 0;
  pingSecondCounter = 0;

  //***********************************************//
  Serial.println("SYSTEM READY");
  Serial.flush();
  //***********************************************//
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void loop() {
  // put your main code here, to run repeatedly:
  //**************************************************************//
  
    Serial.println(F("Mode 1"));
    mTime = 0;
    if (gpsSecondCounter >= gpsCounterTarget)
    {
      recGPS();
      gpsSecondCounter = 0;
      if ((mTime/1000) < (gpsFrequency*60))
      {
        gpsCounterTarget = (gpsFrequency*60)-(mTime/1000);
      }else{
        gpsCounterTarget = gpsFrequency*60;
      }
    }
    if (pingSecondCounter >= pingCounterTarget)
    {
      digitalWrite(GSM_PIN, HIGH);
      Serial.print("IRIDIUM STARTING");
      delay(10000);
      do
        {
          Serial.println("Init Stream");
          irdSend();
          delay(50);
          rAdd = rAdd + 15;
          Serial.println(rAdd);
        
        } while (rAdd <= wAdd);
      digitalWrite(GSM_PIN, LOW);
      delay(5000);

      Serial.println("Competed Ird Transmit");
      
      pingSecondCounter = 0;
      if ((mTime/1000) < (transmissionFrequency*60))
      {
        pingCounterTarget = (transmissionFrequency*60)-(mTime/1000);
      }else{
        pingCounterTarget = transmissionFrequency*60;
      }
    }
    Serial.print(gpsSecondCounter); Serial.println(pingSecondCounter);
    Serial.flush();
    sleep_cpu();
    
}  

//********************************************************************//
//************************* END OF CODE ******************************//
//********************************************************************//
