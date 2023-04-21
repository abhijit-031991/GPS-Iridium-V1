#define TINY_GSM_MODEM_SIM800
#define ARDUINOJSON_USE_LONG_LONG 1

#include <Arduino.h>
#include <permaDefs.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SPIMemory.h>
#include <elapsedMillis.h>
#include <MCP79412RTC.h>
#include <DFRobot_LIS2DW12.h>
#include <avr/sleep.h>
#include <TimeLib.h>
#include <time.h>
#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

// Definittions //

#define SerialMon Serial
#define IridiumSerial Serial2
#define DIAGNOSTICS true // Change this to see diagnostics


// Library Definitions //

elapsedMillis mTime;
elapsedMillis Btime;
TinyGPSPlus gps;
SPIFlash flash(FCS);
MCP79412RTC rtc(false);
DFRobot_LIS2DW12_I2C acce;
IridiumSBD modem(IridiumSerial);


//*********** Setting Variables ***********//

// Mode Control variable //
int mainMode = 0;                   // switch Mode variable(0 = main, 1 = live tracking)

// GPS Control Variables //
unsigned int gpsTimeout = 120;      // GPS Timeout in seconds  *** USER CONFIG ***
int gpsFrequency = 15;              // GPS Frequency in Minutes *** USER CONFIG ***
int gpsHdop = 5;                    // GPS HDOP *** USER CONFIG ***
int sleepCounter = 0;               // Sleep Counter for live traccking mode

// GSM Control Variables //
int transmissionFrequency = 360;     // Transmission Frequency in MINS

// GPS Storage Variables // 
double lat;
double lng;
unsigned int count; 

// Memory Variables //
unsigned long wAdd = 1;
unsigned long rAdd = 0;
unsigned int cnt = 0;

// Time Variables //
time_t last_act_trigger;
time_t mortality_trigger_time;       // Mortality mode is triggered after this time
time_t strtTime;
time_t next_gps_wakeup;
time_t next_gsm_wakeup;
time_t currentTime;
time_t live_tracking_start;
time_t live_tracking_end;


// Boolean Variables //
bool activate = true;
bool ret;
bool act_mode = false;              // Activity Mode flag - shows if device is in activity mode
bool mortality = false;             // Mortality Flag - show inactivity
bool activity_enabled = false;      //** Activity Enabled Flag - 
bool rtc_int_triggered = false;
bool act_int_triggered = false;
bool wipe_memory = true;           //** Memory Wipe flag -
bool noSleepLt = false;

// Control Variables //
uint16_t pingSecondCounter;
uint16_t gpsSecondCounter;
uint16_t pingCounterTarget;
uint16_t gpsCounterTarget;

//************************************************//
//***************    FUNCTIONS    ****************//
//************************************************//
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

void recGPS(){
  Btime = 0;
  digitalWrite(GPS_PIN, HIGH);
  Serial.println(gpsTimeout*1000);
  while (Btime <= (gpsTimeout*1000))
  {
    while (Serial1.available())
    {
      if (!gps.encode(Serial1.read()))
      {
        if (!gps.location.isValid())
        {
          Serial.println(F("Acquiring"));
        }else{
          Serial.println(gps.location.isUpdated());
          Serial.print(F("Location Age:"));
          Serial.println(gps.location.age());
          Serial.print(F("Time Age:"));
          Serial.println(gps.time.age());
          Serial.print(F("Date Age:"));
          Serial.println(gps.date.age());
          Serial.print(F("Satellites:"));
          Serial.println(gps.satellites.value());
          Serial.print(F("HDOP:"));
          Serial.println(gps.hdop.hdop());
        }       
      }      
    }
    if (gps.hdop.hdop() < (double)gpsHdop && gps.location.age() < 1000 && gps.time.age() < 1000 && mTime > 3000)
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
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    dat.datetime = (uint32_t)now();
    dat.locktime = mTime/1000;
    dat.hdop = gps.hdop.hdop();
    dat.act = act_mode;
    
    
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

void read_send(){ 
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
  err = modem.sendSBDBinary((uint8_t*)&dat,sizeof(dat));
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Send Failed - "));
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println(F("Try again!!."));
  }    
}

void mortalityCheck(bool m){
  if (m == true)
  {
    if (currentTime - (last_act_trigger + 86400) > 86400)
    {
      mortality = true; 
    }    
  }  
}

void risr(){
  rtc_int_triggered = true;
  detachInterrupt(RINT);
  detachInterrupt(AINT1);
}

void aisr(){
  act_int_triggered = true;
  mortality = false;
  detachInterrupt(AINT1);
  detachInterrupt(RINT);
}

void validateAlarms(){
  if (rtc.alarm(0))
  {
    Serial.println(F("Resetting Alarm 0 Flag"));
  }
  if (rtc.alarm(1))
  {
    Serial.println(F("Resetting Alarm 0 Flag"));
  }
  time_t x = rtc.get();
  Serial.println(x);
  if (x > next_gps_wakeup)
  {
    Serial.println(F("GPS Alarm Missed"));
    next_gps_wakeup = x + 10;
    rtc.setAlarm(0, next_gps_wakeup);      
    rtc.enableAlarm(0, ALM_MATCH_DATETIME);
  }
  if (x > next_gsm_wakeup)
  {
    Serial.println(F("GSM Alarm Missed"));
    next_gsm_wakeup = x + 120;
    rtc.setAlarm(1, next_gsm_wakeup);      
    rtc.enableAlarm(1, ALM_MATCH_DATETIME);
  }  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(GPS_PIN, OUTPUT);
  pinMode(GSM_PIN, OUTPUT);
  pinMode(RTC_PIN, OUTPUT);
  Wire.swapModule(&TWI1);
  Wire.usePullups();
  Wire.begin();
  SPI.begin();
  RTC_init();

  Serial.print(F("Tag ID :")); Serial.println(tag);Serial.println();
  Serial.println(F("Initializing..."));
  
  //***************************************************//
  

  //***************************************************//

  if(!acce.begin()){
  Serial.println(F("Acc Error"));
  delay(1000);
  }else{
  Serial.print(F("chip id : "));
  Serial.println(acce.getID(),HEX);
  }
  acce.softReset();
  acce.setRange(DFRobot_LIS2DW12::e4_g);
  acce.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acce.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acce.setWakeUpDur(/*dur = */2);
  acce.setWakeUpThreshold(/*threshold = */0.3);
  acce.setPowerMode(DFRobot_LIS2DW12::eContLowPwrLowNoise1_12bit);
  acce.setActMode(DFRobot_LIS2DW12::eDetectAct);
  acce.setInt1Event(DFRobot_LIS2DW12::eWakeUp);
  acce.setDataRate(DFRobot_LIS2DW12::eRate_100hz);
  if (activity_enabled == true)
  {
    attachInterrupt(digitalPinToInterrupt(AINT1), aisr, CHANGE);
  }else{
    detachInterrupt(AINT1);
  }

//***************************************************//

  if(flash.powerUp()){
    Serial.println(F("Powered Up1"));
  }
  if(!flash.begin()){
    Serial.println(F("Starting Flash"));
    Serial.println(flash.error(VERBOSE));
  } 
  Serial.println(flash.getManID());
  if(flash.powerUp()){
    Serial.println(F("Powered Up"));
  }else{
    Serial.println(F("PWR UP Failed!"));
    Serial.println(flash.error(VERBOSE));
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
    rAdd = flash.getAddress(16);
    wAdd = flash.getAddress(16);
  }    
  if(flash.powerDown()){
    Serial.println("Powered Down");
    digitalWrite(1, HIGH);
  }else{
    Serial.println(flash.error(VERBOSE));
  }

//***************************************************//
  if (activate == true)
  {
    acqGPS();
    // strtTime = 1672052568; 
    Serial.println(strtTime);
    next_gps_wakeup = strtTime + (gpsFrequency*60);
    
    next_gsm_wakeup = strtTime + (transmissionFrequency*60);

    digitalWrite(RTC_PIN, HIGH);
    rtc.set(strtTime);
    Serial.println(rtc.get());
    delay(100);
    rtc.alarmPolarity(HIGH);
    rtc.setAlarm(0, next_gps_wakeup);
    rtc.setAlarm(1, next_gsm_wakeup);
    rtc.enableAlarm(0, ALM_MATCH_DATETIME);
    rtc.enableAlarm(1, ALM_MATCH_DATETIME);
    digitalWrite(RTC_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(RINT), risr, CHANGE);
  }

  gpsCounterTarget = gpsFrequency*60;
  pingCounterTarget = transmissionFrequency*60;  

//***************************************************//
  
  Serial.println("SYSTEM READY");
  Serial.flush();
//***************************************************//
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (mainMode)
  {
  case 0: // Normal Mode loop
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
      Serial.println(F("Ping"));
      digitalWrite(GSM_PIN, HIGH);
      delay(20000);
      Btime = 0;
      int sq;
      while (Btime < 60000)
      {
        modem.getSignalQuality(sq);
        if (sq >= 2)
        {
          break;
        }        
      }
      while (rAdd < wAdd)
      {
        if (sq >= 2)
        {
          read_send();
        }
        rAdd = rAdd + 19;
      }            
      digitalWrite(GSM_PIN, LOW);    
      
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
    break;
    
    default:
    break;
  }    
}
