/*
 * Project CarrierTest
 * Description: Tests the 3rd Generation Device Carrier Boron / Xenon / Argon Carrier
 * Author: Charles McClelland
 * Date: Started 11-17-2019 
 * 
 * Implements the following Tests
 * 1 - Test i2c bus and FRAM functionality
 * 2 - Test the TMP-36 for temperature
 * 3 - Test the User Switch - Requires physical Press
 * 4 - Test that the RTC is keeping time
 * 5 - Test an alarm on the RTC
 * 6 - Test Battery charging - Can take a while based on state of charge
 * 7 - Test Watchdog timer interval - Takes an hour
 * 8 - Test Deep Sleep functionality
 * 
 * v0.10 - Initial release under version control
 * v0.20 - Added the MCP79410 Library
 * v0.30 - Redid the program structure
 */

namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Where we store the memory map version number
    controlRegisterAddr   = 0x01,
    currentTestAddr       = 0x02,                   // What test are we on?  Some perform reset
    timeZoneAddr          = 0x03,                   // Store the local time zone data
    tomeZoneOffsetAddr    = 0x04,                   // Store the DST offset value 0 - 2
    deepSleepStartAddr    = 0x05,                   // Time we started the deep sleep test
  };
};

const char releaseNumber[6] = "0.3";                // Displays the release on the menu ****  this is not a production release ****
const int FRAMversionNumber = 1;

 // Included Libraries
 #include "Adafruit_FRAM_I2C.h"                     // Library for FRAM functions
 #include "FRAM-Library-Extensions.h"               // Extends the FRAM Library
 #include "3rdGenDevicePinoutdoc.h"                 // Documents pinout
 #include "MCP79410RK.h"

SerialLogHandler logHandler;                        // For RTC alerts and events

 // Prototypes and System Mode calls
 SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
 FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
 MCP79410 rtc;                   // Rickkas MCP79410 libarary

 enum State {INITIALIZATION_STATE, FRAM_TEST, TMP36_TEST, USERSW_TEST, RTCTIME_TEST, RTCALARM_TEST, CHARGING_TEST, WATCHDOG_TEST, DEEPSLEEP_TEST, ERROR_STATE, IDLE_STATE};
 State state = INITIALIZATION_STATE;
 
 // Pin Constants for Boron
 const int blueLED  = D7;                                         // This LED is on the Electron itself
 const int userSwitch = D4;                                       // User switch with a pull-up resistor
 const int tmp36Pin = A4;                                         // Simple Analog temperature sensor
 const int donePin = A3;                                          // Pin the Electron uses to "pet" the watchdog
 const int wakeUpPin = D8;                                        // This is the Particle Electron WKP pin
 const int DeepSleepPin = D6;                                     // Power Cycles the Particle Device and the Carrier Board only RTC Alarm can wake

 // Program Variables
 volatile bool watchdogInterrupt = false;                         // variable used to see if the watchdogInterrupt had fired
 char resultStr[64];


// setup() runs once, when the device is first turned on.
void setup() {
  pinMode(userSwitch,INPUT);                                      // Button for user input
  pinMode(wakeUpPin,INPUT);                                       // This pin is active HIGH
  pinMode(blueLED, OUTPUT);                                       // declare the Blue LED Pin as an output
  pinMode(donePin,OUTPUT);                                        // Allows us to pet the watchdog
  digitalWrite(donePin,HIGH);
  digitalWrite(donePin,LOW);                                      // Pet the watchdog
  pinMode(DeepSleepPin ,OUTPUT);                                  // For a hard reset active HIGH

  Particle.variable("Release",releaseNumber);
  //Particle.variable("stateOfChg", stateOfCharge);
  //Particle.function("HardReset",hardResetNow);
  //Particle.function("Set-Timezone",setTimeZone);
  //Particle.function("Set-DSTOffset",setDSTOffset);

  if (!Particle.connected()) {                                     // Only going to connect if we are in connectionMode
    Particle.connect();
    waitFor(Particle.connected,90000);                             // 60 seconds then we timeout  -- *** need to add disconnected option and test
    Particle.process();
  }

  rtc.setup();                                                     // Start the RTC code

  attachInterrupt(wakeUpPin, watchdogISR, RISING);                 // Need to pet the watchdog when needed

  state = FRAM_TEST;                                               // Start the tests
  Particle.publish("Test Start", "Beginning Test Run",PRIVATE);
}


void loop() {
  rtc.loop();                                                           // Need to run this in the main loop
  switch (state) {
    case IDLE_STATE:
    break;
    case FRAM_TEST:
      framTest() ? state = TMP36_TEST : state=ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr,PRIVATE);
    break;
    case TMP36_TEST:
      getTemperature() ? state = USERSW_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr,PRIVATE);
    break;
    case USERSW_TEST: {                                                             // Test the user switch
      static bool firstPublish = false;
      if (!firstPublish) {
        waitUntil(meterParticlePublish);
        Particle.publish("Prompt","Please press user switch", PRIVATE);
        firstPublish = true;
      }
      //if (digitalRead(userSwitch)) {
        waitUntil(meterParticlePublish);
        Particle.publish("Result","Switch Test Passed - Press detected", PRIVATE);
        state = RTCTIME_TEST;
      //}
    } break;
    case RTCTIME_TEST:
      rtcClockTest() ? state = RTCALARM_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr, PRIVATE);
    break;
    case RTCALARM_TEST: 
      rtcAlarmTest() ? state = CHARGING_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr, PRIVATE);
    break;
    case CHARGING_TEST:
      if (batteryChargeTest()) {
        waitUntil(meterParticlePublish);
        Particle.publish("Result",resultStr, PRIVATE);
      }
    break;
    case ERROR_STATE: 
      waitUntil(meterParticlePublish);
      Particle.publish("Error","Testing halted",PRIVATE);
      state = IDLE_STATE;
    break;
    default:
      state = IDLE_STATE;
    break;
  }
}


/*
 




  Particle.publish("Test #6", "Battery charge test passed", PRIVATE);
  time_t beginTime = Time.now();
  watchdogISR();
  watchdogInterrupt = false;

  if (Particle.connected()) Particle.publish("Test #7 Started","Expect this test to take ~60 minutes",PRIVATE);
  delay(1000);
  Particle.process();

  while(!watchdogInterrupt) {
    Particle.process();
    delay(1000);
  }

  int elapsedMinutes = (Time.now() - beginTime)/60;
  snprintf(data, sizeof(data), "Elapsed time in minutes is %i", elapsedMinutes);
  if (Particle.connected()) Particle.publish("Test #7 Finished", data ,PRIVATE);
  delay(1000);
  Particle.process();

}
*/

bool framTest() {
  if (!fram.begin()) {                                                // You can stick the new i2c addr in here, e.g. begin(0x51);
    snprintf(resultStr, sizeof(resultStr),"FRAM Test Failed - Missing FRAM");
    return 0;
  }
  else if (FRAMread8(FRAM::versionAddr) != FRAMversionNumber) {                 // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                      // Reset the FRAM to correct the issue
  }

  if (FRAMread8(FRAM::versionAddr) != FRAMversionNumber) {
    snprintf(resultStr, sizeof(resultStr),"FRAM Test Failed - FRAM Read Error");
    return 0;
  } 
  else  {
    snprintf(resultStr, sizeof(resultStr),"FRAM Test Passed");
    return 1;
  }
}

bool getTemperature() {
  int reading = analogRead(tmp36Pin);   //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;        // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                    // Electron is different than the Arduino where there are only 1024 steps
  float temperatureC = (voltage - 0.5) * 100.0;  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;  // now convert to Fahrenheit
  if (temperatureF < 60.0 || temperatureF > 75.0) {             // Reasonable range for room temperature
    snprintf(resultStr, sizeof(resultStr),"Temp seems whack: %3.1f", temperatureF);
    return 0;
  }
  else {
    snprintf(resultStr, sizeof(resultStr),"Temperature is: %3.1f", temperatureF);
    return 1;
  }
}

bool rtcClockTest() {
  if (!rtc.isRTCValid()) {
    snprintf(resultStr, sizeof(resultStr),"RTC Clock Test Failed");
    return 0;
  }
  else {
    snprintf(resultStr, sizeof(resultStr),"RTC Clock Passes - Time is %s GMT",(const char*)Time.timeStr(rtc.getRTCTime()));
    return 1;
  }
}

bool rtcAlarmTest() {                                 // This is a miss need to connect to a pin
  waitUntil(meterParticlePublish);
  Particle.publish("Information", "Setting an alarm for 10 seconds", PRIVATE);

  // Need to connect the MFP pin from the RTC to the Boron and set an interrupt here
  // Will make this connection and add the code here.

  time_t RTCtime = rtc.getRTCTime();
  rtc.setAlarm(RTCtime + 10);

  delay(11000);
  //waitFor(rtc.getInterrupt,15);

  if (!rtc.getInterrupt()) {
    snprintf(resultStr, sizeof(resultStr),"RTC Alarm Test Failed");
    return 0;
  }
  else {
    snprintf(resultStr, sizeof(resultStr),"RTC Alarm Test Passed");
    rtc.clearInterrupt();
    return 1;
  }
}

bool batteryChargeTest() {
    static unsigned long lastUpdate = 0;
    int stateOfCharge = int(batteryMonitor.getSoC());
    if (stateOfCharge <= 65 && millis() - lastUpdate >= 60000) {
      snprintf(resultStr, sizeof(resultStr), "Battery charge level = %i", stateOfCharge);
      waitUntil(meterParticlePublish);
      Particle.publish("Update", resultStr, PRIVATE);
      return 0;
    }
    else if (stateOfCharge <= 65) return 0;
    else {
      snprintf(resultStr, sizeof(resultStr),"RTC Alarm Test Passed");
      rtc.clearInterrupt();
      return 1;
    }
}


// Utility Functions Area

void watchdogISR()
{
  watchdogInterrupt = true;
  digitalWrite(donePin, HIGH);                              // Pet the watchdog
  digitalWrite(donePin, LOW);
}

void BlinkForever() {
  delay(1000);
  Particle.publish("Test Failed" "Reset Device to Continue", PRIVATE);
  while(1) {
    digitalWrite(blueLED,HIGH);
    delay(2000);
    digitalWrite(blueLED,LOW);
    delay(2000);
    Particle.process();
  }
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
   // digitalWrite(hardResetPin,HIGH);                                  // This will cut all power to the Electron AND the carrir board
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

bool meterParticlePublish(void) {                                       // Enforces Particle's limit on 1 publish a second
  static unsigned long lastPublish=0;                                   // Initialize and store value here
  if(millis() - lastPublish >= 1000) {                                  // Particle rate limits at 1 publish per second
    lastPublish = millis();
    return 1;
  }
  else return 0;
}
