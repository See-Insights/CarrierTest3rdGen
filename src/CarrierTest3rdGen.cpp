/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/CarrierTest3rdGen/src/CarrierTest3rdGen.ino"
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
* v0.40 - Moved to the new library for FRAM
* v0.50 - Changed pin definitions for the new v1.2 Boron Carrier
* v0.60 - Added i2c scan and improved the RTC alarm testing
* v0.65 - Added signal strength messaging at connection.  This is Boron Specific / need to fix this for Xenon / Argon
* v0.70 - Updated for new carrier board 1.3
*/

void setup();
void loop();
bool i2cScan();
bool framTest();
bool getTemperature();
bool rtcClockTest();
bool rtcAlarmTest();
bool batteryChargeTest();
bool watchdogTest();
void watchdogISR();
void BlinkForever();
int hardResetNow(String command);
bool meterParticlePublish(void);
void getSignalStrength();
int measureNow(String command);
#line 27 "/Users/chipmc/Documents/Maker/Particle/Projects/CarrierTest3rdGen/src/CarrierTest3rdGen.ino"
namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x00,                   // Where we store the memory map version number
    controlRegisterAddr   = 0x01,
    currentTestAddr       = 0x02,                   // What test are we on?  Some perform reset
    timeZoneAddr          = 0x03,                   // Store the local time zone data
    tomeZoneOffsetAddr    = 0x04,                   // Store the DST offset value 0 - 2
    deepSleepStartAddr    = 0x05,                   // Time we started the deep sleep test
    testdataAddr          = 0x09,
  };
};

const char releaseNumber[6] = "0.7";                // Displays the release on the menu ****  this is not a production release ****
const int FRAMversionNumber = 1;

// Included Libraries
#include "3rdGenDevicePinoutdoc.h"                 // Documents pinout
#include "MCP79410RK.h"
#include "MB85RC256V-FRAM-RK.h"

SerialLogHandler logHandler;                        // For RTC alerts and events

// Prototypes and System Mode calls
SYSTEM_THREAD(ENABLED);         // Means my code will not be held up by Particle processes.
FuelGauge batteryMonitor;       // Prototype for the fuel gauge (included in Particle core library)
MCP79410 rtc;                   // Rickkas MCP79410 libarary
MB85RC64 fram(Wire, 0);

enum State {INITIALIZATION_STATE, I2C_SCAN, FRAM_TEST, TMP36_TEST, USERSW_TEST, RTCTIME_TEST, RTCALARM_TEST, CHARGING_TEST, WATCHDOG_TEST, DEEPSLEEP_TEST, ERROR_STATE, IDLE_STATE};
State state = INITIALIZATION_STATE;

// Pin Constants for Boron
const int blueLED  = D7;                                         // This LED is on the Electron itself
const int userSwitch = D4;                                       // User switch with a pull-up resistor
const int tmp36Pin = A4;                                         // Simple Analog temperature sensor
const int donePin = D5;                                          // Pin the Electron uses to "pet" the watchdog
const int wakeUpPin = D8;                                        // This is the Particle Electron WKP pin
const int DeepSleepPin = D6;                                     // Power Cycles the Particle Device and the Carrier Board only RTC Alarm can wake

// Program Variables
byte currentState;                                               // Store the current state for the tests that might cause a reset
volatile bool watchdogInterrupt = false;                         // variable used to see if the watchdogInterrupt had fired
char resultStr[64];
char SignalString[64];                              // Used to communicate Wireless RSSI and Description


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
  Particle.variable("Signal", SignalString);

  Particle.function("MeasureNow",measureNow);


  detachInterrupt(LOW_BAT_UC);
  // Delay for up to two system power manager loop invocations
  delay(2000);
  // Change PMIC settings
  PMIC pmic;
  pmic.setInputVoltageLimit(4640);

  if (!Particle.connected()) {                                     // Only going to connect if we are in connectionMode
    Particle.connect();
    waitFor(Particle.connected,90000);                             // 60 seconds then we timeout  -- *** need to add disconnected option and test
    Particle.process();
  }

  getSignalStrength();
  Particle.publish("Status",resultStr,PRIVATE);


  waitUntil(meterParticlePublish);
  Particle.publish("Status", "Beginning Test Run",PRIVATE);


  rtc.setup();                                                     // Start the RTC code
  fram.begin();                                                    // Initializes Wire but does not return a boolean on successful initialization
  fram.get(FRAM::currentTestAddr,currentState);

  attachInterrupt(wakeUpPin, watchdogISR, RISING);                 // Need to pet the watchdog when needed

  byte rebootOrNot = EEPROM.read(0);
  (rebootOrNot) ? state = CHARGING_TEST : state = I2C_SCAN;  // Start the tests
  EEPROM.write(0,0);

}


void loop() {
  rtc.loop();                                                           // Need to run this in the main loop
  switch (state) {
    case IDLE_STATE: {
      if (currentState == 1) {
        waitUntil(meterParticlePublish);
        Particle.publish("Result","Deep Sleep Failed - Testing complete",PRIVATE);
        fram.put(FRAM::currentTestAddr,0);
        currentState = 0;
      }
    } break;
    case I2C_SCAN:
      i2cScan() ? state = FRAM_TEST : state = ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr, PRIVATE);
    break;
    case FRAM_TEST:
      framTest() ? state = TMP36_TEST : state=ERROR_STATE;
      waitUntil(meterParticlePublish);
      Particle.publish("Result",resultStr,PRIVATE);
      if (currentState == 1) {                        // The last test resets the device - we catch it here
        waitUntil(meterParticlePublish);
        Particle.publish("Result","Deep Sleep successful - Testing complete",PRIVATE);
        fram.put(FRAM::currentTestAddr,0);
        currentState = 0;
        state = IDLE_STATE;
      }
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
      if (digitalRead(userSwitch) == LOW) {
        waitUntil(meterParticlePublish);
        Particle.publish("Result","Switch Test Passed - Press detected", PRIVATE);
        state = RTCTIME_TEST;
      }
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
        state = WATCHDOG_TEST;
      }
    break;
    case WATCHDOG_TEST: {
      static time_t beginTime = Time.now();
      if (!watchdogTest())  state = ERROR_STATE;
      else if (watchdogInterrupt) {
        int elapsedMinutes = (Time.now() - beginTime)/60;
        snprintf(resultStr, sizeof(resultStr), "Watchdog Test - Passed elapsed time %i mins", elapsedMinutes);
        waitUntil(meterParticlePublish);
        Particle.publish("Result",resultStr, PRIVATE);
        state = DEEPSLEEP_TEST;
        watchdogInterrupt = false;
      }
    } break;
    case DEEPSLEEP_TEST:
      digitalWrite(DeepSleepPin,HIGH);
      fram.put(FRAM::currentTestAddr,1);                            // Put a flag in FRAM since we will be resetting device
      waitUntil(meterParticlePublish);            
      Particle.publish("Information","Deep Sleep Test - 10 seconds",PRIVATE);
      rtc.setAlarm(10);
      delay(11000);
      state = IDLE_STATE;                                             // If test is successful we will not get to this step
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

bool i2cScan() {                                            // Scan the i2c bus and publish the list of devices found
	byte error, address;
	int nDevices = 0;
  strncpy(resultStr,"i2c device(s) found at: ",sizeof(resultStr));

	for(address = 1; address < 127; address++ )
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
      char tempString[4];
      snprintf(tempString, sizeof(tempString), "%02X ",address);
      strncat(resultStr,tempString,4);
			nDevices++;
      if (nDevices == 9) break;                    // All we have space to report in resultStr
		}

		else if (error==4) {
      snprintf(resultStr,sizeof(resultStr),"Unknown error at address %02X", address);
      return 0;
		}
	}

	if (nDevices == 0) {
    snprintf(resultStr,sizeof(resultStr),"No I2C devices found");
    return 0;
  }

  return 1;
}


bool framTest() {
  int tempVersion;
  fram.get(FRAM::versionAddr, tempVersion);
  if (tempVersion != FRAMversionNumber) {                 // Check to see if the memory map in the sketch matches the data on the chip
    fram.erase();                                                          // Reset the FRAM to correct the issue
    fram.put(FRAM::versionAddr,FRAMversionNumber);                         // Put the right value in
  }

  int randomNumberWrote = random(10, 1000);
  int randomNumberRead;
  fram.put(FRAM::testdataAddr,randomNumberWrote);
  fram.get(FRAM::testdataAddr,randomNumberRead);

  if (randomNumberRead != randomNumberWrote) {
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
  if (temperatureF < 60.0 || temperatureF > 85.0) {             // Reasonable range for room temperature
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

bool rtcAlarmTest() {                                                                 // RTC Alarm and Watchdog share access to Wake Pin via an OR gate
  waitUntil(meterParticlePublish);
  Particle.publish("Information", "This alarm will reset the device", PRIVATE);
  EEPROM.write(0,1);
  delay(1000);
  rtc.setAlarm(10);
  Particle.publish("Result","If you see this - RTC Alarm test failed",PRIVATE);
  return 1;
}

bool batteryChargeTest() {
  static bool initialMessage = false;
  static unsigned long lastUpdate = 0;
  int stateOfCharge = int(batteryMonitor.getSoC());

  if (!initialMessage) {
    snprintf(resultStr, sizeof(resultStr), "Battery charge level = %i", stateOfCharge);
    waitUntil(meterParticlePublish);
    Particle.publish("Update", resultStr, PRIVATE);
    initialMessage = true;
  }

  if (stateOfCharge <= 65 && millis() - lastUpdate >= 60000) {
    snprintf(resultStr, sizeof(resultStr), "Battery charge level = %i", stateOfCharge);
    waitUntil(meterParticlePublish);
    Particle.publish("Update", resultStr, PRIVATE);
    lastUpdate = millis();
    return 0;
  }
  else if (stateOfCharge <= 65) return 0;
  else {
    snprintf(resultStr, sizeof(resultStr),"Battery Charge Test - Passed");
    return 1;
  }
}

bool watchdogTest() {
  static bool initialMessage = false;
  static time_t beginTime = Time.now();
  if (!initialMessage) {
    waitUntil(meterParticlePublish);
    Particle.publish("Information","Watchdog timer test, expect this to take about an hour", PRIVATE);
    watchdogISR();
    watchdogInterrupt = false;
    initialMessage = true;
  }

  if(Time.now() - beginTime > 4200 ) {
    snprintf(resultStr, sizeof(resultStr),"Watchdog Test Failed > 70 mins");
    return 0;
  }
  else return 1;
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

void getSignalStrength()
{
  const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};

  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();
 
  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
  snprintf(resultStr,sizeof(resultStr), "Connected: %s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int measureNow(String command) {                                           // Function to force sending data in current hour

  if (command == "1")
  {
    getSignalStrength();
    return 1;
  }
  else return 0;
}