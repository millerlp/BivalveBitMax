/* BivalveBitMax_logger.ino
 *  BivalveBit data logger program designed for use with 
 *  MAX3010x heart sensor and Allegro A1395 Hall effect sensor
 *  
 *  Samples gape sensor every minute. Set the heart
 *  sampling interval by changing the value for the variable heartMinute
 *  below.
 *  
 *  If the device just slowly flashes red after startup, the real time clock
 *  needs to be reset. Open the serial monitor and enter the correct date and
 *  time (in the UTC time zone preferably) using the command format:
 *  SETDATE YYYY-MM-DD HH:MM:SS
 *  
 *  
 *  If the device slowly flashes red 15 times at startup, the heart sensor was not detected
 *  The device will still try to proceed with data collection (getting gape data hopefully)
 *  but will flash red rapidly whenever it attempts to collect heart data (at every heart
 *  sampling interval). Rapid green flashes at the heart data interval represent good (non-zero)
 *  readings coming from the heart sensor. 
 *  
 *  
 *  If you want to manually reset the clock via the Serial monitor in this program, either 
 *  remove the backup battery temporarily to stop the clock, or unplug the heart monitor,
 *  and use the Serial monitor to reset the time during the red LED flashing. 
 */


#include "SdFat.h" // https://github.com/greiman/SdFat (you must be using ver. 2.1.2 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/cpufunc.h>
//#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
//#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940  Real time clock
#include "MAX30105.h"         // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library

#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib

/* Define heartMinute, to be used to set which minutes of the hour to take
 *  heart rate samples. This value will be divided into the current minute
 *  value, and if the modulo (remainder) is 0, then the heart rate sampling
 *  will be activated. 
 *  A value of 1 would sample every minute
 *  A value of 2 would sample every even-numbered minute
 *  A value of 5 would sample every 5 minutes
 */
unsigned int heartMinute = 2; 
// Define number of heart samples to take in 1 minute 
// (240 @ 8Hz = 30sec, 360 = 45sec, 400 @ 8 Hz = 50 sec, 480 @ 8Hz = 1minute)
#define HEART_SAMPLE_LENGTH 400 
unsigned int heartCount = 0; // Current number of heart samples taken in this minute
uint32_t heartBuffer[HEART_SAMPLE_LENGTH] = {0}; // array to save heart measurements before writing to SD

/* Define gapeMinute, to be used to set which minutes of the hour to take
 *  gape samples. This value will be divided into the current minute
 *  value, and if the modulo (remainder) is 0, then the gape sampling
 *  will be activated. 
 *  A value of 1 would sample every minute
 *  A value of 2 would sample every even-numbered minute
 *  A value of 5 would sample every 5 minutes
 */
unsigned int gapeMinute = 1;
bool writeGapeFlag = false; // Used to flag whether to write new hall data to SD

// -------------------------------------------
// ***** STATE MACHINE TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_1MINUTE_SLEEP, // sleep until new minute turns over based on RTC alarm
  STATE_FAST_SAMPLE,  // sleep for short interval based on RTC wakeup signal (8Hz)
  STATE_SHUTDOWN // close data file, shut down data collection due to low battery
} mainState_t;
// main state machine variable, this takes on the various
// values defined for the STATE typedef above. 
mainState_t mainState;

// Type definition for a case structure to write to SD card or skip writing
typedef enum WRITE_SD
{
  WRITE_HALL_TEMP_VOLTS, // collect Hall sensor data and battery voltage data
  WRITE_HEART, // collect heart rate data at sub-second intervals
  WRITE_NOTHING // don't write anything
  
} writeState_t;
writeState_t writeState;

/*******************************************************
 * SD card objects
 *******************************************************/
// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6
SdFat sd; 
//File IRFile; //SD card object 1 (IR heart rate data)
//File GAPEFile; //SD card object 2 (Gape, Temp, Battery voltage data)
SdFile IRFile; //SD card object 1 (IR heart rate data)
SdFile GAPEFile; //SD card object 2 (Gape, Temp, Battery voltage data)
bool SDfailFlag = false;
// Placeholder serialNumber
char serialNumber[] = "SN000";
bool serialValid = false;
// Declare initial name for output files written to SD card
char heartfilename[] =  "YYYYMMDD_HHMM_00_SN000_IR.csv";
char gapefilename[] =   "YYYYMMDD_HHMM_00_SN000_GAPE.csv";


/************************************************************
 *  // MAX30105 heart sensor parameters
 */

MAX30105 max3010x; // MAX3010x sensor object (MAX30101, MAX30102, MAX30105 all interchangeable)
// sensor configurations
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. Only use 2
byte REDledBrightness = 1; // low value of 0 shuts it off, 1 is barely on
byte IRledBrightness = 30;  // Starting value around 30-60 is probably reasonable for bivalves
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32, but only use 1 or 2. The others are too slow
int pulseWidth = 215; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs. Recommend 215
// For 118us, max sampleRate = 1000; for 215us, max sampleRate = 800, for 411us, max sampleRate = 400
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384. 4096 is standard

float tempC;    // output variable for temperature from MAX3010x

// Prototype for the restartAndSampleMAX3010x function (see full function at bottom of file)
uint32_t restartAndSampleMAX3010x(MAX30105 &max3010x, byte IRledBrightness, \
                                  byte sampleAverage, byte ledMode, int sampleRate,\
                                  int pulseWidth, int adcRange, byte REDledBrightness,\
                                  bool EnableTemp=false);
/************************************************************
 * Hall effect sensor definitions
 ************************************************************/
#define ANALOG_IN A0  // Analog input pin for Hall sensor
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
unsigned int HallValue = 0; // Variable for Hall sensor reading

/***************************************************************************************************
** Declare global variables and instantiate classes for MCP79400 real time clock                                              
*   This library is written for the MCP7940, but works with the BivalveBit MCP79400
***************************************************************************************************/
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940

const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
DateTime now; // Variable to hold current time
DateTime firsttimestamp; // Variable to track fast sampling start 
DateTime lasttimestamp; // Variable to track fast sampling end time
byte oldday1; // Used to track midnight change
byte oldday2; // Used to track midnight change
/*! ///< Enumeration of MCP7940 alarm types */
enum alarmTypes {
  matchSeconds,
  matchMinutes,
  matchHours,
  matchDayOfWeek,
  matchDayOfMonth,
  Unused1,
  Unused2,
  matchAll,
  Unknown
};

//------------------------------------------------------------
//  Battery monitor 
#define BATT_MONITOR_EN  9 // digital output channel to turn on battery voltage check
#define BATT_MONITOR  A1  // analog input channel to sense battery voltage
float dividerRatio = 2; // Ratio of voltage divider (47k + 47k) / 47k = 2
float refVoltage = 3.00; // Voltage at AREF pin on ATmega microcontroller, measured per board
float batteryVolts = 0; // Estimated battery voltage returned from readBatteryVoltage function
float minimumVoltage = 3.4; // Minimum safe voltage for a Li-Ion battery
unsigned int lowVoltageCount = 0; // Count how many times voltage is too low
unsigned int lowVoltageCountLimit = 10; // Number of loops after which the program should be shutdown
#define REDLED 11   // Red LED pin
#define GRNLED 8    // Green LED pin
#define VREG_EN 24  // voltage regulator enable


void test_pin_init(void){
    /* Make High (OFF) */
    PORTC.OUT |= PIN0_bm; // BivalveBit green led on pin PC0 (arduino pin 8)
    /* Make output */
    PORTC.DIR |= PIN0_bm;

    /* Make High (OFF) */
    PORTC.OUT |= PIN3_bm; // BivalveBit red led on pin PC3 (arduino pin 11)
    /* Make output */
    PORTC.DIR |= PIN3_bm;
    
    /* Make High (OFF) */
    PORTD.OUT |= PIN2_bm; // Pin PD2 (probe with scope/analyzer)
    /* Make output */
    PORTD.DIR |= PIN2_bm;
}

/*****************************************************************
 * Interrupt service routine for the periodic interrupt timer,
 * used to generate 8Hz wakeup interrupts. Setup of the 8Hz timer
 * is done in BivalveBit_lib using PIT_init() function
 *****************************************************************/
ISR(RTC_PIT_vect)
{
    /* If you have arrived in this ISR due to a PIT interrupt, 
     *  You must clear PIT interrupt flag by writing '1': 
     */
    RTC.PITINTFLAGS = RTC_PI_bm; // clear the interrupt flag
//    PORTD.OUTTGL |= PIN2_bm; // Toggle PD2 (probe with scope/analyzer)
//    PORTC.OUTTGL |= PIN0_bm; // BivalveBit green led on pin PC0 
}



//---------------------------------------------------------------
//-------- Setup
//---------------------------------------------------------------
void setup() {
  Serial.begin(57600);
  Serial.println("Hello");
  analogReference(EXTERNAL);
  setUnusedPins(); // in BivalveBit_lib
  disableUnusedPeripherals(); // in BivalveBit_lib
  pinMode(VREG_EN, OUTPUT);   // Voltage regulator pin
  digitalWrite(VREG_EN, LOW); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(20, INPUT_PULLUP); // pin PF0, attached to RTC multi-function pin
  // Battery monitor pins
  pinMode(BATT_MONITOR, INPUT); // Battery voltage input channel
  pinMode(BATT_MONITOR_EN, OUTPUT); // Battery monitor enable pin
  digitalWrite(BATT_MONITOR_EN, LOW); // pull low to turn off battery monitor circuit
  test_pin_init(); // Set up the LEDs and PD2 output pin
  //---------------------------------
  // Retrieve board serial number
  EEPROM.get(0, serialNumber);
  if (serialNumber[0] == 'S') {
    serialValid = true; // set flag   
    Serial.print("Serial number: "); Serial.println(serialNumber); 
  } else {
    serialValid = false;
    Serial.print("No serial number");
  }
  //----------------------------------------------------------
  // SD card initialization
  pinMode(SD_CHIP_SELECT, OUTPUT); // SD card chip select pin
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    SDfailFlag = true;
    for (int i = 0; i < 50; i++){
      digitalWrite(REDLED, !digitalRead(REDLED));
      delay(100);
      digitalWrite(GRNLED, !digitalRead(GRNLED));
      delay(100);
    }
    return;
  } else {
    Serial.println("SD Card initialized.");
  }
  digitalWrite(GRNLED,LOW); // set low to turn on
  delay(500);
  digitalWrite(GRNLED,HIGH); // set high to turn off

  

   /************************************************************
   *  Real Time Clock startup
   ***********************************************************/
  Wire.begin();
  Wire.setClock(100000);
  MCP7940setup();
  // Turn on battery backup, default is off
  MCP7940.setBattery(true); // enable battery backup mode
  Serial.print("Battery Backup mode is ");
  if (MCP7940.getBattery()) {
   Serial.println("enabled.");
  } else {
   Serial.println("disabled.");
  }
  now = MCP7940.now();  // get the current time
  // Use sprintf() to pretty print date/time with leading zeroes
  sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.print("Date/Time: "); Serial.println(inputBuffer);
  
  bool clockErrorFlag = true;
  while (clockErrorFlag){
    // Check that real time clock has a reasonable time value
    if ( (now.year() < 2024) | (now.year() > 2035) ) {
        Serial.println("Please set clock to current UTC time");
        Serial.println("Use format SETDATE YYYY-MM-DD HH:MM:SS");
       // Error, clock isn't set
       while(clockErrorFlag){
        digitalWrite(REDLED, !digitalRead(REDLED));
        delay(500);
        readCommand();
        now = MCP7940.now();  // get the updated time
        if ( (now.year() >= 2024) & (now.year() <= 2035) ){
          // If the year is now within bounds, break out of this while statement
          clockErrorFlag = false;
          sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
          Serial.print("Date/Time: "); Serial.println(inputBuffer);
          break;
        }
        
       }
    } else {
      // If the time's year was in bound, continue with the program
      clockErrorFlag = false;
    }
  }
  
   //---------------------------------------------------------------
  // Enable voltage regulator to power MAX3010x heart sensor
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  delay(250);

  /*****************************************
   * Start up MAX3010x sensor (heart sensor)
   *****************************************/
  if (max3010x.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 100kHz speed
  {
    Serial.println(F("MAX heart sensor initialized"));
    digitalWrite(GRNLED,LOW); // set low to turn on
    delay(250);
    digitalWrite(GRNLED,HIGH); // set high to turn off
//    oled.println("Heart sensor on");
  } else {
//    oled.println("Heart sensor fail");
    Serial.println(F("Did not find MAX heart sensor"));
    for (int c = 0; c < 15; c++){
      digitalWrite(REDLED,LOW); // set low to turn on, leave on due to the error
      delay(500);
      digitalWrite(REDLED,HIGH); // turn off
      delay(500);
      readCommand();
      now = MCP7940.now();  // get the updated time
      sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
      Serial.print("Date/Time: "); Serial.println(inputBuffer);
    }
    digitalWrite(REDLED,LOW); // set low to turn on, leave on due to the error
  }
  
  digitalWrite(VREG_EN, LOW); // set low to turn off voltage regulator for MAX3010x and Hall sensor
 
  
  

  oldday1 = oldday2 = now.day(); // Store current day's value
  // Initialize two data files
  initHeartFileName(sd, IRFile, now, heartfilename, serialValid, serialNumber);
  initGapeFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
  digitalWrite(REDLED, HIGH); // turn off
  Serial.print("Gape sample interval: "); Serial.print(gapeMinute); Serial.println(" minutes");
  Serial.print("Heart sample interval: "); Serial.print(heartMinute); Serial.println(" minutes");
  
  MCP7940Alarm1Minute(now); // Set RTC multifunction pin to alarm when new minute hits
  attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC

  if (gapeMinute > heartMinute){
    // Make sure gape is sampling at least as often as the heart sensor
    gapeMinute = heartMinute;
  }
  
  mainState = STATE_1MINUTE_SLEEP;
  writeState = WRITE_NOTHING;
  SLPCTRL_init(); // in BivalveBitlib - sets up sleep mode
  sleep_cpu();    // put the cpu to sleep

}     // end setup


//--------------------------------------------------------------------
//----------------- Main loop
//--------------------------------------------------------------------
void loop() {
//        unsigned long m1 = millis();  

  switch(mainState) {
    case STATE_1MINUTE_SLEEP:
    {
        /* You arrived here because you're in the 1-minute sleep cycle
         *  
         */
        now = MCP7940.now();  // get the current time
        // Start by clearing the RTC alarm
        MCP7940.clearAlarm(0); // clear the alarm pin (reset the pin to high)
        Serial.println("Woke from 1 minute alarm");
        sprintf(inputBuffer, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
        Serial.println(inputBuffer);        delay(10);

        if ( now.minute() % gapeMinute == 0) {
            writeGapeFlag = true;
            // Since a new minute just started, sample Hall & battery, and 
            // then decide whether to switch to 8Hz fast sampling or stay in 
            // 1-minute sleep cycle
            bitWrite(PORTC.OUT, 0, 0); // Set PC0 low to turn on green LED
            unsigned long firstmillis = millis();
            digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator
            
            batteryVolts = readBatteryVoltage(BATT_MONITOR_EN, BATT_MONITOR,\
                                                dividerRatio, refVoltage);
  //        Serial.print("Battery: ");Serial.print(batteryVolts,2);Serial.println("V");delay(10);
            
            HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP); 
            
            // Enable the MAX3010x sensor so we can get the temperature 
            uint32_t junk = restartAndSampleMAX3010x(max3010x, IRledBrightness, \
                  sampleAverage, ledMode, sampleRate, pulseWidth, \
                  adcRange, REDledBrightness, true);
            tempC = max3010x.readTemperature();  // May take at least 29ms, up to 100ms
                                                      
            bitWrite(PORTC.OUT, 0, 1); // Turn off PC0 (green LED) by setting pin high
            if (batteryVolts < minimumVoltage) {
              ++lowVoltageCount; // Increment the counter
            }
          } else {
            // It's not a gapeMinute, so don't do much
            writeGapeFlag = false; // Don't try to write data to SD card this time
            writeState = WRITE_NOTHING;
          }
          digitalWrite(VREG_EN, LOW); // set low to turn off after sampling

        
        // Check time, decide what kind of wake interval and sampling to proceed with
        if ( now.minute() % heartMinute == 0) {
          /* If it's a heart-sampling minute change mainState to STATE_FAST_SAMPLE
           * to switch to 8Hz wake interval
          */
          if (lowVoltageCount < lowVoltageCountLimit) {
            // Continue normal sampling routine
            mainState = STATE_FAST_SAMPLE; // switch to fast sampling next              
            writeState = WRITE_HALL_TEMP_VOLTS; // Save the gape/temp/volts data at start of fast sampling
          } else {
            // The battery voltage has been low for too long, shutdown
            mainState = STATE_SHUTDOWN;
            writeState = WRITE_HALL_TEMP_VOLTS;
          }
        } else {
          /* If it's a non-heart minute, update 1 minute wake interval,
          * and remain in STATE_1MINUTE_SLEEP
          */    
          if ( (lowVoltageCount < lowVoltageCountLimit) & writeGapeFlag) {
            mainState = STATE_1MINUTE_SLEEP; // Remain in 1-minute sleep mode  
            writeState = WRITE_HALL_TEMP_VOLTS; // Write gape, temperature, voltage to SD
          } else if ((lowVoltageCount < lowVoltageCountLimit) & !writeGapeFlag) {
            // It's not a writeGape minute, so don't write to SD
            mainState = STATE_1MINUTE_SLEEP; // Remain in 1-minute sleep mode
            writeState = WRITE_NOTHING; // Don't write anything this round
          } else if ( (lowVoltageCount >= lowVoltageCountLimit) & writeGapeFlag ) {
            // The battery voltage has been low for too long, shutdown
            mainState = STATE_SHUTDOWN;
            writeState = WRITE_HALL_TEMP_VOLTS;
          } else if ( (lowVoltageCount >= lowVoltageCountLimit) & writeGapeFlag ) {
            // The battery voltage has been low for too long, shutdown
            mainState = STATE_SHUTDOWN;
            writeState = WRITE_NOTHING; // don't write anything this round
          }
//          Serial.println("Staying in 1 minute sleep loop"); delay(10);
          // Re-enable 1-minute wakeup interrupt
          attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC          
        }
    }
    break;

    case STATE_FAST_SAMPLE:
    {
        // If you arrive here from STATE_1MINUTE_SLEEP, the 8Hz wakeup signal should
        // be in effect, and the Hall sensor will already have been read, so just
        // take a heart sensor reading and add it to the buffer
        // Keep count of how many times you visit this section, and revert to 
        // STATE_1MINUTE_SLEEP after you've taken enough heart readings (30 or 60 seconds) 

        // Take heart sensor sample, add it to the buffer
        
        digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator
        // Flash the green LED briefly, this also gives the voltage regulator time to 
        // stabilize
        if (heartCount > 0) {
          if (heartBuffer[ (heartCount-1)] > 0){
            // Flash the green LED to denote successful sampling
            bitWrite(PORTC.OUT, 0, 0); // Set PC0 low to turn on green LED
            delayMicroseconds(150); // Give voltage regulator time to stabilize
            bitWrite(PORTC.OUT, 0, 1); // Turn off PC0 by setting pin high
          } else if (heartBuffer[ (heartCount-1)] == 0) {
            // If the heart sensor is returning 0's (because it's not attached or malfunctioning)
            // then flash the red LED instead
            bitWrite(PORTC.OUT, 3, 0); // Set PC3 low to turn on red LED
            delayMicroseconds(150); // Give voltage regulator time to stabilize
            bitWrite(PORTC.OUT, 3, 1); // Turn off PC3 by setting pin high
          }
        }
        

        /*********************************************
         *  Collect a sample from the MAX3010x sensor
         *********************************************/
         heartBuffer[heartCount] = restartAndSampleMAX3010x(max3010x, IRledBrightness,\
                  sampleAverage, ledMode, sampleRate, pulseWidth, adcRange, \
                  REDledBrightness, false);
                
        ++heartCount; // Increment heart sample counter
        digitalWrite(VREG_EN, LOW); // set low to disable voltage regulator
        
//        Serial.println(heartBuffer[heartCount-1]);
//        delay(5);
                
        // If enough samples have been taken, revert to 1 minute intervals
        if ( heartCount > (HEART_SAMPLE_LENGTH-1) ) {
          heartCount = 0; // reset for next sampling round        
          writeState = WRITE_HEART; // switch to write SD state 
          lasttimestamp = MCP7940.now(); // Store last time stamp of fast sampling run        
          digitalWrite(VREG_EN, LOW); // set low to disable voltage regulator
        }  
    }
    break;


    case STATE_SHUTDOWN:
    {
        // If you arrive here due to too many low battery voltage readings, 
        // turn off the RTC alarm, detach the interrupt, close all files, and let the 
        // device go into sleep mode one more time (permanently)
        Serial.println("Oops, shutdown"); delay(10);
        digitalWrite(VREG_EN, LOW); // set low to disable voltage regulator
        detachInterrupt(digitalPinToInterrupt(20)); // Remove the current 1 minute interrupt
        MCP7940.setSQWState(false); // turn off square wave output if currently on
        RTC.PITINTCTRL &= ~RTC_PI_bm; /* Periodic Interrupt: disabled, cancels 8Hz wakeup */
        MCP7940.setAlarmState(0, false); // Deactivate alarm
        // Now allow cpu to re-enter sleep at the end of the main loop
    }
    break;
  }   // end of mainState switch block


  switch (writeState){
    case WRITE_NOTHING:
        // Do nothing here, it's not time to write any data to SD
    break;
    case WRITE_HALL_TEMP_VOLTS:
      // You arrive here because the hall sensor, temperature and battery
      // voltage have all been sampled. Write them to SD
//      Serial.println("Write gape, temp, volts"); delay(8);
      // Check if a new day has started, start a new data file if so.
      if (now.day() != oldday1) {
        GAPEFile.close(); // close old file
        // Open new file
        initGapeFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
        oldday1 = now.day(); // update oldday1
      }
      // Write hall, temperature, battery voltage to SD
      writeGapeToSD(now); // See function at bottom of program
      
      // Reset to write-nothing state 
      writeState = WRITE_NOTHING;
      // The main state could be going back to 1-minute intervals or switching
      // to fast sampling heart rate, depending on what happened in the 
      // mainState switch above. Check which state we're in to decide what to do next 
      switch(mainState){
        // If we've arrived here and are switching to fast sample, activate the PIT timer
        case (STATE_FAST_SAMPLE):
          firsttimestamp = MCP7940.now(); // store value for start of IR samples
          detachInterrupt(digitalPinToInterrupt(20)); // Remove the current 1 minute interrupt
//          digitalWrite(VREG_EN, HIGH); // set high to enable voltage regulator here if you wanted to 
                                        // make sure it's running at the very start of the fast sampling
//          setupVCNL4040(); // reset the IR sensor after power-up of the voltage regulator
          MCP7940sqw32768(); // Reactivate the RTC's 32.768kHz clock output

          PIT_init(); // set up the Periodic Interrupt Timer to wake at 8Hz (BivalveBitlib)
          
          sei();  // enable global interrupts
          // head to end of main loop to go to sleep
        break;
        
        case (STATE_1MINUTE_SLEEP):
          // If we've arrived here and it's remaining a 1-minute sleep interval, do nothing      
        break;
        default:
          // Do nothing in the default case
        break;
      }
    break;

    case WRITE_HEART:
      // You arrive here after doing the fast heart rate samples and filling the
      // buffer. Write those data to the SD card heart rate file, and then 
      // reset the mainState to 1-minute intervals

      // Cancel 8Hz wakeups
      RTC.PITINTCTRL &= ~RTC_PI_bm; /* Periodic Interrupt: disabled, cancels 8Hz wakeup */ 
      
//      Serial.println("Write heart data"); delay(8);
      // Check if a new day has started, if so open a new file
      if (firsttimestamp.day() != oldday2){
        IRFile.close();
        initHeartFileName(sd, IRFile, firsttimestamp, heartfilename, serialValid, serialNumber);
        oldday2 = firsttimestamp.day(); // update oldday2
      }
      
      writeHeartToSD(firsttimestamp,lasttimestamp); // write IR data to SD card
      // Revert to 1-minute wake interrupts
      mainState = STATE_1MINUTE_SLEEP; // reset to 1 minute sleep interval
      writeState = WRITE_NOTHING; // reset to write-nothing state 
      MCP7940Alarm1Minute(MCP7940.now()); // Reset 1 minute alarm
      attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC
    break;

    default:
      // Nothing in default case
    break;
  } // end of writeState switch statement

  /***************************************************************** 
   *  At the end of the main loop, put the cpu to sleep. When it
   *  wakes up, detach the RTC's alarm pin interrupt and go back 
   *  to the top of the main loop to check the time and decide what
   *  to do next in the mainState chunk
   */
//  Serial.print( (millis() - m1) ); Serial.println(" <- cycle time"); delay(8);
  sleep_cpu(); 
//  detachInterrupt(digitalPinToInterrupt(20));
}     // end main loop


//---------------------------------------------------------
// Interrupt for pin 20, connected to RTC MFP
// Used for the 1-minute alarms
void RTC1MinuteInterrupt() {
  // Do nothing, this just wakes the microcontroller from sleep
}


//------------------------------------------------------
void MCP7940setup() {
   while (!MCP7940.begin())  // Loop until the RTC communications are established
  {
    Serial.println(F("Unable to find MCP7940. Checking again in 3s."));
    delay(3000);
  }  // of loop until device is located
  Serial.println(F("MCP7940 initialized."));
  while (!MCP7940.deviceStatus())  // Turn oscillator on if necessary
  {
    Serial.println(F("Oscillator is off, turning it on."));
    bool deviceStatus = MCP7940.deviceStart();  // Start oscillator and return state
    if (!deviceStatus)                          // If it didn't start
    {
      Serial.println(F("Oscillator did not start, trying again."));
      delay(1000);
    }  // of if-then oscillator didn't start
  }    // of while the oscillator is of
  MCP7940.setSQWState(false); // turn off square wave output if currently on

}

//---------------------------------------------------------
void MCP7940Alarm1Minute(DateTime currentTime){
  MCP7940.setSQWState(false); // turn off square wave output if currently on
  MCP7940.setAlarmPolarity(false); // pin goes low when alarm is triggered
  Serial.println("Setting alarm for every minute at 0 seconds.");
  MCP7940.setAlarm(0, matchSeconds, currentTime - TimeSpan(0, 0, 0, currentTime.second()), true);  // Match once a minute at 0 seconds
                   
  
//  DateTime currentTime2 = currentTime - TimeSpan(0, 0, 0, currentTime.second());  // Add interval to current time
////  DateTime  dt=  currentTime - TimeSpan(0,0,0,currentTime.second() );
////  Serial.print("Time diff = "); Serial.println(dt.unixtime());
//   // Use sprintf() to pretty print date/time with leading zeroes
//   char          inputBuffer2[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
//  sprintf(inputBuffer2, "%04d-%02d-%02d %02d:%02d:%02d", currentTime.year(), 
//          currentTime2.month(), currentTime2.day(),
//          currentTime2.hour(), currentTime2.minute(), currentTime2.second());
//  Serial.print("Next alarm check: "); Serial.println(inputBuffer2);

  delay(10);
}

//--------------------------------------------
// Set RTC to generate a 32.768kHz signal on its multifunction pin
void MCP7940sqw32768(){
//  MCP7940.setAlarm(0,0, DateTime(2020,1,1,1,1,1)); // Deactivate alarm
  MCP7940.setAlarmState(0, false); // Deactivate the alarm
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency (0=1Hz, 1 = 4.096kHz, 2 = 8.192kHz, 3 = 32.768kHz)
  MCP7940.setSQWState(true); // turn on the square wave output pin
}


//------------- writeGapeToSD -----------------------------------------------
// writeGapeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeGapeToSD (DateTime timestamp) {
  // Reopen logfile. If opening fails, notify the user
  if (!GAPEFile.isOpen()) {
    if (!GAPEFile.open(gapefilename, O_RDWR | O_CREAT | O_AT_END)) {
      digitalWrite(REDLED, HIGH); // turn on error LED
    }
  }
  // Write the unixtime
  GAPEFile.print(timestamp.unixtime(), DEC);GAPEFile.print(F(",")); // POSIX time value
  printTimeToSD(GAPEFile, timestamp); GAPEFile.print(F(","));// human-readable time stamp
  GAPEFile.print(serialNumber); GAPEFile.print(F(",")); // Serial number
  GAPEFile.print(HallValue); GAPEFile.print(F(","));  // Hall sensor value
  GAPEFile.print(tempC); GAPEFile.print(F(","));      // Temperature sensor value
  GAPEFile.print(batteryVolts); // Battery voltage
  GAPEFile.println();
  // GAPEFile.close(); // force the buffer to empty
  GAPEFile.timestamp(T_WRITE, timestamp.year(),timestamp.month(), \
      timestamp.day(),timestamp.hour(),timestamp.minute(),timestamp.second());
  
}


//------------- writeHeartToSD -----------------------------------------------
// writeHeartToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeHeartToSD (DateTime firsttimestamp, DateTime lasttimestamp) {
  // Reopen logfile. If opening fails, notify the user
  if (!IRFile.isOpen()) {
    if (!IRFile.open(heartfilename, O_RDWR | O_CREAT | O_AT_END)) {
      digitalWrite(REDLED, HIGH); // turn on error LED
    }
  }
  for (unsigned int bufferIndex = 0; bufferIndex < HEART_SAMPLE_LENGTH; bufferIndex++){
      if (bufferIndex == 0) {
        // For first value in buffer, write first time stamp
        IRFile.print(firsttimestamp.unixtime(), DEC); IRFile.print(F(","));// POSIX time value
        printTimeToSD(IRFile, firsttimestamp); IRFile.print(F(",")); // Human readable date time
        IRFile.print(serialNumber); IRFile.print(F(",")); // Serial number
        IRFile.println(heartBuffer[bufferIndex]);         // Heart IR value
      } else if ( (bufferIndex > 0) & (bufferIndex < (HEART_SAMPLE_LENGTH - 1)) ){
        IRFile.print(",,,"); // write empty time values for most heart values
        IRFile.println(heartBuffer[bufferIndex]);         // Heart IR value
      } else if (bufferIndex == (HEART_SAMPLE_LENGTH - 1) ){
        // For last value in buffer, write last time stamp
        IRFile.print(lasttimestamp.unixtime(), DEC); IRFile.print(F(","));// POSIX time value
        printTimeToSD(IRFile, lasttimestamp); IRFile.print(F(",")); // Human readable date time
        IRFile.print(serialNumber); IRFile.print(F(",")); // Serial number
        IRFile.println(heartBuffer[bufferIndex]);
      } 
    
  }
  // IRFile.close(); // force the buffer to empty

  IRFile.timestamp(T_WRITE, lasttimestamp.year(),lasttimestamp.month(), \
      lasttimestamp.day(),lasttimestamp.hour(),lasttimestamp.minute(),lasttimestamp.second());
  
}



/***************************************************************************************************
** Method readCommand(). This function checks the serial port to see if there has been any input. **
** If there is data it is read until a terminator is discovered and then the command is parsed    **
** and acted upon                                                                                 **
***************************************************************************************************/
void readCommand() {
  static uint8_t inputBytes = 0;              // Variable for buffer position
  while (Serial.available()) {                // Loop while incoming serial data
    inputBuffer[inputBytes] = Serial.read();  // Get the next byte of data
    if (inputBuffer[inputBytes] != '\n' &&
        inputBytes < SPRINTF_BUFFER_SIZE)  // keep on reading until a newline
      inputBytes++;                        // shows up or the buffer is full
    else {
      inputBuffer[inputBytes] = 0;                 // Add the termination character
      for (uint8_t i = 0; i < inputBytes; i++)     // Convert the whole input buffer
        inputBuffer[i] = toupper(inputBuffer[i]);  // to uppercase characters
      Serial.print(F("\nCommand \""));
      Serial.write(inputBuffer);
      Serial.print(F("\" received.\n"));
      /**********************************************************************************************
      ** Parse the single-line command and perform the appropriate action. The current list of **
      ** commands understood are as follows: **
      ** **
      ** SETDATE      - Set the device time **
      ** CALDATE      - Calibrate device time **
      ** **
      **********************************************************************************************/
      enum commands { SetDate, CalDate, Unknown_Command };  // of commands enumerated type
      commands command;                                     // declare enumerated type
      char     workBuffer[10];                              // Buffer to hold string compare
      sscanf(inputBuffer, "%s %*s", workBuffer);            // Parse the string for first word
      if (!strcmp(workBuffer, "SETDATE"))
        command = SetDate;  // Set command number when found
      else if (!strcmp(workBuffer, "CALDATE"))
        command = CalDate;  // Set command number when found
      else
        command = Unknown_Command;                              // Otherwise set to not found
      uint16_t tokens, year, month, day, hour, minute, second;  // Variables to hold parsed dt/tm
      switch (command) {                                        // Action depending upon command
        /*******************************************************************************************
        ** Set the device time and date                                                           **
        *******************************************************************************************/
        case SetDate:  // Set the RTC date/time
          tokens = sscanf(inputBuffer, "%*s %hu-%hu-%hu %hu:%hu:%hu;", &year, &month, &day, &hour,
                          &minute, &second);
          if (tokens != 6)  // Check to see if it was parsed
            Serial.print(F("Unable to parse date/time\n"));
          else {
            MCP7940.adjust(
                DateTime(year, month, day, hour, minute, second));  // Adjust the RTC date/time
            Serial.println(F("Date has been set."));
          }       // of if-then-else the date could be parsed
          break;  //
        /*******************************************************************************************
        ** Calibrate the RTC and reset the time                                                   **
        *******************************************************************************************/
        case CalDate:  // Calibrate the RTC
          tokens = sscanf(inputBuffer,
                          "%*s %hu-%hu-%hu %hu:%hu:%hu;",  // Use sscanf() to parse the date/
                          &year, &month, &day, &hour, &minute, &second);  // time into variables
          if (tokens != 6)  // Check to see if it was parsed
            Serial.print(F("Unable to parse date/time\n"));
          else {
            int8_t trim =
                MCP7940.calibrate(DateTime(year, month, day,  // Calibrate the crystal and return
                                           hour, minute, second));  // the new trim offset value
            Serial.print(F("Trim value set to "));
            Serial.print(trim * 2);  // Each trim tick is 2 cycles
            Serial.println(F(" clock cycles every minute"));
          }  // of if-then-else the date could be parsed
          break;
        /*******************************************************************************************
        ** Unknown command                                                                        **
        *******************************************************************************************/
        case Unknown_Command:  // Show options on bad command
        default:
          Serial.println(F("Unknown command. Valid commands are:"));
          Serial.println(F("SETDATE yyyy-mm-dd hh:mm:ss"));
          Serial.println(F("CALDATE yyyy-mm-dd hh:mm:ss"));
      }                // of switch statement to execute commands
      inputBytes = 0;  // reset the counter
    }                  // of if-then-else we've received full command
  }                    // of if-then there is something in our input buffer
}  // of method readCommand


/**********************************************************************
 * restartMAX3010x
 * 
 * Re-set the MAX3010x sampling settings, assuming it has just returned
 * from a hard power-down and restart. Take 1 sample and return the
 * uint32_t 4-byte value from the IR channel
 **********************************************************************/
uint32_t restartAndSampleMAX3010x(MAX30105 &max3010x, byte IRledBrightness, byte sampleAverage, \
                                  byte ledMode, int sampleRate, int pulseWidth, int adcRange, \
                                  byte REDledBrightness, bool EnableTemp)
{
    uint32_t heartValue = 0;

    max3010x.setup(IRledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    if (EnableTemp){
       max3010x.enableDIETEMPRDY(); //enable temp ready interrupt. Required to log temp, but each read takes 29ms
    }
//   
    // Tweak individual settings
    max3010x.setPulseAmplitudeRed(REDledBrightness); // essentially turn off red LED to save power, we only want IR LED. **** commented for testing only
    max3010x.setPulseAmplitudeIR(IRledBrightness); // set IR led brightness to user's chosen value 0x00 (off) to 0xFF(full power)
    delay(6); // This delay seems to be necessary to give the sensor time to start reading reasonable values (5ms minimum)
    max3010x.clearFIFO(); // Clearing FIFO potentially lets you only have to grab one value from the FIFO once it starts to refill
    
    long watchdog = millis();
    bool sampleFlag = false;
    // Here we use a watchdog to make sure the collection of a new sample doesn't take
    // longer than our pre-defined watchdog time (in milliseconds)
    while( millis() - watchdog < 5) {
        // With a cleared FIFO these two pointers will match initially
        byte readPointer = max3010x.getReadPointer();
        byte writePointer = max3010x.getWritePointer();
        
        if (readPointer != writePointer){
          // If they don't match, that means a new sample arrived in the FIFO buffer on the MAX3010x
          sampleFlag = true;
          break; // escape the while loop once a new sample has appeared
        }
        delayMicroseconds(20);
        sampleFlag = false;
    }
    if (sampleFlag){
        max3010x.check();  // retrieve the new sample(s) and put them in the max3010x private buffer
        // Calling getIR() should get the most recent value from the buffer of values
        heartValue = max3010x.getIR();
//        Serial.println(heartValue);  // modify getIR in the library to remove safeCheck() function        
    } else {
        // If sampleFlag was still false, a new sample didn't arrive from the MAX3010x sensor in time
//        Serial.println("0");  // modify getIR in the library to remove safeCheck() function   
        heartValue = 0;
    }

    return(heartValue); // Return the heartValue as output
    
} // end of restartAndSampleMAX3010x()
