/* BivalveBitMax_logger_gape_only.ino
 *  BivalveBit data logger program with sleep modes and
 *  state machine to run things. Designed for use with 
 *  Allegro A1395 Hall effect sensor to just record bivalve shell gape. 
 *  
 *  Samples gape sensor every minute. 
 *  
 *  If the device just slowly flashes red after startup, the real time clock
 *  needs to be reset. Open the serial monitor and enter the correct date and
 *  time (in the UTC time zone preferably) using the command format:
 *  SETDATE YYYY-MM-DD HH:MM:SS
 *  
 *  
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
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib

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
//  STATE_FAST_SAMPLE,  // sleep for short interval based on RTC wakeup signal (8Hz)
  STATE_SHUTDOWN // close data file, shut down data collection due to low battery
} mainState_t;
// main state machine variable, this takes on the various
// values defined for the STATE typedef above. 
mainState_t mainState;

// Type definition for a case structure to write to SD card or skip writing
typedef enum WRITE_SD
{
  WRITE_HALL_TEMP_VOLTS, // collect Hall sensor data and battery voltage data
//  WRITE_HEART, // collect heart rate data at sub-second intervals
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
//File GAPEFile; //SD card object 2 (Gape, Temp, Battery voltage data)
SdFile GAPEFile; //SD card object 2 (Gape, Temp, Battery voltage data)
bool SDfailFlag = false;
// Placeholder serialNumber
char serialNumber[] = "SN000";
bool serialValid = false;
// Declare initial name for output file written to SD card
char gapefilename[] =   "YYYYMMDD_HHMM_00_SN000_GAPE.csv";

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
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940 . check time

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
  //---------------------------------------------------------------
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  delayMicroseconds(250);
 
  digitalWrite(VREG_EN, LOW); // set low to turn off voltage regulator for Hall sensor
  
  /************************************************************
   *  Real Time Clock startup
   ***********************************************************/
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
  oldday1 = oldday2 = now.day(); // Store current day's value
  // Initialize data file
  initGapeOnlyFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
  digitalWrite(REDLED, HIGH); // turn off
  Serial.print("Gape sample interval: "); Serial.print(gapeMinute); Serial.println(" minutes");
  
  MCP7940Alarm1Minute(now); // Set RTC multifunction pin to alarm when new minute hits
  attachInterrupt(digitalPinToInterrupt(20),RTC1MinuteInterrupt, CHANGE); // pin 20 to RTC

//  if (gapeMinute > heartMinute){
//    // Make sure gape is sampling at least as often as the heart sensor
//    gapeMinute = heartMinute;
//  }
  
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
            // Flash the green LED briefly, this also gives the voltage regulator time to 
            // stabilize
            bitWrite(PORTC.OUT, 0, 0); // Set PC0 low to turn on green LED
            delayMicroseconds(150); // Give voltage regulator time to stabilize
            bitWrite(PORTC.OUT, 0, 1); // Turn off PC0 by setting pin high
            batteryVolts = readBatteryVoltage(BATT_MONITOR_EN, BATT_MONITOR,\
                                                dividerRatio, refVoltage);
  //        Serial.print("Battery: ");Serial.print(batteryVolts,2);Serial.println("V");delay(10);
            
            HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP); 

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

          /* Update 1 minute wake interval,
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
      Serial.print("Hall: ");Serial.println(HallValue); delay(8);
      // Check if a new day has started, start a new data file if so.
      if (now.day() != oldday1) {
        GAPEFile.close(); // close old file
        // Open new file
        initGapeOnlyFileName(sd, GAPEFile, now, gapefilename, serialValid, serialNumber);
        oldday1 = now.day(); // update oldday1
      }
      // Write hall, temperature, battery voltage to SD
      writeGapeToSD(now); // See function at bottom of program
      
      // Reset to write-nothing state 
      writeState = WRITE_NOTHING;
      // The main state could be going back to 1-minute intervals or switching
      // to fast sampling heart rate, depending on what happened in the 
      // mainState switch above. Check which state we're in to decide what to do next 
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
    }  // end of if-then oscillator didn't start
  }    // end of while the oscillator is off
  MCP7940.setSQWState(false); // turn off square wave output if currently on

}

//---------------------------------------------------------
void MCP7940Alarm1Minute(DateTime currentTime){
  MCP7940.setSQWState(false); // turn off square wave output if currently on
  MCP7940.setAlarmPolarity(false); // pin goes low when alarm is triggered
  Serial.println("Setting alarm for every minute at 0 seconds.");
  MCP7940.setAlarm(0, matchSeconds, currentTime - TimeSpan(0, 0, 0, currentTime.second()), true);  // Match once a minute at 0 seconds
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
//  GAPEFile.print(tempC); GAPEFile.print(F(","));      // Temperature sensor value
  GAPEFile.print(batteryVolts); // Battery voltage
  GAPEFile.println();
  // GAPEFile.close(); // force the buffer to empty
  GAPEFile.timestamp(T_WRITE, timestamp.year(),timestamp.month(), \
      timestamp.day(),timestamp.hour(),timestamp.minute(),timestamp.second());
  
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
