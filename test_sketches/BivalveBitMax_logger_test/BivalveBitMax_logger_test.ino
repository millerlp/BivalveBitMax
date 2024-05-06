/* BivalveBitMax_logger_test.ino
 *  Use to test all of the sensors, clock, SD card on a BivalveBit board
 *  Made to work with MAX3010x heart sensors
 *  
 */

#include "SdFat.h" // https://github.com/greiman/SdFat-beta (you must be using ver. 2.1.0 or higher)
#include <Wire.h>  // built in library, for I2C communications
#include <EEPROM.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include "MCP7940.h"  // https://github.com/Zanduino/MCP7940  Real time clock
#include "MAX30105.h"         // https://github.com/millerlp/SparkFun_MAX3010x_Sensor_Library
#include "BivalveBit_lib.h" // https://github.com/millerlp/BivalveBit_lib

/*******************************************************
 * SD card objects
 *******************************************************/
// SPI pins for SD card
const uint8_t SD_CHIP_SELECT = 7;
#define MOSI 4
#define MISO 5
#define SCK 6
SdFat sd;  // sd card object
bool SDfailFlag = false;
SdFile logfile;  // for sd card, this is the file object to be written to
char filename[] = "YYYYMMDD_HHMM_00_SN000.csv";



// Placeholder serialNumber
char serialNumber[] = "SN000";
bool serialValid = false;

/************************************************************
 *  // MAX30105 heart sensor parameters
 **************************************************************/

MAX30105 max3010x; // MAX3010x sensor object (MAX30101, MAX30102, MAX30105 all interchangeable)
// sensor configurations
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green. Only use 2
byte REDledBrightness = 1; // low value of 0 shuts it off, 1 is barely on
byte IRledBrightness = 60;  // Starting value around 60 is probably reasonable for bivalves
byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32, but only use 1 or 2. The others are too slow
int pulseWidth = 215; //Options: 69, 118, 215, 411, units microseconds. Applies to all active LEDs. Recommend 215
// For 118us, max sampleRate = 1000; for 215us, max sampleRate = 800, for 411us, max sampleRate = 400
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int adcRange = 4096; //Options: 2048, 4096, 8192, 16384. 4096 is standard
// Prototype for the restartAndSampleMAX3010x function (see full function at bottom of file)
uint32_t restartAndSampleMAX3010x(MAX30105 &max3010x, byte IRledBrightness, byte sampleAverage, \
                                  byte ledMode, int sampleRate, int pulseWidth, int adcRange, \
                                  byte REDledBrightness, bool EnableTemp=false);
float tempC; // Used to hold temperature from MAX3010x sensor                                  
/************************************************************
 * Hall effect sensor definitions
 ************************************************************/
#define ANALOG_IN A0  // Analog input pin for Hall sensor
#define HALL_SLEEP 10 // Hall sensor sleep pin, pull high to wake
unsigned int HallValue = 0; // Variable for Hall sensor reading

/***************************************************************************************************
**  MCP7940 real time clock chip                                            **
***************************************************************************************************/
const uint32_t SERIAL_SPEED{57600};     // Set the baud rate for Serial I/O
const uint8_t  SPRINTF_BUFFER_SIZE{32};  // Buffer size for sprintf()
MCP7940_Class MCP7940;                           // Create an instance of the MCP7940
char          inputBuffer[SPRINTF_BUFFER_SIZE];  // Buffer for sprintf()/sscanf()
DateTime now; // Variable to hold current time
DateTime newtime;
DateTime oldtime;
uint32_t oldmillis;
uint32_t updateInterval = 50; // milliseconds
//*******************************************

/* ***************************************
 *  OLED display objects ****
**********************************************/
SSD1306AsciiWire oled; // create OLED display object, using I2C Wire1 port
#define I2C_ADDRESS1 0x3C // for OLED. DIY mall units list 0x78 as address, but need 0x3C to work here

/********************************************************
 * Miscellaneous definitions
 *******************************************************/
#define REDLED 11   // Red LED pin
#define GRNLED 8    // Green LED pin
#define VREG_EN 24  // voltage regulator enable


/************************************************************
 *  Battery monitor variables
 ***********************************************************/
const byte BATT_MONITOR_EN = 9; // digital output channel to turn on battery voltage check
const byte BATT_MONITOR = A1;  // analog input channel to sense battery voltage

const float dividerRatio = 2; // Ratio of voltage divider (47k + 47k) / 47k = 2
const float refVoltage = 3.00; // Voltage at AREF pin on ATmega microcontroller, measured per board
//float refVoltage = 2.5; // Internal 2.5V voltage reference value (INTERNAL2V5)
float batteryVolts = 0; // Estimated battery voltage returned from readBatteryVoltage function

/**************************************************************************
 * Setup loop
 **************************************************************************/
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(REDLED, HIGH); // set high to turn OFF
  digitalWrite(GRNLED, HIGH); // set high to turn OFF
  pinMode(VREG_EN, OUTPUT);
  digitalWrite(VREG_EN, HIGH); // set low to turn off, high to turn on (~150usec to wake)
  pinMode(HALL_SLEEP, OUTPUT);
  digitalWrite(HALL_SLEEP, LOW); // set high to wake, set low to sleep (~60usec to wake)
  analogReference(EXTERNAL); // using voltage regulator value on external pin
  pinMode(ANALOG_IN, INPUT); // Hall sensor input channel
  pinMode(BATT_MONITOR, INPUT); // Battery voltage input channel
  pinMode(BATT_MONITOR_EN, OUTPUT); // Battery monitor enable pin
  digitalWrite(BATT_MONITOR_EN, LOW); // pull low to turn off battery monitor circuit

  Serial.begin(57600);
//  while (!Serial) { delay(1); } // Wait until serial port is opened
  Serial.println("Hi");

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

  //---------------------------------------------------------
  // OLED display setup
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS1);  
  oled.setFont(Adafruit5x7);    
  oled.clear(); 
  oled.home();
//  oled.println("Hello");

  
  /************************************************************
   *  Real Time Clock startup
   ***********************************************************/
  MCP7940setup();  // See function at bottom of file
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
    if ( (now.year() < 2022) | (now.year() > 2035) ) {
        Serial.println("Please set clock to current UTC time");
        Serial.println("Use format SETDATE YYYY-MM-DD HH:MM:SS");
       // Error, clock isn't set
       while(clockErrorFlag){
        digitalWrite(REDLED, !digitalRead(REDLED));
        delay(500);
        readCommand();
        now = MCP7940.now();  // get the updated time
        if ( (now.year() >= 2023) & (now.year() <= 2035) ){
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
  // Turn on the square wave output pin of the RTC chip
  MCP7940.setSQWSpeed(3); // set SQW frequency to 32768 Hz
  MCP7940.setSQWState(true); // turn on the square wave output pin
  newtime = MCP7940.now();
  // TODO: Print date and time to OLED at startup
  printTimeOLED(newtime, oled);
  oled.println();
  delay(2000);
  
  //----------------------------------------------------------
  // SD card initialization
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    oled.println("SD card fail");
    SDfailFlag = true;
    digitalWrite(REDLED, LOW);
    delay(100);
    digitalWrite(REDLED, HIGH);
    delay(100);
    
    return;
  }
  Serial.println("SD card initialized.");
  oled.println("SD card found");
  digitalWrite(GRNLED,LOW);
  delay(100);
  digitalWrite(GRNLED,HIGH);
  delay(500);

  //------------------------------------------------------------------
  // Enable 3.0V voltage regulator so that you can talk to the heart and gape sensors
  digitalWrite(VREG_EN, HIGH); // Set high to turn on
  delayMicroseconds(250); // Give at least 150us for regulator to turn on
  
  oled.home();
  oled.clear();
  //-----------------------------------------------------------------
  // Initialize Hall sensor
  digitalWrite(HALL_SLEEP, HIGH); // turn on hall effect sensor
  delayMicroseconds(50);
  HallValue = readHall(ANALOG_IN); // Function in BivalveBit_lib 
  digitalWrite(HALL_SLEEP, LOW); // put hall sensor to sleep
  if ( (HallValue > 0) & (HallValue < 1023) ) {
    Serial.print("Hall sensor: ");
    Serial.println(HallValue);
    oled.print("Hall: ");
    oled.println(HallValue);
    digitalWrite(GRNLED, LOW);
    delay(100);
    digitalWrite(GRNLED, HIGH);
    delay(100);
  } else {
    Serial.println("Hall sensor problem");
    oled.println("Hall sensor fail");
    digitalWrite(REDLED, LOW);
    delay(200);
    digitalWrite(REDLED, HIGH);
    delay(100);
  }
  delay(500);
  
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
    for (int c = 0; c < 10; c++){
      digitalWrite(REDLED,LOW); // set low to turn on, leave on due to the error
      delay(250);
      digitalWrite(REDLED,HIGH); // turn off
      delay(500);
    }
    digitalWrite(REDLED,LOW); // set low to turn on, leave on due to the error
  }
  //----------------------------------------------------



    //------------------------------------------------------------
    // Battery voltage circuit test
    batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage);
    oled.print("Battery: ");
    oled.print(batteryVolts,3);
    oled.println("V");
    Serial.print("Battery: ");
    Serial.print(batteryVolts,3);
    Serial.println("V");
    delay(500);

    //---------------------------------------
    // Voltage regulator can be turned off again
//    digitalWrite(VREG_EN, LOW); // set low to turn off

    oldmillis = millis();
}   // end of setup loop


//-------------------------------------------------------------
void loop() {

  
  digitalWrite(GRNLED,!digitalRead(GRNLED));
//  delay(500);
  
  if (millis() - oldmillis > updateInterval) {
    digitalWrite(VREG_EN, HIGH); // set low to turn on after sampling
    delayMicroseconds(150);
    HallValue = readWakeHall(ANALOG_IN, HALL_SLEEP); // Function in BivalveBit_lib 
    Serial.print("Hall: ");
    Serial.print(HallValue);
    Serial.print("  Heart: ");
    // Enable the MAX3010x sensor so we can get the temperature 
    uint32_t Prox = restartAndSampleMAX3010x(max3010x, IRledBrightness, \
          sampleAverage, ledMode, sampleRate, pulseWidth, \
          adcRange, REDledBrightness, true); // enable temperature reading also
    tempC = max3010x.readTemperature();  // May take at least 29ms, up to 100ms
                                                 
  
    Serial.print(Prox);
    Serial.print("  Temp: ");
    // Print temperature in °C
    Serial.print(tempC);
    Serial.print("C");
    batteryVolts = readBatteryVoltage(BATT_MONITOR_EN,BATT_MONITOR,dividerRatio,refVoltage);
    Serial.print("  Battery: ");
    Serial.print(batteryVolts,3);
    Serial.println("V");
    Serial.println();

    digitalWrite(VREG_EN, LOW); // set low to turn off after sampling  
    
    oled.home();
    oled.clear();
    oled.print("Hall: ");
    oled.println(HallValue);
    oled.print("Heart: ");
    oled.println(Prox);
    oled.print("Temp: ");
    oled.println(tempC,2);
    oled.print("Battery: ");
    oled.println(batteryVolts,3);
    readCommand(); // check for clock update
  }
  




} //************************* End of main loop


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
            delay(2000);
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
uint32_t restartAndSampleMAX3010x(MAX30105 &max3010x, byte IRledBrightness, \
                                  byte sampleAverage, byte ledMode, int sampleRate,\
                                  int pulseWidth, int adcRange, byte REDledBrightness, \
                                  bool EnableTemp)
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
