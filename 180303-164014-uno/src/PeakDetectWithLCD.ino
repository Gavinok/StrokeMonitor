/*
 * @CreateTime: Dec 25, 2017 9:41 PM
 * @Author: Gavin Jaeger-Freeborn
 * @Contact: gavinfreeborn@gmail.com
 * @Last Modified By:  Gavin Jaeger-Freeborn
 * @Last Modified Time: Dec 25, 2017 11:55 PM
 * @Description:  This code requires the installation of the Adafruit_GPS.h library for it to work
 *                This code allows the arduino device to store the time, and speed of the GPS unit
 *                and accelerometer values in m/s^2 to a micro SD card where the file creates under
 *                the name GPSData.TXT.
 * @Pin Layout: 
 *        GPS Breakout: VIN to +5V
 *                      GND to Ground
 *                      RX to digital 2
 *                      TX to digital 3
 * 
 *    SD Card Breakout: MOSI to digital 11
 *                      MISO to digital 12
 *                      SCK (CLK) to digital 13
 *                      CS to digital 4
 *       Accelerometer: GND to Ground
 *                      X_OUT to analog 0
 *                      Y_OUT to analog 1
 *                      Z_OUT to analog 2
 *                      VCC to +5V
 *                 LED: pin 8
 *             2X16LCD: rs     1
 *                      enable 5
 *                      d4     6
 *                      d5     7
 *                      d6     8
 *                      d7     9
 */

//====================Important definitions================//
  #define debug
  //if true the program will also read accelerometer values.
  #define Accelerometer  true

  //prints raw accelerometer values to sd card ans serial
//#define RawAccelerometer //works perfect

  //if true the program will also wright values to the SDCard.
  #define SDCard  true

  //if true the program will also wright gps values.
  #define Gps  true

  //use low peak detection 
  #define LowPeek

  //if using 2X16 LCD
  //#define LCD
//=======================================================//
#ifdef SDCard
    #include <SD.h> //Load SD card library
    #include<SPI.h> //Load SPI Library
#endif
#ifdef Gps
  #include <Adafruit_GPS.h>    //Install the adafruit GPS library
#endif
#include <SoftwareSerial.h> //Load the Software Serial library
#include <LiquidCrystal.h>// include the library code
//========================Accelerometer pins=================//
  //we are assuming the x axis is in the same direction as the boat
  // x-axis of the accelerometer
  #define xpin A0
  // y-axis of the accelerometer
  #define ypin  A1
  // z-axis of the accelerometer
  #define zpin  A2 
//=======================================================//

//========================LCD pins===================//
#ifdef LCD
    LiquidCrystal lcd(1, 5, 6, 7, 8, 9);
#endif

//=====================Loop Values=========================//
  //used to count loops before resetting the threshold
  uint8_t loops = 0;
  //the limit of how many loops can occur before the threshold is reset
  #define loopLimit 5
  //number of strokes in between writing to sd.
  uint8_t Strokes = -1;
  //array of low peaks used to make the threshold dynamic
  int Low[loopLimit];
  uint8_t LowScan = 0;
  int Lowest = 1000;
  #define minimumLowest 300 // Lowest values that can be considered as legit
//=======================================================//

//=================LowPeekDetection Thresholds and Values===========================//
#ifdef LowPeek

  #define numaverages 5  // number of averages to take (removes noise)
  int oldaverage = 0; // previous averaged reading
  bool OldPositionNegative = false;  // sign of previous slope, true = negative
  //the lowest the threshold can go
  #define ThreshholdMinimum -0.25
  //limits what drops in acceleration are considered the recovery phase.
  int threshhold = 1000;
  #define StaticChangeThreshhold 1.05 //1-2//% difference between the previous acceleration and current acceleration to indicate a peek(Lower means that the difference in acceleration when taking a stroke must be larger)
  #define StaticPercentOfThreshhold 1.2 //1-2//what % of the threshold does the previous reading need to be to indicate deceleration(lower means that the negative portion has to be lower)

#endif
//=======================================================//

#ifdef SDCard
  //chipSelect pin for the SD card Reader
  #define chipSelect  4 
#endif

#ifdef Gps

  SoftwareSerial mySerial(3, 2);

  Adafruit_GPS GPS(&mySerial);

  // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
  // Set to 'true' if you want to debug and listen to the raw GPS sentences. 
  #define GPSECHO  false

  // this keeps track of whether we're using the interrupt
  // off by default!
  boolean usingInterrupt = true;
  void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
#endif

#ifdef Gps
  File GPSlog; //Data object you will write your sensor data to
#endif
//====================================function prototypes=========//
void resetTHreshhold();
int findLowest();
boolean LowPeekDetection();
void setup()  
{ 

  #ifdef LCD
    lcd.begin(16, 2); // set up the LCD's number of columns and rows:
    lcd.print(F("Gavin")); // Print a message to the LCD.
     delay(1000);
  #else
    // connect at 115200 so we can read the GPS fast enough.
    Serial.begin(115200);
    Serial.println(F("Running Gavins Dope Ass Paddling Program"));
  #endif
  #ifdef Gps
    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);
    // timer0 interrupt every 1 millisecond, and read data from the GPS.
    useInterrupt(true);
  #endif
  delay(1000);

  #ifdef SDCard
    pinMode(10, OUTPUT); //Must declare 10 an output and reserve it to keep SD card happy
    DDRB |=(1<<DDB0); // led pin set to output
    SD.begin(chipSelect); //Initialize the SD card reader
  #endif
}
#ifdef Gps
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  SIGNAL(TIMER0_COMPA_vect)
  {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
  }
#endif

#ifdef Gps
  void useInterrupt(boolean v) {
    if (v) {
      // Timer0 is already used for millis() - we'll just interrupt somewhere
      // in the middle and call the "Compare A" function above
      OCR0A = 0xAF;
      TIMSK0 |= _BV(OCIE0A);
      usingInterrupt = true;
    } else {
      // do not call the interrupt function COMPA anymore
      TIMSK0 &= ~_BV(OCIE0A);
      usingInterrupt = false;
    }
  }
#endif

#ifdef RawAccelerometer
  void AccelerometerWright()
  {
    PORTB |=(1<<PORTB0); //led pin high
    GPSlog = SD.open("GPSData.txt", FILE_WRITE); //Open file on SD card for writing
    //---------------File opened -------------------//
    PORTB |=(1<<PORTB0); //led pin high
    if (GPSlog)
    {
      GPSlog.print(analogRead(xpin));   GPSlog.print(F(" , ")); // write ANALOG0 (X) to SDGPSlog.print(" , ");      
      GPSlog.print(analogRead(ypin));   GPSlog.print(F(" , ")); // write ANALOG1 (Y) to SD     
      GPSlog.println(analogRead(zpin));    // write ANALOG2 (Z) to SD
            
            
      Serial.print(analogRead(xpin));   Serial.print(F(" , ")); // write ANALOG0 (X) to SDGPSlog.print(" , ");      
      Serial.print(analogRead(ypin));   Serial.print(F(" , ")); // write ANALOG1 (Y) to SD     
      Serial.println(analogRead(zpin));    // write ANALOG2 (Z) to SD
    }
    else 
    { // if the file didn't open
    //if no longer connected to the serial monitor simply comment out the following line
    Serial.println(F("wrighting Accelerometer Data failed!"));// replace with led
    }
    GPSlog.close();
    //---------------File closed -------------------//
    PORTB &=~(1<<PORTB0); //PIN LOW
    GPSlog.close();
    //---------------File closed -------------------//
    PORTB &=~(1<<PORTB0); //PIN LOW
  }
#endif

//Initialize timer
uint32_t timer = millis();
uint32_t timer1 = millis();
uint32_t NonStrokeTimer = millis();

/*float StrokeRate()
{
  int strokespermilli = (Strokes/(millis() - timer));
  return 1000*strokespermilli;
}*/

void loop()                     // run over and over again
{
    #ifdef LCD
    if ((millis() - timer > 1000))
    {

      #ifdef debug
        Serial.println("Strokes Reset");
      #endif
      lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left corner.
      lcd.setCursor(0, 0);
      lcd.print("strokes "); // Print a message to the LCD.
      lcd.setCursor(0, 1);
      lcd.print(Strokes, DEC); // Print a message to the LCD.
      //Strokes = 0;
      timer = millis(); // reset the timer1
    }
    #endif
  #ifdef debug
    Serial.print(F("strokes "));
    Serial.println(Strokes);
  #endif
  #ifdef Gps
    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (! usingInterrupt) {
      // read data from the GPS in the 'main loop'
      char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
    }
  #endif
  // if a sentence is received, we can check the checksum, parse it...
  #ifdef Gps
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences! 
      // so be very wary if using OUTPUT_ALLDATA and trying to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
  #endif
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  if (NonStrokeTimer > millis())  NonStrokeTimer = millis();
  /*
  Accelerometer wrights to SD Card
  */
  #ifdef Accelerometer

        //if a new peek below the threshold is found add a stroke
        #ifdef LowPeek
          LowPeekDetection();
          resetTHreshhold();
        #endif
        
      #ifdef SDCard
        #ifdef RawAccelerometer
          AccelerometerWright();
        #endif
      #endif
  #endif
  
  /* 
  after approximately one second has passed collect the GPS values
  */
  if ((millis() - timer > 1000) && false) { 
    //-------newcode start----------//
    #ifdef SDCard
      GPSlog = SD.open("GPSData.txt", FILE_WRITE); //Open file on SD card for writing
        //---------------File Opened -------------------//
        if (GPSlog)
        { 
          #ifdef Gps
          GPSlog.print(F("Stroke Rate: "));
          GPSlog.println(StrokeRate());
          #endif
          PORTB &=~(1<<PORTB0); //PIN LOW
        //---------------File closed -------------------//
        }
        else
        {
          Serial.print(F("Stroke Rate failed "));
        }
    #endif
    #ifdef debug
      Serial.print(F("Stroke Rate: "));
      Serial.print(StrokeRate());
    #endif

    timer = millis(); // reset the timer
    
    
    if(false)
    {
      #ifdef Gps
        Serial.print(F("\nTime: "));
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print(F("Date: "));
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print(F("/20"));
        Serial.println(GPS.year, DEC);
        Serial.println(GPS.speed, DEC);
        Serial.print(F("Fix: ")); Serial.println((short)GPS.fix);
      #endif
    }
   // Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
  #ifdef Gps
    if (GPS.fix || true ) 
    {
      #ifdef SDCard

        PORTB |=(1<<PORTB0); //led pin high
        #ifdef debug
          Serial.println("led on");
        #endif
        GPSlog = SD.open("GPSData.txt", FILE_WRITE); //Open file on SD card for writing
        //---------------File Opened -------------------//

        if (GPSlog)
        { 
          GPSlog.print(F("\nTime: "));
          GPSlog.print(GPS.hour, DEC); GPSlog.print(':');
          GPSlog.print(GPS.minute, DEC); GPSlog.print(':');
          GPSlog.print(GPS.seconds, DEC); GPSlog.print('.');
          GPSlog.println(GPS.milliseconds);
          GPSlog.print(F("Date: "));
          GPSlog.print(GPS.day, DEC); GPSlog.print('/');
          GPSlog.print(GPS.month, DEC); GPSlog.print("/20");
          GPSlog.println(GPS.year, DEC);
          GPSlog.print(F("Location (in degrees, works with Google Maps): "));
          GPSlog.print(GPS.latitudeDegrees, 4);
          GPSlog.print(F(", ")); 
          GPSlog.println(GPS.longitudeDegrees, 4);
          GPSlog.print(F("Speed (m): ")); GPSlog.println(GPS.speed);
        }
        else 
        { 
          // if the file didn't open
          //if no longer connected to the serial monitor simply comment out the following line
          Serial.println(F("wrighting GPS Data failed!"));// replace with led
        }
        PORTB &=~(1<<PORTB0); //PIN LOW
        //---------------File closed -------------------//
        GPSlog.close();
      #endif

    }
  #endif
  }
}

#ifdef LowPeek
  boolean LowPeekDetection()
  {
    int currentaverage = 0;    // current averaged reading
    for (int scan=0; scan<= numaverages; scan++){       // loop for numaverages
      currentaverage += analogRead(xpin);   // sum up readings assuming that the xaxis is in the same direction as the boat
    }
    currentaverage /= numaverages;     // divide total sum by num. avg. to get average
    #ifdef debug
      Serial.print(F("currentaverage "));
      Serial.println(currentaverage);
      Serial.print(F("oldaverage "));
      Serial.println(oldaverage);
      //this is used to avoid registering values when the boat is stationary
     // #ifdef debug
     //   if(currentaverage < 100 )
         // currentaverage = oldaverage;
     // #endif

    #endif
    if((millis() - NonStrokeTimer > 900))
    {
      if ((currentaverage > oldaverage * StaticChangeThreshhold)){    // if current is greater than previous (negative slope) and old slope was negative, a local minima was reached
      #ifdef debug
        Serial.print(F("peekdetection begin "));
        Serial.println(OldPositionNegative);
        Serial.print(F("Total ChangeThreshhold "));
        Serial.println(oldaverage * StaticChangeThreshhold);
      #endif
        if(OldPositionNegative == false){
          if(oldaverage < (threshhold * StaticPercentOfThreshhold)){
            // local minima value, do stuff here
            #ifdef debug
                Serial.print(F("TotalThreshhold "));
                Serial.println(threshhold * StaticPercentOfThreshhold);
            #endif
            OldPositionNegative = true;      // the if statement already checked for positive slope, so it makes sense to set the value for the next pass here
            Low[LowScan] = currentaverage;  //add this minima to the array of minima
            
            #ifdef debug
              Serial.println(F("loop count "));
              Serial.print(loops);
            #endif
            Strokes++;
            LowScan++;
            NonStrokeTimer = millis();
          }
        }
      }
    }
    #ifdef debug
      Serial.print(F("peekdetection end "));
      Serial.println(OldPositionNegative);
    #endif
    if (currentaverage - oldaverage < 0){  // set old slope variable to negative if applicable
        if(OldPositionNegative == true)
        {
            int peak = oldaverage;
            Serial.print(F("peak"));
            Serial.println(peak);
        }
      OldPositionNegative = false;
    }
    oldaverage = currentaverage;    // sets current values to old values
    loops++;
    return OldPositionNegative;
  }
#endif

int findLowest()
{

  Lowest = 1000;
  for(uint8_t scan; scan < loopLimit; scan++)
  {
    #ifdef debug
      Serial.print(F("low[scan] here ........................"));
      Serial.println(Low[scan]);
    #endif
    if((Lowest > Low[scan]) && !(Low[scan] ==0))
    {
      Lowest = Low[scan];
    }
  }
  return Lowest;
}
void resetTHreshhold();
{
  if(loops >= loopLimit)
  {
    //sets Lowest to the lowest value in the array low[]
    int Lowest = findLowest();
    #ifdef debug
      Serial.print(F("lowest "));
      Serial.println(Lowest);
    #endif
    //if the new threshold is above the ThreshholdMinimum replace the threshold
    if(Lowest > minimumLowest)
    {
      #ifdef debug
          Serial.print(F("lowest "));
          Serial.println(Lowest);
      #endif
      threshhold = Lowest;
      #ifdef debug
        Serial.println(F("threshhold reset "));
        Serial.println(threshhold);
      #endif
    }
    loops = 0;
  }
}