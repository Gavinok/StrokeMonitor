#include <Arduino.h>
/*
 * @CreateTime: Dec 25, 2017 9:41 PM
 * @Author: Gavin Jaeger-Freeborn
 * @Contact: gavinfreeborn@gmail.com
 * @Last Modified By:  Gavin Jaeger-Freeborn
 * @Last Modified Time: Dec 25, 2017 11:55 PM
 * @Description:  This code requires the installation of the Adafruit_GPS.h library for it to work
 *                This code allows the Arduino device to store the time, and speed of the GPS unit
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
 *                 LED: digital 8
 *             2X16LCD: rs     A3
 *                      enable 5
 *                      d4     6
 *                      d5     7
 *                      d6     8
 *                      d7     9
 *            ButtonPin:      A4
 */

//====================Important definitions================//
  //#define DEBUG
  //if true the program will also read accelerometer values.
  #define Accelerometer  
  #define STROKE_RATE
  //prints raw accelerometer values to sd card ans serial
//#define RawAccelerometer //works perfect

  //if true the program will also wright values to the SD_CARD.
  //#define SD_CARD  true

  //if true the program will also wright GPS values.
 // #define GPS_  true

  //use low peak detection 
  #define LOWPEEK

  //if using 2X16 LCD
  #define LCD
//=======================================================//
#ifdef SD_CARD
    #include <SD.h> //Load SD card library
    #include<SPI.h> //Load SPI Library
#endif
#ifdef GPS_
  #include <Adafruit_GPS.h>    //Install the adafruit GPS library
#endif
#include <SoftwareSerial.h> //Load the Software Serial library
#include <LiquidCrystal.h>// include the library code
//========================Accelerometer pins=================//
  //we are assuming the x axis is in the same direction as the boat
  // x-axis of the accelerometer
  #define XPIN A0
  // y-axis of the accelerometer
  #define YPIN  A1
  // z-axis of the accelerometer
  #define ZPIN  A2 
//=======================================================//

//========================LCD pins===================//
#ifdef LCD
    LiquidCrystal lcd(A3, 5, 6, 7, 8, 9);
#endif
#define BUTTON_PIN A4
//=====================Loop Values=========================//
  //used to count loops before resetting the threshold
  uint8_t loops = 0;
  //the limit of how many loops can occur before the threshold is reset
  #define LOOP_LIMIT 5
  //number of strokes in between writing to sd.
  uint16_t Strokes = -1;
  //array of low peaks used to make the threshold dynamic
  int Low[LOOP_LIMIT];
  uint8_t LowScan = 0;
  #define MINIMUM_LOWEST 300 // Lowest values that can be considered as legit
//=======================================================//

//=================LOWPEEKDetection Thresholds and Values===========================//
#ifdef LOWPEEK

  #define NUMBER_OF_AVERAGES 6  // number of averages to take (removes noise)
  int oldaverage = 0; // previous averaged reading
  bool OldPositionNegative = false;  // sign of previous slope, true = negative
  //limits what drops in acceleration are considered the recovery phase.
  int threshhold = 1000;
  #define STATIC_CHANGE_THRESHHOLD 1.02 //1-2//% difference between the previous acceleration and current acceleration to indicate a peek(Lower means that the difference in acceleration when taking a stroke must be larger)
  #define STATIC_PERCENT_THRESHHOLD 1.2 //1-2//what % of the threshold does the previous reading need to be to indicate deceleration(lower means that the negative portion has to be lower)
  uint16_t NonStrokeTimerThreshhold = 900; // this is the threshhold for how far apart a stroke must be to count.
  uint8_t AxisPin = 69;// this is the axis used for detecting strokes 0 for x, 1 for y, 2 for z
#endif
//=======================================================//
#ifdef STROKE_RATE
  int Stroke_Rate_Strokes;
  uint32_t StrokeRateTimer = millis();
#endif

#ifdef SD_CARD
  //CHIP_SELECT pin for the SD card Reader
  #define CHIP_SELECT  4 
#endif

#ifdef GPS_

  SoftwareSerial mySerial(3, 2);

  Adafruit_GPS GPS(&mySerial);

  // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
  // Set to 'true' if you want to DEBUG and listen to the raw GPS sentences. 
  #define GPSECHO  false

  // this keeps track of whether we're using the interrupt
  // off by default!
  boolean usingInterrupt = true;
  void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
#endif

#ifdef GPS_
  File GPSlog; //Data object you will write your sensor data to
#endif
//====================================function prototypes=========//
void resetThreshhold();
int findLowest();
void LOWPEEKDetection();
uint8_t InitializeAxis();
void ExtractDynamicValues(int *loopsNumber, long *longterm, int *dynamic);

uint32_t NonStrokeTimer = millis();
uint32_t totalTimer = millis();
void setup()  
{ 
  analogReference(EXTERNAL);
  #ifdef LCD
    lcd.begin(16, 2); // set up the LCD's number of columns and rows:
    lcd.setCursor(0, 0);
    lcd.print(F("Running Dope Ass")); // Print a message to the LCD.
    lcd.setCursor(0, 1);
    lcd.print(F("Paddling Program")); // Print a message to the LCD.
    delay(1000);
  #else
  // connect at 115200 so we can read the GPS fast enough.
    Serial.begin(115200);
    Serial.println(F("Running Dope Ass Paddling Program"));
  #endif
// connect at 115200 so we can read the GPS fast enough.
    Serial.begin(115200);
    Serial.println(F("Running Dope Ass Paddling Program"));
  #ifdef GPS_
    // connect at 115200 so we can read the GPS
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

  #ifdef SD_CARD
    pinMode(10, OUTPUT); //Must declare 10 an output and reserve it to keep SD card happy
    //DDRB |=(1<<DDB0); // led pin set to output
    SD.begin(CHIP_SELECT); //Initialize the SD card reader
  #endif
//=================Initialize axis here============
  pinMode(BUTTON_PIN, INPUT);
  while(!digitalRead(BUTTON_PIN))
  {
    digitalRead(BUTTON_PIN);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Please Start"));
  lcd.setCursor(0, 1);
  lcd.print(F("Paddling"));
  AxisPin = InitializeAxis();
  lcd.setCursor(0, 0);
  /* lcd.print(F("The Axis is "));
  switch(AxisPin)
  {
    case XPIN: lcd.print(F("X"));
             break;
    case YPIN: lcd.print(F("Y"));
             break;
    case ZPIN: lcd.print(F("Z"));
             break;
    default: lcd.print(F("ERROR"));
  } */
//==============done initializing==================
}
#ifdef GPS_
  // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  SIGNAL(TIMER0_COMPA_vect)
  {
    char c = GPS.read();
    // if you want to DEBUG, this is a good time to do it!
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;  
      // writing direct to UDR0 is much much faster than Serial.print 
      // but only one character can be written at a time. 
  #endif
  }
#endif

#ifdef GPS_
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

//Initialize timer
uint32_t timer = millis();
uint32_t timer1 = millis();

#ifdef STROKE_RATE
float Stroke_Rate()
{
  float test = Stroke_Rate_Strokes/10;
  Serial.println(test);
  Serial.print("+++++++++++++++++++++++Time");
  Serial.println(Stroke_Rate_Strokes);
  float StrokeRate = ((Stroke_Rate_Strokes/10)*6);
  Stroke_Rate_Strokes = 0;
  return StrokeRate;
}
#endif

void loop()     // run over and over again
{
  pinMode(A4, INPUT);
  #ifdef DEBUG
    Serial.print(F("=================================== LowScan"));
    Serial.println(LowScan);
    Serial.print(F("=================================== NonStrokeTimerThreshhold"));
    Serial.println(NonStrokeTimerThreshhold);
  #endif
  #ifdef LCD
   /* if ((millis() - timer > 1000))
    {

      #ifdef DEBUG
        Serial.println("Strokes Reset");
      #endif
      lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left corner.
      lcd.setCursor(0, 0);
      lcd.print("strokes"); // Print a message to the LCD.
      lcd.setCursor(0, 1);
      lcd.print(Strokes, DEC); // Print a message to the LCD.
      lcd.setCursor(3, 1);
      lcd.print(AxisPin); // Print a message to the LCD.
      //Strokes = 0;
      timer = millis(); // reset the timer1
    }*/
  #endif
  #ifdef DEBUG
    Serial.print(F("strokes "));
    Serial.println(Strokes);
  #endif
  #ifdef GPS_
    // in case you are not using the interrupt above, you'll
    // need to 'hand query' the GPS, not suggested :(
    if (! usingInterrupt) {
      // read data from the GPS in the 'main loop'
      char c = GPS.read();
      // if you want to DEBUG, this is a good time to do it!
      if (GPSECHO)
        if (c) Serial.print(c);
    }
  #endif
  // if a sentence is received, we can check the checksum, parse it...
  #ifdef GPS_
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
        #ifdef LOWPEEK
          LOWPEEKDetection();
          resetThreshhold();
          #ifdef STROKE_RATE
          if(millis() - StrokeRateTimer > 10000)
          {
            Serial.println(millis() - StrokeRateTimer > 10000);
            lcd.setCursor(0, 0);
            lcd.print("Stroke Rate ");
            lcd.print(Stroke_Rate());
            StrokeRateTimer = millis();
          }
          #endif
        #endif
        
      #ifdef SD_CARD
        //updates the sdcard every second.
        if ((millis() - timer > 1000))
        {
          File Datalog = SD.open("GPSData.txt", FILE_WRITE); //Open file on SD card for writing
          if (Datalog)
          { 
            lcd.print("tru");
            Datalog.println(millis() - totalTimer); // print the total time
            Datalog.println(Strokes); // print the total Strokes
            Datalog.close();
            
          }else 
          { 
            // if the file didn't open
            //if no longer connected to the serial monitor simply comment out the following line
            lcd.print(F("wrighting SD Data failed!"));// replace with led
          }
          timer = millis();
        }
        #ifdef RawAccelerometer
          AccelerometerWright();
        #endif
      #endif
  #endif
  
  /* 
    after approximately one second has passed collect the GPS values
  */
  /*if ((millis() - timer > 1000) && false) { 
    #ifdef SD_CARD
      GPS.fixquality); 
  #ifdef GPS_
    if (GPS.fix || true ) 
    {
      #ifdef SD_CARD

        PORTB |=(1<<PORTB0); //led pin high
        #ifdef DEBUG
          Serial.println("led on");
        #endif
        GPSlog = SD.open("GPSData.txt", FILE_WRITE); //Open file on SD card for writing
        //---------------File Opened -------------------//

        if (GPSlog)
        { 
          GPS.speed);
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
  }*/
}
#ifdef LOWPEEK
/**
 * @brief 
 * 
 * requires: Low[], NonStrokeTimer, NonStrokeTimerThreshhold, (deffined)STATIC_CHANGE_THRESHHOLD, 
 *           threshhold, LowScan, OldPositionNegative, (Deffined)LOOP_LIMIT, (deffined)STATIC_PERCENT_THRESHHOLD
 * 
 *  
 */
  void LOWPEEKDetection()
  {
    int currentaverage = 0;    // current averaged reading
    for (int scan=0; scan<= NUMBER_OF_AVERAGES; scan++){       // loop for NUMBER_OF_AVERAGES
      currentaverage += analogRead(AxisPin);   // sum up readings assuming that the xaxis is in the same direction as the boat
    }
    currentaverage /= NUMBER_OF_AVERAGES;     // divide total sum by num. avg. to get average
    #ifdef DEBUG
      Serial.print(F("currentaverage "));
      Serial.println(currentaverage);
      Serial.print(F("oldaverage "));
      Serial.println(oldaverage);
      //this is used to avoid registering values when the boat is stationary
     // #ifdef DEBUG
     //   if(currentaverage < 100 )
         // currentaverage = oldaverage;
     // #endif

    #endif
    if(millis() - NonStrokeTimer > 2000)
    {
      NonStrokeTimerThreshhold = 700;
    }
    #ifdef DEBUG
        Serial.print(F("peekdetection begin "));
        Serial.println(OldPositionNegative);
        lcd.setCursor(11, 1);
        lcd.print(OldPositionNegative); // Print a message to the LCD.
        Serial.print(F("Total ChangeThreshhold "));
        Serial.println(oldaverage * STATIC_CHANGE_THRESHHOLD);
      #endif
    if((millis() - NonStrokeTimer > NonStrokeTimerThreshhold))
    {
      if ((currentaverage > oldaverage * STATIC_CHANGE_THRESHHOLD)){    // if current is greater than previous (negative slope) and old slope was negative, a local minima was reached
        if(OldPositionNegative == false){
          if(oldaverage < (threshhold * STATIC_PERCENT_THRESHHOLD)){
            // local minima value, do stuff here
            #ifdef DEBUG
                Serial.print(F("TotalThreshhold "));
                Serial.println(threshhold * STATIC_PERCENT_THRESHHOLD);
            #endif
            OldPositionNegative = true;      // the if statement already checked for positive slope, so it makes sense to set the value for the next pass here
            Low[LowScan] = currentaverage;  //add this minima to the array of minima
            
            /* #ifdef DEBUG
              Serial.println(F("oop count "));
              Serial.print(loops);
            #endif */
            Strokes++;
            Stroke_Rate_Strokes++;
            lcd.setCursor(0, 1);
            lcd.print(F("Strokes ")); // Print a message to the LCD.
            lcd.print(Strokes, DEC); // Print a message to the LCD.
            if(LowScan < LOOP_LIMIT - 3)
            { 
              LowScan++;
            }
            else
            {
              LowScan = 0;
             /*  if 2 strokes have been counted the threshhold can be lowered to half the 
              time of the last gap. */
            if((millis() - NonStrokeTimer) > 500  && (millis() - NonStrokeTimer) < 2000)
                          NonStrokeTimerThreshhold = ((millis() - NonStrokeTimer) * 0.7);
              Serial.print(F("++++++++++++++++++++non stroke timer threshhold"));
              Serial.println(NonStrokeTimerThreshhold);
            }
            NonStrokeTimer = millis();
            
          }
        }
      }
    }
    #ifdef DEBUG
      Serial.print(F("peekdetection end "));
      Serial.println(OldPositionNegative);
      
      lcd.setCursor(12, 1);
      lcd.print(OldPositionNegative); // Print a message to the LCD.
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
    //return OldPositionNegative;
    
  }
#endif
/**
 * @brief finds the lowest value in an array low scan
 * requires: Low[], and Lowest
 * @return int lowest value in an array
 */
int findLowest()
{
  int Lowest = 1000;
  for(uint8_t scan; scan < LOOP_LIMIT; scan++)
  {
    #ifdef DEBUG
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
/**
 * @brief resets the threshhold based on the lowest value it can find in the array low
 * Requires: loops, , LOOP_LIMIT, Lowest,  MINIMUM_LOWEST, and threshhold
 * 
 */
void resetThreshhold()
{
  if(loops >= LOOP_LIMIT)
  {
    //sets Lowest to the lowest value in the array low[]
    int Lowest = findLowest();
    #ifdef DEBUG
      Serial.print(F("lowest "));
      Serial.println(Lowest);
    #endif
    //if the new threshold is above the ThreshholdMinimum replace the threshold
    if(Lowest > MINIMUM_LOWEST)
    {
      #ifdef DEBUG
          Serial.print(F("lowest "));
          Serial.println(Lowest);
      #endif
      threshhold = Lowest;
      #ifdef DEBUG
        Serial.println(F("threshhold reset "));
        Serial.println(threshhold);
      #endif
    }
    loops = 0;
  }
}
/**
 * @brief ExtractDynamicValues simply extracts the dynamic component of acceleration 
 *  in each direction and stores it in the provided array pointer *dynamic. 
 *  this is done by subtracting the current accell value from the long term average.
 * 
 * 
 * @param loopsNumber pointer to the number of loops in the original function
 * @param longterm  pointer an array containing the longterm average in each axis
 * @param dynamic pointer an array containing the dynamic acceleration in each axis
 */
void ExtractDynamicValues(int *loopsNumber, long *longterm, int *dynamic)
{
  int currentaverage[3] = {0, 0, 0};    // current averaged reading
  for (int scan=0; scan<= NUMBER_OF_AVERAGES; scan++){       // loop for NUMBER_OF_AVERAGES
        currentaverage[0] += analogRead(XPIN);   // sum up readings assuming that the xaxis is in the same direction as the boat
        currentaverage[1] += analogRead(YPIN);
        currentaverage[2] += analogRead(ZPIN);   
      }
     currentaverage[0] /= NUMBER_OF_AVERAGES;     // divide total sum by num. avg. to get average
     currentaverage[1] /= NUMBER_OF_AVERAGES;     
     currentaverage[2] /= NUMBER_OF_AVERAGES;
          
     for(int i=0; i < 3; i++)
     {
       *(longterm+i) = (*(longterm+i) + (currentaverage[i]));
       *(dynamic+i) = ((currentaverage[i]) - *(longterm+i) / *(loopsNumber));
     }
     
     *loopsNumber = *loopsNumber + 1;
}
/**
 * @brief  InitializeAxis determins the axis on the accelerometer to use for determining a stroke.
 *  this is done by finding the highest dynamic values in each axis in an interval.
 * 
 * @return uint8_t the chosen axis on the accelerometer to use for stroke detection 0 is x, 1 is y, 2 is z,
 */
uint8_t InitializeAxis()
{
  uint8_t axis = 0;
  long longterm[3] = {0, 0, 0};
  int dynamic[3];
  int loopsNumber = 0;
  int largestAmplitude = 0;
  //first we let the dynamic readings stabilize
  while((dynamic[0] > 50) || (dynamic[1] > 50) || (dynamic[2] > 50))
  {
     ExtractDynamicValues(&loopsNumber, longterm, dynamic);
  } 
  
  //second we wait for the individual to start paddling
  while((dynamic[0] < 5) || (dynamic[1] < 5) || (dynamic[2] < 5))
  {
     ExtractDynamicValues(&loopsNumber, longterm, dynamic);
  }
  lcd.print("pass");
  //finally we determin the axes to measure the acceleration in
  for(int i = 0; i < 500; i++)
  {
     ExtractDynamicValues(&loopsNumber, longterm, dynamic);
     //if it has the largest delta but is not the highest in total(that is from gravity)
    /*  Serial.println(longterm[0]);
     Serial.println(longterm[1]);
     Serial.println(longterm[2]);
     Serial.println(); */
     if((largestAmplitude < dynamic[0]) && !((longterm[0] > longterm[2]) && (longterm[0] > longterm[1])))
      {
        largestAmplitude = dynamic[0];
        axis = XPIN;
      }
      if((largestAmplitude < dynamic[1]) && !((longterm[1] > longterm[2]) && (longterm[1] > longterm[0])))
      {
        largestAmplitude = dynamic[1];
        axis = YPIN;
      }
      if((largestAmplitude < dynamic[2]) && !((longterm[2] > longterm[0]) && (longterm[2] > longterm[1])))
      {
        largestAmplitude = dynamic[2];
        axis = ZPIN;
      }
  }
  Serial.print(F("the axis is"));
  Serial.println(axis);
  return axis;
}