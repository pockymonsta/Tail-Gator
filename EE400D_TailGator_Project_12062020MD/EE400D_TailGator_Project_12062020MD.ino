

// TTGO T-Call pin definitions
      #define TOUCH_THRESHHOLD     40
      
                #define MODEM_RST            5
                #define MODEM_PWKEY          4
                #define MODEM_POWER_ON       23
                #define MODEM_TX             27
                #define MODEM_RX             26
                #define I2C_SDA              21
                #define I2C_SCL              22
                #define GPS_ON               32
                #define Lidar_ON             33
                #define Camera_ON            14
                #define RLed                 25
                #define YLed                 2
                #define Motion_ON            36
               // #define SMS_Button           34
                #define flash                39
                #define buzzer               12
                      
      
      #define BLYNK_PRINT Serial
      #define BLYNK_HEARTBEAT 30
      #define TINY_GSM_MODEM_SIM800
      
      #include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus
      //#include <AceButton.h> // https://github.com/bxparks/AceButton
      #include <TinyGsmClient.h>
      #include <SoftwareSerial.h>
      #include <Wire.h>
      #include "utilities.h"
      #include <TinyGsmClient.h> // https://github.com/vshymanskyy/TinyGSM
      #include <BlynkSimpleTinyGSM.h>
      #include <HardwareSerial.h>
      
      HardwareSerial myserial2(2);
      HardwareSerial myserial1(1);
      uint8_t byteFromSerial;

      static const int RXPin = 13, TXPin = 1000;
      static const uint32_t GPSBaud = 9600;
      SoftwareSerial ss(RXPin, TXPin);
      

      #define RX_PIN2       15
      #define TX_PIN2       120

//      #define RX_PIN       13
//      #define TX_PIN       121
      
      #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
      #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


//      using namespace ace_button;

//Buttons


// Emergency Number and Message

            String message = "Tailgating incident has been recorded, please check the Blynk App. For more information or live stream video. I am currently at the following location:";
            String mobile_number = "9513979669";
            String message_with_data;

// Variables for storing GPS Data
            float latitude;
            float longitude;
            float speed;
            int satellites;
            int hourVal;
            int minuteVal;
            int secVal; 
            int timeVal;
            String direction;

            int tailgate_level = 0;
            float meanDistance = 0;
            float readingsPerSecond = 0;
            
            
// Switch
//            ButtonConfig config1;
//            AceButton call_button(&config1);
//            ButtonConfig config2;
//            AceButton sms_button(&config2);
//            
//            void handleEvent_call(AceButton*, uint8_t, uint8_t);
//            void handleEvent_sms(AceButton*, uint8_t, uint8_t);

// Set serial for GPS Module
 
           #define SerialMon Serial

// Hardware Serial for builtin GSM Module

            #define SerialAT Serial1

            const char apn[]  = "h2g2";
            const char user[] = "";
            const char pass[] = "";

// Auth Token in the Blynk App.

            const char auth[] = "HrCOApP0he0uqfb7PZTx3Rt54NE9hebw";

            TinyGPSPlus gps;
            WidgetMap myMap(V0);
            BlynkTimer timer;
            TinyGsm modem(SerialAT);
            
            unsigned int move_index = 1;

/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void setup()
{ 
// go to sleep  
// Set console baud rate

      Serial.begin(115200);
      delay(100);
      ss.begin(GPSBaud);
      delay(100);
      myserial2.begin(115200, SERIAL_8N1, RX_PIN2, TX_PIN2); //<- set your RX/TX Pin
      //myserial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); //<- set your RX/TX Pin
      delay(100);

// Keep power when running from battery

      Wire.begin(I2C_SDA, I2C_SCL);
      bool   isOk = setPowerBoostKeepOn(1);
      SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

//     ledcSetup(channel, freq, resolution);
//     ledcAttachPin(12, channel);
     

//Set-up modem reset, enable, power pins
  
            pinMode(MODEM_PWKEY,OUTPUT);
            pinMode(MODEM_RST,OUTPUT);
            pinMode(MODEM_POWER_ON,OUTPUT);
            pinMode(Lidar_ON,OUTPUT);//33
            pinMode(Camera_ON,OUTPUT);//14
            pinMode(GPS_ON,OUTPUT);//32
            pinMode(RLed, OUTPUT);//25
            pinMode(YLed, OUTPUT);//2
            pinMode(buzzer, OUTPUT);//12
            pinMode(Motion_ON,INPUT);//0
            pinMode(flash,OUTPUT);//35
          
          
          
            digitalWrite(MODEM_PWKEY, LOW);
            digitalWrite(MODEM_RST, HIGH);
            digitalWrite(MODEM_POWER_ON, HIGH);
            digitalWrite(Camera_ON, LOW);//14
            digitalWrite(Lidar_ON, LOW);//33
            digitalWrite(GPS_ON, LOW);//32
            digitalWrite(RLed, LOW); //25
            digitalWrite(YLed, LOW); //2
            digitalWrite(buzzer, LOW); //12
            digitalWrite(flash, LOW); //35


            
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set GSM module baud rate and UART pins
        SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
        delay(3000);

// Restart takes quite some time
// To skip it, call init() instead of restart()
  
        SerialMon.println("Initializing modem...");
        modem.restart();

        String modemInfo = modem.getModemInfo();
        SerialMon.print("Modem: ");
        SerialMon.println(modemInfo);

        SerialMon.print("Waiting for network...");
        if (!modem.waitForNetwork(240000L)) {
          SerialMon.println(" fail");
          delay(10000);
          return;
        }
        SerialMon.println(" OK");
      
        if (modem.isNetworkConnected()) {
          SerialMon.println("Network connected");
        }
      
        SerialMon.print(F("Connecting to APN: "));
        SerialMon.print(apn);
        if (!modem.gprsConnect(apn, user, pass)) {
          SerialMon.println(" fail");
          delay(10000);
          return;
        }
        SerialMon.println(" OK");
        Blynk.begin(auth, modem, apn, user, pass);
        digitalWrite(GPS_ON, HIGH);//32
        
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void checkGPS()
{                
//// This sketch displays information every time a new sentence is correctly encoded.

        smartDelay(200);
        displayInfo();
        digitalWrite(Lidar_ON, HIGH);//33
        
            }  

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
        checkGPS(); 
        speedTest();
        Blynk.run();
        timer.run();
//        sms_button.check();
//        call_button.check();
        
        if (meanDistance < ((speed / 10) * 450)){
          tailgate_degree();
        }
        else {
          digitalWrite(Camera_ON, LOW);
        }
        
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void displayInfo()
{
   
       if ( gps.location.isValid())
    {
    
    latitude = (gps.location.lat());     //Storing the Lat. and Lon.
    longitude = (gps.location.lng());
    timeVal = (gps.time.value());

    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);

    Blynk.virtualWrite(V1, String(latitude, 6));
    Blynk.virtualWrite(V2, String(longitude, 6));
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    speed =((gps.speed.kmph())/1.6214);               //get speed
    Blynk.virtualWrite(V3, speed);


    direction = TinyGPSPlus::cardinal(gps.course.value()); // get the direction
    Blynk.virtualWrite(V4, direction);

    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);

    
    satellites = (gps.satellites.value());    //get number of satellites
    Blynk.virtualWrite(V5, satellites);
    float Distance= meanDistance;
    Blynk.virtualWrite(V7, Distance);
    float RPS= readingsPerSecond;
    Blynk.virtualWrite(V8, RPS);

// time variables from GPS

// Adjust for our time zone

    if ((gps.time.hour())>= 12){
      hourVal = ((gps.time.hour())-8);
    }
    else {
      hourVal = ((gps.time.hour())+4);
    }  
    minuteVal = gps.time.minute();
    secVal = gps.time.second();
    
//timeVal = String(hourVal) + ":" + String(minuteVal) + ":" + String(secVal); // concatenate strings to create time variable
   
// display time in Blynk app 

    Blynk.virtualWrite(V6, String(hourVal) + ":" + String(minuteVal) + ":" + String(secVal));

    Distance= meanDistance;
    Blynk.virtualWrite(V7, Distance);
    RPS= readingsPerSecond;
    Blynk.virtualWrite(V8, RPS);

    Serial.println( "\n\nSpeed test:" );
    Serial.printf( "%f readings per second.\n", readingsPerSecond );
    Serial.printf( "%f mean read distance.\n", meanDistance );
    Serial.println();
     if (gps.time.isValid())
    {
    if (gps.time.hour() < 10)

    Serial.print("TIME: ");
    Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
   
    if (gps.time.minute() < 10) 
    
    Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    
    if (gps.time.second() < 10) 
    
    Serial.print(F("0"));
    Serial.print(gps.time.second());

  }
   
   Serial.println();

}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
                   
void speedTest() {

  while ( myserial2.available() > 0 ) {
    myserial2.read();
  }

  long t0 = millis();

  #define NUM_READINGS 50

  long accum = 0;

  for ( int i = 0; i < NUM_READINGS; i++ ) {

    accum += readLIDAR( 100 );
  
  }

  long t1 = millis();

  readingsPerSecond = NUM_READINGS * 1000.0f / ( t1 - t0 );

  meanDistance = ((float)accum) / NUM_READINGS;

    while ( myserial2.available() == 0 ) {
    delay( 5 );
  }
  while ( myserial2.available() > 0 ) {
    myserial2.read();
  }

}

unsigned int readLIDAR( long timeout ) {

  unsigned char readBuffer[ 9 ];

  long t0 = millis();

  while ( myserial2.available() < 9 ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    delay( 5 );
  }

  for ( int i = 0; i < 9; i++ ) {
    readBuffer[ i ] = myserial2.read();
  }

  while ( ! detectFrame( readBuffer ) ) {

    if ( millis() - t0 > timeout ) {
      // Timeout
      return 0;
    }

    while ( myserial2.available() == 0 ) {
      delay( 5 );
    }

    for ( int i = 0; i < 8; i++ ) {
      readBuffer[ i ] = readBuffer[ i + 1 ];
    }

    readBuffer[ 8 ] = myserial2.read();

  }

  // Distance is in bytes 2 and 3 of the 9 byte frame.
  unsigned int distance = ( (unsigned int)( readBuffer[ 2 ] ) ) |
                          ( ( (unsigned int)( readBuffer[ 3 ] ) ) << 8 );

  return distance;


}

bool detectFrame( unsigned char *readBuffer ) {

  return  readBuffer[ 0 ] == 0x59 &&
          readBuffer[ 1 ] == 0x59 &&
          (unsigned char)(
            0x59 +
            0x59 +
            readBuffer[ 2 ] + 
            readBuffer[ 3 ] + 
            readBuffer[ 4 ] +
            readBuffer[ 5 ] + 
            readBuffer[ 6 ] + 
            readBuffer[ 7 ]
          ) == readBuffer[ 8 ];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void tailgate_degree()
{
   if (meanDistance < ((speed / 10) * 400))
  {
    tailgate_level = 0;
    digitalWrite(Camera_ON, HIGH);
    Serial.println();
    Serial.println("Camera initialized....");
    Serial.println();
    
  }
  else{
    digitalWrite(Camera_ON, LOW);
    Serial.println();
    Serial.println("Camera is OFF");
    Serial.println();
  }  
  
  if (meanDistance < ((speed / 10) * 350)&& meanDistance >= ((speed / 10) * 250))
  {
    tailgate_level = 1;
    digitalWrite(YLed, HIGH);
    digitalWrite(RLed, LOW);
    Serial.println();
    Serial.println("RedLED_OFF, YelowLED_ON");
    Serial.println();
    alarm_1();
    digitalWrite(YLed, LOW);
  }
  else{
    digitalWrite(YLed, LOW);
   // ledcWrite(channel, 0);
  }
  
  if (meanDistance < ((speed / 10) * 250))
  {
    tailgate_level = 2;
    digitalWrite(YLed, LOW);
    digitalWrite(RLed, HIGH);
    Serial.println();
    Serial.println("RedLED_ON, YelowLED_OFF");
    Serial.println();
    alarm_2();
    message_with_data = message + "  Latitude is: " + (gps.location.lat()) + "  And my Longitude is: " + (gps.location.lng())+ "  The Time Now is: " + (gps.time.value());
    modem.sendSMS(mobile_number, message_with_data);
    message_with_data = "";
    digitalWrite(RLed, LOW);
  }
  else{
    digitalWrite(RLed, LOW);
    //ledcWrite(channel, 0);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////

void alarm_1()
{   
   digitalWrite(buzzer, HIGH);
   delay(150);
   digitalWrite(buzzer, LOW);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void alarm_2()
{ 
   digitalWrite(buzzer, HIGH);
   delay(150);
   digitalWrite(buzzer, LOW);
   delay(150);
   digitalWrite(buzzer, HIGH);
   delay(150);
   digitalWrite(buzzer, LOW);
   
}


static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());     
  } 
  while (millis() - start < ms);

 if (millis() > 5000 && gps.charsProcessed() < 10)
 
  {
    
    Serial.println(F("No GPS detected: check wiring void"));
    Blynk.virtualWrite(V4, "Outdoor");
    Blynk.virtualWrite(V3, "Try to");
    Blynk.virtualWrite(V2, "detected:");
    Blynk.virtualWrite(V1, "No GPS");
    Blynk.virtualWrite(V5, "Move");
    float Distance= meanDistance;
    Blynk.virtualWrite(V7, Distance);
    float RPS= readingsPerSecond;
    Blynk.virtualWrite(V8, RPS);
    Blynk.virtualWrite(V6, String(hourVal) + ":" + String(minuteVal) + ":" + String(secVal));
    Distance= meanDistance;
    Blynk.virtualWrite(V7, Distance);
    RPS= readingsPerSecond;
    Blynk.virtualWrite(V8, RPS);
    digitalWrite(Lidar_ON, LOW);//33
    smartDelay(200);
  } 
  
}

//void repeatMeFiveTimes() {
//// do something
//}
//
//timerId = timer.setTimer(1000, repeatMeFiveTimes, 5);
//}
