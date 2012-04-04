////////////////////////////////////////////
#include <SoftwareSerial.h>

#include <TinyGPS.h>

//////
#include <LiquidCrystal.h>

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"



TinyGPS gps;
SoftwareSerial ss(8, 9);


LiquidCrystal lcd(A2,A3, 7,6,5,4);
#define LED A4
boolean g_showLat = true;


 // interrupt driven pulse counts 
#define INTERRUPT_IR        1    // ATmega 168 and 328 - interrupt 0 = pin 2, 1 = pin 3
volatile boolean    g_PPSOn;


// routine called when external interrupt is triggered
void interruptHandlerIR() 
{
  if( g_PPSOn )
      digitalWrite(LED, LOW);
  else
      digitalWrite(LED, HIGH);
  g_PPSOn = !g_PPSOn;
}

static void lcdInt (byte x, byte y, unsigned int value, char fill =' ') {
  lcd.setCursor(x, y);
  int places = log10(value);
  for(int i=0; i<4-places;i++)
  lcd.print(fill);
  lcd.print(value);  
}  

void setup()
{
    Serial.begin(115200);
    
    lcd.begin(16,2);
    
    Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
    Serial.println("by Mikal Hart");
    Serial.println();    

    lcd.setCursor(0,0);
    lcd.print("Tiny GPS v");
    lcd.print(TinyGPS::library_version());
    lcd.setCursor(0,1);
    
    pinMode(LED, OUTPUT);     
    g_showLat = true;
    
    g_PPSOn = true;
    attachInterrupt(1, interruptHandlerIR, CHANGE);      

    ss.begin(9600);
    // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC only (see above)
    ss.println(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    float fspeed, fcourse;
    
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    fspeed = gps.f_speed_kmph();
    fcourse = gps.f_course();
    
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.print(" SPEED=");
    Serial.print(fspeed == TinyGPS::GPS_INVALID_SPEED ? 0 : fspeed/100.0,1);
    Serial.print(" HEADING=");
    Serial.print(fcourse == TinyGPS::GPS_INVALID_ANGLE ? 0 : fcourse/100.0,1);


    lcd.setCursor(0,0); 
    if( g_showLat )
    {
      lcd.setCursor(0,0); 
      //lcd.print("Lt");
      lcd.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 12);
      lcd.print("      ");
      lcd.setCursor(0,1); 
      //lcd.print("Lo");
      lcd.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 12);
      lcd.print("      ");      
    }
    else
    {
      lcd.setCursor(0,0); 
      lcd.print("Sat=");
      lcd.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      lcd.print(" Prc=");
      lcd.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
      lcd.print("      ");
      lcd.setCursor(0,1); 
      lcd.print("Sd=");
      lcd.print(fspeed == TinyGPS::GPS_INVALID_SPEED ? 0 : fabs(fspeed/100.0),2);
      lcd.print(" Cs=");
      lcd.print(fcourse == TinyGPS::GPS_INVALID_ANGLE ? 0 : fcourse/100.0,2);
      lcd.print("      ");
    }
    
    g_showLat = !g_showLat;
    
  }
  else
  {
    lcd.setCursor(0,1); 
    lcd.print("C=");
    lcd.print(chars);
    lcd.print(" S=");
    lcd.print(sentences);
    lcd.print(" ERR=");
    lcd.print(failed); 
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);  
  
  
//  digitalWrite(LED, HIGH);
//  delay(100);
//  digitalWrite(LED, LOW);
//  delay(900);
}
