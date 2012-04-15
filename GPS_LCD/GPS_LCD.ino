////////////////////////////////////////////
#include <SoftwareSerial.h>

#include <TinyGPS.h>

//#undef WEB_SUPPORT
#define WEB_SUPPORT


/////////////////////////////////////////
#ifdef WEB_SUPPORT
#include <EtherCard.h>
// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
static byte myip[] = { 10,0,0,10 }; 
//static byte myip[] = { 192,168,0,15 };
byte Ethernet::buffer[500];
BufferFiller bfill;
#endif
/////////////////////////////////////////

/////
#include <LiquidCrystal.h>

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_GGARMCGSA "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGSA "$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"



TinyGPS gps;
SoftwareSerial ss(8, 9);


LiquidCrystal lcd(A2,A3, 7,6,5,4);

#define MAX_COUNT  15
#define LED A4
boolean g_showLat = true;
char printBuf[16];
char printBuf2[16];
float g_flat, g_flon;
float g_fspeed, g_fcourse;
int     g_numCounts[MAX_COUNT];
double  g_sumSpeed[MAX_COUNT];


 // interrupt driven pulse counts 
#define INTERRUPT_IR        1    // ATmega 168 and 328 - interrupt 0 = pin 2, 1 = pin 3
volatile boolean    g_PPSOn;
volatile boolean    g_hasPPSSignal;
volatile long       g_timePPS;
volatile int        g_logCount;
int                 g_lastLogCount;
long                g_ms;
boolean             g_toggleHomePage;

// routine called when external interrupt is triggered
void interruptHandlerGPS_PPS() 
{
  if( g_PPSOn )
      digitalWrite(LED, LOW);
  else
      digitalWrite(LED, HIGH);
  g_PPSOn = !g_PPSOn;
  if( g_PPSOn )
  {
    g_lastLogCount = g_logCount;
    g_logCount = 0;
    
    g_timePPS = millis();
    
    if( g_lastLogCount < MAX_COUNT )
    {
       g_numCounts[g_lastLogCount] ++;
       g_sumSpeed[g_lastLogCount] += g_fspeed;   
    }
    else
    {
       g_numCounts[MAX_COUNT-1] ++;
       g_sumSpeed[MAX_COUNT-1] += g_fspeed;   
    }
  }
}

void interruptHandlerLogCounter() 
{
  g_logCount++;
}


extern int __bss_end;
extern void *__brkval;

int get_free_memory()
{
  int free_memory;

  if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}

static void lcdInt (byte x, byte y, unsigned int value, char fill =' ') 
{
  lcd.setCursor(x, y);
  int places = log10(value);
  for(int i=0; i<4-places;i++)
  lcd.print(fill);
  lcd.print(value);  
}  

static char* printDMS(float degrees, char* buf )
{
  float fValue = fabs(degrees);
  int deg = (int) fValue;
  float remainder = fValue-deg;
  fValue = remainder*60.0;
  int mins = (int) fValue;
  remainder = fValue - mins;
  fValue = remainder * 100.0;
  int secs = (int) fValue;
  remainder = fValue- secs;
  fValue = remainder *1000;
  int ptSec = (int) fValue;
  sprintf(buf, "%d:%d:%d.%d",deg,mins,secs, ptSec);

  return buf;
}



#ifdef WEB_SUPPORT
static word homePage() 
{
  g_toggleHomePage = !g_toggleHomePage;
  if( g_toggleHomePage )
  {
    long t = millis() / 1000;
    word h = t / 3600;
    byte m = (t / 60) % 60;
    byte s = t % 60;
    bfill = ether.tcpOffset();
    bfill.emit_p(PSTR(
      "HTTP/1.0 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Pragma: no-cache\r\n"
      "\r\n"
      "<meta http-equiv='refresh' content='1'/>"
      "<title>Enchantee Log</title>"));
  
    printDMS(g_flat, printBuf); 
    bfill.emit_p(PSTR("<h1>Lat $S<h1>"), printBuf );
    printDMS(g_flon, printBuf);
    bfill.emit_p(PSTR("<h1>Lon $S<h1>"), printBuf );
    bfill.emit_p(PSTR("<h1>Speed $D.$D Course $D.$D</h1>"), (int)g_fspeed, (int) g_fspeed*10, (int)g_fcourse, (int) g_fcourse*10 );
    bfill.emit_p(PSTR("<h1>Water log counter $D</h1>"), g_logCount );
    bfill.emit_p(PSTR("<h1>Running $D$D:$D$D:$D$D</h1>"), h/10, h%10, m/10, m%10, s/10, s%10);
  }
  else
  {
    bfill = ether.tcpOffset();
    bfill.emit_p(PSTR(
      "HTTP/1.0 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Pragma: no-cache\r\n"
      "\r\n"
      "<meta http-equiv='refresh' content='1'/>"
      "<title>Enchantee Log</title>"));
    bfill.emit_p(PSTR("<h1>Log Wheel\tCOunt\tAvg speed<h1><pre>"));
    for(int i=0; i< MAX_COUNT; i++ )
    {
      if( g_numCounts[i] == 0 ) 
        bfill.emit_p(PSTR("$D\t0\t0.0\n"),i );
      else
        bfill.emit_p(PSTR("$D\t$D\t$D.$D\n"), i, g_numCounts[i], g_sumSpeed[i]/g_numCounts[i] );
    }  
  }
  bfill.emit_p(PSTR("Free mem $D"), get_free_memory() );
 
  return bfill.position();
}
#endif


  
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
    
    g_PPSOn = digitalRead(LED);
    g_timePPS = 0;
    g_toggleHomePage = false;
    
    attachInterrupt(1, interruptHandlerGPS_PPS, CHANGE);      

    //set up the interrupt handler that will count the log counter wheel
    g_logCount = 0;
    g_lastLogCount = 0;
    pinMode(3, INPUT);     
    digitalWrite(3, HIGH);   
    attachInterrupt(0, interruptHandlerLogCounter, RISING);  


    ss.begin(9600);
    // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC only (see above)
    ss.println(PMTK_SET_NMEA_UPDATE_1HZ);
    //ss.println(PMTK_SET_NMEA_UPDATE_5HZ);
    //ss.println(PMTK_SET_NMEA_OUTPUT_GGARMCGSA);
    //ss.println(PMTK_SET_NMEA_OUTPUT_RMCGSA);
    ss.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    g_ms = millis();
    
    for(int i=0; i<MAX_COUNT;i++)
    {
       g_numCounts[i] = 0;
       g_sumSpeed[i] = 0.0; 
      
    }
    
#ifdef WEB_SUPPORT
  if (ether.begin(sizeof Ethernet::buffer, mymac, 10) == 0)
    Serial.println( "Failed to access Ethernet controller");
  ether.staticSetup(myip);
#endif
    
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 450;)
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
    unsigned long age;

    gps.f_get_position(&g_flat, &g_flon, &age);
    g_fspeed = gps.f_speed_kmph();
    g_fcourse = gps.f_course();
    
    Serial.print("LAT=");
    Serial.print(g_flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : g_flat, 6);
    Serial.print(" LON=");
    Serial.print(g_flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : g_flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.print(" SPEED=");
    Serial.print(g_fspeed == TinyGPS::GPS_INVALID_SPEED ? 0 : g_fspeed/100.0,1);
    Serial.print(" HEADING=");
    Serial.print(g_fcourse == TinyGPS::GPS_INVALID_ANGLE ? 0 : g_fcourse/100.0,1);


    lcd.setCursor(0,0); 
    if( g_showLat )
    {
      lcd.setCursor(0,0); 
      //lcd.print("Lt");
      //lcdDMS( flat );
      lcd.print( printDMS( g_flat, printBuf ));
      //lcd.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 12);
      lcd.print("      ");
      lcd.setCursor(0,1); 
      lcd.print( printDMS( g_flon, printBuf ));
      //lcd.print("Lo");
      //lcdDMS( flon );
      //lcd.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 12);
      lcd.print("      ");      
    }
    else
    {
      lcd.setCursor(0,0); 
      lcd.print("Sd=");
      lcd.print(g_fspeed == TinyGPS::GPS_INVALID_SPEED ? 0 : fabs(g_fspeed),1);
      lcd.print(" Cs=");
      lcd.print(g_fcourse == TinyGPS::GPS_INVALID_ANGLE ? 0 : g_fcourse,1);
      lcd.print("      ");
      lcd.setCursor(0,1); 
      lcd.print("WaterLog=");
      lcd.print( g_lastLogCount );
      lcd.print(" / ");
      lcd.print( g_logCount );
      //lcd.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      //lcd.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      //lcd.print(" Prc=");
      //lcd.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop()/100.0,1);
      lcd.print("      ");
    }
    
    if( millis() - g_ms > 10000 )
    {      
      g_showLat = !g_showLat;
      g_ms = millis();
    }
    
  }
  
  //if no PPS signal
  if( (millis() - g_timePPS)  > 1000 )
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
  
#ifdef WEB_SUPPORT
 word len = ether.packetReceive();
  word pos = ether.packetLoop(len);
  
  if (pos)  // check if valid tcp data is received
    ether.httpServerReply(homePage()); // send web page data
#endif

//  digitalWrite(LED, HIGH);
//  delay(100);
//  digitalWrite(LED, LOW);
//  delay(900);
}
