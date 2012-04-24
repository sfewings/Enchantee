////////////////////////////////////////////
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <EtherCard.h>
#include <LiquidCrystal.h>

/////////////////////////////////////////
// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x32 };
static byte myip[] = { 10,0,0,15 }; 
//static byte myip[] = { 192,168,0,15 };
byte Ethernet::buffer[550];
BufferFiller bfill;


/////////////////////////////////////////
// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// turn on only the second sentence (GPRMC)
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA   "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_GGARMCGSA "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_RMCGSA    "$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_RMCONLY   "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

// set baud rate
#define PMTK_SET_BAUD_4800   "$PMTK251,4800*14"
#define PMTK_SET_BAUD_9600   "$PMTK251,9600*17"
#define PMTK_SET_BAUD_19200  "$PMTK251,19200*22"
#define PMTK_SET_BAUD_38400  "$PMTK251,38400*27"
#define PMTK_SET_BAUD_57600  "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"


TinyGPS gps;
SoftwareSerial ss(8, 9);


////////////////////////////////////////////////
LiquidCrystal lcd(A2,A3, 7,6,5,4);

#define MAX_COUNT  25
#define LED_PIN A4
#define PPS_PIN  3
#define LOGWHEEL_PIN  2

char printBuf[16];
boolean g_toggleDisplay;
float   g_flat;
float   g_flon;
float   g_fspeed;
float   g_fcourse;
int     g_numCounts[MAX_COUNT];
double  g_sumSpeed[MAX_COUNT];
int     g_lastLogCount;
long    g_ms;


// interrupt driven pulse counts 
// ATmega 168 and 328 - interrupt 0 = pin 2, 1 = pin 3
volatile boolean    g_PPSOn;
volatile boolean    g_hasPPSSignal;
volatile long       g_timePPS;
volatile int        g_logCount;

////////////////////////////////////////////////
// routine called when external interrupt is triggered
void interruptHandlerGPS_PPS() 
{
  if( g_PPSOn )
      digitalWrite(LED_PIN, LOW);
  else
      digitalWrite(LED_PIN, HIGH);
  g_PPSOn = !g_PPSOn;
  g_timePPS = millis();
  
  
  //update the log count calibration
  g_lastLogCount = g_logCount;
  g_logCount = 0;
  
  if( g_lastLogCount/2 < MAX_COUNT )
  {
   g_numCounts[g_lastLogCount/2] ++;
   g_sumSpeed[g_lastLogCount/2] += g_fspeed;   
  }
  else
  {
   g_numCounts[MAX_COUNT-1] ++;
   g_sumSpeed[MAX_COUNT-1] += g_fspeed;   
  }
}

void interruptHandlerLogCounter() 
{
  g_logCount++;
}

/////////////////////////////////////////////////
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

static char* printMSTime(long ms, char* buf)
{
  long t = ms / 1000;
  word h = t / 3600;
  byte m = (t / 60) % 60;
  byte s = t % 60;
  byte ss= (ms % 1000)/100;
  sprintf(buf, "%d%d:%0d%d:%d%d.%d", h/10, h%10, m/10, m%10, s/10, s%10, ss );
  return buf;
}
  
////////////////////////////////////////////////
//Web support
static word servePage(word pos) 
{
  bfill = ether.tcpOffset();
  
  char* data = (char *) Ethernet::buffer + pos;
#if SERIAL
        Serial.println(data);
#endif
  // receive buf hasn't been clobbered by reply yet
  if (strncmp("GET /c", data, 6) == 0)      //calibration details
  {
    bfill = ether.tcpOffset();
    bfill.emit_p(PSTR(
      "HTTP/1.0 200 OK\r\n"
      "Content-Type: text/html\r\n"
      "Pragma: no-cache\r\n"
      "\r\n"
      "<meta http-equiv='refresh' content='1'/>"
      "<title>Enchantee Log</title>"));
    bfill.emit_p(PSTR("<h1>Log Wheel\tCount\tAvg speed<h1><pre>"));
    for(int i=0; i< MAX_COUNT; i++ )
    {
      if( g_numCounts[i] == 0 ) 
        bfill.emit_p(PSTR("$D\t0\t0.0\n"),i*2 );
      else
      {
        float val = fabs(g_sumSpeed[i]/g_numCounts[i]);
        bfill.emit_p(PSTR("$D\t$D\t$D.$D\n"), i*2, g_numCounts[i], (int)val,(int)((val-(int)val)*100));
      }
    }  
  }
  else  //general home page
  {
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
    bfill.emit_p(PSTR("<h1>Speed $D.$D</h1>"), (int)g_fspeed, (int)((g_fspeed-(int)g_fspeed)*10) );
    bfill.emit_p(PSTR("<h1>Course $D.$D</h1>"), (int)g_fcourse, (int)((g_fcourse - (int)g_fcourse)*10) );
    bfill.emit_p(PSTR("<h1>Satellites $D</h1>"), gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites() );
    int hDop = gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop();  
    bfill.emit_p(PSTR("<h1>Horz DOP $D.$D</h1>"), (int)hDop/100, (int)((hDop/100.0 - (int)(hDop/100))*100));
    bfill.emit_p(PSTR("<h1>Water log counter $D</h1>"), g_lastLogCount );
  }
  bfill.emit_p(PSTR("Running $S\n"), printMSTime(millis(), printBuf));
  bfill.emit_p(PSTR("Free mem $D\n"), get_free_memory() );
  bfill.emit_p(PSTR("Since PPS $S\n"), printMSTime(millis()-g_timePPS, printBuf));
 
  return bfill.position();
}

////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  
  Serial.print("Simple TinyGPS library v. "); 
  Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();    

  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("Tiny GPS v");
  lcd.print(TinyGPS::library_version());
  lcd.setCursor(0,1);
  
  g_toggleDisplay = true;
  g_ms = millis();
  
  //initialise GPS PPS interrups
  pinMode(LED_PIN, OUTPUT);     
  digitalWrite(LED_PIN, LOW );
  g_PPSOn = false;
  g_timePPS = millis();
  pinMode(PPS_PIN, INPUT);     
  digitalWrite(PPS_PIN, HIGH);   
  attachInterrupt(1, interruptHandlerGPS_PPS, CHANGE);      

  //set up the interrupt handler that will count the log counter wheel
  g_logCount = 0;
  g_lastLogCount = 0;
  for(int i=0; i<MAX_COUNT;i++)
  {
     g_numCounts[i] = 0;
     g_sumSpeed[i] = 0.0;  
  }
  pinMode(LOGWHEEL_PIN, INPUT);     
  digitalWrite(LOGWHEEL_PIN, HIGH);   
  attachInterrupt(0, interruptHandlerLogCounter, RISING);  
    

  //initialise the GPS module
  ss.begin( 9600 );
  ss.println( PMTK_SET_BAUD_9600 );
  ss.begin(19200);
  ss.println( PMTK_SET_BAUD_9600 );
  ss.begin(38400);
  ss.println( PMTK_SET_BAUD_9600 );
  ss.begin(57600);
  ss.println( PMTK_SET_BAUD_9600 );
  ss.begin(115200);
  ss.println( PMTK_SET_BAUD_9600 );

  ss.begin(9600);

  //have to set SoftwareSerial.h _SS_MAX_RX_BUFF 64 to 128 to receive more than one NEMEA string
  //ss.println(PMTK_SET_NMEA_OUTPUT_GGARMCGSA);
  //ss.println(PMTK_SET_NMEA_OUTPUT_RMCGSA);
  ss.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //ss.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  ss.println(PMTK_SET_NMEA_UPDATE_1HZ);
  //ss.println(PMTK_SET_NMEA_UPDATE_5HZ);
  
    
  //for Nanode    if (ether.begin(sizeof Ethernet::buffer, mymac, 8) == 0)
  if (ether.begin(sizeof Ethernet::buffer, mymac, 10) == 0)
    Serial.println( "Failed to access Ethernet controller");
  ether.staticSetup(myip);
  //1000ms delay required for Netcom 3G9WB modem
  delay(1000);
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  boolean hasPPS = (millis()-g_timePPS < 1000);
  
  while (ss.available())
  {
    char c = ss.read();
    Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
    {
      newData = true;
      Serial.write("==>Received");
    }
  }

  if (newData)
  {
    unsigned long age;

    gps.f_get_position(&g_flat, &g_flon, &age);
    g_fspeed = gps.f_speed_kmph();
    g_fcourse = gps.f_course();
    
    Serial.print("Has PPS=");
    Serial.print(hasPPS);
    Serial.print(" LAT=");
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
    Serial.println(g_fcourse == TinyGPS::GPS_INVALID_ANGLE ? 0 : g_fcourse/100.0,1);


    lcd.setCursor(0,0); 
//    if( g_toggleDisplay )
//    {
//      lcd.setCursor(0,0); 
//      lcd.print( printDMS( g_flat, printBuf ));
//      lcd.print("      ");
//      lcd.setCursor(0,1); 
//      lcd.print( printDMS( g_flon, printBuf ));
//      //lcd.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 12);
//      lcd.print("      ");      
//    }
//    else
    {
      lcd.setCursor(0,0); 
      lcd.print("Sd=");
      lcd.print(g_fspeed == TinyGPS::GPS_INVALID_SPEED ? 0 : fabs(g_fspeed),1);
      lcd.print(" Cs=");
      lcd.print(g_fcourse == TinyGPS::GPS_INVALID_ANGLE ? 0 : g_fcourse,1);
      lcd.print("      ");
 
      lcd.setCursor(0,1); 
      lcd.print("Log=");
      lcd.print( g_lastLogCount );
      lcd.print("/");
      lcd.print( g_logCount );
      if( hasPPS )
      {
        lcd.print(" Sat=");
        lcd.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
        lcd.print("   ");
      }
      else
        lcd.print(" Sat=0  ");
    }
  }

  if( millis() - g_ms > 1000 )
  {    
    g_toggleDisplay = !g_toggleDisplay;
    g_ms = millis();

    Serial.print("Has PPS=");
    Serial.print(hasPPS);

    if( !hasPPS )
    {
      //we are not getting PPS. Update the last second log count
      g_lastLogCount = g_logCount;
      g_logCount = 0;
      lcd.setCursor(0,1); 
      lcd.print("Log=");
      lcd.print( g_lastLogCount );
      lcd.print(" Sat=0  ");
    }

    Serial.print("Log count=");
    Serial.println( g_lastLogCount );
  }

  //service web server requests  
  word len = ether.packetReceive();
  word pos = ether.packetLoop(len);
 
  if (pos)  // check if valid tcp data is received
  {
    ether.httpServerReply(servePage(pos)); // send web page data
  }
}
