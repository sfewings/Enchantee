////////////////////////////////////////////
//--------------------------------------------------------------------------------------------
// Ethernet support
//-------------------------------------------------------------------------------------------- 

#include <EtherCard.h>
#include <TinyNMEA.h>

static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
static byte myip[] = { 192,168,0,40 };

byte Ethernet::buffer[500];
BufferFiller bfill;

#define BUF_SIZE 200
int pos=0;
int lastLen=0;
char NMEASentence[BUF_SIZE+1];
char LastNMEASentence[BUF_SIZE+1];
bool validSentence = false;

TinyNMEA nmeaParser;

void setup()
{
    Serial.begin(4800);

    Serial.print("Anemometer test @4800 8N1");

    NMEASentence[BUF_SIZE] = '\0';
    LastNMEASentence[BUF_SIZE] = '\0';
    pos = lastLen = 0;

    if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
        Serial.println( "Failed to access Ethernet controller");
    ether.staticSetup(myip);
    ether.persistTcpConnection(true);
}

void loop()
{
    if( millis() % 10 == 0 )
    {
        word len = ether.packetReceive();
        word pos = ether.packetLoop(len);
  
        if (len)  // check if valid tcp data is received
        {
            Serial.print(len);
            Serial.print(",");
            if( pos )
            {
                for(int i=0; i<pos; i++ );
                    Serial.print(Ethernet::buffer[pos]);
            }
            bfill = ether.tcpOffset();
            bfill.emit_raw( LastNMEASentence, lastLen );
            byte sent = ether.tcpSend();
            Serial.print( "S" );
            Serial.println( sent );
        }
    }

    
    while (Serial.available())
    {
        char c = Serial.read();
        //Serial.write(c);

        if( c == '$' )
        {
            validSentence = true;
            pos = 0;
        }
        NMEASentence[pos++] = c;
        
        nmeaParser.encode(c);

        if( pos == BUF_SIZE )
        {
            Serial.println("Buffer too small for sencene");
            Serial.println(NMEASentence);
            validSentence = false;
            pos =0;
        }

        if( c == '\r' && validSentence )
        {
            NMEASentence[pos] = '\0';   //null terminate
            //Serial.println(NMEASentence);
            memcpy(LastNMEASentence, NMEASentence, pos );
            lastLen = pos;
            Serial.print("D,S,T=");
            Serial.print(nmeaParser.windDirection());
            Serial.print(",");
            Serial.print(nmeaParser.windSpeed());
            Serial.print(",");
            Serial.println(nmeaParser.temperature());
            //if( ether.isLinkUp() )
            //{
                 //bfill = ether.tcpOffset();
                 //bfill.emit_p( PSTR("$S"), NMEASentence );
                 //byte sent = ether.tcpSend();
                 //Serial.print( "S" );
                 //Serial.println( sent );
            //}
        }
    }

//    Serial.write(".\n\r");
    //delay(400);
}
