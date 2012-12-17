// Demo of the Heading Board based on the HDPM01 (with barometer and compass)
// 2010-03-22 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

HeadingBoard sensor (4);
MilliTimer measureTimer;

unsigned int Xsens, Ysens;
unsigned int Xoffset, Yoffset;
unsigned int angle ;
unsigned char cAtanStep[19]={0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23};
int  iAtanPoint[19]={ 0x020C,0x0998,0x0d60,0x1057,0x12cd,0x1508,0x1702,0x18ca,0x1a8c,0x1c2c,0x1da6,0x1f12,0x206e,0x21b8,0x2258,0x244d,0x2576,0x26a8,0x2710};
int iAtanAngle[19]={0x001e,0x008a,0x00bd,0x00e3,0x0101,0x011b,0x0131,0x0144,0x0156,0x0166,0x0174,0x0181,0x018d,0x0198,0x01a3,0x01ad,0x01b6,0x01bf,0x01c2};


unsigned int step=0;


void compass_reset_calibration(void)
{
    unsigned int XRaw,YRaw;
    unsigned int i;
    unsigned int Xmax = 0,
    Xmin = 4095,
    Ymax = 0,
    Ymin = 4095;
    //compass_set_reset();
    //compass_set_reset();
    //compass_set_reset();
    for(i=0;i<60;i++)
    {
        int x, y;
        sensor.heading(x, y);
        XRaw = x + 2048;
        YRaw = y + 2048;

        if( XRaw > Xmax )
            Xmax = XRaw;
        if( XRaw < Xmin )
            Xmin = XRaw;
        if( YRaw > Ymax )
            Ymax = YRaw;
        if( YRaw < Ymin )
            Ymin = YRaw;
    }
    Xoffset = (Xmax + Xmin) / 2;
    Yoffset = (Ymax + Ymin) / 2;
    Xsens = (Xmax - Xmin) / 2;
    Ysens = (Ymax - Ymin) / 2;
}

void get_compass_data(unsigned int &XRaw, unsigned int &YRaw)
{
    unsigned char i;
    unsigned int x_value, y_value, x_max, y_max,x_min, y_min ;

    x_max = y_max = x_value = y_value = 0;
    x_min = y_min = 4095;

    for(i = 0; i<6; i++)
    {
        int x, y;
        sensor.heading(x, y);
        XRaw = x + 2048;
        YRaw = y + 2048;

        x_value += XRaw;
        y_value += YRaw;
        if(XRaw > x_max ) 
            x_max = XRaw;
        if(YRaw > y_max ) 
            y_max = YRaw;
        if(XRaw < x_min ) 
            x_min = XRaw;
        if(YRaw < y_min ) 
            y_min = YRaw;
    }
    x_value -= x_max;
    x_value -= x_min;
    y_value -= y_max;
    y_value -= y_min;
    XRaw = x_value/4;
    YRaw = y_value/4;
}

void cal_compass(void)
{
    unsigned int XRaw, YRaw;
    unsigned int X_raw, Y_raw;
    unsigned long int lAcc;
    
    get_compass_data( XRaw, YRaw );

    if( XRaw > Xoffset )
    {
        X_raw = XRaw - Xoffset;
    }
    else
    {
        X_raw = Xoffset - XRaw ;
    }
    if( YRaw > Yoffset)
    {
        Y_raw = YRaw - Yoffset;
    }
    else
    {
        Y_raw = Yoffset - YRaw ;
    }
    if( Xsens > Ysens )
        Y_raw = ( Y_raw * Xsens ) / Ysens;
    else
        X_raw = ( X_raw * Ysens ) / Xsens;
    if( X_raw >= Y_raw)
    {
        lAcc = Y_raw;
        lAcc *= 1000;
        lAcc /= X_raw;
        lAcc /= 1;
        lAcc = arcTan(lAcc);
        angle = lAcc / 10;
    }
    else
    {
        lAcc = X_raw;
        lAcc *= 1000;
        lAcc = lAcc/Y_raw;
        lAcc = arcTan(lAcc);
        angle = 90 - (lAcc/10);
    }
    if((XRaw < Xoffset) && (YRaw > Yoffset)) 
        angle = 180 - angle;
    else if((XRaw < Xoffset) && (YRaw < Yoffset) ) 
        angle = 180 + angle;
    else if((XRaw > Xoffset) && (YRaw < Yoffset) )
        angle = 360 - angle;
    if(angle>=360) 
        angle -=360;
    else if(angle<0) 
        angle +=360;
}



unsigned long int arcTan(unsigned long int lAcc)
{
    char I;
    lAcc *= 10;
    if (lAcc>10000)
    lAcc=10000;
    for (I=0x00;I<0x19;I++)
    {
        if (lAcc<=iAtanPoint[I])
        {
            if (I== 0)
            {
                lAcc/=cAtanStep[0];
            }
            else
            {
                lAcc-=iAtanPoint[I-1];
                lAcc/=cAtanStep[I-1];
                lAcc+=iAtanAngle[I-1];
            }
            break;
        }
    }
    return(lAcc);
}


void do_compass_calibrate()
{
    compass_reset_calibration();
    Serial.print("Calibration ");
    char buf[120];
    sprintf(buf, "%u, %u, %u, %u", Xoffset, Yoffset, Xsens, Ysens );
    Serial.println(buf);
}



void setup () {
    //Serial.begin(57600);
    Serial.begin(9600);
    Serial.println("\n[heading_demo]");
    rf12_initialize(7, RF12_915MHZ, 5);
    sensor.begin();

    do_compass_calibrate();
}


void loop () 
{
    if (measureTimer.poll(200)) 
    {
        struct { int temp, pres, xaxis, yaxis; } payload;

        sensor.pressure(payload.temp, payload.pres);
        sensor.heading(payload.xaxis, payload.yaxis);
        
        while (!rf12_canSend())
            rf12_recvDone();
        rf12_sendStart(0, &payload, sizeof payload, 2);

        Serial.print("HDPM ");
        Serial.print(payload.temp);
        Serial.print(' ');
        Serial.print(payload.pres);
        Serial.print(' ');
        Serial.print(payload.xaxis);
        Serial.print(' ');
        Serial.print(payload.yaxis);

        cal_compass();
        //Serial.print("Angle");
        Serial.print(' ');
        Serial.println(angle);
    }
}
