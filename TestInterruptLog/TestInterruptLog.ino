
 // interrupt driven pulse counts 
#define INTERRUPT_IR        1    // ATmega 168 and 328 - interrupt 0 = pin 2, 1 = pin 3

volatile unsigned long    g_toggles = 0;


// routine called when external interrupt is triggered
void interruptHandlerIR() 
{
  g_toggles = g_toggles + 1;
}





void setup()
{
  pinMode(13, OUTPUT);  
  Serial.begin(57600);
 
  //set up the interrupt handler that will count the IR LED watt pulse counts
  pinMode(2, INPUT);     
  digitalWrite(2, HIGH);   
  attachInterrupt(1, interruptHandlerIR, CHANGE);  
}

void loop() 
{
  // read the value from the sensor:
  int toggles = 0;

  delay(100);
  uint8_t oldSREG = SREG;          // save interrupt register
  cli();                           // prevent interrupts while accessing the count   

  toggles = g_toggles;
  g_toggles = 0;  
  SREG = oldSREG;                  // restore interrupts

  
  Serial.println( toggles );
 
  // turn the ledPin on  for a period relative to number of toggles
  digitalWrite(13, HIGH);  
  //delay(toggles);     
  digitalWrite(13, LOW);   
}
