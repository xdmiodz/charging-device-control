#include <LiquidCrystal.h>
#define FIELD_SENSOR A0    // select the input pin for the potentiometer

int referenceVal = 500;

#define MAX_MSG_LEN (128)

byte reply[MAX_MSG_LEN] = {0};
byte replylen = 0;

#define SERIAL_SPEED 9600 
#define RESPONSE_TIMEOUT  500 //in ms

#define DEV_MODE_TX HIGH
#define DEV_MODE_RX LOW
#define PfeiferSerial Serial2
#define PfeiferControlPin 36



struct timer 
{
  unsigned long startTime;
  unsigned long periodTime;
  void (*callback) (void*);
  void (*nocallback) (void*); //This callback is called if the timer is still running
};

//timer structure for Pfeifer
struct timer PfeiferTimer;

//timer structure for AnalogInput
struct timer MFieldTimer;

//timer structure for Button
struct timer ButtonTimer;

struct _mfieldvals
{
  long currentField;
  long accumulatedRaw;
  long accumulatedRawCurrent;
  long setField;
  float rawCorrection;
  byte sensorPin;
  LiquidCrystal *lcd;
  unsigned long counts;
  
  /*coeeficients to  parabola*/
  float a; 
  float b;
  float c;
  
#define MAX_VOLTAGE 500
#define MIN_VOLTAGE 100
  long maxval;
  long minval;
} mfieldvals;

#define BUTTON_PIN 42
struct _button
{
#define BUTTON_MODE_COARSE 0
#define BUTTON_MODE_FINE   1
  byte buttonMode;

#define BUTTON_STATE_ON     HIGH
#define BUTTON_STATE_OFF    LOW
  byte buttonState;
  
  byte buttonPin;
#define BUTTON_MODE_CHANGE_TIMEOUT 2000
  unsigned long pushDown; //time of recent push down
  unsigned long pushUp; //time of recent push up  
  
  byte previousState;
  struct _mfieldvals * mcontrol;
  
} button;

struct _rs485
{
#define MAX_LEN_MSG 128
  unsigned byte sent;
  char sendbuf[MAX_LEN_MSG];
  unsigned byte recvd;
  char recvbuf[MAX_LEN_MSG];
  unsigned int baudrate;

  unsigned byte cmdlen;

  /*number of printed bytes*/
  unsigned byte printd;
  
#define RS485_MODE_TX HIGH
#define RS485_MODE_RX LOW  
  boolean mode;
  byte controlPin;
  
  HardwareSerial* serial;
};

// set contol pins for lcd
LiquidCrystal lcd(32, 22, 24, 26, 28, 30);

void setup()
{
  // start lcd
  lcd.begin(16, 4);
    lcd.print("P: "); 
  lcd.setCursor(0, 1);

  lcd.print("B: "); 
  // initialize the pushbutton pin as an input:
  //pinMode(buttonPin, INPUT);   
  
  //Set Serial for pfeifer device
  pinMode(PfeiferControlPin, OUTPUT); 
  PfeiferSerial.begin(SERIAL_SPEED);
  setDevMode(PfeiferControlPin, DEV_MODE_RX);
  
  //init timer for Pfeifer Device
  PfeiferTimer.startTime = millis();
  PfeiferTimer.periodTime = 1000;
  PfeiferTimer.callback = updatePfeifer;
  PfeiferTimer.nocallback = NULL;
  
  //init values for magnetic field varistor
  mfieldvals.currentField = 0;
  mfieldvals.accumulatedRaw = 0;
  mfieldvals.lcd = &lcd;
  mfieldvals.sensorPin = FIELD_SENSOR;
  mfieldvals.counts = 0;
  mfieldvals.maxval = MAX_VOLTAGE;
  mfieldvals.minval = MIN_VOLTAGE;
  mfieldvals.rawCorrection = 0;
  mfieldvals.a = 0;
  mfieldvals.b = MAX_VOLTAGE - MIN_VOLTAGE;
  mfieldvals.c = MIN_VOLTAGE;
  
  //init timer for mfield varistor
  MFieldTimer.startTime = millis();
  MFieldTimer.periodTime = 100;
  MFieldTimer.callback = printMfieldResistorValue;
  MFieldTimer.nocallback = accumulateAnalogRead;
  

  //init button
  button.buttonMode = BUTTON_MODE_COARSE;
  button.buttonState = BUTTON_STATE_OFF;
  button.buttonPin = BUTTON_PIN;
  button.mcontrol = &mfieldvals;
  button.pushDown = millis();
  button.pushUp = button.pushDown;
  button.previousState = BUTTON_STATE_OFF;
  
  //init button timer
  ButtonTimer.nocallback = buttonModeStateTransition;
  ButtonTimer.callback = NULL;  
}

void setSerialMode (byte controlPin, boolean devMode)
{
  digitalWrite(controlPin, devMode);
}

byte calcDelay(byte len, unsigned int speed)
{
   return ceil(8*len/(speed/1000.));
}


void recvSerialData(void* arg)
{
   struct _rs485 * rs485 = (struct _rs485 *)arg;
   rs485->mode = RS485_MODE_TX;
   setSerialMode (rs485->controlPin, RS485_MODE_TX);
   if (rs485->serial.available())
     {
       rs485->recvbuf[rs485->recvd] = rs485->serail.read();
       rs485->recvd++;
     }
   if (rs485->recvd == MAX_LEN_MSG)
     rs485->recvd = 0;
}

void sendSerialData(void* arg)
{
  struct _rs485 * rs485 = (struct _rs485 *)arg;
  if (rs485->sent <= rs485->cmdlen)
    {
      rs485->serial.write(rs485->sendbuf[rs485->sent]);
      delay(calDelay(1, rs485->baudrate));
      rs485->sent++;
    }
  else
    {
      rs485->sent = 0;
    }
}

void printButtonMode(struct _button * mbutton)
{
    struct _mfieldvals * mfv = mbutton->mcontrol;
    mfv->lcd->setCursor (14,1);
    if (BUTTON_MODE_FINE == mbutton->buttonMode)
    {
      mfv->lcd->print (char(0x5E));
      mfv->lcd->print (char(0x5E));
      return;
    }
    if (BUTTON_MODE_COARSE == mbutton->buttonMode)
    {
      mfv->lcd->print (" ");
      mfv->lcd->print (char(0x5E));
      return;
    }
}

void setCoarseMode (struct _button * mbutton)
{
   struct _mfieldvals * mfv = mbutton->mcontrol;
   mbutton->buttonMode = BUTTON_MODE_COARSE;   

   mfv->rawCorrection = 0.;
   float x = mfv->accumulatedRawCurrent/1024.;
   if ((x > 0) && (x < 1))
   {
        mfv->a = (1/x/(x-1))*(mfv->currentField - MIN_VOLTAGE - x*(MAX_VOLTAGE - MIN_VOLTAGE));
   }
   else
       mfv->a = 0;
       
   mfv->b = (MAX_VOLTAGE - MIN_VOLTAGE) - mfv->a;
   mfv->c = MIN_VOLTAGE;
   
   printButtonMode(mbutton);
}

void setFineMode (struct _button * mbutton)
{
   struct _mfieldvals * mfv = mbutton->mcontrol;
   mbutton->buttonMode = BUTTON_MODE_FINE;
   long currentField = mfv->currentField;
   mfv->minval = (currentField / 100)*100;
   mfv->maxval =  mfv->minval + 100;
   long k2 = (mfv->maxval -  mfv->minval);
   long b2 = mfv->minval;
   
   mfv->rawCorrection = (k2*mfv->accumulatedRawCurrent/1024. + b2 - currentField)/k2;   
   
   mfv->a = 0;    
   mfv->b = k2;
   mfv->c = b2;
   
  printButtonMode(mbutton);
   
}

void changeButtonMode (struct _button * mbutton)
{
   if (BUTTON_MODE_COARSE == mbutton->buttonMode)
   {
     setFineMode (mbutton);
     return;
    }
    if (BUTTON_MODE_FINE == mbutton->buttonMode)
    {
      setCoarseMode (mbutton);
      return;
    }
}

void buttonModeStateTransition (void* arg)
{
  struct _button * mbutton = (struct _button *)arg;
  byte state = digitalRead(mbutton->buttonPin);
  struct _mfieldvals * mfv = mbutton->mcontrol;
  
  printButtonMode(mbutton);
  
  if (BUTTON_STATE_ON == state && BUTTON_STATE_OFF == mbutton->previousState)
  {
     mbutton->previousState = BUTTON_STATE_ON;
     mbutton->pushDown = millis();
     return;
  }
  
  if (BUTTON_STATE_OFF == state && BUTTON_STATE_ON == mbutton->previousState)
  {
     mbutton->previousState = BUTTON_STATE_OFF;
     mbutton->pushUp = millis();

     if (mbutton->pushUp < mbutton->pushDown)
       return;
       
     if ((mbutton->pushUp - mbutton->pushDown) < BUTTON_MODE_CHANGE_TIMEOUT)
     {
         //fast click
         changeButtonMode(mbutton);
         return;
     }
     else
     {
         //long click;
        return;
     }
     return;
  }
  if (BUTTON_STATE_ON == state && BUTTON_STATE_ON == mbutton->previousState)
  {
    unsigned long currentTime = millis();
    if (currentTime < mbutton->pushDown)
      return;
     
    if ((currentTime - mbutton->pushDown) < BUTTON_MODE_CHANGE_TIMEOUT)
    {
      changeButtonMode(mbutton);
      mbutton->pushDown = currentTime;
      return;
    }
  }
  
}

void printMfieldResistorValue (void* arg)
{
  struct _mfieldvals * mfv = (struct _mfieldvals *)arg;
  mfv->accumulatedRawCurrent = ceil(mfv->accumulatedRaw/mfv->counts);
  float x = mfv->accumulatedRawCurrent/1024. - mfv->rawCorrection;
  long currentField = floor(mfv->a*x*x + mfv->b*x + mfv->c);
  if (currentField != mfv->currentField)
  {
    printEmptyLine(mfv->lcd, 3, 1, 4);
    mfv->lcd->setCursor(3, 1);
    mfv->lcd->print(currentField); 
    mfv->currentField = currentField; 
  }
  mfv->accumulatedRaw = 0;
  mfv->counts = 0;
}

void accumulateAnalogRead(void* arg)
{
   struct _mfieldvals * mfv = (struct _mfieldvals *)arg;
   
   mfv->accumulatedRaw += analogRead(mfv->sensorPin);
   mfv->counts++;
}

byte readMsg(HardwareSerial serial, byte* buffer)
{  
    unsigned long startt = millis();
    byte len = 0;
    while ((len < MAX_MSG_LEN) && ((millis() - startt) < RESPONSE_TIMEOUT))
    {
        if (serial.available())
        {
            buffer[len] = serial.read();
            len++;
        }
    }
    return len;
}

void checkTimer (struct timer* Timer, void* arg)
{
  if (Timer->callback)
  {
    unsigned long currTime = millis();
    if (currTime - Timer->startTime > Timer->periodTime)
    {
      Timer->callback(arg);
      Timer->startTime = currTime;
      return;
    }
  }
  if (Timer->nocallback)
    Timer->nocallback (arg);
}

void updatePfeifer(void *arg)
{
  //PfeiferSerial.flush();
   //delay(5);
   setDevMode(PfeiferControlPin, DEV_MODE_TX);
   
   //PfeiferSerial.println("MOD?");
   PfeiferSerial.write((byte('M')));
   delay(calcDelay(1, SERIAL_SPEED));
   PfeiferSerial.write((byte('O')));
   delay(calcDelay(1, SERIAL_SPEED));
   PfeiferSerial.write((byte('D')));
   delay(calcDelay(1, SERIAL_SPEED));
   PfeiferSerial.write((byte('?')));
   delay(calcDelay(1, SERIAL_SPEED));
   PfeiferSerial.write((byte('\r')));
   delay(calcDelay(1, SERIAL_SPEED));
   PfeiferSerial.write((byte('\n')));
   delay(calcDelay(1, SERIAL_SPEED));
   //delay(50);
   setDevMode(PfeiferControlPin, DEV_MODE_RX); 
   if ((replylen = readMsg(PfeiferSerial, reply)) > 0)
   { 
     printEmptyLine(&lcd,3, 0, 13);
     lcd.setCursor(3, 0);
    // lcd.print("W");
     lcd.print((const char*)reply);
   }
}


 
void printEmptyLine(LiquidCrystal* lcd, byte startPos, byte startLine, byte maxLen)
{
   byte i;
   lcd->setCursor(startPos, startLine);
   for (i = 0; i < maxLen; ++i)
     lcd->print(" ");
}


int readVoltage()
{
  return 500;
}



void loop()
{ 
  //checkTimer (&PfeiferTimer, NULL);
  checkTimer (&MFieldTimer, &mfieldvals);
  checkTimer (&ButtonTimer, &button);
}
