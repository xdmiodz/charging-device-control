#include <LiquidCrystal.h>
#define FIELD_SENSOR A0    // select the input pin for the potentiometer

#define PfeiferSerial Serial3
#define PfeiferSerialControlPin 38
#define PfeiferSerialBaudrate 9600

#define ChargerSerial Serial2
#define ChargerSerialControlPin 36
#define ChargerSerialBaudrate 9600


/*Global defines*/
#define TRUE     1
#define FALSE    0

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

//timer for Charger Device
struct timer ChargerTimer;

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
#define BUTTON_MODE_SET    2
  byte buttonMode;

#define BUTTON_STATE_ON     HIGH
#define BUTTON_STATE_OFF    LOW
  byte buttonState;
  
  byte buttonPin;
#define BUTTON_MODE_CHANGE_TIMEOUT 2000
  unsigned long pushDown; //time of recent push down
  unsigned long pushUp; //time of recent push up  
  
  byte previousState;
  
} button;

struct _rs485
{
#define MAX_LEN_MSG (128)
  byte sent;
  char sendbuf[MAX_LEN_MSG];
  byte recvd;
  char recvbuf[MAX_LEN_MSG];
  unsigned int baudrate;

  byte cmdlen;
  
  /*function to check if received data is full response*/
  boolean (*checkRecvd)(byte * data, byte len);

  /*set if info is already up to date*/
  boolean updateInfo;
  
  /**/
  boolean transmitEnabled;
  
#define RS485_MODE_TX HIGH
#define RS485_MODE_RX LOW  
  boolean mode;
  byte controlPin;
  
  HardwareSerial* serial;
  LiquidCrystal *lcd;
};

struct _chargerControl
{
  struct _rs485 * rs485;
  
  #define CHARGER_GET_VOLTAGE  0
  #define CHARGER_SET_VOLTAGE  1
  byte status;
  
  /*current voltage on the device*/
  unsigned int voltage;
} chargerControl;

struct _mfieldcontrol
{
  struct _chargerControl * charger;
  struct _button * button;
  struct _mfieldvals * mvals;
} mfieldcontrol;

struct _lcdControl
{
  LiquidCrystal * lcd;
  struct _button * button;
  
} lcdControl;

/*RS485 device for Pfeifer RVC300*/
struct _rs485 rs485Pfeifer; 

/*RS485 device for Charger Device*/
struct _rs485 rs485Charger;

// set contol pins for lcd
LiquidCrystal lcd(32, 22, 24, 26, 28, 30);

void setup()
{
  // start lcd
  lcd.begin(16, 4);
    lcd.print("P: "); 
  lcd.setCursor(0, 1);

  lcd.print("B: "); 
  
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

  //init pfeifer serial control
  rs485Pfeifer.sent = 0;
  rs485Pfeifer.recvd = 0;
  rs485Pfeifer.cmdlen = 0;
  rs485Pfeifer.baudrate = PfeiferSerialBaudrate;
  rs485Pfeifer.controlPin = PfeiferSerialControlPin;
  rs485Pfeifer.serial = &PfeiferSerial;
  rs485Pfeifer.lcd = &lcd;
  rs485Pfeifer.transmitEnabled = 0;
  rs485Pfeifer.checkRecvd = pfeiferCheckString;
  PfeiferSerial.begin(PfeiferSerialBaudrate);
  setSerialCmd ("PRI?\r\n", &rs485Pfeifer);

  pinMode(rs485Pfeifer.controlPin, OUTPUT); 
  setSerialMode (&rs485Pfeifer, RS485_MODE_RX);

  //init timer for Pfeifer Device
  PfeiferTimer.startTime = millis();
  PfeiferTimer.periodTime = 5000;
  PfeiferTimer.callback = updatePfeiferInfo;
  PfeiferTimer.nocallback = checkPfeiferInfo;


  //init pfeifer serial control
  rs485Charger.sent = 0;
  rs485Charger.recvd = 0;
  rs485Charger.cmdlen = 0;
  rs485Charger.baudrate = ChargerSerialBaudrate;
  rs485Charger.controlPin = ChargerSerialControlPin;
  rs485Charger.serial = &ChargerSerial;
  rs485Charger.lcd = &lcd;
  rs485Charger.transmitEnabled = 0;
  rs485Charger.checkRecvd = chargerCheckReply;
  ChargerSerial.begin(ChargerSerialBaudrate);
  setSerialCmdStr ("PRI?\r\n", &rs485Charger);

  pinMode(rs485Charger.controlPin, OUTPUT); 
  setSerialMode (&rs485Charger, RS485_MODE_RX);

  //init timer for Pfeifer Device
  ChargerTimer.startTime = millis();
  ChargerTimer.periodTime = 1000;
  ChargerTimer.callback = updateChargerInfo;
  ChargerTimer.nocallback = checkChargerInfo;

  //init charger control
  chargerControl.lcd = &lcd;
  chargercontrol.rs485 = &rs485Charger;
  chargerControl.mcontrol = &mfieldvals;
  chargerControl.status = CHARGER_READ_VOLTAGE;
}


byte checksum(const byte * cmd, byte len)
{
  byte chk = 0;
  byte i;
  for (i = 0; i < len; ++i)
    {
      chk ^= cmd[i];
    }
  return ~chk;
}

void setSerialCmdBin (struct _rs485 * rs485, const byte * cmd,
		      byte len)
{
  rs485->cmdlen = len;
  memcpy((void*)rs485->sendbuf, (void*)cmd, len);
}

/*set cmd for setting info to Charger Device*/
void setChargerVoltage(struct _rs485 * charger, unsigned int voltage)
{
  if ((voltage < MIN_VOLTAGE) || (voltage > MAX_VOLTAGE))
    return;
  
  float fVolt = float(voltage)/100.;
  byte * pbVolt = &fVolt;
  byte cmd[7] = {0};
  
  cmd[0] = 1;
  cmd[1] = byte('F');
  cmd[2] = pbVolt[0];
  cmd[3] = pbVolt[1];
  cmd[4] = pbVolt[2];
  cmd[5] = pbVolt[3];
  cmd[6] = checksum(cmd, 6);
  setSerialCmdBin (charger, cmd, 7);
  return;
}

/*set cmd for updating info from Charger Device*/
void getChargerVoltage(struct _rs485 * charger)
{
  byte cmd[4] = {0};
  
  cmd[0] = 1;
  cmd[1] = byte('B');
  cmd[2] = 0;
  cmd[3] = checksum(cmd, 3);
  setSerialCmdBin (charger, cmd, 4);
  return;
}


void setSerialCmdStr (const char* cmd, struct _rs485 * rs485)
{
  rs485->cmdlen = strlen(cmd);
  strncpy ((char*)rs485->sendbuf, cmd,  rs485->cmdlen);
}



void setSerialMode (struct _rs485* rs485, boolean devMode)
{
  digitalWrite(rs485->controlPin, devMode);
  rs485->mode = devMode;
}

byte calcDelay(byte len, unsigned int speed)
{
  return ceil(8*len/(speed/1000.));
}


void recvSerialData(struct _rs485 * rs485)
{
  if (rs485->updateInfo)
    {
      /*Received info is not updated, so I'm not going to receive more */
      rs485->serial->flush();
      return;
    }

  if (rs485->serial->available())
    {
      rs485->recvbuf[rs485->recvd] = rs485->serial->read();
      rs485->recvd++;
      
      /*Wow! We have a reply!*/
      if (rs485->checkRecvd && 
	  rs485->checkRecvd((byte*)rs485->recvbuf, rs485->recvd))
	{
	  rs485->updateInfo = 1;
	}


    }
  if (rs485->recvd == MAX_LEN_MSG)
    rs485->recvd = 0;
}

void sendSerialData(struct _rs485 * rs485)
{
  if (!rs485->transmitEnabled)
    return;
  if (rs485->updateInfo)
    return;

  if (rs485->cmdlen == 0)
    return;

  if (rs485->sent < rs485->cmdlen)
    {
      setSerialMode (rs485, RS485_MODE_TX);
      rs485->serial->write(rs485->sendbuf[rs485->sent]);
      delay(calcDelay(1, rs485->baudrate));
      rs485->sent++;
      setSerialMode (rs485, RS485_MODE_RX);
    }
  else
    {
      rs485->sent = 0;
      rs485->transmitEnabled = 0;
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
  if (BUTTON_MODE_COARSE == mode)
    {
      mbutton->button
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
       mbutton->buttonMode = BUTTON_MODE_SET;
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

boolean pfeiferCheckString (byte * data, byte len)
{
  if ((len > 2) && (data[len-2] == byte('\r')) 
      && (data[len-1] == byte('\n')))
    return 1;
  return 0;
}

boolean chargerCheckReply (const byte * data, byte len)
{
  if (data[0] != 1)
    return 0;
  if (len > 1)
    {
      byte msg_len = data[1] + 2;
      if (len == msg_len)
	if (data[len - 1] == checksum(data, len))
	  return 1;
	else
	  return 0;
      else
	return 0;
    }
  else
    return 0;
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

void checkTimer (struct timer* Timer, void * arg)
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

void checkPfeiferInfo(void *arg)
{
  struct _rs485 * rs485 = (struct _rs485 *)arg;
  sendSerialData (rs485);
  recvSerialData (rs485);
}



void updateChargerInfo(void * arg)
{
  struct _rs485 * chargerControl = (struct _chargerControl *)arg;
  struct _rs485 * rs485 = chargerControl->rs485;

  rs485->transmitEnabled = 1;
  
  if (rs485->updateInfo == 0)
    return;

  rs485->recvd = 0;
  rs485->sent = 0;

  if (CHARGER_GET_VOLTAGE == chargerControl->status)
    {
      if (byte('B') == rs485->recvbuf[2])
	{
	  float * recvvoltage = &(rs485->recvbuf[4]);
	  chargerControl->voltage = long((*recvvoltage)*100.);
	  return;
	} 
      else
	return;
    }
  if (CHARGER_SET_VOLTAGE == chargerControl->status)
    {
      setChargerVoltage(rs485, chargerControl->voltage);
      chargerControl->status = CHARGER_GET_VOLTAGE;
      return;
    }
}


void checkChargerInfo(void * arg)
{
  struct _rs485 * chargerControl = (struct _chargerControl *)arg;
  struct _rs485 * rs485 = chargerControl->rs485;
  sendSerialData (rs485);
  recvSerialData (rs485);
}


void updatePfeiferInfo (void *arg)
{
  struct _rs485 * rs485 = (struct _rs485 *)arg;
  rs485->transmitEnabled = 1;
  if (rs485->updateInfo && rs485->recvd > 7)
    {
      byte i = 0;
      rs485->updateInfo = 0;
      printEmptyLine (rs485->lcd, 3, 0, 14);
      rs485->lcd->setCursor(3,0);
      for (i = 5; i < rs485->recvd - 2; ++i)
	rs485->lcd->write(rs485->recvbuf[i]);
      rs485->recvd = 0;
      rs485->sent = 0;
    }
}

 
void printEmptyLine(LiquidCrystal* lcd, byte startPos, byte startLine, 
		    byte maxLen)
{
   byte i;
   lcd->setCursor(startPos, startLine);
   for (i = 0; i < maxLen; ++i)
     lcd->print(" ");
}


void loop()
{ 
  checkTimer (&chargerTimer, &chargerControl);
  checkTimer (&PfeiferTimer, &rs485Pfeifer);
  checkTimer (&MFieldTimer, &mfieldvals);
  checkTimer (&ButtonTimer, &button);
}
