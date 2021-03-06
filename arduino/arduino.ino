#include <LiquidCrystal.h>
#define FIELD_SENSOR A0    // select the input pin for the potentiometer

#define PfeiferSerial Serial3
#define PfeiferSerialControlPin 38
#define PfeiferSerialBaudrate 9600

#define ChargerSerial Serial1
#define ChargerSerialControlPin 34
#define ChargerSerialBaudrate 9600

#define PfeiferUpdateTimeout 5000
#define ChargeUpdateTimeout  1200
#define VaristorUpdatTime    100

/*Global defines*/
#define TRUE     1
#define FALSE    0

struct vband
{
    byte id;
    unsigned int minv;
    unsigned int maxv;
};

#define MIN_VBAND_ID 0
#define MAX_VBAND_ID 8

//voltage bands available for setting
struct vband vbands[9] = {{0, 100, 200},
                         {1, 200, 300},
                         {2, 300, 400},
                         {3, 400, 500},
                         {4, 500, 600},
                         {5, 600, 700},
                         {6, 700, 800},
                         {7, 800, 900},
                         {8, 900, 1000}};

struct timer 
{
  unsigned long startTime;
  unsigned long periodTime;
  void (*callback) (void*);
  void (*idle_callback) (void*); //This callback is called if the timer is still running
};

//timer structure for Pfeifer
struct timer pfeiferTimer;

//timer structure for AnalogInput
struct timer MFieldTimer;

//timer structure for Button
struct timer LCDTimer;

//timer for Charger Device
struct timer chargerTimer;

struct _mfieldvals
{
  long currentField;
  long accumulatedRaw;
  long accumulatedRawCurrent;
  long setField;
  
  byte vband;
  
  byte sensorPin;
  unsigned long counts;
  
#define MAX_VOLTAGE 1000
#define MIN_VOLTAGE 100
  long maxval;
  long minval;
} mfieldvals;

#define BUTTON_PIN 42
struct _button
{
#define BUTTON_MODE_VBAND  0
#define BUTTON_MODE_SET    1
  byte buttonMode;
  byte buttonPreviousMode;

#define BUTTON_STATE_ON     HIGH
#define BUTTON_STATE_OFF    LOW
  byte buttonState;

  
  byte buttonPin;
#define BUTTON_MODE_LONG_CLICK_TIMEOUT 1000
#define BUTTON_MODE_FAST_CLICK_TIMEOUT 50
  unsigned long pushDown; //time of recent push down
  unsigned long pushUp; //time of recent push up  
  
  byte previousState;

#define BUTTON_BLOCKED   FALSE
#define BUTTON_ENABLED   TRUE
  boolean buttonEnabled;

  boolean buttonStateUpdate;
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
  boolean (*checkRecvd)(const byte * data, byte len);

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
  long voltage;

  /*set voltage on the device*/
  long setVoltage;

  boolean setFieldUpdate;
} chargerControl;

struct _mfieldcontrol
{
  struct _chargerControl * charger;
  struct _button * button;
  struct _mfieldvals * mvals;
} mfieldcontrol;

struct _pfeiferControl
{
  struct _rs485 * rs485;
  char pressure[MAX_LEN_MSG];
  boolean warning;
} pfeiferControl;

struct _lcdControl
{
  LiquidCrystal * lcd;
  struct _button * button;
  struct _mfieldvals * mvals;
  struct _pfeiferControl * pfeiferControl;

  byte buttonMode;
  long setField;
  long varistorField;
  
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
  mfieldvals.accumulatedRaw = 0;
  mfieldvals.sensorPin = FIELD_SENSOR;
  mfieldvals.counts = 0;
  mfieldvals.vband = 0;
  mfieldvals.maxval = vbands[0].maxv;
  mfieldvals.minval = vbands[0].minv;
  mfieldvals.setField = mfieldvals.minval;
  mfieldvals.currentField = 0;

  //init timer for mfield varistor
  MFieldTimer.startTime = millis();
  MFieldTimer.periodTime = VaristorUpdatTime;
  MFieldTimer.callback = updateMfieldValue;
  MFieldTimer.idle_callback = accumulateAnalogRead;
  

  //init button
  button.buttonMode = BUTTON_MODE_VBAND;
  button.buttonState = BUTTON_STATE_OFF;
  button.buttonPin = BUTTON_PIN;
  button.pushDown = millis();
  button.pushUp = button.pushDown;
  button.previousState = BUTTON_STATE_OFF;
  button.buttonEnabled = BUTTON_ENABLED;
  button.buttonStateUpdate = TRUE;

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
  setSerialCmdStr ("PRI?\r\n", &rs485Pfeifer);

  pinMode(rs485Pfeifer.controlPin, OUTPUT); 
  setSerialMode (&rs485Pfeifer, RS485_MODE_RX);

  /*init pfeifer control*/
  pfeiferControl.rs485 = &rs485Pfeifer;
  pfeiferControl.pressure[0] = '\0';
  pfeiferControl.warning = TRUE;

  //init timer for Pfeifer Device
  pfeiferTimer.startTime = millis();
  pfeiferTimer.periodTime = PfeiferUpdateTimeout;
  pfeiferTimer.callback = updatePfeiferInfo;
  pfeiferTimer.idle_callback = checkPfeiferInfo;


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
  setChargerVoltage(&rs485Charger, MIN_VOLTAGE);

  pinMode(rs485Charger.controlPin, OUTPUT); 
  setSerialMode (&rs485Charger, RS485_MODE_RX);

  //init timer for Charger Device
  chargerTimer.startTime = millis();
  chargerTimer.periodTime = ChargeUpdateTimeout;
  chargerTimer.callback = updateChargerInfo;
  chargerTimer.idle_callback = checkChargerInfo;

  //init charger control
  chargerControl.rs485 = &rs485Charger;
  chargerControl.status = CHARGER_SET_VOLTAGE;
  chargerControl.setVoltage = MIN_VOLTAGE;
  chargerControl.setFieldUpdate = TRUE;

  //init filed control
  mfieldcontrol.charger = &chargerControl;
  mfieldcontrol.button = &button;
  mfieldcontrol.mvals = &mfieldvals;

  /*init lcd control*/
  lcdControl.lcd = &lcd;
  lcdControl.button = &button;
  lcdControl.mvals = &mfieldvals;
  lcdControl.buttonMode = button.buttonMode;
  lcdControl.setField = mfieldvals.setField;
  lcdControl.varistorField = mfieldvals.currentField;
  lcdControl.pfeiferControl = &pfeiferControl;
  
  /*lcd timer*/
  LCDTimer.startTime = millis();
  LCDTimer.periodTime = 1000;
  LCDTimer.callback = NULL;
  LCDTimer.idle_callback = updateLCD;  
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

/*set cmd for setting info to Charger Device*/
void setChargerVoltage(struct _rs485 * charger, long voltage)
{
  if ((voltage < MIN_VOLTAGE) || (voltage > MAX_VOLTAGE))
    return;
  
  float fVolt = float(voltage)/1000.;
  byte * pbVolt = (byte *)&fVolt;
  byte cmd[8] = {0};
  
  cmd[0] = 1;
  cmd[1] = 6;
  cmd[2] = byte('F');
  cmd[3] = pbVolt[0];
  cmd[4] = pbVolt[1];
  cmd[5] = pbVolt[2];
  cmd[6] = pbVolt[3];
  cmd[7] = checksum(cmd, 7);
  setSerialCmdBin (charger, cmd, 8);
  return;
}

/*set cmd for updating info from Charger Device*/
void getChargerVoltage(struct _rs485 * charger)
{
  byte cmd[5] = {0};
  
  cmd[0] = 1;
  cmd[1] = 3;
  cmd[2] = byte('B');
  cmd[3] = 0;
  cmd[4] = checksum(cmd, 4);
  setSerialCmdBin (charger, cmd, 5);
  return;
}

void setSerialCmdBin (struct _rs485 * rs485, const byte * cmd,
                      byte len)
{
  rs485->cmdlen = len;
  byte i;
  memcpy((void*)rs485->sendbuf, (void*)cmd, len);
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
      
    }
  else
    {
      setSerialMode (rs485, RS485_MODE_RX);
      rs485->sent = 0;
      rs485->transmitEnabled = 0;
    }
}

void printButtonMode(LiquidCrystal * lcd,
                     struct _button * mbutton,
                     struct _mfieldvals * mvals)
{
  
  switch (mbutton->buttonMode)
    {
    case BUTTON_MODE_VBAND:
      lcd->setCursor (13,1);
      lcd->print ("[");
      lcd->print (mvals->vband + 1);
      lcd->print ("]");
      break;
    case BUTTON_MODE_SET:
      lcd->setCursor (13,1);
      lcd->print ("  ");
      lcd->print (char(0x9B)); //battery symbol
      break;
    default:
      break;
    }
}

void printSetField(LiquidCrystal * lcd, long val)
{
  lcd->setCursor (7, 1);
  lcd->print("      ");
  lcd->setCursor (7, 1);
  lcd->print ("/ ");
  lcd->print (val);
}


void printCurrentField(LiquidCrystal * lcd, long val)
{
  lcd->setCursor (3, 1);
  lcd->print("    ");
  lcd->setCursor (3, 1);
  lcd->print (val);
}

void printPressure(LiquidCrystal * lcd, char * val,
                  boolean warning)
{
  //printEmptyLine (lcd, 2, 0, 14);
  lcd->setCursor (2,0);
  if (warning)
  {
    lcd->print("!");  
  }  
  else
    {
      lcd->print(" ");
    }

  lcd->print (val);
}


void updateLCD(void * arg)
{
  struct _lcdControl * lcdControl = (struct _lcdControl *)arg;
  struct _button * mbutton = lcdControl->button;
  struct _mfieldvals * mvals = lcdControl->mvals;
  struct _pfeiferControl * pfeifer = lcdControl->pfeiferControl;
  LiquidCrystal * lcd = lcdControl->lcd;
  
  if (mbutton->buttonStateUpdate)
    {
      printButtonMode (lcd, mbutton, mvals);
      mbutton->buttonStateUpdate = FALSE;
    }  
  if (mvals->setField != lcdControl->setField)
    {
      lcdControl->setField = mvals->setField; 
      printSetField(lcd, lcdControl->setField);
    }

  if (mvals->currentField != lcdControl->varistorField)
    {
      lcdControl->varistorField = mvals->currentField;
      printCurrentField(lcd, lcdControl->varistorField);
    }
  printPressure(lcd, pfeifer->pressure, pfeifer->warning);
}

void changeVbandMode (struct _button * mbutton, 
                      struct _mfieldvals * mfv)
{
  mbutton->buttonMode = BUTTON_MODE_VBAND;   
  mbutton->buttonPreviousMode = BUTTON_MODE_VBAND;
  
  if (mfv->vband < MAX_VBAND_ID)
    mfv->vband++;
  else
    mfv->vband = 0;
  
  mfv->minval = vbands[mfv->vband].minv;
  mfv->maxval = vbands[mfv->vband].maxv;
  mbutton->buttonStateUpdate = TRUE;
}

void changeButtonMode (struct _button * mbutton,
                       struct _mfieldvals * mfv)
{
  if (BUTTON_MODE_VBAND == mbutton->buttonMode)
    changeVbandMode (mbutton, mfv);
  
  return;
}

void buttonModeStateTransition (struct _button * mbutton,
                                struct _mfieldvals * mfv)
{
  byte state = digitalRead(mbutton->buttonPin);

  if  (mbutton->buttonEnabled == BUTTON_BLOCKED)
    return;
      
  if (BUTTON_STATE_ON == state && 
      BUTTON_STATE_OFF == mbutton->previousState)
    {
      mbutton->previousState = BUTTON_STATE_ON;
      mbutton->pushDown = millis();
      return;
    }
  
  if (BUTTON_STATE_OFF == state && 
      BUTTON_STATE_ON == mbutton->previousState)
    {
      mbutton->previousState = BUTTON_STATE_OFF;
      mbutton->pushUp = millis();

      if (mbutton->pushUp < mbutton->pushDown)
        return;

      unsigned long presstime = mbutton->pushUp - mbutton->pushDown;
      if (presstime < BUTTON_MODE_FAST_CLICK_TIMEOUT)
        return;
       
      if ((presstime < BUTTON_MODE_LONG_CLICK_TIMEOUT) && 
          (presstime > BUTTON_MODE_FAST_CLICK_TIMEOUT))
        {
          //fast click
          changeButtonMode(mbutton, mfv);
          return;
        }
      if (presstime > BUTTON_MODE_LONG_CLICK_TIMEOUT)
        {
          //long click 
          mbutton->buttonStateUpdate = TRUE;
          mbutton->buttonPreviousMode = mbutton->buttonMode;
          mbutton->buttonEnabled = BUTTON_BLOCKED;
          mbutton->buttonMode = BUTTON_MODE_SET;
        }
      return;
    }
  if (BUTTON_STATE_ON == state &&
      BUTTON_STATE_ON == mbutton->previousState)
    {
      unsigned long currentTime = millis();
      if (currentTime < mbutton->pushDown)
        return;
     
      unsigned long presstime = currentTime - mbutton->pushDown;     
    
      if (presstime > BUTTON_MODE_LONG_CLICK_TIMEOUT)
        {
          //long click
          mbutton->buttonStateUpdate = TRUE;
          mbutton->buttonPreviousMode = mbutton->buttonMode;
          mbutton->buttonMode = BUTTON_MODE_SET;
          mbutton->buttonEnabled = BUTTON_BLOCKED;
          return;
        }
    }
  
}

boolean pfeiferCheckString (const byte * data, byte len)
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
    {
     if (data[len - 1] == checksum(data, len - 1))
       return 1;
     else
       return 0;
   }
   else
     return 0;
 }
 else
  return 0;
}

void updateMfieldValue (void* arg)
{
  struct _mfieldcontrol * mcontrol = (struct _mfieldcontrol *)arg;
  struct _mfieldvals * mvals = mcontrol->mvals;
  struct _button * mbutton = mcontrol->button;
  struct _chargerControl * charger =  mcontrol->charger;

  long k = (mvals->maxval - mvals->minval);
  long b = mvals->minval;

  mvals->setField = charger->voltage;
  mvals->accumulatedRawCurrent = ceil(mvals->accumulatedRaw/mvals->counts);
  float x = mvals->accumulatedRawCurrent/1024.;
  long realField = ceil(k*x + b);
  mvals->currentField = realField;

  mvals->accumulatedRaw = 0;
  mvals->counts = 0;
  
  if (BUTTON_MODE_SET == mbutton->buttonMode)
    {
      charger->setVoltage = mvals->currentField;
      charger->setFieldUpdate = TRUE;
      mvals->setField =  mvals->currentField;
      mbutton->buttonMode = mbutton->buttonPreviousMode;
      mbutton->buttonEnabled = BUTTON_ENABLED;
      mbutton->buttonStateUpdate = TRUE;
    }
}

void accumulateAnalogRead(void* arg)
{
   struct _mfieldcontrol * mcontrol = (struct _mfieldcontrol *)arg;
   struct _mfieldvals * mvals = mcontrol->mvals;
   struct _button * mbutton = mcontrol->button;
   
   /*update state of varistor*/
   mvals->accumulatedRaw += analogRead(mvals->sensorPin);
   mvals->counts++;
   
   /*update button state*/
   buttonModeStateTransition (mbutton, mvals); 
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
  if (Timer->idle_callback)
    Timer->idle_callback (arg);
}

void checkPfeiferInfo(void *arg)
{
  struct _pfeiferControl * pfeifer = (struct _pfeiferControl *)arg;
  struct _rs485 * rs485 = pfeifer->rs485;

  sendSerialData (rs485);
  recvSerialData (rs485);
}



void updateChargerInfo(void * arg)
{
  struct _chargerControl * chargerControl = (struct _chargerControl *)arg;
  struct _rs485 * rs485 = chargerControl->rs485;

  rs485->transmitEnabled = 1;
  
  if (rs485->updateInfo == 0)
    return;

  rs485->recvd = 0;
  rs485->sent = 0;

  if (CHARGER_GET_VOLTAGE == chargerControl->status)
  {
    rs485->updateInfo = 0;
    if (byte('B') == rs485->recvbuf[2])
    {
     float * recvvoltage = (float*)&(rs485->recvbuf[4]);
     long newvoltage = round(((*recvvoltage)*1000.));
     chargerControl->voltage = newvoltage;

      if (chargerControl->setFieldUpdate)
      {
        setChargerVoltage(rs485, chargerControl->setVoltage);
        chargerControl->setFieldUpdate = FALSE;
        chargerControl->status =  CHARGER_SET_VOLTAGE; 
      }

      return;
    } 
    else
     return;
  }

  if (CHARGER_SET_VOLTAGE == chargerControl->status)
  {
    rs485->updateInfo = 0;
    getChargerVoltage(rs485);
    chargerControl->status = CHARGER_GET_VOLTAGE;
  }
  return;
}


void checkChargerInfo(void * arg)
{
  struct _chargerControl * chargerControl = (struct _chargerControl *)arg;
  struct _rs485 * rs485 = chargerControl->rs485;
  sendSerialData (rs485);
  recvSerialData (rs485);
}


void updatePfeiferInfo (void *arg)
{
  struct _pfeiferControl * pfeifer = (struct _pfeiferControl *)arg;
  struct _rs485 * rs485 = pfeifer->rs485;
  rs485->transmitEnabled = 1;
  if (rs485->updateInfo && rs485->recvd > 7)
    {
      byte i = 0;
      rs485->updateInfo = 0;
      pfeifer->warning = FALSE;
      strncpy(pfeifer->pressure, &rs485->recvbuf[5], rs485->recvd - 7);
      pfeifer->pressure[rs485->recvd - 7] = byte('\0');
      rs485->recvd = 0;
      rs485->sent = 0;
    }
    else
      pfeifer->warning = TRUE;

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
  checkTimer (&pfeiferTimer, &pfeiferControl);
  checkTimer (&MFieldTimer, &mfieldcontrol);
  checkTimer (&LCDTimer, &lcdControl);
}
