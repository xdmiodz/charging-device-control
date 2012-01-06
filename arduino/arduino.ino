#define HOST_SERIAL Serial
#define DEV_SERIAL Serial1
#define TEST_SERIAL Serial2
#define SERIAL_SPEED 9600 
#define MSG_MAX_SIZE 128

#define RESPONSE_TIMEOUT  10 //in ms
#define MAX_MSG_LEN (int)(9.6*RESPONSE_TIMEOUT/8)

#define PIN_MODE 22
#define PIN_MODE_TEST 23
#define DEV_MODE_TX HIGH
#define DEV_MODE_RX LOW

#define DEBUG

byte READY = 0;

byte hostmsg[MAX_MSG_LEN] = {0};
byte devmsg[MAX_MSG_LEN] = {0};
byte testmsg[MAX_MSG_LEN] = {0};
byte msglen = 0;


void setDevMode(byte pinMode, byte devMode)
{
  digitalWrite(pinMode, devMode);
}

void setup()
{
  pinMode(PIN_MODE, OUTPUT);
  pinMode(PIN_MODE_TEST, OUTPUT);
  setDevMode(PIN_MODE, DEV_MODE_RX);
  setDevMode(PIN_MODE_TEST, DEV_MODE_RX);
  HOST_SERIAL.begin(SERIAL_SPEED);
  DEV_SERIAL.begin(SERIAL_SPEED);
  TEST_SERIAL.begin(SERIAL_SPEED);
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

void sendMsgBin(HardwareSerial serial, byte* buffer, byte len, byte rxPin)
{
    byte i;
    if (rxPin)
      setDevMode(rxPin, DEV_MODE_TX);
      
    for (i = 0; i < len; ++i)
    {
        serial.write(buffer[i]);
        delay(calcDelay(1, SERIAL_SPEED));
    }
    if (rxPin)  
      setDevMode(rxPin, DEV_MODE_RX);
}

void sendMsgHexToHost(HardwareSerial serial, byte* buffer, byte len)
{
    byte i;
    for (i = 0; i < len; ++i)
    {
      serial.print(buffer[i], HEX);
      delay(calcDelay(2, SERIAL_SPEED));
      serial.print(" ");
      delay(calcDelay(2, SERIAL_SPEED));
    }
   serial.println("");   
   delay(calcDelay(2, SERIAL_SPEED));
}

byte calcDelay(byte len, int speed)
{
   return ceil(8*len/(speed/1000.));
}

void loop()
{
  if (!READY)
    {
      delay(10);
      HOST_SERIAL.flush();
      DEV_SERIAL.flush();
      #ifdef DEBUG
      TEST_SERIAL.flush();
      #endif
      HOST_SERIAL.println("READY!");
      READY = 1;
      delay(200);
    }
   msglen = readMsg(HOST_SERIAL, hostmsg);
   if (msglen)
   {
      sendMsgBin(DEV_SERIAL, hostmsg, msglen, PIN_MODE);
      #ifdef DEBUG
        msglen = readMsg(TEST_SERIAL, hostmsg);
        if (msglen)
          sendMsgHexToHost(HOST_SERIAL, hostmsg, msglen);
      #endif
   }
   delay(10);    
}
