#include <stdlib.h>

#define HOST_SERIAL Serial
#define DEV_SERIAL Serial1
#define TEST_SERIAL Serial2
#define SERIAL_SPEED 9600 
#define MSG_MAX_SIZE 128

#define DEV_RESPONSE_TIMEOUT  10 //in ms

#define PIN_MODE 22
#define PIN_MODE_TEST 23
#define DEV_MODE_TX HIGH
#define DEV_MODE_RX LOW

byte READY = 0;

char hostmsg = 0;
char devmsg = 0;
char reply = 0;

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


byte calcDelay(byte len, int speed)
{
   return ceil(8*len/(speed/1000.));
}

void devSendCmd(byte cmd)
{
  setDevMode(PIN_MODE, DEV_MODE_TX);
  DEV_SERIAL.write(cmd);
  delay(calcDelay(1, SERIAL_SPEED));
  setDevMode(PIN_MODE, DEV_MODE_RX);
}

void hostSendMsg(byte cmd)
{ 
  HOST_SERIAL.print(cmd, HEX);
  delay(calcDelay(1, SERIAL_SPEED));
  HOST_SERIAL.print(" ");
  delay(calcDelay(1, SERIAL_SPEED));
  HOST_SERIAL.println("");
}

void loop()
{
  if (!READY)
    {
      HOST_SERIAL.println("READY!");
      READY = 1;
    }
  while (HOST_SERIAL.available())
    {
      hostmsg = HOST_SERIAL.read();
      devSendCmd(hostmsg);
    }
  delay(3);
  while(DEV_SERIAL.available())
    {
      devmsg = DEV_SERIAL.read();
      hostSendMsg(devmsg);
    }
  delay(3);
}
