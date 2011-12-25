#include<stdlib.h>

#define HOST_SERIAL Serial
#define DEV_SERIAL Serial1
#define SERIAL_SPEED 9600 
#define MSG_MAX_SIZE 128

#define DEV_RESPONSE_TIMEOUT  10 //in ms

#define SET_MSG  0
#define GET_MSG  1
#define ERR_MSG  3
#define MIN_VOLTAGE 100
#define MAX_VOLTAGE 700


#define DEV_MODE_TX_PIN 20
#define DEV_MODE_RX_PIN 21

#define ERR 1
#define OK  0

char hostmsg[MSG_MAX_SIZE] = {0};
char devmsg[MSG_MAX_SIZE] = {0};
char reply[MSG_MAX_SIZE] = {0};
float setvoltage = 100;

void setDevTx(byte pinTx, byte pinRx)
{
  digitalWrite(pinRx, LOW);
  digitalWrite(pinTx, HIGH)
}


void setDevRx(byte pinTx, byte pinRx)
{
  digitalWrite(pinTx, LOW);
  digitalWrite(pinRx, HIGH)
}

void setup()
{
  pinMode(DEV_MODE_TX_PIN, OUTPUT);
  pinMode(DEV_MODE_RX_PIN, OUTPUT);
  setDevRx(DEV_MODE_TX_PIN, DEV_MODE_RX_PIN);
  HOST_SERIAL.begin(SERIAL_SPEED);
  DEV_SERIAL.begin(SERIAL_SPEED);
}


byte read_host_serial()
{
  byte i = 0;
  if (HOST_SERIAL.available() > 0)
    {
      while (!HOST_SERIAL.available())
	{
	  hostmsg[i] = HOST_SERIAL.read();
	  i++;
	}
    }
  return i;
}

byte checkstart(char *buf, char *str, byte len)
{
  byte i = 0;
  for (i; i < len; ++i)
    {
      if (buf[i] != str[i])
	return ERR;
    }
  return OK;
}

void setVoltage()
{
  byte cmd[5];
  if ((setvolatge >= MIN_VOLTAGE) && (setvoltage <= MAX_VOLTAGE))
    {
      printf(reply, "Going to set voltage to: %f", setvoltage);
      HOST_SERIAL.println(reply);
      //setDevTx(DEV_MODE_TX_PIN, DEV_MODE_RX_PIN);
    }
  else
    {
      printf(reply, "Incorrect voltage: %f", setvoltage);
      HOST_SERIAL.println(reply);
      return;
    }
}

byte parse_host_msg(byte len)
{
  char *endtr;
  if (OK == checkstart(hostmsg, "get", 3))
    {
      return GET_MSG;
    }
  if (OK == checkstart(hostmsg, "set", 3))
    {
      char *endptr;
      setvoltage = strtof(hostmsg[4], endptr);
      return SET_MSG;
    }
  return ERR_MSG;
}


void loop()
{
  byte len = 0;
  if ((len = read_host_serial()) > 0)
    {
      byte type = parse_host_msg(len);
      switch type
	{
	case GET_MSG:
	  printf(reply, "Got get command: %s", hostmsg);
	  HOST_SERIAL.println(reply);
	case SET_MSG:
	  printf(reply, "Got set command: %s", hostmsg);
	  HOST_SERIAL.println(reply);
	  setVoltage();
	default:
	  printf(reply, "Got wrong command %s", hostmsg);
	  HOST_SERIAL.println(reply);
	}
    }
}

