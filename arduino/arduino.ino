#include <stdlib.h>

#define HOST_SERIAL Serial
#define DEV_SERIAL Serial1
#define TEST_SERIAL Serial2
#define SERIAL_SPEED 9600 
#define MSG_MAX_SIZE 128

#define DEV_RESPONSE_TIMEOUT  10 //in ms

#define SET_MSG  0
#define GET_MSG  1
#define ERR_MSG  3
#define MIN_VOLTAGE 100
#define MAX_VOLTAGE 700


#define PIN_MODE 22
#define PIN_MODE_TEST 23
#define DEV_MODE_TX HIGH
#define DEV_MODE_RX LOW

#define LINE_COMPLETE 0
#define LINE_NOT_COMPLETE 1
#define ERR 1
#define OK  0

byte READY = 0;

char hostmsg[MSG_MAX_SIZE] = {0};
char devmsg[MSG_MAX_SIZE] = {0};
char reply[MSG_MAX_SIZE] = {0};
float setvoltage = 100;
byte host_msg_len = 0;

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

byte readhostline()
{
  while (HOST_SERIAL.available())
	{
	  if (host_msg_len < MSG_MAX_SIZE)
		{
		  hostmsg[host_msg_len] = HOST_SERIAL.read();	
		  if (byte('\n') == hostmsg[host_msg_len])
			{
			  return LINE_COMPLETE;
			}
		  host_msg_len++;
		}
	  else
		{
		  return LINE_COMPLETE;
		}
	}
  return LINE_NOT_COMPLETE;
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

byte calcDelay(byte len, int speed)
{
   return ceil(8*len/(speed/1000.));
}

void devSendCmd(byte* cmd, byte len)
{
  setDevMode(PIN_MODE, DEV_MODE_TX);
  DEV_SERIAL.write(cmd, len);
  delay(calcDelay(len, SERIAL_SPEED));
  setDevMode(PIN_MODE, DEV_MODE_RX);
}

void setVoltage()
{
  byte cmd[3] = {85, 0, 85};
  if ((setvoltage >= MIN_VOLTAGE) && (setvoltage <= MAX_VOLTAGE))
    {
      HOST_SERIAL.print("Going to set voltage to: ");
      HOST_SERIAL.println(setvoltage, DEC);
      devSendCmd(cmd, 3);
    }
  else
    {
      HOST_SERIAL.print("Incorrect voltage: ");
      HOST_SERIAL.println(setvoltage, DEC);
      return;
    }
}



byte cs(byte* cmd, byte len)
{
  byte i;
  byte sum = cmd[0];
  for (i = 1; i < len; ++i)
	sum ^= cmd[i];
  return ~sum;
}

void send_get_cmd()
{
  byte cmd[5] = {0};
  cmd[0] = 1; //device number
  cmd[1] = 3; //cmd length
  cmd[2] = byte('A'); //get cmd code
  cmd[3] = 1;
  cmd[4] = cs(cmd, 4);
  devSendCmd(cmd, 5);
}

byte recvn_dev(byte n, byte* buffer, unsigned long timeout) //receive n bytes from device
{
  unsigned long start = millis();
  unsigned long stopt = start;
  byte received = 0;
  while (received < n)
    {
	  if (TEST_SERIAL.available())
        {
          buffer[received] = TEST_SERIAL.read();
          received++;
        }
	  stopt = millis();
	  if ((stopt - start) > timeout)
        {
          return ERR;
        }
    }
  return OK;
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
      setvoltage = (float)strtod(&hostmsg[4], &endptr);
      return SET_MSG;
    }
  return ERR_MSG;
}

void printMsgToHost(byte* msg, byte len)
{
   byte i;
   for (i = 0; i < len; ++i)
   {
      HOST_SERIAL.print(msg[i], HEX);
      HOST_SERIAL.print(" ");
   }
   HOST_SERIAL.println("");
}

void loop()
{
  if (!READY)
	{
      HOST_SERIAL.println("READY!");
      READY = 1;
	}
  if (LINE_COMPLETE == readhostline())
	{
	  HOST_SERIAL.print("GOT MSG: ");
	  HOST_SERIAL.println(hostmsg);
	  byte ret = parse_host_msg(host_msg_len);
	  switch (ret)
		{
		case GET_MSG:
		  HOST_SERIAL.println("GOT GET MSG");
		  send_get_cmd();
                  setDevMode(PIN_MODE_TEST, DEV_MODE_RX);
		  if (OK == recvn_dev(5, (byte*)devmsg, 1000))
                  {
                        HOST_SERIAL.println("GOT RESPONSE");
			printMsgToHost((byte*)devmsg,5);
                  }
		  else
			HOST_SERIAL.println("NOTHING RECEIVED");
		  break;
		case SET_MSG:
		  HOST_SERIAL.println("GOT SET MSG");
		  setVoltage();
		  break;
		default:  
		  HOST_SERIAL.println("GOT WRONG MSG");
        
		}
	  host_msg_len = 0;
	}
}
