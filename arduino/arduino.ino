#include "stdlib.h"

#define HOST_SERIAL Serial
#define DEV_SERIAL Serial1
#define TEST_SERIAL Serial2
#define SERIAL_SPEED 9600 
#define MSG_MAX_SIZE 128

#define RESPONSE_TIMEOUT  100 //in ms
#define MAX_MSG_LEN (256)

#define PIN_MODE 22
#define PIN_MODE_TEST 23
#define DEV_MODE_TX HIGH
#define DEV_MODE_RX LOW
#define MAX_FUNC_NAME 16
#define DEBUG
#define CD_MAGIC_CONST (1000*5/(4095*4.5))

#define MIN_VOLTAGE 100
#define MAX_VOLTAGE 500

#define OK 0
#define ERROR 1

byte READY = 0;

byte hostmsg[MAX_MSG_LEN] = {0};
byte msglen = 0;

#ifdef DEBUG
byte testmsg[MAX_MSG_LEN] = {0};
byte testmsglen = 0;
#endif

typedef int(*cli_func_t)(void*);

typedef struct cli
{
  char name[MAX_FUNC_NAME];
  cli_func_t cli_func;
}cli_t; 

const cli_t cli[] = {{"help",        &display_help}, 
		     {"set-voltage", &set_voltage},
		     {"send-cmd",    &send_cmd}};

byte chksum(byte* cmd, byte len)
{
  byte i;
  byte chks = 0;
  for (i = 0; i < len - 1; ++i)
    chks ^= cmd[i];
  return ~chks;
}


int send_cmd(void *arg)
{
  byte cmd[MAX_MSG_LEN];
  char* nstr = NULL;
  char* endptr = NULL;
  byte cmdlen = 0;
  byte i;
  if (NULL == arg)
    return ERROR;
  nstr = (char*)arg;
#ifdef DEBUG
  HOST_SERIAL.print("send-cmd: arg is: ");
  HOST_SERIAL.println((char*)arg);
#endif
  while (nstr = strtok(nstr, " "))
    {
      endptr = nstr;
#ifdef DEBUG
      HOST_SERIAL.print("send-cmd: token is: ");
      HOST_SERIAL.println(nstr);
#endif
      cmd[cmdlen] = (byte)strtol((const char*)nstr, &endptr, 16);
#ifdef DEBUG
      HOST_SERIAL.print("send-cmd: token is converted to: ");
      HOST_SERIAL.println(cmd[cmdlen], HEX);
#endif
      if (endptr == nstr)
	{
	  return ERROR;
	}
      cmdlen++;
      nstr = NULL;
    }
#ifdef DEBUG
  HOST_SERIAL.print("send-cmd: going to send the following symbols: ");
  for (i = 0; i < cmdlen; ++i)
    {
      HOST_SERIAL.print(cmd[i], HEX);
      HOST_SERIAL.print(" ");
    }
  HOST_SERIAL.println("");
#endif
  sendMsgBin(DEV_SERIAL, cmd, cmdlen, PIN_MODE);
  return OK;
}

int display_help(void* arg)
{
  return ERROR;
}


int set_voltage(void *arg)
{
  float voltage = 0;
  char* endptr, *str;
  byte cmd[8] = {0};
  if (arg != NULL)
    {
      str =  (char*)arg;
#ifdef DEBUG
      HOST_SERIAL.print("Arg is: ");
      HOST_SERIAL.println(str);
#endif
      voltage = (float)strtod(str, &endptr);
      if ((voltage < MIN_VOLTAGE) || (voltage > MAX_VOLTAGE))
	return ERROR;
      voltage = ceil(voltage/CD_MAGIC_CONST);
#ifdef DEBUG
      HOST_SERIAL.print("Going to set voltage to: ");
      HOST_SERIAL.println(voltage, 2);
#endif
    }
  else
    return ERROR;
  cmd[0] = 1; //device number
  cmd[1] = 6; //length of the cmd
  cmd[2] = byte('F'); //command code
  cmd[3] = ((byte*)(&voltage))[0];
  cmd[4] = ((byte*)(&voltage))[1];
  cmd[5] = ((byte*)(&voltage))[2];
  cmd[6] = ((byte*)(&voltage))[3];
  cmd[7] = chksum(cmd, 8);
  sendMsgBin(DEV_SERIAL, cmd, 8, PIN_MODE);
  return OK;
}

int cli_parser(byte* string, byte len)
{
  static byte nclis = sizeof(cli)/sizeof(cli_t);
  byte i = 0;
  byte slen = strlen((char*)string);
  for (i = 0; i < nclis; ++i)
    {
      byte fnamelen = strlen(cli[i].name);
      if (0 == strncmp((const char*)string, 
		       (const char*)cli[i].name, 
		       fnamelen))
	{
	  if ((fnamelen == slen) || (string[fnamelen + 1] == byte('\0')))
	    return ERROR;
	  char* arg = (char*)&string[fnamelen + 1];
	  
	  return cli[i].cli_func((void*)arg);
	}
    }
  HOST_SERIAL.println("I can't parse your line");
  return ERROR;
}

void setDevMode(byte pinMode, byte devMode)
{
  digitalWrite(pinMode, devMode);
}

void setup()
{
  READY = 0;
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

byte readHostLine(HardwareSerial serial, byte* buffer)
{
  static byte reads = 0;
  while (serial.available())
    {
      buffer[reads] = serial.read();
      if (buffer[reads] == byte('\n'))
	{
	  buffer[reads] = '\0';
	  byte res = reads;
	  reads = 0;
	  return res;
	}
      reads++;
    }
  return 0;
}

void sendMsgBin(HardwareSerial serial, byte* buffer, byte len, byte rxPin)
{
    byte i;
    if (rxPin)
      setDevMode(rxPin, DEV_MODE_TX);
    delay(5);  
    serial.write(buffer, len);
    delay(calcDelay(len + 1, SERIAL_SPEED));
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
    HOST_SERIAL.println("READY!");
    READY = 1;
    delay(200);
  }
  if ((msglen = readHostLine(HOST_SERIAL, hostmsg)) > 0)
    {
      #ifdef DEBUG
      TEST_SERIAL.flush();
      #endif
      byte res = cli_parser(hostmsg, msglen);
      #ifdef DEBUG
      if ((OK == res) && (testmsglen = readMsg(TEST_SERIAL, testmsg)) > 0)
	{
	  sendMsgHexToHost(HOST_SERIAL, testmsg, testmsglen);
	}
      #endif
    }
  delay(200);    
}
