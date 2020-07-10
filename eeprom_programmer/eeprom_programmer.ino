// EEPROM Programmer - code for an Arduino Mega 2560
//
// Written by K Adcock.
//       Jan 2016 - Initial release
//       Dec 2017 - Slide code tartups, to remove compiler errors for new Arduino IDE (1.8.5).
//   7th Dec 2017 - Updates from Dave Curran of Tynemouth Software, adding commands to enable/disable SDP.
//  10th Dec 2017 - Fixed one-byte EEPROM corruption (always byte 0) when unprotecting an EEPROM
//                  (doesn't matter if you write a ROM immediately after, but does matter if you use -unprotect in isolation)
//                - refactored code a bit (split loop() into different functions)
//                - properly looked at timings on the Atmel datasheet, and worked out that my delays
//                  during reads and writes were about 10,000 times too big!
//                  Reading and writing is now orders-of-magnitude quicker.
//  21st Feb 2018 - P. Sieg
//                  static const long int k_uTime_WriteDelay_uS = 500; // delay between byte writes - needed for at28c16
//                  delayMicroseconds(k_uTime_WritePulse_uS);
//  06th Oct 2018 - P. Sieg
//                - corrected SDP (un)protect adresses & k_uTime_WriteDelay_uS
//                - Set parameters -A=28C16; -B=28C64; -C=28C256
//  29th Jan 2019 - P. Sieg
//                - Introduced + and - to alter k_uTime_WritePulse_uS
//  05th Jul 2020 - P. Mans
//                - Changed pinout to modify 512
//
//
// Distributed under an acknowledgement licence, because I'm a shallow, attention-seeking tart. :)
//
// http://danceswithferrets.org/geekblog/?page_id=903
//
// This software presents a 9600-8N1 serial port.
//
// R[hex address]                         - reads 16 bytes of data from the EEPROM
// W[hex address]:[data in two-char hex]  - writes up to 16 bytes of data to the EEPROM
// P                                      - set write-protection bit (Atmels only, AFAIK)
// U                                      - clear write-protection bit (ditto)
// V                                      - prints the version string
// A                                      - set parameters for 28C16
// B                                      - set parameters for 28C64
// C                                      - set parameters for 28C256
// +                                      - k_uTime_WritePulse_uS + 50
// -                                      - k_uTime_WritePulse_uS - 25
//
// Any data read from the EEPROM will have a CRC checksum appended to it (separated by a comma).
// If a string of data is sent with an optional checksum, then this will be checked
// before anything is written.
//

#include <avr/pgmspace.h>
// include the library code:
 #include <LiquidCrystal.h>
 
// initialize the library with the numbers of the interface pins
 LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

const char hex[] =
{
  '0', '1', '2', '3', '4', '5', '6', '7',
  '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};

const char version_string[] = {"Version=0.15 PM"};

//Definition for 27256
static const int kPin_nOE     = 37; //22 22
static const int kPin_nCE     = 41; //20 20 
static const int kPin_nWE     = 24;  //1  22

//definition for 27512
//static const int kPin_nCE     = 25; //20 20 
//static const int kPin_nOE     = 37; //22 22
//static const int kPin_nWE     = 41;  //1  22
//static const int kPin_Addr15  = 24; //1  1

//Generic definitions
static const int kPin_Addr14  = 27; //1  27
static const int kPin_Addr13  = 29; //26 26
static const int kPin_Addr12  = 26; //2  2
static const int kPin_Addr11  = 35; //23 23
static const int kPin_Addr10  = 39; //21 21
static const int kPin_Addr9   = 33; //24 24
static const int kPin_Addr8   = 31; //25 25
static const int kPin_Addr7   = 28; //3  3
static const int kPin_Addr6   = 30; //4  4
static const int kPin_Addr5   = 32; //5  5
static const int kPin_Addr4   = 34; //6  6
static const int kPin_Addr3   = 36; //7  7 
static const int kPin_Addr2   = 38; //8  8 
static const int kPin_Addr1   = 40; //9  9
static const int kPin_Addr0   = 42; //10 10

static const int kPin_Data7   = 43; //19 19
static const int kPin_Data6   = 45; //18 18
static const int kPin_Data5   = 47; //17 17
static const int kPin_Data4   = 49; //16 16
static const int kPin_Data3   = 51; //15 15
static const int kPin_Data2   = 48; //13 13
static const int kPin_Data1   = 46; //12 12
static const int kPin_Data0   = 44; //11 11
static const int kPin_WaitingForInput  = 14;
static const int kPin_LED_Red = 22;
static const int kPin_LED_Grn = 53;

byte g_cmd[80]; // strings received from the controller will go in here
static const int kMaxBufferSize = 16;
byte buffer[kMaxBufferSize];

static const long int k_uTime_WritePulse_uS = 25; 
static const long int k_uTime_ReadPulse_uS = 25;
             long int k_uTime_WriteDelay_uS = 50; // delay between byte writes - needed for at28c16
// (to be honest, both of the above are about ten times too big - but the Arduino won't reliably
// delay down at the nanosecond level, so this is the best we can do.)
             long int SDPadr1=0x5555, SDPadr2=0x2AAA; // default 28C256
// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);
//  Serial.println(version_string);
  lcd.begin(8, 2);
  pinMode(kPin_WaitingForInput, OUTPUT); digitalWrite(kPin_WaitingForInput, HIGH);
  pinMode(kPin_LED_Red, OUTPUT); digitalWrite(kPin_LED_Red, LOW);
  pinMode(kPin_LED_Grn, OUTPUT); digitalWrite(kPin_LED_Grn, LOW);

  // address lines are ALWAYS outputs
  pinMode(kPin_Addr0,  OUTPUT);
  pinMode(kPin_Addr1,  OUTPUT);
  pinMode(kPin_Addr2,  OUTPUT);
  pinMode(kPin_Addr3,  OUTPUT);
  pinMode(kPin_Addr4,  OUTPUT);
  pinMode(kPin_Addr5,  OUTPUT);
  pinMode(kPin_Addr6,  OUTPUT);
  pinMode(kPin_Addr7,  OUTPUT);
  pinMode(kPin_Addr8,  OUTPUT);
  pinMode(kPin_Addr9,  OUTPUT);
  pinMode(kPin_Addr10, OUTPUT);
  pinMode(kPin_Addr11, OUTPUT);
  pinMode(kPin_Addr12, OUTPUT);
  pinMode(kPin_Addr13, OUTPUT);
  pinMode(kPin_Addr14, OUTPUT);
  //pinMode(kPin_Addr15, OUTPUT);
  
 //lcd.print("EEprommer Initialized!");
// lcd.print("Eepromme");
// lcd.setCursor(0, 1);
// lcd.print("r Init! ");
  LCDPrint("Eeprommer Init!");
  delay(5000);
  LCDPrint(version_string);
  // control lines are ALWAYS outputs
  pinMode(kPin_nCE, OUTPUT); digitalWrite(kPin_nCE, LOW); // might as well keep the chip enabled ALL the time
  pinMode(kPin_nOE, OUTPUT); digitalWrite(kPin_nOE, HIGH);
  pinMode(kPin_nWE, OUTPUT); digitalWrite(kPin_nWE, HIGH); // not writing
  SetDataLinesAsInputs();
  SetAddress(0);
}

void loop()
{
  while (true)
  {
    digitalWrite(kPin_WaitingForInput, HIGH);
    ReadString();
    digitalWrite(kPin_WaitingForInput, LOW);
    
    switch (g_cmd[0])
    {
      case 'V': Serial.println(version_string); break;
      case 'P': SetSDPState(true); break;
      case 'U': SetSDPState(false); break;
      case 'R': ReadEEPROM(); break;
      case 'W': WriteEEPROM(); break;
      case 'A': k_uTime_WriteDelay_uS=100;  
                SDPadr1=-1; SDPadr2=-1; Serial.println("Set params for 28C16");break;
      case 'B': k_uTime_WriteDelay_uS=5;  
                SDPadr1=0x1555; SDPadr2=0x0AAA; Serial.println("Set params for 28C64");break;
      case 'C': k_uTime_WriteDelay_uS=25;  
                SDPadr1=0x5555; SDPadr2=0x2AAA; Serial.println("Set params for 28C256");break;
      case '+': k_uTime_WriteDelay_uS=k_uTime_WriteDelay_uS+50;  
                if (k_uTime_WriteDelay_uS > 500) k_uTime_WriteDelay_uS = 500; 
                Serial.print("k_uTime_WriteDelay_uS=");Serial.println(k_uTime_WriteDelay_uS,DEC); break;
      case '-': k_uTime_WriteDelay_uS=k_uTime_WriteDelay_uS-25; 
                if (k_uTime_WriteDelay_uS < 5) k_uTime_WriteDelay_uS = 5; 
                Serial.print("k_uTime_WriteDelay_uS=");Serial.println(k_uTime_WriteDelay_uS,DEC); break;
      case 0: break; // empty string. Don't mind ignoring this.
      default: Serial.println("ERR Unrecognised command"); break;
    }
  }
}

void LCDPrint (String STR)
{
//   var String STR;
   String LINE1 = STR.substring(0, 8);
   String LINE2 = STR.substring(8);
   lcd.setCursor(0,0);
   lcd.print(LINE1);
   lcd.setCursor(0, 1);
   lcd.print(LINE2 + "          ");
}

void ReadEEPROM() // R<address>  - read kMaxBufferSize bytes from EEPROM, beginning at <address> (in hex)
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    LCDPrint("Error in reading Eeprom");
    return;
  }

  // decode ASCII representation of address (in hex) into an actual value
  int addr = 0;
  int x = 1;
  while (x < 5 && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x++]);
  }     

  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsInputs();
  digitalWrite(kPin_nOE, LOW); // makes the EEPROM output the byte
  delayMicroseconds(2);

  String ADDRS = String(addr, HEX);
  LCDPrint("reading: "+ ADDRS + "       ");
  ReadEEPROMIntoBuffer(addr, kMaxBufferSize);

  // now print the results, starting with the address as hex ...
  Serial.print(hex[ (addr & 0xF000) >> 12 ]);
  Serial.print(hex[ (addr & 0x0F00) >> 8  ]);
  Serial.print(hex[ (addr & 0x00F0) >> 4  ]);
  Serial.print(hex[ (addr & 0x000F)       ]);
  Serial.print(":");
  PrintBuffer(kMaxBufferSize);

  Serial.println("OK");
//  LCDPrint("Reading OK        ")
  digitalWrite(kPin_nOE, HIGH); // stops the EEPROM outputting the byte
}

void WriteEEPROM() // W<four byte hex address>:<data in hex, two characters per byte, max of 16 bytes per line>
{
  if (g_cmd[1] == 0)
  {
    Serial.println("ERR");
    LCDPrint("Error in writing Eeprom");
    return;
  }

  int addr = 0;
  int x = 1;
  while (g_cmd[x] != ':' && g_cmd[x] != 0)
  {
    addr = addr << 4;
    addr |= HexToVal(g_cmd[x]);
    ++x;
  }

  // g_cmd[x] should now be a :
  if (g_cmd[x] != ':')
  {
    Serial.println("ERR");
    return;
  }
  
  x++; // now points to beginning of data
  uint8_t iBufferUsed = 0;
  while (g_cmd[x] && g_cmd[x+1] && iBufferUsed < kMaxBufferSize && g_cmd[x] != ',')
  {
    uint8_t c = (HexToVal(g_cmd[x]) << 4) | HexToVal(g_cmd[x+1]);
    buffer[iBufferUsed++] = c;
    x += 2;
  }

  // if we're pointing to a comma, then the optional checksum has been provided!
  if (g_cmd[x] == ',' && g_cmd[x+1] && g_cmd[x+2])
  {
    byte checksum = (HexToVal(g_cmd[x+1]) << 4) | HexToVal(g_cmd[x+2]);

    byte our_checksum = CalcBufferChecksum(iBufferUsed);

    if (our_checksum != checksum)
    {
      // checksum fail!
      iBufferUsed = -1;
      Serial.print("ERR ");
      Serial.print(checksum, HEX);
      Serial.print(" ");
      Serial.print(our_checksum, HEX);
      Serial.println("");
      return;
    }
  }

  // buffer should now contains some data
  if (iBufferUsed > 0)
  {
    String ADDRS = String(addr, HEX);
    LCDPrint("Writing: "+ ADDRS + "       ");
    WriteBufferToEEPROM(addr, iBufferUsed);
  }

  if (iBufferUsed > -1)
  {
    Serial.println("OK");
  }
}

// Important note: the EEPROM needs to have data written to it immediately after sending the "unprotect" command, so that the buffer is flushed.
// So we read byte 0 from the EEPROM first, then use that as the dummy write afterwards.
// It wouldn't matter if this facility was used immediately before writing an EEPROM anyway ... but it DOES matter if you use this option
// in isolation (unprotecting the EEPROM but not changing it).

void SetSDPState(bool bWriteProtect)
{
  if (SDPadr1 < 0) {
    Serial.println("SDP feature not supported by chip");
    return;
  }
  digitalWrite(kPin_LED_Red, HIGH);

  digitalWrite(kPin_nWE, HIGH); // disables write
  digitalWrite(kPin_nOE, LOW); // makes the EEPROM output the byte
  SetDataLinesAsInputs();
  
  byte bytezero = ReadByteFrom(0);
  
  digitalWrite(kPin_nOE, HIGH); // stop EEPROM from outputting byte
  digitalWrite(kPin_nCE, HIGH);
  SetDataLinesAsOutputs();

  if (bWriteProtect)
  {
    WriteByteTo(SDPadr1, 0xAA);
    WriteByteTo(SDPadr2, 0x55);
    WriteByteTo(SDPadr1, 0xA0);
  }
  else
  {
    WriteByteTo(SDPadr1, 0xAA);
    WriteByteTo(SDPadr2, 0x55);
    WriteByteTo(SDPadr1, 0x80);
    WriteByteTo(SDPadr1, 0xAA);
    WriteByteTo(SDPadr2, 0x55);
    WriteByteTo(SDPadr1, 0x20);
  }
  
  WriteByteTo(0x0000, bytezero); // this "dummy" write is required so that the EEPROM will flush its buffer of commands.

  digitalWrite(kPin_nCE, LOW); // return to on by default for the rest of the code
  digitalWrite(kPin_LED_Red, LOW);

  Serial.print("OK SDP ");
  if (bWriteProtect)
  {
    Serial.println("enabled");
  }
  else
  {
    Serial.println("disabled");
  }
}

// ----------------------------------------------------------------------------------------

void ReadEEPROMIntoBuffer(int addr, int size)
{
  digitalWrite(kPin_LED_Grn, HIGH);
  digitalWrite(kPin_nWE, HIGH);
  SetDataLinesAsInputs();
  delayMicroseconds(1);
  digitalWrite(kPin_nOE, LOW);
//  delayMicroseconds(2);
  
  for (int x = 0; x < size; ++x)
  {
    buffer[x] = ReadByteFrom(addr + x);
//    delayMicroseconds(5);
  }

  digitalWrite(kPin_nOE, HIGH);
  digitalWrite(kPin_LED_Grn, LOW);
}

void WriteBufferToEEPROM(int addr, int size)
{
  digitalWrite(kPin_LED_Red, HIGH);
  digitalWrite(kPin_nOE, HIGH); // stop EEPROM from outputting byte
  digitalWrite(kPin_nWE, HIGH); // disables write
  SetDataLinesAsOutputs();

  for (uint8_t x = 0; x < size; ++x)
  {
    WriteByteTo(addr + x, buffer[x]);
    delayMicroseconds(k_uTime_WriteDelay_uS);    
  }
  
  digitalWrite(kPin_LED_Red, LOW);
}

// ----------------------------------------------------------------------------------------

// this function assumes that data lines have already been set as INPUTS, and that
// nOE is set LOW.
byte ReadByteFrom(int addr)
{
  SetAddress(addr);
  digitalWrite(kPin_nCE, LOW);
  delayMicroseconds(k_uTime_ReadPulse_uS);
  byte b = ReadData();
  digitalWrite(kPin_nCE, HIGH);

  return b;
}

// this function assumes that data lines have already been set as OUTPUTS, and that
// nOE is set HIGH.
void WriteByteTo(int addr, byte b)
{
  SetAddress(addr);
  SetData(b);

  digitalWrite(kPin_nCE, LOW);
  digitalWrite(kPin_nWE, LOW); // enable write  
  delayMicroseconds(k_uTime_WritePulse_uS);
  
  digitalWrite(kPin_nWE, HIGH); // disable write
  digitalWrite(kPin_nCE, HIGH); 
}

// ----------------------------------------------------------------------------------------

void SetDataLinesAsInputs()
{
  pinMode(kPin_Data0, INPUT);
  pinMode(kPin_Data1, INPUT);
  pinMode(kPin_Data2, INPUT);
  pinMode(kPin_Data3, INPUT);
  pinMode(kPin_Data4, INPUT);
  pinMode(kPin_Data5, INPUT);
  pinMode(kPin_Data6, INPUT);
  pinMode(kPin_Data7, INPUT);
}

void SetDataLinesAsOutputs()
{
  pinMode(kPin_Data0, OUTPUT);
  pinMode(kPin_Data1, OUTPUT);
  pinMode(kPin_Data2, OUTPUT);
  pinMode(kPin_Data3, OUTPUT);
  pinMode(kPin_Data4, OUTPUT);
  pinMode(kPin_Data5, OUTPUT);
  pinMode(kPin_Data6, OUTPUT);
  pinMode(kPin_Data7, OUTPUT);
}

void SetAddress(int a)
{
  digitalWrite(kPin_Addr0,  (a&1)?HIGH:LOW    );
  digitalWrite(kPin_Addr1,  (a&2)?HIGH:LOW    );
  digitalWrite(kPin_Addr2,  (a&4)?HIGH:LOW    );
  digitalWrite(kPin_Addr3,  (a&8)?HIGH:LOW    );
  digitalWrite(kPin_Addr4,  (a&16)?HIGH:LOW   );
  digitalWrite(kPin_Addr5,  (a&32)?HIGH:LOW   );
  digitalWrite(kPin_Addr6,  (a&64)?HIGH:LOW   );
  digitalWrite(kPin_Addr7,  (a&128)?HIGH:LOW  );
  digitalWrite(kPin_Addr8,  (a&256)?HIGH:LOW  );
  digitalWrite(kPin_Addr9,  (a&512)?HIGH:LOW  );
  digitalWrite(kPin_Addr10, (a&1024)?HIGH:LOW );
  digitalWrite(kPin_Addr11, (a&2048)?HIGH:LOW );
  digitalWrite(kPin_Addr12, (a&4096)?HIGH:LOW );
  digitalWrite(kPin_Addr13, (a&8192)?HIGH:LOW );
  digitalWrite(kPin_Addr14, (a&16384)?HIGH:LOW);
 // digitalWrite(kPin_Addr15, (a&32768)?HIGH:LOW);
}

// this function assumes that data lines have already been set as OUTPUTS.
void SetData(byte b)
{
  digitalWrite(kPin_Data0, (b&1)?HIGH:LOW  );
  digitalWrite(kPin_Data1, (b&2)?HIGH:LOW  );
  digitalWrite(kPin_Data2, (b&4)?HIGH:LOW  );
  digitalWrite(kPin_Data3, (b&8)?HIGH:LOW  );
  digitalWrite(kPin_Data4, (b&16)?HIGH:LOW );
  digitalWrite(kPin_Data5, (b&32)?HIGH:LOW );
  digitalWrite(kPin_Data6, (b&64)?HIGH:LOW );
  digitalWrite(kPin_Data7, (b&128)?HIGH:LOW);
}

// this function assumes that data lines have already been set as INPUTS.
byte ReadData()
{
  byte b = 0;

  if (digitalRead(kPin_Data0) == HIGH) b |= 1;
  if (digitalRead(kPin_Data1) == HIGH) b |= 2;
  if (digitalRead(kPin_Data2) == HIGH) b |= 4;
  if (digitalRead(kPin_Data3) == HIGH) b |= 8;
  if (digitalRead(kPin_Data4) == HIGH) b |= 16;
  if (digitalRead(kPin_Data5) == HIGH) b |= 32;
  if (digitalRead(kPin_Data6) == HIGH) b |= 64;
  if (digitalRead(kPin_Data7) == HIGH) b |= 128;

  return(b);
}

// ----------------------------------------------------------------------------------------

void PrintBuffer(int size)
{
  uint8_t chk = 0;

  for (uint8_t x = 0; x < size; ++x)
  {
    Serial.print(hex[ (buffer[x] & 0xF0) >> 4 ]);
    Serial.print(hex[ (buffer[x] & 0x0F)      ]);

    chk = chk ^ buffer[x];
  }

  Serial.print(",");
  Serial.print(hex[ (chk & 0xF0) >> 4 ]);
  Serial.print(hex[ (chk & 0x0F)      ]);
  Serial.println("");
}

void ReadString()
{
  int i = 0;
  byte c;

  g_cmd[0] = 0;
  do
  {
    if (Serial.available())
    {
      c = Serial.read();
      if (c > 31)
      {
        g_cmd[i++] = c;
        g_cmd[i] = 0;
      }
    }
  } 
  while (c != 10);
}

uint8_t CalcBufferChecksum(uint8_t size)
{
  uint8_t chk = 0;

  for (uint8_t x = 0; x < size; ++x)
  {
    chk = chk ^  buffer[x];
  }

  return(chk);
}

// converts one character of a HEX value into its absolute value (nibble)
byte HexToVal(byte b)
{
  if (b >= '0' && b <= '9') return(b - '0');
  if (b >= 'A' && b <= 'F') return((b - 'A') + 10);
  if (b >= 'a' && b <= 'f') return((b - 'a') + 10);
  return(0);
}
