// -----------------------------------------------------------------------------
// Paper Tape Reader
// Copyright (C) 2020 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
// -----------------------------------------------------------------------------


// Pins:
// D0 : serial RX (unused)
// D1 : serial TX
// D2 : cannot be used on rev0 PCB due to layout error, free on rev1
// D3 : motor power (uses PWM)
// D4-D11 : data hole photo-transistors
// D12 : index hole photo-transistor
// D13 : test signal "reading"
// A0  : rotary encoder pin A
// A1  : rotary encoder pin B
// A2  : rotary encoder button
// A3  : relay
// A4  : I2C SDA (OLED display)
// A5  : I2C SCL (OLED display)
//
// Board     : Arduino Pro or Pro Mini,
// Processor : ATMega328P (3.3V, 8MHz)
//
// To restore boot loader, compile with Arduino then upload
// C:\Users\hansel\AppData\Local\Temp\arduino_build_*\PaperTapeReader.ino.with_bootloader.hex
// with High Fuse Byte set to 0xD2
//
// To eliminate boot loader, compile with Arduino then upload
// C:\Users\hansel\AppData\Local\Temp\arduino_build_*\PaperTapeReader.ino.hex
// with High Fuse Byte set to 0xD7
//
// Fuse settings:         Low   High   Ext
// with    boot loader:  0xFE   0xD2  0xFD
// without boot loader:  0xFE   0xD7  0xFD

#include <PID_v1.h>
#include <EEPROM.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>


// include some pre-defined tapes in the program (to aid testing, see bottom of this file)
#define COMPARE_PROG_SET 0

// print debug information to serial interface (instead of regular output data)
#define DEBUG  0

// size of internal buffer
#define BUFSIZE 500

long CPS, LDL, baud, bits, parity, stopbits, motor_min_pwr, motor_max_pwr, motor_initial_pwr;
long motor_P, motor_I, motor_D, compare_mode = '-', show_stats;
unsigned long UPC, data_start_time, data_stop_time, data_num_chars;


// -------------------------------------------------------------------------------------------------
//                                    Rotary encoder handling
// -------------------------------------------------------------------------------------------------

volatile int rotationCount;
volatile bool CLKstate, DATAval;


inline void delay_button_debounce()
{
  delay(10);
}

inline bool is_button_pressed()
{
  return !digitalRead(A2);
}


inline void wait_button_press()
{
  while( !is_button_pressed() );
  delay_button_debounce();
}


inline void wait_button_release()
{
  while( is_button_pressed() );
  delay_button_debounce();
}


// returns rotation step count since previous call
// => positive=clockwise, negative=counter-clockwise
int get_rotation()
{
  int r;

  noInterrupts();
  r = rotationCount;
  rotationCount = 0;
  interrupts();
  
  return r;
}


// timeout on timer1 (called from ISR(TIMER1_COMPA_vect) below)
// used to debounce the rotary encoder
void rotary_timer_isr()
{
  bool CLK = PINC & 0x01;

  // if CLK value is still different from the previous state then this was not a bounce
  if( CLK != CLKstate )
    {
      // DATA is valid on HIGH->LOW edge of clock
      if( !CLK ) rotationCount += DATAval ? -1 : 1;

      CLKstate = CLK;
    }
  
  // re-enable pin change interrupts for rotary encoder
  PCIFR |= bit(PCIF1);
  PCICR |= bit(PCIE1);
}


// pin change interrupt handler for PCINT8-15, only PCINT8 (A0, PC0) is enabled
// => trigger debounce timer for capturing rotary encode state
ISR(PCINT1_vect)
{
  bool CLK = PINC & 0x01;

  // change in CLK triggers a DATA read
  if( CLK != CLKstate )
    {
      DATAval = PINC & 0x02; // remember current DATA value
      TCNT1   = 0;           // reset timer
      OCR1A   = 5000;        // set up timer to expire at 5 milliseconds
      TIFR1   = bit(OCF1A);  // reset output compare match A interrupt flag
      TIMSK1  = bit(OCIE1A); // enable timer interrupt on output compare match A
      PCICR  &= ~bit(PCIE1); // disable pin change interrupts for rotary encoder (until timer expires)
    }
}


void rotary_enable(bool b)
{
  if( b )
    {
      CLKstate = true;
      rotationCount = 0;
      PCMSK1 |= bit(PCINT8); // select pin-change interrupt on rotary encoder CLK (A0, PCINT8)
      PCIFR  |= bit(PCIF1);  // reset pin-change interrupt flag for PCINT8-15
      PCICR  |= bit(PCIE1);  // enable pin-change interrupts for PCINT8-15
    }
  else
    {
      PCICR &= ~bit(PCIE1);  // disable pin change interrupts for PCINT8-15
      rotationCount = 0;
    }
}


// -------------------------------------------------------------------------------------------------
//                                            Menu handling
// -------------------------------------------------------------------------------------------------


SSD1306AsciiAvrI2c oled;
int  currentItem;
bool modifyMode = false;
byte currentScreen, bufferFillDisplay = 0;


struct MenuItemStruct
{
  byte screen;
  byte col;
  byte row;
  byte len;
  long values[15];
  long def;
  long *value;
};


#if COMPARE_PROG_SET==0
#define MENUITEMNUM 16
#else
#define MENUITEMNUM 17
#endif

#define MAXLONG 0x7FFFFFFF
struct MenuItemStruct currentItemInfo;
const MenuItemStruct menuItems[MENUITEMNUM] PROGMEM =
  {
    {1, 2, 0, 0, {0}, 0, NULL},
    {1, 2, 1, 0, {0}, 0, NULL},
    {1, 2, 2, 0, {0}, 0, NULL},
    {2, 5, 0, 4, {13, 10, 30, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, MAXLONG}, MAXLONG, &CPS},
    {2, 5, 1, 4, {-1, 50, 50, 500}, 150, &LDL},
    {2, 5, 2, 6, {14, 55, 75, 110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200}, 9600, &baud},
    {2, 5, 3, 1, {4, 5, 6, 7, 8}, 8, &bits},
    {2, 7, 3, 1, {3+128, 'E', 'N', 'O'}, 'N', &parity},
    {2, 9, 3, 1, {2, 1, 2}, 1, &stopbits},
    {3, 5, 1, 3, {-1, 0, 1, 255},  15, &motor_min_pwr},
    {3, 5, 2, 3, {-1, 0, 1, 255}, 255, &motor_max_pwr},
    {3, 5, 3, 3, {-1, 0, 5, 255}, 200, &motor_initial_pwr},
    {4, 5, 1, 3, {-1, 0, 1, 999},  10, &motor_P},
    {4, 5, 2, 3, {-1, 0, 1, 999},  10, &motor_I},
    {4, 5, 3, 3, {-1, 0, 1, 999},   0, &motor_D},
    {5, 8, 2, 1, {2+128, 'N', 'Y'}, 'N', &show_stats},
#if COMPARE_PROG_SET==1
    {5, 8, 3, 1, {3+128, '-', 'B', 'L'}, '-', &compare_mode},
#elif COMPARE_PROG_SET==2
    {5, 8, 3, 1, {2+128, '-', 'W'}, '-', &compare_mode},
#elif COMPARE_PROG_SET==3
    {5, 8, 3, 1, {2+128, '-', 'T'}, '-', &compare_mode},
#endif
  };


uint32_t getSerialConfig()
{
  int v = bits * 8 + stopbits-1;
  if     ( parity=='E' ) v += 2;
  else if( parity=='O' ) v += 4;
  
  switch( v )
    {
    case 0x00: return SERIAL_5N1;
    case 0x01: return SERIAL_5N2;
    case 0x02: return SERIAL_5E1;
    case 0x03: return SERIAL_5E2;
    case 0x04: return SERIAL_5O1;
    case 0x05: return SERIAL_5O2;

    case 0x08: return SERIAL_6N1;
    case 0x09: return SERIAL_6N2;
    case 0x0A: return SERIAL_6E1;
    case 0x0B: return SERIAL_6E2;
    case 0x0C: return SERIAL_6O1;
    case 0x0D: return SERIAL_6O2;
  
    case 0x10: return SERIAL_7N1;
    case 0x11: return SERIAL_7N2;
    case 0x12: return SERIAL_7E1;
    case 0x13: return SERIAL_7E2;
    case 0x14: return SERIAL_7O1;
    case 0x15: return SERIAL_7O2;

    case 0x18: return SERIAL_8N1;
    case 0x19: return SERIAL_8N2;
    case 0x1A: return SERIAL_8E1;
    case 0x1B: return SERIAL_8E2;
    case 0x1C: return SERIAL_8O1;
    case 0x1D: return SERIAL_8O2;

    // fall back to default 8N1 settings
    default  : return SERIAL_8N1;
    }
}


bool stepItemValue(struct MenuItemStruct *info, char direction)
{
  int i;
  if( info->value!=NULL )
    {
      long v = *(info->value);
      int n = info->values[0];
      if( n>128 ) n-=128;

      if( n<0 )
        {
          if( direction>0 ) { v += info->values[2]; if( v > info->values[3] ) v = info->values[1]; }
          if( direction<0 ) { v -= info->values[2]; if( v < info->values[1] ) v = info->values[3]; }
        }
      else if( n>0 )
        {
          if( direction<0 )
            {
              for(i=n; i>0; i--)
                if( info->values[i] < v )
                  { v = info->values[i]; break; }
              if( i==0 ) v = info->values[n];
            }
          else if( direction>0 )
            {
              for(i=1; i<=n; i++)
                if( info->values[i] > v )
                  { v = info->values[i]; break; }
              if( i>n ) v = info->values[1];
            }
        }

      if( v != *(info->value) )
        {
          *(info->value) = v;
          return true;
        }
    }

  return false;
}


bool stepCurrentItemValue(char direction)
{
  bool res = stepItemValue(&currentItemInfo, direction);
  if( res && currentItemInfo.screen == currentScreen )
    printItemValue(&currentItemInfo);
    
  return res;
}


void saveCurrentItemValue()
{
  EEPROM.put(4+4*currentItem, *(currentItemInfo.value));
}


void getItemInfo(byte item, struct MenuItemStruct *info)
{
  if( item>=0 && item<MENUITEMNUM )
    memcpy_P(info, &(menuItems[item]), sizeof(MenuItemStruct));
}


void displayParams()
{
  oled.set1X();
  oled.setCursor(0, 7);
  oled.print(baud); oled.print(' '); oled.print(bits); oled.write(parity); oled.print(stopbits);
  if( compare_mode!='-' ) { oled.setCursor(11*6, 7); oled.print('C'); oled.print((char) compare_mode); }
  oled.setCursor((CPS<100 ? 15 : 14)*6, 7);
  if( CPS==MAXLONG ) oled.print(F("MAX")); else oled.print(CPS); 
  oled.print(F(" CPS"));
  oled.set2X();  
}

void displayScreen(int n)
{
  if( currentScreen==n ) return;
  currentScreen = n;
  oled.clear();
  oled.set2X();  

  if( n==1 )
    {
      oled.println(F("  PLAY"));
      oled.println(F("  PLAY+LDL"));
      oled.println(F("  FFWD"));

      oled.set1X();
      oled.setCursor(0, 6);
      if( UPC>=10000 ) for(int i = (buffer_size()*20)/BUFSIZE; i>0; i--) oled.write('=');

      displayParams();
    }
  else if( n==2 )
    {
      oled.println(F("CPS: "));
      oled.println(F("LDL: "));
      oled.println(F("SER: "));
    }
  else if( n==3 )
    {
      oled.println(F("MOTOR"));
      oled.println(F("MIN: "));
      oled.println(F("MAX: "));
      oled.println(F("INI: "));
    }
  else if( n==4 )
    {
      oled.println(F("CONTROLLER"));
      oled.println(F("P  : "));
      oled.println(F("I  : "));
      oled.println(F("D  : "));
    }
  else if( n==5 )
    {
      oled.println(F("TESTING"));
      oled.println();
      oled.println(F("STATS : "));
#if COMPARE_PROG_SET>0
      oled.println(F("COMP  : "));
#endif
    }

  for(int i=0; i<MENUITEMNUM; i++)
    {
      MenuItemStruct info;
      getItemInfo(i, &info);
      if( info.screen==currentScreen ) printItemValue(&info);
    }
}


void setCursor(byte col, byte row)
{
  oled.setCursor(col*12, row*2);
}


void showCursor(int onOff)
{
  setCursor(currentItemInfo.col-1, currentItemInfo.row);
  oled.set2X();
  oled.write(onOff==0 ? ' ' : (onOff==1 ? '>' : 0x7e));
}


void printItemValue(struct MenuItemStruct *info)
{
  if( info->value!=NULL )
    {
      setCursor(info->col, info->row);
      for(int i=0; i<info->len; i++) oled.print(' ');
      setCursor(info->col, info->row);
      if( info->values[0] > 128 )
        oled.write(*(info->value));
      else if( *(info->value) == MAXLONG )
        oled.print(F("MAX"));
      else
        oled.print((unsigned long) *(info->value));
    }
}

void readSettings()
{
  unsigned long v;
  EEPROM.get(0, v);
  if( v!=0x12345678 ) { v = 0x12345678; EEPROM.put(0, v); v = 0; }
  for(int i=0; i<MENUITEMNUM; i++) 
    {
      struct MenuItemStruct itemInfo;
      getItemInfo(i, &itemInfo);
      if( itemInfo.value!=NULL ) 
        {
          if( v==0x12345678 )
            {
              EEPROM.get(4+i*4, *(itemInfo.value));
              if( itemInfo.values[0]<0 && (*(itemInfo.value)<itemInfo.values[1] || *(itemInfo.value)>itemInfo.values[3]) )
                *(itemInfo.value) = itemInfo.def;
            }
          else
            {
              *(itemInfo.value) = itemInfo.def;
              EEPROM.put(4+i*4, *(itemInfo.value));
            }
        }
    }
}


void show_menu()
{
  currentScreen = 0;
  displayScreen(currentItemInfo.screen);
  showCursor(1);
}


void resetMenu()
{
  currentItem = 0;
  getItemInfo(currentItem, &currentItemInfo);
  show_menu();
}


void setupMenu()
{
  oled.begin(&Adafruit128x64, 0x3C);
  oled.setFont(System5x7);

  // 400kHz I2C clock at 8MHz system clock is borderline but seems to 
  // work with my display
  oled.setI2cClock(400000);

  readSettings();
  resetMenu();
}


void handle_menu()
{
  int rotated = get_rotation();
  if( rotated )
    {
      if( modifyMode )
        stepCurrentItemValue(rotated);
      else
        {
          showCursor(0);
          currentItem += rotated;
          while( currentItem<0 ) currentItem += MENUITEMNUM;
          currentItem = currentItem % MENUITEMNUM;

          getItemInfo(currentItem, &currentItemInfo);
          displayScreen(currentItemInfo.screen);
          showCursor(1);
        }
    }
  else if( is_button_pressed() && currentItemInfo.value!=NULL )
    {
      unsigned long t = millis();
      delay_button_debounce();
      while( is_button_pressed() && millis()-t<250 );

      if( millis()-t<250 && !modifyMode )
        {
          if( stepCurrentItemValue(1) ) saveCurrentItemValue();
        }
      else if( modifyMode )
        {
          modifyMode = false;
          saveCurrentItemValue();
          showCursor(1);
        }
      else
        {
          modifyMode = true;
          showCursor(2);
        }
         
      delay_button_debounce();
      wait_button_release();
    }
}



// -------------------------------------------------------------------------------------------------
//                              Use software serial for less than 150 baud
// -------------------------------------------------------------------------------------------------


static bool useSoftSerial;
static byte bitCounter, totalBits;
static unsigned int microsPerBit, regShiftOut, stopBitMask;
static unsigned long prevSend;

static byte parityTable[32] = {0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69,
                               0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96,
                               0x69, 0x96, 0x96, 0x69, 0x96, 0x69, 0x69, 0x96,
                               0x96, 0x69, 0x69, 0x96, 0x69, 0x96, 0x96, 0x69};

#define GET_EVEN_PARITY(d) (parityTable[(d)/8] & (1<<((d)&7)))

#define SERIAL_BUFFER_SIZE (useSoftSerial ? 1 : 64)

void beginSoftSerial(unsigned int baud)
{
  microsPerBit  = 1000000ul / baud;
  bitCounter = 0xff;

  totalBits = bits + stopbits + (parity!='N');
  stopBitMask = ((1<<stopbits)-1) << (bits + (parity!='N'));
}


void writeSoftSerial(byte data)
{
  if( bitCounter==0xff )
    {
      // copy data to shift register
      regShiftOut = data & ((1<<bits)-1);

      // compute parity
      if( (parity=='O' && !GET_EVEN_PARITY(regShiftOut)) || (parity=='E' && GET_EVEN_PARITY(regShiftOut)) )
        regShiftOut |= 1 << bits;

      regShiftOut |= stopBitMask;
      bitCounter = totalBits;

      // send start bit
      PORTD &= ~0x02;
      prevSend = micros();
    }
}


void handleSoftSerial()
{
  // send next data bit
  if( bitCounter<0xff && (micros()-prevSend)>=microsPerBit )
    {
      if( bitCounter>0 ) { if( regShiftOut & 0x01 ) PORTD |= 0x02; else PORTD &= ~0x02; }
      prevSend += microsPerBit;
      regShiftOut >>= 1;
      bitCounter--;
    }
}


void initSerial(long baud)
{
  // set UART parameters
  if( baud < 150 )
    {
      digitalWrite(1, HIGH);
      Serial.end();
      beginSoftSerial(baud);
      useSoftSerial = true;
      bitCounter=0xff;
    }
  else
    {
      Serial.end();
      Serial.begin(baud, getSerialConfig());
      useSoftSerial = false;
    }
}


inline void writeSerial(byte data)
{
  if( useSoftSerial )
    writeSoftSerial(data);
  else
    Serial.write(data);
}


inline int serialAvailableForWrite()
{
  if( useSoftSerial )
    return bitCounter==0xFF ? 1 : 0;
  else
    return Serial.availableForWrite();
}


// -------------------------------------------------------------------------------------------------
//                                           Main functions   
// -------------------------------------------------------------------------------------------------


#define MOTOR_AUTO   0
#define MOTOR_BOOST1 1
#define MOTOR_BOOST2 2
#define MOTOR_BOOST3 3
#define MOTOR_BOOST4 4
#define MOTOR_DRAIN  10

double buffer_len_setpoint, buffer_len_setpoint_max, motor_power;
int motorMode;
char error_msg1[20], error_msg2[20];

volatile double buffer_len = 0;
volatile byte buffer[BUFSIZE];
volatile int  buffer_start = 0, buffer_end = 0;
volatile bool running = false;
volatile int nIndexPulses = 0, compare_data_ptr = 0, error = 0;
volatile unsigned long prevIndexPulseTime = 0, read_timeout = 0;

PID myPID((double *) &buffer_len, &motor_power, &buffer_len_setpoint, 2, 5, 1, DIRECT);


inline int  compare_data_size(int i);
inline byte compare_data_read(int i, int n);


inline bool buffer_empty()
{
  return buffer_len==0;
}


inline int buffer_size()
{
  return buffer_len;
}


inline byte buffer_get()
{
  if( buffer_len > 0 )
    {
      noInterrupts();
      byte b = buffer[buffer_start++];
      if ( buffer_start == BUFSIZE ) buffer_start = 0;
      buffer_len--;
      interrupts();
      return b;
    }
  else
    return 0;
}


inline bool buffer_add(byte b)
{
  if ( buffer_len < BUFSIZE - 1 )
    {
      buffer[buffer_end++] = b;
      if ( buffer_end == BUFSIZE ) buffer_end = 0;
      buffer_len++;
      return true;
    }
  else
    {
#if DEBUG>0
      Serial.println(F("OVERFLOW"));
#endif
      return false;
    }
}


void send_data_stop()
{
#if DEBUG>0
  Serial.println(F("SEND_STOP"));
#endif
  digitalWrite(A3, LOW);
}


void send_data_start()
{
#if DEBUG>0
  Serial.print(F("SEND_START: "));
  Serial.print(baud); Serial.print(F(" / 0x")); Serial.println(getSerialConfig(), HEX);
#else
  initSerial(baud);
#endif
  digitalWrite(A3, HIGH);
}


void showStats()
{
  oled.clear();
  oled.set2X();  
  oled.setCursor(0, 0);

  oled.print(F("CHR: ")); 
  oled.println(data_num_chars);

  // subtract 1s from time since that is the timeout for detecting
  // the end of tape
  oled.print(F("SEC: ")); 
  unsigned long ds = (data_stop_time-data_start_time)/100;
  if( (data_stop_time-data_start_time)%100 >= 50 ) ds++;
  oled.print(ds/10);
  oled.print('.');
  oled.println(ds%10);

  oled.print(F("CPS: ")); 
  unsigned int i = 10000*data_num_chars/(data_stop_time-data_start_time);
  oled.println((i+5)/10);

  oled.print(F("   >OK"));
  wait_button_press();
  wait_button_release();
}


void fast_fwd()
{
  oled.clear();
  oled.set2X();
  setCursor(0, 0);
  oled.print(F("FAST FWD..."));
  setCursor(2, 2);
  oled.println(F(">STOP"));
          
  analogWrite(3, 255);

  // button may still be pressed when entering fast_fwd
  delay_button_debounce();
  wait_button_release();

  wait_button_press();
  analogWrite(3, 0);
  wait_button_release();
  show_menu();
}


void test()
{
  unsigned long t;

  // test mode
  oled.clear();
  oled.set2X();
  setCursor(1, 1); oled.print(F("0x00 ^"));
  setCursor(0, 3); oled.print(F("MOTOR: 0"));
          
  int i, pwr = 0, prev = -1;
  byte go = 1, stepsize = 5;
  while ( go )
    {
      int  idx = ((~PINB) & 0x10)<<4;
      byte b = ~((PIND & 0xF0) | (PINB & 0x0F));
              
      if( (b|idx)!=prev )
        {
          // display data/index hole status (1=open/0=closed)
          setCursor(1, 0);
          for(i = 7; i >= 0; i--) 
            {
              oled.write( b & (1 << i) ? '1' : '0');
              if( i==3 ) oled.write(idx ? '1' : '0'); 
            }

          // display byte in hex and ASCII
          setCursor(3, 1); 
          if( b<0x10 ) oled.write('0');
          oled.print(b, HEX);
          setCursor(9, 1);
          oled.write(b >= 32 && b < 127 ? b : '.');

          prev = b|idx;
        }
              
      int r = get_rotation();
      if( r!=0 )
        {
          // change motor power
          pwr += r * stepsize;
          if( pwr<0 ) pwr = 255;
          else if( pwr>255 ) pwr = 0;
          analogWrite(3, pwr);

          // display motor power
          setCursor(7, 3);
          oled.print(pwr);
          if( pwr<100 ) oled.write(' ');
          if( pwr<10  ) oled.write(' ');
        }
              
      if( go == 1 && !is_button_pressed() )
        { go = 2; delay_button_debounce(); }
      else if( go == 2 && is_button_pressed() )
        { go = 3; t = millis(); delay_button_debounce();  }
      else if( go == 3 && !is_button_pressed() )
        { 
          // long button push  => toggle motor power step size 1/5
          // short button push => exit test mode
          if( millis()-t > 500 )
            { go = 1; stepsize = stepsize==1 ? 5 : 1; }
          else
            go = 0; 
          delay_button_debounce();
        }
    }

  // turn motor off
  analogWrite(3, 0);
  wait_button_release();
  show_menu();
}


void start_running()
{
  UPC = max(1000000ul/CPS, (long(1 + bits + (parity=='N' ? 0 : 1) + stopbits) * 1000000ul)/baud);
  oled.set2X();
  oled.clear();
  setCursor(0, 0);
  oled.println(F("RUNNING..."));
  if( currentItem==1 ) { oled.set1X(); oled.setCursor(12, 2); oled.print(F("LINE DELAY: ")); oled.print(LDL); oled.print(F("ms")); oled.set2X(); }
  setCursor(2, 2);
  oled.println(F(">STOP"));
  displayParams();
  oled.set1X();
  send_data_start();
  running = true;
  error = 0;
  nIndexPulses = 0;
  bufferFillDisplay = 0;
  int cps = 1000000ul / UPC;
  data_num_chars=0;
  data_start_time=0;
  data_stop_time=0;
  compare_data_ptr = 0;
  prevIndexPulseTime = 0;
  rotary_enable(false); // disable rotary encoder (need timer1 for data holes)
  PCIFR |= bit(PCIF0);  // clear outstanding pin change interrupts
  PCICR |= bit(PCIE0);  // enable pin change interrupts for index hole

  if( motor_max_pwr>0 )
    {
      buffer_len_setpoint_max = max(min(cps, BUFSIZE/5), 25);
      buffer_len_setpoint = buffer_len_setpoint_max;

      // set PID parameters
      myPID.SetOutputLimits(motor_min_pwr, motor_max_pwr);
      myPID.SetTunings(motor_P/100.0, motor_I/100.0, motor_D/100.0);
              
      // compute initial motor power
      if( currentItem==1 || cps<=50 )
        motor_power = motor_min_pwr+5; // using line delay and/or low CPS => start with low power
      else if( cps>=800 )
        motor_power = motor_initial_pwr; // no line delay, high CPS => start with full initial power
      else
        motor_power = map(cps, 50, 1000, motor_min_pwr+5, motor_initial_pwr); // interpolate initial power

#if DEBUG>0
      Serial.println(F("STARTING"));
      Serial.print(F("CPS: ")); Serial.print(CPS); Serial.print(' '); Serial.println(UPC);
#endif

      analogWrite(3, motor_power);
      motorMode = MOTOR_AUTO;
      myPID.SetMode(AUTOMATIC);

      // stop if no index hole is seen within 2 seconds
      read_timeout = millis() + 2000;
    }
}


void stop_running(bool do_show_menu = true)
{
  running = false;
  analogWrite(3, 0);      // turn off motor
  myPID.SetMode(MANUAL);  // disable PID controller
  PCICR &= ~bit(PCIE0);   // disable pin change interrupts for index hole
  TIMSK1 &= ~bit(OCIE1A); // disable timer 1 interrupts
  rotary_enable(true);    // re-enable rotary encoder
  if( do_show_menu ) show_menu();
  if( buffer_empty() ) send_data_stop();
}


void handle_boost()
{
  // boost motor if it appears to be stalled
  unsigned long t = micros()-prevIndexPulseTime;
  if( motor_power < motor_min_pwr+10 && t > 30000 && t > (2*UPC) && motorMode==MOTOR_AUTO )
    {
      motor_power += 3;
      analogWrite(3, motor_power+.5);
#if DEBUG>0
      Serial.print(F("BOOST1:")); Serial.println(motor_power);
#endif        
      myPID.SetMode(MANUAL);
      motorMode = MOTOR_BOOST1;
    }
  else if(motorMode==MOTOR_BOOST1 && t > 60000 )
    {
      motor_power += 5;
      analogWrite(3, motor_power+.5);
#if DEBUG>0
      Serial.print(F("BOOST2:")); Serial.println(motor_power);
#endif        
      motorMode = MOTOR_BOOST2;
    }
  else if( motorMode==MOTOR_BOOST2 && t > 150000 )
    {
      motor_power = max(motor_min_pwr+15, motor_power);
      analogWrite(3, motor_power);
#if DEBUG>0
      Serial.print(F("BOOST3:")); Serial.println(motor_power);
#endif        
      motorMode = MOTOR_BOOST3;
      read_timeout = max(read_timeout, millis() + 1000);
    }
  else if( motorMode==MOTOR_BOOST3 && t > 500000 && nIndexPulses>=5 )
    {
      analogWrite(3, motor_min_pwr*2);
#if DEBUG>0
      Serial.println(F("BOOST4"));
#endif        
      motorMode = MOTOR_BOOST4;
      read_timeout = max(read_timeout, millis() + 1000);
    }
  else if( motorMode>=MOTOR_BOOST1 && motorMode<=MOTOR_BOOST4 && t < 15000 )
    {
      motorMode = MOTOR_AUTO;
      myPID.SetMode(AUTOMATIC);
      read_timeout = millis() + 1000;
#if DEBUG>0
      Serial.println(F("AUTO"));
#endif        
    }
}


void control_motor()
{
  if( motorMode!=MOTOR_DRAIN && buffer_size() > BUFSIZE-50 )
    {
      // the buffer is almost full => stop the motor to drain the buffer
      analogWrite(3, 0);
      motor_power = 0;
      myPID.SetMode(MANUAL);
      motorMode = MOTOR_DRAIN;
#if DEBUG>0
      Serial.println(F("DRAIN"));
#endif        
    }
  else if( motorMode==MOTOR_DRAIN && buffer_size()<buffer_len_setpoint )
    {
      // draining complete => restart the motor
      motor_power = motor_min_pwr+5;
      analogWrite(3, motor_power);
      motorMode = MOTOR_AUTO;
      myPID.SetMode(AUTOMATIC);
      prevIndexPulseTime = micros();
      read_timeout = millis() + 1000;
#if DEBUG>0
      Serial.println(F("RESTART"));
#endif        
    }
  else if( motorMode!=MOTOR_DRAIN )
    {
      // boost motor power if motor appears to be stalled
      handle_boost();

      // compute motor power (happens every 200ms, disabled when boosting)
      if( myPID.Compute() )
        {
#if DEBUG>0
          if( UPC>=20000 )
            { Serial.print(buffer_len_setpoint); Serial.print(' '); Serial.print(buffer_size()); Serial.print(' '); Serial.println(motor_power); }
#endif
          analogWrite(3, motor_power + 0.5);
        }

      // if we have not seen an index hole in "read_timeout" milliseconds then assume the tape has ended
      if( millis() > read_timeout )
        {
#if DEBUG>0
          Serial.println(F("STOPPING (NO DATA)"));
#endif
          if( show_stats=='Y' && error==0 && data_num_chars>0 && data_stop_time>data_start_time )
            {
              stop_running(false);
              if( buffer_empty() ) showStats();
              show_menu();
            }
          else
            stop_running();
        }
    }
}


void error_stop(const char *msg1, const char *msg2)
{
  stop_running(false);
  if( msg1 != error_msg1 ) strcpy(error_msg1, msg1);
  if( msg2 != error_msg2 ) strcpy(error_msg2, msg2);
  error = 1;
}


void error_display()
{
  oled.clear();
  oled.set2X();
  setCursor(0, 0);
  oled.println(F("!! ERROR !!"));
  oled.println(error_msg1);
  oled.println(error_msg2);
  displayParams();

#if DEBUG>0
  Serial.print(error_msg1);
  Serial.print(' ');
  Serial.println(error_msg1);
#endif
  error = 2;
}


// timer interrupt handler => capture data
ISR(TIMER1_COMPA_vect)
{
  TIMSK1 &= ~bit(OCIE1A); // disable timer 1 interrupt

  // when not reading data we're using timer 1 to debounce the rotary encoder
  // (which is not needed while reading data)
  if( !running ) { rotary_timer_isr(); return; }

  PORTB |= 0x20; // set "reading" test signal

  // read data holes
  byte b = ~((PIND & 0xF0) | (PINB & 0x0F));

  // attempt to add newly read data to the buffer
  if( !buffer_add(b) ) error_stop("  BUFFER  ", " OVERFLOW ");

#if COMPARE_PROG_SET>0
  // compare data read against stored program requested
  if( compare_mode!='-' && compare_data_ptr>=0 )
    {
      // skip repeats of the leader byte
      if( compare_data_ptr>0 || b!=compare_data_read(compare_mode, 0) ) compare_data_ptr++;
          
      byte bb = compare_data_read(compare_mode, compare_data_ptr);
      if( b!=bb )
        {
          //digitalWrite(A3, LOW);
          sprintf(error_msg1, "DIFF @%i", compare_data_ptr);
          sprintf(error_msg2, " %02Xh<>%02Xh", b, bb);
          error_stop(error_msg1, error_msg2);
        }
      else if( compare_data_ptr+1 >= compare_data_size(compare_mode) )
        {
#if DEBUG>0
          Serial.println(F("No diffs!"));
#endif
          compare_data_ptr = -1;
        }
    }
#endif

  if( running ) 
    {
      // clear and re-enable pin change interrupts for index hole
      PCIFR |= bit(PCIF0);
      PCICR |= bit(PCIE0);
    }

  PORTB &= ~0x20; // clear "reading" test signal
}


// pin change interrupt handler for PCINT0-7, only PCINT4 (D12) is enabled
// => trigger timer for capturing data if index hole detected
ISR(PCINT0_vect)
{
  // index hole detected if pin PB4 (D12) goes LOW
  if( (PINB & 0x10)==0 )
    {
      unsigned long t    = micros();
      unsigned long td   = t - prevIndexPulseTime;
      prevIndexPulseTime = t;

      // wait to see 5 index holes before actually starting to reading data
      if( nIndexPulses >= 5 )
        {
          // set up timer1 to delay capture until data is more stable to read
          OCR1A  = min(td/4, UPC<50000 ? 5000 : 50000); // set up for output compare match A at N microseconds
          TCNT1  = 0;               // reset timer
          TIFR1  = bit(OCF1A);      // reset output compare match A interrupt flag
          TIMSK1 = bit(OCIE1A);     // enable timer interrupt on output compare match A
          PCICR &= ~bit(PCIE0);     // disable pin change interrupts for index hole (until timer expires)
        }
      else
        nIndexPulses++;

      // stop if we don't see another index hole within half a second
      read_timeout = millis() + 500;
    }
}


void setup()
{
  digitalWrite(1, HIGH);
  pinMode(1, OUTPUT);       
  pinMode(2, INPUT); // MUST be INPUT for PCB version 0 (layout error)
  pinMode(3, OUTPUT);       
  for (int i = 0; i < 8; i++) pinMode(4 + i, INPUT);
  pinMode(12, INPUT);       
  pinMode(2, OUTPUT);
  pinMode(13, OUTPUT);      
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, OUTPUT);

  // set PWM frequency to 15686 Hz (8MHz clock, no prescaling, timer counts to 510)
  TCCR2B = (TCCR2B & B11111000) | B00000001;

  setupMenu();

  myPID.SetSampleTime(200);

#if DEBUG>0
  Serial.begin(115200);
  Serial.println(F("DEBUG"));
#endif
  send_data_stop();

  // select pin-change interrupt on index hole sensor (D12, PCINT4)
  PCMSK0 |= bit(PCINT4);

  // set timer1 to prescaler 8 (at 8MHz clock => 1 microsecond resolution)
  TCCR1A = 0;  
  TCCR1B = bit(CS11);
  TCCR1C = 0;  

  // enable rotary encoder
  rotary_enable(true);

  // initialize ringbuffer
  buffer_start = 0;
  buffer_end   = 0;
  buffer_len   = 0;
}


void loop()
{
  if( running )
    {
      // currently reading tape
      if( is_button_pressed() )
        {
          // button pressed while running => stop
#if DEBUG>0
          Serial.println(F("STOPPING (BUTTON)"));
#endif
          stop_running();
          wait_button_release();
        }
      else if( motor_max_pwr>0 ) 
        {
          // control motor power (if motor_max_pwr==0 then we're in manual mode)
          control_motor();
        }
    }
  else
    {
      // NOT currently reading tape
      if( error==1 )
        {
          // error message was set when reading tape in interrupt handler
          // => display it now
          error_display();
        }
      else if( error==2 && is_button_pressed() )
        {
          // currently displaying an error message 
          // => clear it and show menu
          show_menu();
          wait_button_release();
          error = 0;
        }
      else
        {
          // handle menu navigation via rotary encoder
          handle_menu();

          if( is_button_pressed() && currentItem < 3 )
            {
              // rotary encoder button pressed on first menu page (item 0, 1 or 2)
              delay_button_debounce();
              
              // distinguish between long or short button press
              unsigned long t = millis();
              while( is_button_pressed() && millis()-t<500 );
              bool longPush = millis()-t >= 500;
              
              if( (currentItem==0 || currentItem==1) && !longPush )
                start_running(); // short push on PLAY or PLAY+LDL => start playing
              else if( (currentItem==2 && !longPush) || ((currentItem==0 || currentItem==1) && longPush) )
                fast_fwd(); // long push on PLAY or PLAY+LDL or short push on FFWD => fast forward
              else if( currentItem==2 && longPush )
                test(); // long push on FFWD => go into test mode
            }
        }
    }

  // update buffer fill bar
  if( running || currentScreen==1 )
    {
      static unsigned long next_update = 0;
      if( millis()>next_update )
        {
          byte f = (buffer_size()*20)/BUFSIZE;
          if( f>bufferFillDisplay )
            {
              oled.set1X();
              oled.setCursor(6*bufferFillDisplay, 6);
              oled.write('=');
              bufferFillDisplay++;
            }
          else if( f<bufferFillDisplay )
            {
              oled.set1X();
              bufferFillDisplay--;
              oled.setCursor(6*bufferFillDisplay, 6);
              oled.write(' ');
            }

          next_update = millis()+100;
        }
    }

  // handle software serial if necessary
  if( useSoftSerial ) handleSoftSerial();

  // send buffered data
  unsigned long t;
  static unsigned long sendnext = 0;
  if( !buffer_empty() && (t = micros()) >= sendnext && serialAvailableForWrite()>0 )
    {
      byte b = buffer_get();
#if DEBUG==0
     writeSerial(b);
#endif
      if( show_stats=='Y' )
        {
          if( data_num_chars==0 ) data_start_time = millis();
          data_stop_time = millis();
          data_num_chars++;
        }

      if( (b==10) && currentItem==1 ) 
        {
          // sending newline and line-delay enabled => add delay
          sendnext = t + LDL * 1000l + long(SERIAL_BUFFER_SIZE-serialAvailableForWrite()) * UPC;

          // reduce buffer setpoint (will cause PID to reduce motor speed)
          if( UPC < 10000 )
            {
              buffer_len_setpoint -= 10;
              if( buffer_len_setpoint<25 ) buffer_len_setpoint = 25;
            }
#if DEBUG>0
          Serial.println(F("NL"));
#endif
        }
      else
        {
          // either not using line delay or no newline
          sendnext = t + UPC;

          // slowly increase buffer setpoint for non-newline characters
          if( buffer_len_setpoint<buffer_len_setpoint_max ) buffer_len_setpoint += 0.5;
        }
      
      // stop sending when buffer is empty and motor is no longer running
      if( buffer_empty() && !running )
        {
          send_data_stop();
#if DEBUG>0
          Serial.println(F("BUFFER EMPTY"));
#endif
          if( show_stats=='Y' && error==0 && data_num_chars>0 && data_stop_time>data_start_time )
            {
              showStats();
              show_menu();
            }
        }
    }
}


// -------------------------------------------------------------------------------------------------
//                                        Compare mode functions
// -------------------------------------------------------------------------------------------------


// MITS 4k BASIC 3.2 (binary):
// http://altairclone.com/downloads/basic/Paper%20Tape%20and%20Cassette/4K%20BASIC%20Ver%203-2.tap
#include "basic.h"

// Lunar Lander BASIC program (ASCII):
// http://altairclone.com/downloads/basic/BASIC%20Programs/4K%20BASIC/lander4k.bas
#include "lunar.h"

// Tic-Tac-Toe BASIC program (ASCII):
// http://altairclone.com/downloads/basic/BASIC%20Programs/4K%20BASIC/tictac4k.bas
#include "tictactoe.h"  
  
// Wumpus BASIC program (ASCII):
// http://altairclone.com/downloads/basic/BASIC%20Programs/4K%20BASIC/wumpus4k.bas
#include "wumpus.h"  
  

inline int compare_data_size(int prog)
{
  switch(prog)
    {
#if COMPARE_PROG_SET==1
    case 'B'  : return sizeof(basic);
    case 'L'  : return sizeof(lunar);
#elif COMPARE_PROG_SET==2
    case 'W'  : return sizeof(wumpus);
#elif COMPARE_PROG_SET==3
    case 'T'  : return sizeof(tictac);
#endif
    default   : return 0;
    }
}

  
inline byte compare_data_read(int prog, int i)
{
  switch(prog)
    {
#if COMPARE_PROG_SET==1
    case 'B'  : return pgm_read_byte_near(basic + i);
    case 'L'  : return pgm_read_byte_near(lunar + i);
#elif COMPARE_PROG_SET==2
    case 'W'  : return pgm_read_byte_near(wumpus + i);
#elif COMPARE_PROG_SET==2
    case 'T'  : return pgm_read_byte_near(tictac + i);
#endif
    default   : return 0xFF;
    }
}
