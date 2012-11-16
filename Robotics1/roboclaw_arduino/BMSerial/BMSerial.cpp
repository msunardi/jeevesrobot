/*
Modified version of SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

-- Added support for half duplex single pin operation
-- Added readln,readdec,readhex,readbin
-- Added read timeout option
-- Fixed RX Stop Bit widths(because the bits are read nominally from the center of each bit, the bit width should only be half the normal width for the stop bit)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
//
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "BMSerial.h"

//
// Lookup table
//
typedef struct _DELAY_TABLE
{
	long baud;
	unsigned short rx_delay_centering;
	unsigned short rx_delay_intrabit;
	unsigned short rx_delay_stopbit;
	unsigned short tx_delay;
} DELAY_TABLE;

#if F_CPU == 16000000

static const DELAY_TABLE PROGMEM table[] = 
{
  //  baud    rxcenter   rxintra    rxstop    tx
  { 115200,   1,         17,        8,       12,    },
  { 57600,    10,        37,        18,       33,    },
  { 38400,    25,        57,        28,       54,    },
  { 31250,    31,        70,        35,       68,    },
  { 28800,    34,        77,        38,       74,    },
  { 19200,    54,        117,       58,      114,   },
  { 14400,    74,        156,       78,      153,   },
  { 9600,     114,       236,       118,      233,   },
  { 4800,     233,       474,       237,      471,   },
  { 2400,     471,       950,       475,      947,   },
  { 1200,     947,       1902,      851,     1899,  },
  { 300,      3804,      7617,      3808,     7614,  },
};

const int XMIT_START_ADJUSTMENT = 5;

#elif F_CPU == 8000000

static const DELAY_TABLE table[] PROGMEM = 
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   1,          5,         2,      3,      },
  { 57600,    1,          15,        7,     13,     },
  { 38400,    2,          25,        13,     23,     },
  { 31250,    7,          32,        16,     29,     },
  { 28800,    11,         35,        17,     32,     },
  { 19200,    20,         55,        27,     52,     },
  { 14400,    30,         75,        37,     72,     },
  { 9600,     50,         114,       57,     112,    },
  { 4800,     110,        233,       116,    230,    },
  { 2400,     229,        472,       236,    469,    },
  { 1200,     467,        948,       474,    945,    },
  { 300,      1895,       3805,      1902,   3802,   },
};

const int XMIT_START_ADJUSTMENT = 4;

#elif F_CPU == 20000000

// 20MHz support courtesy of the good people at macegr.com.
// Thanks, Garrett!

static const DELAY_TABLE PROGMEM table[] =
{
  //  baud    rxcenter    rxintra    rxstop  tx
  { 115200,   3,          21,        10,     18,     },
  { 57600,    20,         43,        21,     41,     },
  { 38400,    37,         73,        36,     70,     },
  { 31250,    45,         89,        44,     88,     },
  { 28800,    46,         98,        49,     95,     },
  { 19200,    71,         148,       74,    145,    },
  { 14400,    96,         197,       98,    194,    },
  { 9600,     146,        297,       148,    294,    },
  { 4800,     296,        595,       292,    592,    },
  { 2400,     592,        1189,      594,   1186,   },
  { 1200,     1187,       2379,      1184,   2376,   },
  { 300,      4759,       9523,      4761,   9520,   },
};

const int XMIT_START_ADJUSTMENT = 6;

#else

#error This version of BMSerial supports only 20, 16 and 8MHz processors

#endif

//
// Statics
//
BMSerial *BMSerial::active_object = 0;
char BMSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t BMSerial::_receive_buffer_tail = 0;
volatile uint8_t BMSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
inline void DebugPulse(uint8_t pin, uint8_t count)
{
#if _DEBUG
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
#endif
}

//
// Private methods
//

/* static */ 
inline void BMSerial::tunedDelay(uint16_t delay) { 
	uint8_t tmp=0;
	
	asm volatile("sbiw    %0, 0x01 \n\t"
	  "ldi %1, 0xFF \n\t"
	  "cpi %A0, 0xFF \n\t"
	  "cpc %B0, %1 \n\t"
	  "brne .-10 \n\t"
	  : "+r" (delay), "+a" (tmp)
	  : "0" (delay)
	  );
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool BMSerial::listen()
{
  if (active_object != this)
  {
    _buffer_overflow = false;
    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;
    SREG = oldSREG;
    return true;
  }

  return false;
}

//
// The receive routine called by the interrupt handler
//
void BMSerial::recv()
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=0x1; i; i <<= 1)
    {
      tunedDelay(_rx_delay_intrabit);
      DebugPulse(_DEBUG_PIN2, 1);
      uint8_t noti = ~i;
      if (rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN2, 1);

    if (_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    if ((_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF != _receive_buffer_head) 
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    } 
    else 
    {
#if _DEBUG // for scope: pulse pin as overflow indictator
      DebugPulse(_DEBUG_PIN1, 1);
#endif
      _buffer_overflow = true;
    }
  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}

void BMSerial::tx_pin_write(uint8_t pin_state)
{
	if (pin_state == LOW)
	  *_transmitPortRegister &= ~_transmitBitMask;
	else
	  *_transmitPortRegister |= _transmitBitMask;
}

uint8_t BMSerial::rx_pin_read()
{
	return *_receivePortRegister & _receiveBitMask;
}

int8_t BMSerial::bintoint(uint8_t c)
{
	if (c >= '0' && c <= '1') {
	  return c - '0';
	} else {
	  return -1;    
	}
}

int8_t BMSerial::dectoint(uint8_t c)
{
	if (c >= '0' && c <= '9') {
	  return c - '0';
	} else {
	  return -1;    
	}
}

int8_t BMSerial::hextoint(uint8_t c)
{
	if (c >= '0' && c <= '9') {
	  return c - '0';
	} else if (c >= 'a' && c <= 'f') {
	  return c - 'a' + 10;
	} else if (c >= 'A' && c <= 'F') {
	  return c - 'A' + 10;
	} else {
	  return -1;    
	}
}

//
// Interrupt handling
//

/* static */
inline void BMSerial::handle_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  BMSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect)
{
  BMSerial::handle_interrupt();
}
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect)
{
  BMSerial::handle_interrupt();
}
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect)
{
  BMSerial::handle_interrupt();
}
#endif

//
// Constructor
//
BMSerial::BMSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
	_rx_delay_centering(0),
	_rx_delay_intrabit(0),
	_rx_delay_stopbit(0),
	_tx_delay(0),
	_buffer_overflow(false),
	_inverse_logic(inverse_logic)
{
	setTX(transmitPin);
	setRX(receivePin);
}

//
// Destructor
//
BMSerial::~BMSerial()
{
	end();
}

void BMSerial::setTX(uint8_t tx)
{
	pinMode(tx, OUTPUT);
	digitalWrite(tx, HIGH);
	_transmitPin = tx;
	_transmitBitMask = digitalPinToBitMask(tx);
	uint8_t port = digitalPinToPort(tx);
	_transmitPortRegister = portOutputRegister(port);
}

void BMSerial::setRX(uint8_t rx)
{
	pinMode(rx, INPUT);
	if (!_inverse_logic)
	  digitalWrite(rx, HIGH);  // pullup for normal logic!
	_receivePin = rx;
	_receiveBitMask = digitalPinToBitMask(rx);
	uint8_t port = digitalPinToPort(rx);
	_receivePortRegister = portInputRegister(port);
}

//
// Public methods
//

void BMSerial::begin(long speed)
{
	if(_receivePin==0 && _transmitPin==1){
		Serial.begin(speed);
		return;
	}
	
	_rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;
	
	for (unsigned i=0; i<sizeof(table)/sizeof(table[0]); ++i)
	{
	  long baud = pgm_read_dword(&table[i].baud);
	  if (baud == speed)
	  {
	    _rx_delay_centering = pgm_read_word(&table[i].rx_delay_centering);
	    _rx_delay_intrabit = pgm_read_word(&table[i].rx_delay_intrabit);
	    _rx_delay_stopbit = pgm_read_word(&table[i].rx_delay_stopbit);
	    _tx_delay = pgm_read_word(&table[i].tx_delay);
	    break;
	  }
	}
	
	// Set up RX interrupts, but only if we have a valid RX baud rate
	if (_rx_delay_stopbit)
	{
		if (digitalPinToPCICR(_receivePin))
		{
			*digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
			*digitalPinToPCMSK(_receivePin) |= _BV(digitalPinToPCMSKbit(_receivePin));
		}
		tunedDelay(_tx_delay); // if we were low this establishes the end
	}

#if _DEBUG
	pinMode(_DEBUG_PIN1, OUTPUT);
	pinMode(_DEBUG_PIN2, OUTPUT);
#endif

	listen();
}

void BMSerial::end()
{
	if(_receivePin==0 && _transmitPin==1){
		Serial.end();
		return;
	}
	
	if (digitalPinToPCMSK(_receivePin))
		*digitalPinToPCMSK(_receivePin) &= ~_BV(digitalPinToPCMSKbit(_receivePin));
}

// Read data from buffer
int BMSerial::read(uint32_t timeout)
{
	if(_receivePin==0 && _transmitPin==1){
		uint32_t start = micros();
		// Empty buffer?
		while(!Serial.available()){
		   if((micros()-start)>timeout)
		      return -1;
		}
		return Serial.read();
	}
	
	if (!is_listening())
		return -1;

	uint32_t start = micros();
	// Empty buffer?
	while(_receive_buffer_head == _receive_buffer_tail){
	   if((micros()-start)>timeout)
	      return -1;
	}

	// Read from "head"
	uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
	_receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
	return d;
}

int BMSerial::available()
{
	if(_receivePin==0 && _transmitPin==1){
		return Serial.available();
	}
	
	if (!is_listening())
		return 0;

	return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

uint32_t BMSerial::readdec(uint32_t timeout,bool ignoreleading)
{
	uint32_t data=0;
	uint16_t index=0;
	
	do{
		int8_t c = read(timeout);
		if(c==-1)
			return -1;
		int16_t i = dectoint(c);
		if(i==-1 && !ignoreleading){
			return data;
		}
		if(i!=-1)
			ignoreleading = false;
		if(!ignoreleading){
			data*=10;
			data+=i;
		}	
	}while(1);
}

uint32_t BMSerial::readhex(uint32_t timeout,bool ignoreleading)
{
	uint32_t data=0;
	uint16_t index=0;
	
	do{
		int8_t c = read(timeout);
		if(c==-1)
			return -1;
		int16_t i = hextoint(c);
		if(i==-1 && !ignoreleading){
			return data;
		}
		if(i!=-1)
			ignoreleading = false;
		if(!ignoreleading){
			data<<=4;
			data|=i;
		}	
	}while(1);
}

uint32_t BMSerial::readbin(uint32_t timeout,bool ignoreleading)
{
	uint32_t data=0;
	uint16_t index=0;
	
	do{
		int8_t c = read(timeout);
		if(c==-1)
			return -1;
		int16_t i = bintoint(c);
		if(i==-1 && !ignoreleading){
			return data;
		}
		if(i!=-1)
			ignoreleading = false;
		if(!ignoreleading){
			data<<=1;
			data|=i;
		}	
	}while(1);
}

uint16_t BMSerial::readln(char *data,uint32_t timeout)
{
	uint16_t index=0;
	
	do{
		int16_t c = read(timeout);
		if(c==-1){
			data[index]=0;
			return -1;
		}
		if(c==13){
			data[index]=0;
			return index;
		}
		data[index++]=c;
	}while(1);
}

int BMSerial::read()
{
	if(_receivePin==0 && _transmitPin==1){
		return Serial.read();
	}
	
	return read(0);
}

size_t BMSerial::write(uint8_t b)
{
	if(_receivePin==0 && _transmitPin==1){
		return Serial.write(b);
	}
	
	if (_tx_delay == 0)
	  return 0;
	
	uint8_t oldSREG = SREG;
	cli();  // turn off interrupts for a clean txmit

	if(_receivePin==_transmitPin){
		pinMode(_transmitPin, OUTPUT);
		digitalWrite(_transmitPin, HIGH);
	}
		
	// Write the start bit
	tx_pin_write(_inverse_logic ? HIGH : LOW);
	tunedDelay(_tx_delay + XMIT_START_ADJUSTMENT);
	
	// Write each of the 8 bits
	if (_inverse_logic)
	{
	  for (byte mask = 0x01; mask; mask <<= 1)
	  {
	    if (b & mask) // choose bit
	      tx_pin_write(LOW); // send 1
	    else
	      tx_pin_write(HIGH); // send 0
	  
	    tunedDelay(_tx_delay);
	  }
	
	  tx_pin_write(LOW); // restore pin to natural state
	}
	else
	{
	  for (byte mask = 0x01; mask; mask <<= 1)
	  {
	    if (b & mask) // choose bit
	      tx_pin_write(HIGH); // send 1
	    else
	      tx_pin_write(LOW); // send 0
	  
	    tunedDelay(_tx_delay);
	  }
	
	  tx_pin_write(HIGH); // restore pin to natural state
	}
	
	if(_receivePin==_transmitPin){
		pinMode(_receivePin, INPUT);
		if (!_inverse_logic)
		  digitalWrite(_receivePin, HIGH);  // pullup for normal logic!
	}
	
	SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);
	
	return 1;
}

#if !defined(cbi)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void BMSerial::enable_timer0(bool enable) 
{
  if (enable)
#if defined(__AVR_ATmega8__)
    sbi(TIMSK, TOIE0);
#else
    sbi(TIMSK0, TOIE0);
#endif
  else 
#if defined(__AVR_ATmega8__)
    cbi(TIMSK, TOIE0);
#else
    cbi(TIMSK0, TOIE0);
#endif
}

void BMSerial::flush()
{
	if(_receivePin==0 && _transmitPin==1){
		Serial.flush();
		return;
	}
	
	if (!is_listening())
		return;

	uint8_t oldSREG = SREG;
	cli();
	_receive_buffer_head = _receive_buffer_tail = 0;
	SREG = oldSREG;
}

int BMSerial::peek()
{
	if(_receivePin==0 && _transmitPin==1){
		return Serial.peek();
	}
	
	if (!is_listening())
		return -1;

	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;

	// Read from "head"
		return _receive_buffer[_receive_buffer_head];
}
