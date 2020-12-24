// IMPORTANT: LIBRARY MUST BE SPECIFICALLY CONFIGURED FOR EITHER TFT SHIELD
// OR BREAKOUT BOARD USAGE.  SEE RELEVANT COMMENTS IN TFTLCD_ILI932x.h

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#if defined(__SAM3X8E__)
#include <include/pio.h>
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif
#include "TFTLCD_ILI932x.h"
#include "pin_magic.h"
#include "pins_arduino.h"
#include "wiring_private.h"

//#define TFTWIDTH   320
//#define TFTHEIGHT  480

#define TFTWIDTH 240
#define TFTHEIGHT 320

#include "registers.h"

// Constructor for breakout board (configurable LCD control lines).
// Can still use this w/shield, but parameters are ignored.
TFTLCD_ILI932x::TFTLCD_ILI932x(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd,
                                 uint8_t reset)
    : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {

#ifndef USE_ADAFRUIT_SHIELD_PINOUT
  // Convert pin numbers to registers and bitmasks
  _reset = reset;
#ifdef __AVR__
  csPort = portOutputRegister(digitalPinToPort(cs));
  cdPort = portOutputRegister(digitalPinToPort(cd));
  wrPort = portOutputRegister(digitalPinToPort(wr));
  rdPort = portOutputRegister(digitalPinToPort(rd));
#endif
#if defined(__SAM3X8E__)
  csPort = digitalPinToPort(cs);
  cdPort = digitalPinToPort(cd);
  wrPort = digitalPinToPort(wr);
  rdPort = digitalPinToPort(rd);
#endif
  csPinSet = digitalPinToBitMask(cs);
  cdPinSet = digitalPinToBitMask(cd);
  wrPinSet = digitalPinToBitMask(wr);
  rdPinSet = digitalPinToBitMask(rd);
  csPinUnset = ~csPinSet;
  cdPinUnset = ~cdPinSet;
  wrPinUnset = ~wrPinSet;
  rdPinUnset = ~rdPinSet;
#ifdef __AVR__
  *csPort |= csPinSet; // Set all control bits to HIGH (idle)
  *cdPort |= cdPinSet; // Signals are ACTIVE LOW
  *wrPort |= wrPinSet;
  *rdPort |= rdPinSet;
#endif
#if defined(__SAM3X8E__)
  csPort->PIO_SODR |= csPinSet; // Set all control bits to HIGH (idle)
  cdPort->PIO_SODR |= cdPinSet; // Signals are ACTIVE LOW
  wrPort->PIO_SODR |= wrPinSet;
  rdPort->PIO_SODR |= rdPinSet;
#endif
  pinMode(cs, OUTPUT); // Enable outputs
  pinMode(cd, OUTPUT);
  pinMode(wr, OUTPUT);
  pinMode(rd, OUTPUT);
  if (reset) {
    digitalWrite(reset, HIGH);
    pinMode(reset, OUTPUT);
  }
#endif

  init();
}

// Constructor for shield (fixed LCD control lines)
TFTLCD_ILI932x::TFTLCD_ILI932x(void) : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {
  init();
}

// Initialization common to both shield & breakout configs
void TFTLCD_ILI932x::init(void) {

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  CS_IDLE; // Set all control bits to idle state
  WR_IDLE;
  RD_IDLE;
  CD_DATA;
  digitalWrite(5, HIGH); // Reset line
  pinMode(A3, OUTPUT);   // Enable outputs
  pinMode(A2, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(5, OUTPUT);
#endif

  setWriteDir(); // Set up LCD data port(s) for WRITE operations

  rotation = 0;
  cursor_y = cursor_x = 0;
  textcolor = 0xFFFF;
  _width = TFTWIDTH;
  _height = TFTHEIGHT;
}

// Initialization command tables for different LCD controllers
#define TFTLCD_DELAY 0xFF
static const uint16_t ILI932x_regValues[] PROGMEM = {
    ILI932X_START_OSC,
    0x0001, // Start oscillator
    TFTLCD_DELAY,
    50, // 50 millisecond delay
    ILI932X_DRIV_OUT_CTRL,
    0x0100,
    ILI932X_DRIV_WAV_CTRL,
    0x0700,
    ILI932X_ENTRY_MOD,
    0x1030,
    ILI932X_RESIZE_CTRL,
    0x0000,
    ILI932X_DISP_CTRL2,
    0x0202,
    ILI932X_DISP_CTRL3,
    0x0000,
    ILI932X_DISP_CTRL4,
    0x0000,
    ILI932X_RGB_DISP_IF_CTRL1,
    0x0,
    ILI932X_FRM_MARKER_POS,
    0x0,
    ILI932X_RGB_DISP_IF_CTRL2,
    0x0,
    ILI932X_POW_CTRL1,
    0x0000,
    ILI932X_POW_CTRL2,
    0x0007,
    ILI932X_POW_CTRL3,
    0x0000,
    ILI932X_POW_CTRL4,
    0x0000,
    TFTLCD_DELAY,
    200,
    ILI932X_POW_CTRL1,
    0x1690,
    ILI932X_POW_CTRL2,
    0x0227,
    TFTLCD_DELAY,
    50,
    ILI932X_POW_CTRL3,
    0x001A,
    TFTLCD_DELAY,
    50,
    ILI932X_POW_CTRL4,
    0x1800,
    ILI932X_POW_CTRL7,
    0x002A,
    TFTLCD_DELAY,
    50,
    ILI932X_GAMMA_CTRL1,
    0x0000,
    ILI932X_GAMMA_CTRL2,
    0x0000,
    ILI932X_GAMMA_CTRL3,
    0x0000,
    ILI932X_GAMMA_CTRL4,
    0x0206,
    ILI932X_GAMMA_CTRL5,
    0x0808,
    ILI932X_GAMMA_CTRL6,
    0x0007,
    ILI932X_GAMMA_CTRL7,
    0x0201,
    ILI932X_GAMMA_CTRL8,
    0x0000,
    ILI932X_GAMMA_CTRL9,
    0x0000,
    ILI932X_GAMMA_CTRL10,
    0x0000,
    ILI932X_GRAM_HOR_AD,
    0x0000,
    ILI932X_GRAM_VER_AD,
    0x0000,
    ILI932X_HOR_START_AD,
    0x0000,
    ILI932X_HOR_END_AD,
    0x00EF,
    ILI932X_VER_START_AD,
    0X0000,
    ILI932X_VER_END_AD,
    0x013F,
    ILI932X_GATE_SCAN_CTRL1,
    0xA700, // Driver Output Control (R60h)
    ILI932X_GATE_SCAN_CTRL2,
    0x0003, // Driver Output Control (R61h)
    ILI932X_GATE_SCAN_CTRL3,
    0x0000, // Driver Output Control (R62h)
    ILI932X_PANEL_IF_CTRL1,
    0X0010, // Panel Interface Control 1 (R90h)
    ILI932X_PANEL_IF_CTRL2,
    0X0000,
    ILI932X_PANEL_IF_CTRL3,
    0X0003,
    ILI932X_PANEL_IF_CTRL4,
    0X1100,
    ILI932X_PANEL_IF_CTRL5,
    0X0000,
    ILI932X_PANEL_IF_CTRL6,
    0X0000,
    ILI932X_DISP_CTRL1,
    0x0133, // Main screen turn on
};

void TFTLCD_ILI932x::begin() {
  uint8_t i = 0;

  reset();

  delay(200);
  uint16_t a, d;
  CS_ACTIVE;
  while (i < sizeof(ILI932x_regValues) / sizeof(uint16_t)) {
    a = pgm_read_word(&ILI932x_regValues[i++]);
    d = pgm_read_word(&ILI932x_regValues[i++]);
    if (a == TFTLCD_DELAY)
      delay(d);
    else
      writeRegister16(a, d);
  }
  setRotation(rotation);
  setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
}

void TFTLCD_ILI932x::reset(void) {

  CS_IDLE;
  //  CD_DATA;
  WR_IDLE;
  RD_IDLE;

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
  digitalWrite(5, LOW);
  delay(2);
  digitalWrite(5, HIGH);
#else
  if (_reset) {
    digitalWrite(_reset, LOW);
    delay(2);
    digitalWrite(_reset, HIGH);
  }
#endif

  // Data transfer sync
  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00);
  for (uint8_t i = 0; i < 3; i++)
    WR_STROBE; // Three extra 0x00s
  CS_IDLE;
}

// Sets the LCD address window (and address counter, on 932X).
// Relevant to rect/screen fills and H/V lines.  Input coordinates are
// assumed pre-sorted (e.g. x2 >= x1).
void TFTLCD_ILI932x::setAddrWindow(int x1, int y1, int x2, int y2) {
  CS_ACTIVE;
  // Values passed are in current (possibly rotated) coordinate
  // system.  932X requires hardware-native coords regardless of
  // MADCTL, so rotate inputs as needed.  The address counter is
  // set to the top-left corner -- although fill operations can be
  // done in any direction, the current screen rotation is applied
  // because some users find it disconcerting when a fill does not
  // occur top-to-bottom.
  int x, y, t;
  switch (rotation) {
  default:
    x = x1;
    y = y1;
    break;
  case 1:
    t = y1;
    y1 = x1;
    x1 = TFTWIDTH - 1 - y2;
    y2 = x2;
    x2 = TFTWIDTH - 1 - t;
    x = x2;
    y = y1;
    break;
  case 2:
    t = x1;
    x1 = TFTWIDTH - 1 - x2;
    x2 = TFTWIDTH - 1 - t;
    t = y1;
    y1 = TFTHEIGHT - 1 - y2;
    y2 = TFTHEIGHT - 1 - t;
    x = x2;
    y = y2;
    break;
  case 3:
    t = x1;
    x1 = y1;
    y1 = TFTHEIGHT - 1 - x2;
    x2 = y2;
    y2 = TFTHEIGHT - 1 - t;
    x = x1;
    y = y2;
    break;
  }
  writeRegister16(0x0050, x1); // Set address window
  writeRegister16(0x0051, x2);
  writeRegister16(0x0052, y1);
  writeRegister16(0x0053, y2);
  writeRegister16(0x0020, x); // Set address counter to top left
  writeRegister16(0x0021, y);
  CS_IDLE;
}

// Fast block fill operation for fillScreen, fillRect, H/V line, etc.
// Requires setAddrWindow() has previously been called to set the fill
// bounds.  'len' is inclusive, MUST be >= 1.
void TFTLCD_ILI932x::flood(uint16_t color, uint32_t len) {
  uint16_t blocks;
  uint8_t i, hi = color >> 8, lo = color;

  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00); // High byte of GRAM register...
  write8(0x22); // Write data to GRAM

  // Write first pixel normally, decrement counter by 1
  CD_DATA;
  write8(hi);
  write8(lo);
  len--;

  blocks = (uint16_t)(len / 64); // 64 pixels/block
  if (hi == lo) {
    // High and low bytes are identical.  Leave prior data
    // on the port(s) and just toggle the write strobe.
    while (blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        WR_STROBE;
        WR_STROBE;
        WR_STROBE;
        WR_STROBE; // 2 bytes/pixel
        WR_STROBE;
        WR_STROBE;
        WR_STROBE;
        WR_STROBE; // x 4 pixels
      } while (--i);
    }
    // Fill any remaining pixels (1 to 64)
    for (i = (uint8_t)len & 63; i--;) {
      WR_STROBE;
      WR_STROBE;
    }
  } else {
    while (blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        write8(hi);
        write8(lo);
        write8(hi);
        write8(lo);
        write8(hi);
        write8(lo);
        write8(hi);
        write8(lo);
      } while (--i);
    }
    for (i = (uint8_t)len & 63; i--;) {
      write8(hi);
      write8(lo);
    }
  }
  CS_IDLE;
}

void TFTLCD_ILI932x::drawFastHLine(int16_t x, int16_t y, int16_t length,
                                    uint16_t color) {
  int16_t x2;
  return;

  // Initial off-screen clipping
  if ((length <= 0) || (y < 0) || (y >= _height) || (x >= _width) ||
      ((x2 = (x + length - 1)) < 0))
    return;

  if (x < 0) { // Clip left
    length += x;
    x = 0;
  }
  if (x2 >= _width) { // Clip right
    x2 = _width - 1;
    length = x2 - x + 1;
  }

  setAddrWindow(x, y, x2, y);
  flood(color, length);
  setAddrWindow(0, 0, _width - 1, _height - 1);
}

void TFTLCD_ILI932x::drawFastVLine(int16_t x, int16_t y, int16_t length,
                                    uint16_t color) {
  int16_t y2;

  // Initial off-screen clipping
  if ((length <= 0) || (x < 0) || (x >= _width) || (y >= _height) ||
      ((y2 = (y + length - 1)) < 0))
    return;
  if (y < 0) { // Clip top
    length += y;
    y = 0;
  }
  if (y2 >= _height) { // Clip bottom
    y2 = _height - 1;
    length = y2 - y + 1;
  }

  setAddrWindow(x, y, x, y2);
  flood(color, length);
  setAddrWindow(0, 0, _width - 1, _height - 1);
}

void TFTLCD_ILI932x::fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h,
                               uint16_t fillcolor) {
  int16_t x2, y2;

  // Initial off-screen clipping
  if ((w <= 0) || (h <= 0) || (x1 >= _width) || (y1 >= _height) ||
      ((x2 = x1 + w - 1) < 0) || ((y2 = y1 + h - 1) < 0))
    return;
  if (x1 < 0) { // Clip left
    w += x1;
    x1 = 0;
  }
  if (y1 < 0) { // Clip top
    h += y1;
    y1 = 0;
  }
  if (x2 >= _width) { // Clip right
    x2 = _width - 1;
    w = x2 - x1 + 1;
  }
  if (y2 >= _height) { // Clip bottom
    y2 = _height - 1;
    h = y2 - y1 + 1;
  }

  setAddrWindow(x1, y1, x2, y2);
  flood(fillcolor, (uint32_t)w * (uint32_t)h);
  setAddrWindow(0, 0, _width - 1, _height - 1);
}

void TFTLCD_ILI932x::fillScreen(uint16_t color) {
  // For the 932X, a full-screen address window is already the default
  // state, just need to set the address pointer to the top-left corner.
  // Although we could fill in any direction, the code uses the current
  // screen rotation because some users find it disconcerting when a
  // fill does not occur top-to-bottom.
  uint16_t x, y;
  switch (rotation) {
  default:
    x = 0;
    y = 0;
    break;
  case 1:
    x = TFTWIDTH - 1;
    y = 0;
    break;
  case 2:
    x = TFTWIDTH - 1;
    y = TFTHEIGHT - 1;
    break;
  case 3:
    x = 0;
    y = TFTHEIGHT - 1;
    break;
  }
  CS_ACTIVE;
  writeRegister16(0x0020, x);
  writeRegister16(0x0021, y);
  flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}

void TFTLCD_ILI932x::drawPixel(int16_t x, int16_t y, uint16_t color) {

  // Clip
  if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    return;

  CS_ACTIVE;
  int16_t t;
  switch (rotation) {
  case 1:
    t = x;
    x = TFTWIDTH - 1 - y;
    y = t;
    break;
  case 2:
    x = TFTWIDTH - 1 - x;
    y = TFTHEIGHT - 1 - y;
    break;
  case 3:
    t = x;
    x = y;
    y = TFTHEIGHT - 1 - t;
    break;
  }
  writeRegister16(0x0020, x);
  writeRegister16(0x0021, y);
  writeRegister16(0x0022, color);
  CS_IDLE;
}

// Issues 'raw' an array of 16-bit color values to the LCD; used
// externally by BMP examples.  Assumes that setWindowAddr() has
// previously been set to define the bounds.  Max 255 pixels at
// a time (BMP examples read in small chunks due to limited RAM).
void TFTLCD_ILI932x::pushColors(uint16_t *data, uint8_t len, boolean first) {
  uint16_t color;
  uint8_t hi, lo;
  CS_ACTIVE;
  if (first == true) { // Issue GRAM write command only on first call
    CD_COMMAND;
    write8(0x00);
  }
  CD_DATA;
  while (len--) {
    color = *data++;
    hi = color >> 8; // Don't simplify or merge these
    lo = color;      // lines, there's macro shenanigans
    write8(hi);      // going on.
    write8(lo);
  }
  CS_IDLE;
}

void TFTLCD_ILI932x::setRotation(uint8_t x) {

  // Call parent rotation func first -- sets up rotation flags, etc.
  Adafruit_GFX::setRotation(x);
  // Then perform hardware-specific rotation operations...

  CS_ACTIVE;

  uint16_t t;
  switch (rotation) {
  default:
    t = 0x1030;
    break;
  case 1:
    t = 0x1028;
    break;
  case 2:
    t = 0x1000;
    break;
  case 3:
    t = 0x1018;
    break;
  }
  writeRegister16(0x0003, t); // MADCTL
  // For 932X, init default full-screen address window:
  setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
}

#ifdef read8isFunctionalized
#define read8(x) x = read8fn()
#endif

// Because this function is used infrequently, it configures the ports for
// the read operation, reads the data, then restores the ports to the write
// configuration.  Write operations happen a LOT, so it's advantageous to
// leave the ports in that state as a default.
uint16_t TFTLCD_ILI932x::readPixel(int16_t x, int16_t y) {

  if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    return 0;

  CS_ACTIVE;
  uint8_t hi, lo;
  int16_t t;
  switch (rotation) {
  case 1:
    t = x;
    x = TFTWIDTH - 1 - y;
    y = t;
    break;
  case 2:
    x = TFTWIDTH - 1 - x;
    y = TFTHEIGHT - 1 - y;
    break;
  case 3:
    t = x;
    x = y;
    y = TFTHEIGHT - 1 - t;
    break;
  }
  writeRegister16(0x0020, x);
  writeRegister16(0x0021, y);
  // Inexplicable thing: sometimes pixel read has high/low bytes
  // reversed.  A second read fixes this.  Unsure of reason.  Have
  // tried adjusting timing in read8() etc. to no avail.
  for (uint8_t pass = 0; pass < 2; pass++) {
    CD_COMMAND;
    write8(0x00);
    write8(0x22); // Read data from GRAM
    CD_DATA;
    setReadDir(); // Set up LCD data port(s) for READ operations
    read8(hi);    // First 2 bytes back are a dummy read
    read8(hi);
    read8(hi); // Bytes 3, 4 are actual pixel value
    read8(lo);
    setWriteDir(); // Restore LCD data port(s) to WRITE configuration
  }
  CS_IDLE;
  return ((uint16_t)hi << 8) | lo;
}

// Ditto with the read/write port directions, as above.
uint16_t TFTLCD_ILI932x::readID(void) {
  uint16_t id;

  // retry a bunch!
  for (int i = 0; i < 5; i++) {
    id = (uint16_t)readReg(0xD3);
    delayMicroseconds(50);
    if (id == 0x9341) {
      return id;
    }
  }

  uint8_t hi, lo;

  /*
  for (uint8_t i=0; i<128; i++) {
    Serial.print("$"); Serial.print(i, HEX);
    Serial.print(" = 0x"); Serial.println(readReg(i), HEX);
  }
  */

  if (readReg(0x04) == 0x8000) { // eh close enough
    // setc!
    /*
      Serial.println("!");
      for (uint8_t i=0; i<254; i++) {
      Serial.print("$"); Serial.print(i, HEX);
      Serial.print(" = 0x"); Serial.println(readReg(i), HEX);
      }
    */
    writeRegister24(HX8357D_SETC, 0xFF8357);
    delay(300);
    // Serial.println(readReg(0xD0), HEX);
    if (readReg(0xD0) == 0x990000) {
      return 0x8357;
    }
  }

  CS_ACTIVE;
  CD_COMMAND;
  write8(0x00);
  WR_STROBE;    // Repeat prior byte (0x00)
  setReadDir(); // Set up LCD data port(s) for READ operations
  CD_DATA;
  read8(hi);
  read8(lo);
  setWriteDir(); // Restore LCD data port(s) to WRITE configuration
  CS_IDLE;

  id = hi;
  id <<= 8;
  id |= lo;
  return id;
}

uint32_t TFTLCD_ILI932x::readReg(uint8_t r) {
  uint32_t id;
  uint8_t x;

  // try reading register #4
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  setReadDir(); // Set up LCD data port(s) for READ operations
  CD_DATA;
  delayMicroseconds(50);
  read8(x);
  id = x;   // Do not merge or otherwise simplify
  id <<= 8; // these lines.  It's an unfortunate
  read8(x);
  id |= x;  // shenanigans that are going on.
  id <<= 8; // these lines.  It's an unfortunate
  read8(x);
  id |= x;  // shenanigans that are going on.
  id <<= 8; // these lines.  It's an unfortunate
  read8(x);
  id |= x; // shenanigans that are going on.
  CS_IDLE;
  setWriteDir(); // Restore LCD data port(s) to WRITE configuration

  // Serial.print("Read $"); Serial.print(r, HEX);
  // Serial.print(":\t0x"); Serial.println(id, HEX);
  return id;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t TFTLCD_ILI932x::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// For I/O macros that were left undefined, declare function
// versions that reference the inline macros just once:

#ifndef write8
void TFTLCD_ILI932x::write8(uint8_t value) { write8inline(value); }
#endif

#ifdef read8isFunctionalized
uint8_t TFTLCD_ILI932x::read8fn(void) {
  uint8_t result;
  read8inline(result);
  return result;
}
#endif

#ifndef setWriteDir
void TFTLCD_ILI932x::setWriteDir(void) { setWriteDirInline(); }
#endif

#ifndef setReadDir
void TFTLCD_ILI932x::setReadDir(void) { setReadDirInline(); }
#endif

#ifndef writeRegister8
void TFTLCD_ILI932x::writeRegister8(uint8_t a, uint8_t d) {
  writeRegister8inline(a, d);
}
#endif

#ifndef writeRegister16
void TFTLCD_ILI932x::writeRegister16(uint16_t a, uint16_t d) {
  writeRegister16inline(a, d);
}
#endif

#ifndef writeRegisterPair
void TFTLCD_ILI932x::writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d) {
  writeRegisterPairInline(aH, aL, d);
}
#endif

void TFTLCD_ILI932x::writeRegister24(uint8_t r, uint32_t d) {
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  CD_DATA;
  delayMicroseconds(10);
  write8(d >> 16);
  delayMicroseconds(10);
  write8(d >> 8);
  delayMicroseconds(10);
  write8(d);
  CS_IDLE;
}

void TFTLCD_ILI932x::writeRegister32(uint8_t r, uint32_t d) {
  CS_ACTIVE;
  CD_COMMAND;
  write8(r);
  CD_DATA;
  delayMicroseconds(10);
  write8(d >> 24);
  delayMicroseconds(10);
  write8(d >> 16);
  delayMicroseconds(10);
  write8(d >> 8);
  delayMicroseconds(10);
  write8(d);
  CS_IDLE;
}
