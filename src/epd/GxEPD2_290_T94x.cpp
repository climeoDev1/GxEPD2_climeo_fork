#include "GxEPD2_290_T94x.h"

const unsigned char LUT_DATA1[] = 
    {
        // VS L0
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        // VS L1
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        // VS L2
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        // VS L3
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x01,

        // VS L4
        0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

        // VS L5
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x42,

        0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x41,

        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x01, 0x82, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x01, 0x81, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x01, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved

        0x02, 0x00, 0x00, // FR, XON
        0x22, 0x17, 0x41, 0xA8, 0x32, 0x40,
        // EOPT  VGH   VSH1  VSH2  VSL   VCOM
};

void waitForBusyLow()
{
  pinMode(CONFIG_RM_DISPLAY_BUSY, INPUT);
  while (digitalRead(CONFIG_RM_DISPLAY_BUSY)) // Wait for busy low
  {
    delay(1);
  }
}

void transferCommand(SPIClass &hspi, uint8_t c, std::vector<uint8_t> data = {})
{
  hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CONFIG_RM_DISPLAY_DC, LOW);
  digitalWrite(CONFIG_RM_DISPLAY_CS, LOW);
  hspi.transfer(c);
  digitalWrite(CONFIG_RM_DISPLAY_CS, HIGH);
  digitalWrite(CONFIG_RM_DISPLAY_DC, HIGH);
  hspi.endTransaction();

  if (data.size())
  {
    hspi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CONFIG_RM_DISPLAY_CS, LOW);
    hspi.transfer(data.data(), data.size());
    digitalWrite(CONFIG_RM_DISPLAY_CS, HIGH);
    hspi.endTransaction();
  }
}

void hwReset()
{
  // Hardware reset
  pinMode(CONFIG_RM_DISPLAY_RST, OUTPUT);
  digitalWrite(CONFIG_RM_DISPLAY_RST, LOW);  // RESE# low
  delay(5);                                  // delay 200us
  digitalWrite(CONFIG_RM_DISPLAY_RST, HIGH); // RESE# high
  delay(5);                                  // delay 200us
  waitForBusyLow();
}

void swReset(SPIClass &hspi)
{
  // Software reset
  transferCommand(hspi, 0x12);
  waitForBusyLow();
}

void reset(SPIClass &hspi)
{
  hwReset();
  swReset(hspi);
}

bool fullMode = true;

void initFullUpdate(SPIClass &hspi)
{
  fullMode = true;
  reset(hspi);
  transferCommand(hspi, 0x21, {0x00, 0x80});
  transferCommand(hspi, 0x01, {0x27, 0x01, 0x00});
  transferCommand(hspi, 0x11, {0x03});
  transferCommand(hspi, 0x44, {0x00, 0x0F});
  transferCommand(hspi, 0x45, {0x00, 0x00, 0x27, 0x01});
  transferCommand(hspi, 0x4E, {0x00});
  transferCommand(hspi, 0x4F, {0x00, 0x00});
  transferCommand(hspi, 0x3C, {0x03});
  transferCommand(hspi, 0x03, {0x80});
  transferCommand(hspi, 0x22, {0xB1});
  transferCommand(hspi, 0x21, {0x00, 0x80});
  transferCommand(hspi, 0x20);

  waitForBusyLow();
}

void writeWithMode(SPIClass &hspi,
                   uint8_t mode,
                   const std::vector<uint8_t> &black,
                   const std::vector<uint8_t> &red)
{
  transferCommand(hspi, 0x24, black);
  transferCommand(hspi, 0x26, red);
  transferCommand(hspi, 0x22, {mode});
  transferCommand(hspi, 0x20);
  waitForBusyLow();
}

void initPart(SPIClass &hspi)
{
  fullMode = false;

  std::vector<uint8_t> lutData1;
  std::copy(LUT_DATA1, LUT_DATA1 + 224, std::back_inserter(lutData1));
  transferCommand(hspi, 0x32, lutData1);
  transferCommand(hspi, 0x37, {0xff, 0xff, 0xff, 0xff, 0xff, 0x40, 0x00, 0x00});
  transferCommand(hspi, 0x3C, {0x80});
  transferCommand(hspi, 0x3F, {LUT_DATA1[227]});
  transferCommand(hspi, 0x22, {0xC0});
  transferCommand(hspi, 0x21, {0x00, 0x80});
  transferCommand(hspi, 0x20);
  waitForBusyLow();

  transferCommand(hspi, 0x11, {0x03}); // Y inc, X inc, X data counter upd
  transferCommand(hspi, 0x01, {0x27, 0x01, 0x00});
  transferCommand(hspi, 0x44, {0x00, 0x0F});
  transferCommand(hspi, 0x45, {0x00, 0x00, 0x27, 0x01});
  transferCommand(hspi, 0x4E, {0x00});
  transferCommand(hspi, 0x4F, {0x0, 0x00});
}

void GxEPD2_290_T94x::writeScreen(uint8_t value)
{
    if (xx >= x22)
    {
      xx = x11;
      yy++;
    }
    if (yy >= y22)
    {
      yy = y11;
    }

    int index = yy * WIDTH + xx;
    assert(index < 4736 && "ERROR: Writing data outside the screen buffer");

    currentScreen[index] = value;    
    xx++;
}


GxEPD2_290_T94x::GxEPD2_290_T94x(int16_t cs, int16_t dc, int16_t rst, int16_t busy) :
  GxEPD2_EPD(cs, dc, rst, busy, HIGH, 10000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate),
  currentScreen{std::vector<uint8_t>(4736, 0x00)},
  previousScreen{std::vector<uint8_t>(4736, 0x00)}
{
}

void GxEPD2_290_T94x::clearScreen(uint8_t value)
{
  _writeScreenBuffer(0x24, value); // set current
  refresh(false); // full refresh
  _initial_write = false;
}

void GxEPD2_290_T94x::writeScreenBuffer(uint8_t value)
{
  if (_initial_write) return clearScreen(value);
  _writeScreenBuffer(0x24, value); // set current
}

void GxEPD2_290_T94x::writeScreenBufferAgain(uint8_t value)
{
  _writeScreenBuffer(0x24, value); // set current
}

void GxEPD2_290_T94x::_writeScreenBuffer(uint8_t command, uint8_t value)
{
  if (!_init_display_done) _InitDisplay();
  _setPartialRamArea(0, 0, WIDTH, HEIGHT);

  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    writeScreen(value);
  }
}

void GxEPD2_290_T94x::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImage(0x24, bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_290_T94x::writeImageForFullRefresh(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImage(0x24, bitmap, x, y, w, h, invert, mirror_y, pgm); // set current
}


void GxEPD2_290_T94x::writeImageAgain(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImage(0x24, bitmap, x, y, w, h, invert, mirror_y, pgm); // set current
}

void GxEPD2_290_T94x::_writeImage(uint8_t command, const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  int16_t wb = (w + 7) / 8; // width bytes, bitmaps are padded
  x -= x % 8; // byte boundary
  w = wb * 8; // byte boundary
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_init_display_done) _InitDisplay();
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  _setPartialRamArea(x1, y1, w1, h1);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb, h of bitmap for index!
      int16_t idx = mirror_y ? j + dx / 8 + ((h - 1 - (i + dy))) * wb : j + dx / 8 + (i + dy) * wb;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;    
      writeScreen(data);
    }
  }
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_290_T94x::writeImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImagePart(0x24, bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_290_T94x::writeImagePartAgain(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImagePart(0x24, bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_290_T94x::_writeImagePart(uint8_t command, const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                     int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  if ((w_bitmap < 0) || (h_bitmap < 0) || (w < 0) || (h < 0)) return;
  if ((x_part < 0) || (x_part >= w_bitmap)) return;
  if ((y_part < 0) || (y_part >= h_bitmap)) return;
  int16_t wb_bitmap = (w_bitmap + 7) / 8; // width bytes, bitmaps are padded
  x_part -= x_part % 8; // byte boundary
  w = w_bitmap - x_part < w ? w_bitmap - x_part : w; // limit
  h = h_bitmap - y_part < h ? h_bitmap - y_part : h; // limit
  x -= x % 8; // byte boundary
  w = 8 * ((w + 7) / 8); // byte boundary, bitmaps are padded
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_init_display_done) _InitDisplay();
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  _setPartialRamArea(x1, y1, w1, h1);
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb_bitmap, h_bitmap of bitmap for index!
      int16_t idx = mirror_y ? x_part / 8 + j + dx / 8 + ((h_bitmap - 1 - (y_part + i + dy))) * wb_bitmap : x_part / 8 + j + dx / 8 + (y_part + i + dy) * wb_bitmap;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;
      writeScreen(data);
    }
  }
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_290_T94x::writeImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    writeImage(black, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_290_T94x::writeImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    writeImagePart(black, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_290_T94x::writeNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1)
  {
    writeImage(data1, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_290_T94x::drawImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImageAgain(bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_290_T94x::drawImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                   int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
  writeImagePartAgain(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_290_T94x::drawImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    drawImage(black, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_290_T94x::drawImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
                                   int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    drawImagePart(black, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_290_T94x::drawNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1)
  {
    drawImage(data1, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_290_T94x::refresh(bool partial_update_mode)
{
  if (partial_update_mode) refresh(0, 0, WIDTH, HEIGHT);
  else
  {
    _Update_Full();
    _initial_refresh = false; // initial full update done
  }
}

void GxEPD2_290_T94x::refresh(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if (_initial_refresh) return refresh(false); // initial update needs be full update
  // intersection with screen
  int16_t w1 = x < 0 ? w + x : w; // reduce
  int16_t h1 = y < 0 ? h + y : h; // reduce
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  w1 = x1 + w1 < int16_t(WIDTH) ? w1 : int16_t(WIDTH) - x1; // limit
  h1 = y1 + h1 < int16_t(HEIGHT) ? h1 : int16_t(HEIGHT) - y1; // limit
  if ((w1 <= 0) || (h1 <= 0)) return;
  // make x1, w1 multiple of 8
  w1 += x1 % 8;
  if (w1 % 8 > 0) w1 += 8 - w1 % 8;
  x1 -= x1 % 8;
  _setPartialRamArea(x1, y1, w1, h1);
  _Update_Part();
}

void GxEPD2_290_T94x::powerOff()
{
  _PowerOff();
}

void GxEPD2_290_T94x::hibernate()
{
  _PowerOff();
  if (_rst >= 0)
  {
    _writeCommand(0x10); // deep sleep mode
    _writeData(0x1);     // enter deep sleep
    _hibernating = true;
    _init_display_done = false;
  }
}

uint8_t toX1(int x) 
{
  return x;
}

uint8_t toX2(int x, int w)
{
  return (x + w);
}

uint8_t toY1(int y)
{
  return y;
}

uint8_t toY2(int y, int h)
{
  return y + h;
}

void GxEPD2_290_T94x::_setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  x11 = toX1(x);
  x22 = toX2(x, w);
  y11 = toY1(y);
  y22 = toY2(y, h);
  xx = x11;
  yy = y11;
}

void GxEPD2_290_T94x::_PowerOn()
{
  _power_is_on = true;
}

void GxEPD2_290_T94x::_PowerOff()
{
  _power_is_on = false;
  _using_partial_mode = false;
}

void GxEPD2_290_T94x::_InitDisplay()
{
  reset(*_pSPIx);
  initFullUpdate(*_pSPIx);
  std::vector<uint8_t> whiteData(4736, 0xFF); 
  std::vector<uint8_t> blackData(4736, 0x00);
  writeWithMode(*_pSPIx, 0xC7, whiteData, blackData); 
  _init_display_done = true;
}

void GxEPD2_290_T94x::_Update_Full()
{
  reset(*_pSPIx);
  initFullUpdate(*_pSPIx);
  std::fill(previousScreen.begin(), previousScreen.end(), 0);
  writeWithMode(*_pSPIx, 0xC7, currentScreen, previousScreen);
  previousScreen = currentScreen;
  _waitWhileBusy("_Update_Full", full_refresh_time);
}

void GxEPD2_290_T94x::_Update_Part()
{
  _PowerOn();
  reset(*_pSPIx);
  initPart(*_pSPIx);
  waitForBusyLow();  
  transferCommand(*_pSPIx, 0x24, currentScreen);
  transferCommand(*_pSPIx, 0x26, previousScreen);
  previousScreen = currentScreen;
  transferCommand(*_pSPIx, 0x22, {0x0C});
  transferCommand(*_pSPIx, 0x20, {});
  _waitWhileBusy("_Update_Part", partial_refresh_time);
  transferCommand(*_pSPIx, 0x10, {0x01});
}
