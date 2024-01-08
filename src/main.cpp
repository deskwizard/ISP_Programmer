// **** CUSTOM 'arduino-as-isp', ONLY USE WITH PARTICULAR THIS HARDWARE ****//
//
//  - Reset output polarity inverted for 2N7000 on reset line
//  - LED pins changed
//  - Added CD4053B inhibit pin (outputs HI-Z when not programming)
//  - Added target power enable
//  - Baudrate changed to 115200 (Use 'AVR ISP' as programmer type)
//
//    MCU fuse read test (example):
//    avrdude -v -p atmega328p -c stk500v1 -P /dev/ttyUSB0
//
// TODO:
//    - The error LED or it's code is dodgy AF, not sure what's happening there.

#include "SPI.h"
#include "defines.h"
#include "stk500v1_defs.h"

int16_t errorCount = 0;
bool programming = false;

uint16_t bufferIndex; // Address for reading and writing
uint8_t buffer[256];  // Global buffer

// Heartbeat so you can tell the software is running.
uint8_t heartbeatValue = 128;
int8_t heartbeatDelta = 8;

bool rst_active_high = false;

void setup() {

  Serial.begin(BAUDRATE);

  pinMode(ENABLE_PGM, OUTPUT);

  pinMode(TARGET_PWR, OUTPUT);
  digitalWrite(TARGET_PWR, OFF);

  pinMode(LED_OK, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  digitalWrite(LED_OK, ON);
  digitalWrite(LED_ERROR, OFF);

  pinMode(LED_HB, OUTPUT);
}

void loop(void) {
  /*
    // Is there an errorCount?
    if (errorCount) {
      digitalWrite(LED_ERR, HIGH);
    } else {
      digitalWrite(LED_ERR, LOW);
    }
   */
  // Handle the the LEDs
  handleLEDs();

  if (Serial.available()) {
    avrisp();
  }
}

void handleLEDs() {

  uint32_t currentMillis = millis();
  static uint32_t lastHBLedMillis = 0;
  static uint32_t lastProgLedMillis = 0;
  static bool progLedState;

  if ((uint32_t)(currentMillis - lastProgLedMillis) >= LED_PULSE_TIME &&
      programming) {

    progLedState = !progLedState;
    digitalWrite(LED_OK, !progLedState);
    digitalWrite(LED_ERROR, !progLedState);
    lastProgLedMillis = currentMillis;
  }

  if ((uint32_t)(currentMillis - lastHBLedMillis) >= (LED_PULSE_TIME / 2)) {

    if (heartbeatValue > LED_MAX_PWM || heartbeatValue < LED_MIN_PWM) {
      heartbeatDelta = -heartbeatDelta;
    }

    heartbeatValue += heartbeatDelta;
    analogWrite(LED_HB, heartbeatValue);
    lastHBLedMillis = currentMillis;
  }
}

void reset_target(bool reset) {
  if ((reset && rst_active_high) || (!reset && !rst_active_high)) {
    digitalWrite(RESET, LOW);
  } else {
    digitalWrite(RESET, HIGH);
  }
}

uint8_t getChar() {
  while (!Serial.available())
    ;
  return Serial.read();
}

void fill(uint16_t n) {
  for (uint16_t x = 0; x < n; x++) {
    buffer[x] = getChar();
  }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  SPI.transfer(a);
  SPI.transfer(b);
  SPI.transfer(c);
  return SPI.transfer(d);
}

void replyEmpty() {

  if (Sync_CRC_EOP == getChar()) {
    Serial.write(Resp_STK_INSYNC);
    Serial.write(Resp_STK_OK);
  } else {
    errorCount++;
    Serial.write(Resp_STK_NOSYNC);
  }
}

void reply(uint8_t b) {
  if (Sync_CRC_EOP == getChar()) {
    Serial.write(Resp_STK_INSYNC);
    Serial.write(b);
    Serial.write(Resp_STK_OK);
  } else {
    errorCount++;
    Serial.write(Resp_STK_NOSYNC);
  }
}

void getParameters(uint8_t c) {
  switch (c) {
  case Parm_STK_HW_VER:
    reply(HWVER);
    break;
  case Parm_STK_SW_MAJOR:
    reply(SWMAJ);
    break;
  case Parm_STK_SW_MINOR:
    reply(SWMIN);
    break;
  case Parm_STK_PROGMODE:
    reply('S'); // serial programmer
    break;
  case Parm_STK_VTARGET:
    reply(31);
    break;
  default:
    reply(0);
  }
}

void set_parameters() {
  // call this after reading parameter packet into buffer[]
  param.devicecode = buffer[0];
  param.revision = buffer[1];
  param.progtype = buffer[2];
  param.parmode = buffer[3];
  param.polling = buffer[4];
  param.selftimed = buffer[5];
  param.lockbytes = buffer[6];
  param.fusebytes = buffer[7];
  param.flashpoll = buffer[8];
  // ignore buffer[9] (= buffer[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buffer[10]);
  param.pagesize = beget16(&buffer[12]);
  param.eepromsize = beget16(&buffer[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buffer[16] * 0x01000000 + buffer[17] * 0x00010000 +
                    buffer[18] * 0x00000100 + buffer[19];

  // AVR devices have active low reset, AT89Sx are active high
  rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode() {

  digitalWrite(ENABLE_PGM, LOW);
  digitalWrite(TARGET_PWR, ON);

  delay(20); // random number

  // Reset target before driving SCK or MOSI

  // SPI.begin() will configure SS as output, so SPI master mode is selected.
  // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
  // So we have to configure RESET as output
  // (reset_target() first sets the correct level)
  reset_target(true);

  pinMode(RESET, OUTPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));

  // See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

  // Pulse RESET after SCK is low:
  digitalWrite(SCK, LOW);
  delay(20); // discharge SCK, value arbitrarily chosen

  reset_target(false);

  // Pulse must be minimum 2 target CPU clock cycles so 100 usec
  // is ok for CPU speeds above 20 KHz
  delayMicroseconds(100);

  reset_target(true);

  // Send the enable programming command:
  delay(50); // Must be > 20 msec as per datasheet
  spi_transaction(0xAC, 0x53, 0x00, 0x00);

  programming = true;
}

void end_pmode() {

  SPI.end();

  // We're about to take the target out of reset so configure SPI pins as input
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);

  reset_target(false);

  pinMode(RESET, INPUT);

  programming = false;

  digitalWrite(ENABLE_PGM, HIGH);
  digitalWrite(TARGET_PWR, OFF);

  /////////////////////////////////////////////////////////////////////////////////

  if (errorCount) {
    digitalWrite(LED_OK, OFF);
    digitalWrite(LED_ERROR, ON);
  } else {
    digitalWrite(LED_OK, ON);
    digitalWrite(LED_ERROR, OFF);
  }
}

void universal() {
  uint8_t ch;
  fill(4);
  ch = spi_transaction(buffer[0], buffer[1], buffer[2], buffer[3]);
  reply(ch);
}

void flash(uint8_t hilo, uint16_t addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF, data);
}

void commit(uint16_t addr) {
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

uint16_t current_page() {
  if (param.pagesize == 32) {
    return bufferIndex & 0xFFFFFFF0;
  }
  if (param.pagesize == 64) {
    return bufferIndex & 0xFFFFFFE0;
  }
  if (param.pagesize == 128) {
    return bufferIndex & 0xFFFFFFC0;
  }
  if (param.pagesize == 256) {
    return bufferIndex & 0xFFFFFF80;
  }
  return bufferIndex;
}

uint8_t write_flash_pages(uint16_t length) {

  uint16_t x = 0;

  uint16_t page = current_page();

  while (x < length) {

    if (page != current_page()) {
      commit(page);
      page = current_page();
    }

    flash(LOW, bufferIndex, buffer[x++]);
    flash(HIGH, bufferIndex, buffer[x++]);

    bufferIndex++;
  }

  commit(page);

  return Resp_STK_OK;
}

void write_flash(uint16_t length) {

  fill(length);

  if (Sync_CRC_EOP == getChar()) {
    Serial.write(Resp_STK_INSYNC);
    Serial.write(write_flash_pages(length));
  } else {
    errorCount++;
    Serial.write(Resp_STK_NOSYNC);
  }
}

// Write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(uint16_t start, uint16_t length) {

  // This writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);

  for (uint16_t x = 0; x < length; x++) {
    uint16_t addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buffer[x]);
    delay(45);
  }

  return Resp_STK_OK;
}

uint8_t write_eeprom(uint16_t length) {

  // bufferIndex is a word address, get the byte address
  uint16_t start = bufferIndex * 2;
  uint16_t remaining = length;

  if (length > param.eepromsize) {
    errorCount++;
    return Resp_STK_FAILED;
  }

  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }

  write_eeprom_chunk(start, remaining);

  return Resp_STK_OK;
}

void program_page() {

  uint8_t result = Resp_STK_FAILED;

  uint16_t length = 256 * getChar();

  length += getChar();

  uint8_t memtype = getChar();

  // flash memory @bufferIndex, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }

  if (memtype == 'E') {

    result = write_eeprom(length);

    if (Sync_CRC_EOP == getChar()) {
      Serial.write(Resp_STK_INSYNC);
      Serial.write(result);
    } else {
      errorCount++;
      Serial.write(Resp_STK_NOSYNC);
    }

    return;
  }

  Serial.write(Resp_STK_FAILED);

  return;
}

uint8_t flash_read(uint8_t hilo, uint16_t addr) {
  return spi_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

uint8_t flash_read_page(uint16_t length) {

  for (uint16_t x = 0; x < length; x += 2) {

    uint8_t low = flash_read(LOW, bufferIndex);
    Serial.write(low);

    uint8_t high = flash_read(HIGH, bufferIndex);
    Serial.write(high);

    bufferIndex++;
  }

  return Resp_STK_OK;
}

uint8_t eeprom_read_page(uint16_t length) {

  // bufferIndex again we have a word address (tf...???)
  uint16_t start = bufferIndex * 2;

  for (uint16_t x = 0; x < length; x++) {
    uint16_t addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    Serial.write(ee);
  }

  return Resp_STK_OK;
}

void read_page() {

  uint8_t result = Resp_STK_FAILED;

  uint16_t length = 256 * getChar();
  length += getChar();

  uint8_t memtype = getChar();

  if (Sync_CRC_EOP != getChar()) {
    errorCount++;
    Serial.write(Resp_STK_NOSYNC);
    return;
  }

  Serial.write(Resp_STK_INSYNC);

  if (memtype == 'F') {
    result = flash_read_page(length);
  }
  if (memtype == 'E') {
    result = eeprom_read_page(length);
  }

  Serial.write(result);
}

void read_signature() {

  if (Sync_CRC_EOP != getChar()) {
    errorCount++;
    Serial.write(Resp_STK_NOSYNC);
    return;
  }

  Serial.write(Resp_STK_INSYNC);

  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  Serial.write(high);

  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  Serial.write(middle);

  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  Serial.write(low);

  Serial.write(Resp_STK_OK);
}

////////////////////////////////////////////////////////////////////////

void avrisp() {

  uint8_t ch = getChar();

  switch (ch) {

  case Cmnd_STK_GET_SYNC:
    errorCount = 0;
    replyEmpty();
    break;

  case Cmnd_STK_GET_SIGN_ON:
    if (getChar() == Sync_CRC_EOP) {
      Serial.write(Resp_STK_INSYNC);
      Serial.print(Resp_STK_SIGNON_MSG);
      Serial.write(Resp_STK_OK);
    } else {
      errorCount++;
      Serial.write(Resp_STK_NOSYNC);
    }
    break;

  case Cmnd_STK_GET_PARAMETER:
    getParameters(getChar());
    break;

  case Cmnd_STK_SET_DEVICE:
    fill(20);
    set_parameters();
    replyEmpty();
    break;

  case Cmnd_STK_SET_DEVICE_EXT: // extended parameters - ignore for now
    fill(5);
    replyEmpty();
    break;

  case Cmnd_STK_ENTER_PROGMODE:
    if (!programming) {
      start_pmode();
    }
    replyEmpty();
    break;

  case Cmnd_STK_LOAD_ADDRESS:
    bufferIndex = getChar();
    bufferIndex += 256 * getChar();
    replyEmpty();
    break;

  case Cmnd_STK_PROG_FLASH:
    getChar(); // low addr
    getChar(); // high addr
    replyEmpty();
    break;

  case Cmnd_STK_PROG_DATA:
    getChar(); // data...
    replyEmpty();
    break;

  case Cmnd_STK_PROG_PAGE:
    program_page();
    break;

  case Cmnd_STK_READ_PAGE:
    read_page();
    break;

  case Cmnd_STK_UNIVERSAL:
    universal();
    break;

  case Cmnd_STK_LEAVE_PROGMODE:
    errorCount = 0;
    end_pmode();
    replyEmpty();
    break;

  case Cmnd_STK_READ_SIGN:
    read_signature();
    break;

  // expecting a command, not Sync_CRC_EOP
  // this is how we can get back in sync
  case Sync_CRC_EOP:
    errorCount++;
    Serial.write(Resp_STK_NOSYNC);
    break;

  // anything else we will return Resp_STK_UNKNOWN
  default:
    errorCount++;
    if (Sync_CRC_EOP == getChar()) {
      Serial.write(Resp_STK_UNKNOWN);
    } else {
      Serial.write(Resp_STK_NOSYNC);
    }
  }
}