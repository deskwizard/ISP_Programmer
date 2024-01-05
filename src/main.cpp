// **** CUSTOMIZED VERSION OF arduino-as-isp, ONLY USE FOR THAT HARDWARE ****//
//
//  - Reset output polarity inverted for 2N7000 on reset line
//  - LED pins changed
//  - Added CD4053B inhibit pin (outputs HI-Z when not programming)
//  - Added target power enable

#include "SPI.h"
#include "defines.h"

void avrisp();
void heartbeat();
void pulse(uint8_t pin, uint8_t times);

int ISPError = 0;
bool pgm_mode = false;

unsigned int here;   // address for reading and writing, set by 'U' command
uint8_t buffer[256]; // global block storage

// Heartbeat so you can tell the software is running.
uint8_t heartbeatValue = 128;
int8_t heartbeatDelta = 8;

bool rst_active_high;

void setup() {

  Serial.begin(BAUDRATE);

  pinMode(ENABLE_PGM, OUTPUT);

  digitalWrite(TARGET_PWR, HIGH);
  pinMode(TARGET_PWR, OUTPUT);

  /*   digitalWrite(TARGET_PWR, LOW);
    delay(1000);
    digitalWrite(TARGET_PWR, HIGH); */

  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);

  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);

  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);
}

void loop(void) {

  // is pgm_mode active?
  if (pgm_mode) {
    digitalWrite(LED_PMODE, HIGH);
  } else {
    digitalWrite(LED_PMODE, LOW);
  }

  // is there an error?
  if (ISPError) {
    digitalWrite(LED_ERR, HIGH);
  } else {
    digitalWrite(LED_ERR, LOW);
  }

  // light the heartbeat LED
  heartbeat();

  if (Serial.available()) {
    avrisp();
  }
}

void heartbeat() {

  static unsigned long last_time = 0;
  unsigned long now = millis();

  if ((now - last_time) < 40) {
    return;
  }

  last_time = now;

  if (heartbeatValue > 192) {
    heartbeatDelta = -heartbeatDelta;
  }

  if (heartbeatValue < 16) {
    heartbeatDelta = -heartbeatDelta;
  }

  heartbeatValue += heartbeatDelta;

  analogWrite(LED_HB, heartbeatValue);
}

void reset_target(bool reset) {
  digitalWrite(RESET,
               ((reset && rst_active_high) || (!reset && !rst_active_high))
                   ? LOW
                   : HIGH);
}

uint8_t getChar() {
  while (!Serial.available())
    ;
  return Serial.read();
}

void fill(int n) {
  for (int x = 0; x < n; x++) {
    buffer[x] = getChar();
  }
}

void pulse(uint8_t pin, uint8_t times) {
  do {
    digitalWrite(pin, HIGH);
    delay(LED_PULSE_TIME);
    digitalWrite(pin, LOW);
    delay(LED_PULSE_TIME);
  } while (times--);
}

void prog_lamp(bool state) {
  if (PROG_FLICKER) {
    digitalWrite(LED_PMODE, state);
  }
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  SPI.transfer(a);
  SPI.transfer(b);
  SPI.transfer(c);
  return SPI.transfer(d);
}

void empty_reply() {

  if (CRC_EOP == getChar()) {
    Serial.print((char)STK_INSYNC);
    Serial.print((char)STK_OK);
  } else {
    ISPError++;
    Serial.print((char)STK_NOSYNC);
  }
}

void breply(uint8_t b) {
  if (CRC_EOP == getChar()) {
    Serial.print((char)STK_INSYNC);
    Serial.print((char)b);
    Serial.print((char)STK_OK);
  } else {
    ISPError++;
    Serial.print((char)STK_NOSYNC);
  }
}

void get_version(uint8_t c) {
  switch (c) {
  case 0x80:
    breply(HWVER);
    break;
  case 0x81:
    breply(SWMAJ);
    break;
  case 0x82:
    breply(SWMIN);
    break;
  case 0x93:
    breply('S'); // serial programmer
    break;
  default:
    breply(0);
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
  digitalWrite(TARGET_PWR, LOW);

  delayMicroseconds(40); // random number

  // Reset target before driving SCK or MOSI

  // SPI.begin() will configure SS as output, so SPI master mode is selected.
  // We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
  // So we have to configure RESET as output here,
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

  pgm_mode = true;
}

void end_pmode() {

  SPI.end();

  // We're about to take the target out of reset so configure SPI pins as input
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);

  reset_target(false);

  pinMode(RESET, INPUT);

  pgm_mode = false;

  digitalWrite(ENABLE_PGM, HIGH);
  digitalWrite(TARGET_PWR, HIGH);
}

void universal() {
  uint8_t ch;
  fill(4);
  ch = spi_transaction(buffer[0], buffer[1], buffer[2], buffer[3]);
  breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data) {
  spi_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF, data);
}

void commit(unsigned int addr) {
  if (PROG_FLICKER) {
    prog_lamp(LOW);
  }
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    delay(LED_PULSE_TIME);
    prog_lamp(HIGH);
  }
}

unsigned int current_page() {
  if (param.pagesize == 32) {
    return here & 0xFFFFFFF0;
  }
  if (param.pagesize == 64) {
    return here & 0xFFFFFFE0;
  }
  if (param.pagesize == 128) {
    return here & 0xFFFFFFC0;
  }
  if (param.pagesize == 256) {
    return here & 0xFFFFFF80;
  }
  return here;
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  unsigned int page = current_page();
  while (x < length) {
    if (page != current_page()) {
      commit(page);
      page = current_page();
    }
    flash(LOW, here, buffer[x++]);
    flash(HIGH, here, buffer[x++]);
    here++;
  }

  commit(page);

  return STK_OK;
}

void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getChar()) {
    Serial.print((char)STK_INSYNC);
    Serial.print((char)write_flash_pages(length));
  } else {
    ISPError++;
    Serial.print((char)STK_NOSYNC);
  }
}

// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length) {
  // this writes byte-by-byte, page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (unsigned int x = 0; x < length; x++) {
    unsigned int addr = start + x;
    spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buffer[x]);
    delay(45);
  }
  prog_lamp(HIGH);
  return STK_OK;
}

uint8_t write_eeprom(unsigned int length) {
  // here is a word address, get the byte address
  unsigned int start = here * 2;
  unsigned int remaining = length;
  if (length > param.eepromsize) {
    ISPError++;
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  return STK_OK;
}

void program_page() {
  char result = (char)STK_FAILED;
  unsigned int length = 256 * getChar();
  length += getChar();
  char memtype = getChar();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getChar()) {
      Serial.print((char)STK_INSYNC);
      Serial.print(result);
    } else {
      ISPError++;
      Serial.print((char)STK_NOSYNC);
    }
    return;
  }
  Serial.print((char)STK_FAILED);
  return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr) {
  return spi_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x += 2) {
    uint8_t low = flash_read(LOW, here);
    Serial.print((char)low);
    uint8_t high = flash_read(HIGH, here);
    Serial.print((char)high);
    here++;
  }
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    Serial.print((char)ee);
  }
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getChar();
  length += getChar();
  char memtype = getChar();
  if (CRC_EOP != getChar()) {
    ISPError++;
    Serial.print((char)STK_NOSYNC);
    return;
  }
  Serial.print((char)STK_INSYNC);
  if (memtype == 'F') {
    result = flash_read_page(length);
  }
  if (memtype == 'E') {
    result = eeprom_read_page(length);
  }
  Serial.print(result);
}

void read_signature() {
  if (CRC_EOP != getChar()) {
    ISPError++;
    Serial.print((char)STK_NOSYNC);
    return;
  }
  Serial.print((char)STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  Serial.print((char)high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  Serial.print((char)middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  Serial.print((char)low);
  Serial.print((char)STK_OK);
}
//////////////////////////////////////////
//////////////////////////////////////////

////////////////////////////////////
////////////////////////////////////
void avrisp() {
  uint8_t ch = getChar();
  switch (ch) {
  case '0': // signon
    ISPError = 0;
    empty_reply();
    break;
  case '1':
    if (getChar() == CRC_EOP) {
      Serial.print((char)STK_INSYNC);
      Serial.print("AVR ISP");
      Serial.print((char)STK_OK);
    } else {
      ISPError++;
      Serial.print((char)STK_NOSYNC);
    }
    break;
  case 'A':
    get_version(getChar());
    break;
  case 'B':
    fill(20);
    set_parameters();
    empty_reply();
    break;
  case 'E': // extended parameters - ignore for now
    fill(5);
    empty_reply();
    break;
  case 'P':
    if (!pgm_mode) {
      start_pmode();
    }
    empty_reply();
    break;
  case 'U': // set address (word)
    here = getChar();
    here += 256 * getChar();
    empty_reply();
    break;

  case 0x60:   // STK_PROG_FLASH
    getChar(); // low addr
    getChar(); // high addr
    empty_reply();
    break;
  case 0x61:   // STK_PROG_DATA
    getChar(); // data
    empty_reply();
    break;

  case 0x64: // STK_PROG_PAGE
    program_page();
    break;

  case 0x74: // STK_READ_PAGE 't'
    read_page();
    break;

  case 'V': // 0x56
    universal();
    break;
  case 'Q': // 0x51
    ISPError = 0;
    end_pmode();
    empty_reply();
    break;

  case 0x75: // STK_READ_SIGN 'u'
    read_signature();
    break;

  // expecting a command, not CRC_EOP
  // this is how we can get back in sync
  case CRC_EOP:
    ISPError++;
    Serial.print((char)STK_NOSYNC);
    break;

  // anything else we will return STK_UNKNOWN
  default:
    ISPError++;
    if (CRC_EOP == getChar()) {
      Serial.print((char)STK_UNKNOWN);
    } else {
      Serial.print((char)STK_NOSYNC);
    }
  }
}