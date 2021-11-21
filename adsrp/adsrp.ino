#ifndef ARDUINO_AVR_NANO
    #error "Selected Arduino board is not supported"
#endif
// TODO: Maximize these values (try to fit 768 buffer in 1K memory?)
#ifdef __AVR_ATmega168__
  // Atmega 168 has 1K memory
  #define OPT_MAX_BLOCK_SIZE 256
#elif __AVR_ATmega328P__
  // Atmega 168 has 2K memory
  #define OPT_MAX_BLOCK_SIZE (1024 + 256)
#else
    #error "Selected MCU is not supported"
#endif

#include <CRC16.h>

const uint32_t DEFAULT_BAUD_RATE = 115200;

const int PIN_SHIFT_DATA_0 = 10;
const int PIN_SHIFT_DATA_1 = 11;
const int PIN_SHIFT_DATA_2 = 12;

const int PIN_SHIFT_CLOCK = A1;
const int PIN_SHIFT_LATCH = A2;

const int PIN_ROM_READ_INV = A3;
const int PIN_ROM_WRITE_INV = A4;
const int PIN_ROM_CS_INV = A5;

const int PIN_ROM_D0 = 2;
const int PIN_ROM_D1 = 3;
const int PIN_ROM_D2 = 4;
const int PIN_ROM_D3 = 5;
const int PIN_ROM_D4 = 6;
const int PIN_ROM_D5 = 7;
const int PIN_ROM_D6 = 8;
const int PIN_ROM_D7 = 9;

enum struct DataBusMode {
  Input,
  Output,
  Initial,
};

// --- ROMs ---
struct RomInfo {
  uint32_t manufacturer_id;
  uint32_t chip_id;
  uint32_t rom_size;
  uint32_t sector_size;
};

enum struct RomParam {
  ManufacturerId,
  DeviceId,
};

namespace roms {
  constexpr RomInfo AM29F040 = {0x01, 0xA4, 0x80000, 0x10000};
  constexpr RomInfo SST39SF040 = {0xBF, 0xB7, 0x80000, 0x1000};
}

constexpr const RomInfo SUPPORTED_ROMS[] = {
  roms::AM29F040,
  roms::SST39SF040,
};

// --- Bincode VarInt ---
const uint8_t VARINT_MAX_SINGLE_BYTE = 250;
const uint8_t VARINT_MARKER_U16 = 251;
const uint8_t VARINT_MARKER_U32 = 252;

// --- Frame buffer helper type ---

template<uint32_t SZ>
class FrameBuffer {
private:
  uint8_t m_buffer[SZ];
  uint16_t m_used = 0;
  
public:
  void seek(uint16_t pos) { m_used = pos; }
  void rewind() { m_used = 0; }
  void advance(uint16_t count) { m_used += count; }

  uint16_t used() const { return m_used; }
  uint8_t* ptr() { return m_buffer + m_used; }

  void write_u8(uint8_t data) { m_buffer[m_used++] = data; }
  void write_u16(uint16_t data) { 
    m_buffer[m_used++] = data;
    m_buffer[m_used++] = data >> 8;
  }
  void write_u32(uint32_t data) { 
    m_buffer[m_used++] = data;
    m_buffer[m_used++] = data >> 8;
    m_buffer[m_used++] = data >> 16;
    m_buffer[m_used++] = data >> 24;
  }
  void write_uvarint(uint32_t data) {
    if (data <= VARINT_MAX_SINGLE_BYTE) {
      write_u8(data);
    } else if (data <= 0xFFFF) {
      write_u8(VARINT_MARKER_U16);
      write_u16(data);
    } else {
      write_u8(VARINT_MARKER_U32);
      write_u32(data);
    }
  }


  uint8_t read_u8() {
    return m_buffer[m_used++];
  }
  uint16_t read_u16() {
    uint8_t* base = m_buffer + m_used;
    uint16_t data = static_cast<uint16_t>(*base)
      | (static_cast<uint16_t>(*(base + 1)) << 8);
    m_used += 2;
    return data;
  }
  uint32_t read_u32() {
    uint8_t* base = m_buffer + m_used;
    uint32_t data = static_cast<uint32_t>(*base)
      | (static_cast<uint32_t>(*(base + 1)) << 8)
      | (static_cast<uint32_t>(*(base + 2)) << 16)
      | (static_cast<uint32_t>(*(base + 3)) << 24);
    m_used += 4;
    return data;
  }
  uint32_t read_uvarint() {
    uint8_t head = read_u8();
    if (head <= VARINT_MAX_SINGLE_BYTE) {
      return head;
    } else if (head == VARINT_MARKER_U16) {
      return read_u16();
    } else {
      return read_u32();
    }
  }
};

// --- Transport protocol ---

const uint32_t FRAME_HEADER_SIZE = sizeof(uint16_t);
const uint32_t FRAME_TRAILER_SIZE = sizeof(uint16_t);
const uint16_t MAX_PACKET_FIELDS_SIZE = 32;
const uint16_t FRAME_BUFFER_SIZE = OPT_MAX_BLOCK_SIZE + MAX_PACKET_FIELDS_SIZE;
const uint16_t AUX_FRAME_BUFFER_SIZE = MAX_PACKET_FIELDS_SIZE;

FrameBuffer<FRAME_BUFFER_SIZE> S_FRAME_BUFFER;
FrameBuffer<AUX_FRAME_BUFFER_SIZE> S_AUX_FRAME_BUFFER;
CRC16 S_GLOBAL_CRC;

// Returns size of read frame or 0 if failed to read frame or no input available
uint16_t read_frame() {
  S_FRAME_BUFFER.rewind();
  if (Serial.readBytes(S_FRAME_BUFFER.ptr(), FRAME_HEADER_SIZE) != FRAME_HEADER_SIZE) {
    // Failed to read header
    return 0;
  }
  uint16_t size = S_FRAME_BUFFER.read_u16();

  if (size > FRAME_BUFFER_SIZE) {
    // Purge all remaining data and return with error code
    while (Serial.available() && size--) {
      Serial.read();
    }
    return 0;
  }

  uint16_t remaining_size = size - FRAME_HEADER_SIZE;

  if (Serial.readBytes(S_FRAME_BUFFER.ptr(), remaining_size) != remaining_size) {
    // failed to read remaining data
    return 0;
  }

  uint16_t data_size = size - FRAME_HEADER_SIZE - FRAME_TRAILER_SIZE;
  S_FRAME_BUFFER.advance(data_size);
  uint16_t expected_crc = S_FRAME_BUFFER.read_u16();

  S_FRAME_BUFFER.rewind();
  S_GLOBAL_CRC.restart();
  S_GLOBAL_CRC.add(S_FRAME_BUFFER.ptr(), size - FRAME_TRAILER_SIZE);
  uint16_t actual_crc = S_GLOBAL_CRC.getCRC();

  if (expected_crc != actual_crc) {
    return 0;
  }

  // Prepare buffer to read packet body
  S_FRAME_BUFFER.advance(FRAME_HEADER_SIZE);
  return data_size;
}

// Prepares frame buffer for writing frame data
void new_frame() {
  S_FRAME_BUFFER.rewind();
  S_FRAME_BUFFER.advance(FRAME_HEADER_SIZE);
}

// Finalize frame and write it to the serial
void write_frame() {
  uint16_t tail_pos = S_FRAME_BUFFER.used();
  uint16_t frame_size = tail_pos + FRAME_TRAILER_SIZE;
  
  S_FRAME_BUFFER.rewind();
  S_FRAME_BUFFER.write_u16(frame_size);

  S_FRAME_BUFFER.rewind();
  S_GLOBAL_CRC.restart();
  S_GLOBAL_CRC.add(S_FRAME_BUFFER.ptr(), tail_pos);
  uint16_t crc = S_GLOBAL_CRC.getCRC();
  S_FRAME_BUFFER.advance(tail_pos);
  S_FRAME_BUFFER.write_u16(crc);

  S_FRAME_BUFFER.rewind();
  Serial.write(S_FRAME_BUFFER.ptr(), frame_size);
}

// --- Programming protocol ---

const uint32_t REQUEST_ID_INIT = 0x00;
const uint32_t REQUEST_ID_READ = 0x01;
const uint32_t REQUEST_ID_WRITE = 0x02;
const uint32_t REQUEST_ID_ERASE_SECTOR = 0x03;

const uint32_t RESPONSE_ID_INIT = 0x00;
const uint32_t RESPONSE_ID_READ = 0x01;
const uint32_t RESPONSE_ID_WRITE = 0x02;
const uint32_t RESPONSE_ID_ERASE_SECTOR = 0x03;
const uint32_t RESPONSE_ID_IN_PROGRESS = 0x04;
const uint32_t RESPONSE_ID_ERROR = 0x05;


const uint32_t ERROR_UNSUPPORTED_REQUEST = 0x00;
const uint32_t ERROR_UNSUPPORTED_ROM = 0x01;
const uint32_t ERROR_ROM_INIT_FAILED = 0x02;
const uint32_t ERROR_INVALID_OFFSET = 0x03;
const uint32_t ERROR_INVALID_SIZE = 0x04;
const uint32_t ERROR_SECTOR_ERASE_FAILED = 0x05;
const uint32_t ERROR_WRITE_FAILED = 0x06;
const uint32_t ERROR_ROM_NOT_MATCH = 0x07;
const uint32_t ERROR_NOT_INITIALIZED = 0x08;
const uint32_t ERROR_INVALID_SECTOR = 0x09;

const RomInfo* current_rom = nullptr;

void send_rom_info(const RomInfo* rom) {
  new_frame();
  S_FRAME_BUFFER.write_uvarint(RESPONSE_ID_INIT);
  S_FRAME_BUFFER.write_uvarint(OPT_MAX_BLOCK_SIZE);
  S_FRAME_BUFFER.write_uvarint(rom->rom_size);
  S_FRAME_BUFFER.write_uvarint(rom->sector_size);
  S_FRAME_BUFFER.write_uvarint(rom->manufacturer_id);
  S_FRAME_BUFFER.write_uvarint(rom->chip_id);
  write_frame();
}

void send_error(uint32_t error) {
  new_frame();
  S_FRAME_BUFFER.write_uvarint(RESPONSE_ID_ERROR);
  S_FRAME_BUFFER.write_uvarint(error);
  write_frame();
}

// We use aux buffer here to send frame, because while in progress,
// we should not corrupt the main frame
void send_in_progress() {
  S_AUX_FRAME_BUFFER.rewind();
  S_AUX_FRAME_BUFFER.advance(FRAME_HEADER_SIZE);
  S_AUX_FRAME_BUFFER.write_uvarint(RESPONSE_ID_IN_PROGRESS);
  uint16_t tail_pos = S_AUX_FRAME_BUFFER.used();
  uint16_t frame_size = tail_pos + FRAME_TRAILER_SIZE;
  S_AUX_FRAME_BUFFER.rewind();
  S_AUX_FRAME_BUFFER.write_u16(frame_size);

  S_AUX_FRAME_BUFFER.rewind();
  S_GLOBAL_CRC.restart();
  S_GLOBAL_CRC.add(S_AUX_FRAME_BUFFER.ptr(), tail_pos);
  uint16_t crc = S_GLOBAL_CRC.getCRC();
  S_AUX_FRAME_BUFFER.advance(tail_pos);
  S_AUX_FRAME_BUFFER.write_u16(crc);

  S_AUX_FRAME_BUFFER.rewind();
  Serial.write(S_AUX_FRAME_BUFFER.ptr(), frame_size);
}

const RomInfo* get_supported_rom(uint32_t manufacturer_id, uint32_t chip_id) {
  for (int i = 0; i < sizeof(SUPPORTED_ROMS); ++i) {
    const auto* rom = SUPPORTED_ROMS + i;
    if ((rom->manufacturer_id == manufacturer_id) && (rom->chip_id == chip_id)) {
      return rom;
    }
  }
  return nullptr;
}

void process_init() {
  auto manufacturer_id_hint = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());
  auto chip_id_hint = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());
  bool exact = static_cast<uint8_t>(S_FRAME_BUFFER.read_u8());

  // NOTE: in  the future manufacturer_id_hint and chip_id_hint can be used to select
  // chip identification scheme
  const auto* id_detect_rom = get_supported_rom(manufacturer_id_hint, chip_id_hint);
  if (id_detect_rom == nullptr) {
      send_error(ERROR_UNSUPPORTED_ROM);
      return;
  }

  auto manufacturer_id = rom_read_chip_param(RomParam::ManufacturerId);
  auto device_id = rom_read_chip_param(RomParam::DeviceId);

  const auto* detected_rom = get_supported_rom(manufacturer_id, device_id);

  if (detected_rom == nullptr) {
      send_error(ERROR_UNSUPPORTED_ROM);
      return;
  }

  if (exact) {
    if ((manufacturer_id != manufacturer_id_hint) || (device_id != chip_id_hint)) {
      send_error(ERROR_ROM_NOT_MATCH);
      return;
    }
  }
  current_rom = detected_rom;

  send_rom_info(detected_rom);
}

void process_read() {
  auto offset = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());
  auto block_size = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());

  if (current_rom == nullptr) {
    send_error(ERROR_NOT_INITIALIZED);
    return;
  }

  if (offset > current_rom->rom_size) {
    send_error(ERROR_INVALID_OFFSET);
    return;
  }
  if (offset + block_size > current_rom->rom_size) {
    send_error(ERROR_INVALID_SIZE);
    return;
  }

  new_frame();
  S_FRAME_BUFFER.write_uvarint(RESPONSE_ID_READ);
  S_FRAME_BUFFER.write_uvarint(block_size);
  if (!rom_read_block(offset, block_size)) {
    return;
  }
  write_frame();
}

void process_erase_sector() {
  auto sector = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());
  
  auto sector_count = current_rom->rom_size / current_rom->sector_size;

  if (current_rom == nullptr) {
    send_error(ERROR_NOT_INITIALIZED);
    return;
  }

  if (sector >= sector_count) {
    send_error(ERROR_INVALID_SECTOR);
    return;
  }

  if (!rom_erase_sector(sector)) {
    return;
  }

  new_frame();
  S_FRAME_BUFFER.write_uvarint(RESPONSE_ID_ERASE_SECTOR);
  write_frame();
}

void process_write() {
  if (current_rom == nullptr) {
    send_error(ERROR_NOT_INITIALIZED);
    return;
  }

  auto offset = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());
  auto block_size = static_cast<uint32_t>(S_FRAME_BUFFER.read_uvarint());

  if (offset > current_rom->rom_size) {
    send_error(ERROR_INVALID_OFFSET);
    return;
  }
  if (offset + block_size > current_rom->rom_size) {
    send_error(ERROR_INVALID_SIZE);
    return;
  }

  if (!rom_write_block(offset, block_size)) {
    return;
  }

  new_frame();
  S_FRAME_BUFFER.write_uvarint(RESPONSE_ID_WRITE);
  write_frame();
}

void process_request() {
  auto request_id = static_cast<uint8_t>(S_FRAME_BUFFER.read_uvarint());
  switch (request_id) {
    case REQUEST_ID_INIT: { process_init(); break; }
    case REQUEST_ID_READ: { process_read(); break; }
    case REQUEST_ID_WRITE: { process_write(); break; }
    case REQUEST_ID_ERASE_SECTOR: { process_erase_sector(); break; }
    default: { send_error(ERROR_UNSUPPORTED_REQUEST); break; }
  }
}

// --- Programming ---

void set_rom_address(uint32_t addr) {
  const uint32_t DATA_0_MASK = 0x00000080;
  const uint32_t DATA_1_MASK = 0x00008000;
  const uint32_t DATA_2_MASK = 0x00800000;
  for (int i = 0; i < 8; i++) {
    digitalWrite(PIN_SHIFT_DATA_0, (addr & DATA_0_MASK) ? HIGH : LOW);
    digitalWrite(PIN_SHIFT_DATA_1, (addr & DATA_1_MASK) ? HIGH : LOW);
    digitalWrite(PIN_SHIFT_DATA_2, (addr & DATA_2_MASK) ? HIGH : LOW);
    digitalWrite(PIN_SHIFT_CLOCK, HIGH);
    digitalWrite(PIN_SHIFT_CLOCK, LOW);
    addr <<= 1;
  }

  digitalWrite(PIN_SHIFT_LATCH, HIGH);
  digitalWrite(PIN_SHIFT_LATCH, LOW);
}

void set_data_bus_mode(DataBusMode new_mode) {
  static DataBusMode current_mode = DataBusMode::Initial;

  if (new_mode == current_mode) {
    return;
  }

  if (new_mode == DataBusMode::Output) {
    pinMode(PIN_ROM_D0, OUTPUT);
    pinMode(PIN_ROM_D1, OUTPUT);
    pinMode(PIN_ROM_D2, OUTPUT);
    pinMode(PIN_ROM_D3, OUTPUT);
    pinMode(PIN_ROM_D4, OUTPUT);
    pinMode(PIN_ROM_D5, OUTPUT);
    pinMode(PIN_ROM_D6, OUTPUT);
    pinMode(PIN_ROM_D7, OUTPUT);
  } else {
    pinMode(PIN_ROM_D0, INPUT);
    pinMode(PIN_ROM_D1, INPUT);
    pinMode(PIN_ROM_D2, INPUT);
    pinMode(PIN_ROM_D3, INPUT);
    pinMode(PIN_ROM_D4, INPUT);
    pinMode(PIN_ROM_D5, INPUT);
    pinMode(PIN_ROM_D6, INPUT);
    pinMode(PIN_ROM_D7, INPUT);
  }

  current_mode = new_mode;
}

void rom_reset() {
  rom_write_cycle(0x0000, 0xF0);
  delay(10);
}

void rom_write_unlock_sequence(uint8_t cmd) {
  rom_write_cycle(0x5555, 0xAA);
  rom_write_cycle(0x2AAA, 0x55);
  rom_write_cycle(0x5555, cmd);
}

bool rom_erase_sector(uint32_t sector) {
  uint32_t sector_addr = current_rom->sector_size * sector;
  rom_write_unlock_sequence(0x80);
  rom_write_cycle(0x5555, 0xAA);
  rom_write_cycle(0x2AAA, 0x55);
  rom_write_cycle(sector_addr, 0x30);

  const uint32_t ROM_ERASE_TIMEOUT = 10 * 1000; // 10 seconds for sector at max
  const uint32_t UPDATE_INTERVAL = 100; // 100 ms should be enough

  uint8_t previous_dq6 = rom_read_cycle(sector_addr) & 0x40;
  uint32_t started_at = millis();
  uint32_t last_update_at = started_at;
  for(;;) {
    uint32_t current_time = millis();
    // Periodic update message
    if (current_time - last_update_at > UPDATE_INTERVAL) {
      send_in_progress();
      last_update_at = current_time;
    }
    // Check for timeout
    if (current_time - started_at > ROM_ERASE_TIMEOUT) {
      rom_reset(); // Reset required after error
      send_error(ERROR_SECTOR_ERASE_FAILED);
      return false;
    }
    // Check toggle bit
    uint8_t current_dq6 = rom_read_cycle(sector_addr) & 0x40;
    if (previous_dq6 == current_dq6) {
      // 2 consecutive bits equal => finished
      break;
    }
    previous_dq6 = current_dq6;
  }
  rom_reset(); // Return to read state
  return true;
}

uint8_t rom_read_chip_param(RomParam param) {
  rom_write_unlock_sequence(0x90);
  
  uint32_t addr = 0x0000;
  switch (param) {
    case RomParam::ManufacturerId: {addr = 0x0000; break; }
    case RomParam::DeviceId: {addr = 0x0001; break; }
  }

  auto data = rom_read_cycle(addr);
  rom_reset();
  return data;
}

bool rom_write_cycle(uint32_t addr, uint8_t data) {
  set_rom_address(addr);
  // Double-ensure that ROM OE is OFF to avoid magic smoke
  digitalWrite(PIN_ROM_READ_INV, HIGH);
  set_data_bus_mode(DataBusMode::Output);
  digitalWrite(PIN_ROM_D0, (data & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D1, (data & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D2, (data & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D3, (data & 0x08) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D4, (data & 0x10) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D5, (data & 0x20) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D6, (data & 0x40) ? HIGH : LOW);
  digitalWrite(PIN_ROM_D7, (data & 0x80) ? HIGH : LOW);
  digitalWrite(PIN_ROM_CS_INV, LOW);
  digitalWrite(PIN_ROM_WRITE_INV, LOW);
  digitalWrite(PIN_ROM_WRITE_INV, HIGH);
  digitalWrite(PIN_ROM_CS_INV, HIGH);
  // Go back to input mode to prevent accidents
  set_data_bus_mode(DataBusMode::Input);
}

uint8_t rom_read_cycle(uint32_t addr) {
  // TODO: SPEEDUP via direct writes/reads to/from PORTx/PINx
  // Curently it takes 5uS between consecutive digitalWrite's, but 
  // in reality it could be done much faster (62.5 nS specifically)
  uint8_t b = 0;
    
  set_rom_address(addr);
  digitalWrite(PIN_ROM_CS_INV, LOW);
  digitalWrite(PIN_ROM_READ_INV, LOW);

  if (digitalRead(PIN_ROM_D0)) b |= 0x01;
  if (digitalRead(PIN_ROM_D1)) b |= 0x02;
  if (digitalRead(PIN_ROM_D2)) b |= 0x04;
  if (digitalRead(PIN_ROM_D3)) b |= 0x08;
  if (digitalRead(PIN_ROM_D4)) b |= 0x10;
  if (digitalRead(PIN_ROM_D5)) b |= 0x20;
  if (digitalRead(PIN_ROM_D6)) b |= 0x40;
  if (digitalRead(PIN_ROM_D7)) b |= 0x80;
  
  digitalWrite(PIN_ROM_READ_INV, HIGH);
  digitalWrite(PIN_ROM_CS_INV, HIGH);

  return b;
}

bool rom_read_block(uint32_t offset, uint32_t block_size) {
  set_data_bus_mode(DataBusMode::Input);
  while (block_size--) {
    auto rom_byte = rom_read_cycle(offset++);
    S_FRAME_BUFFER.write_u8(rom_byte);
  }
  return true;
}

bool rom_write_block(uint32_t offset, uint32_t block_size) {
  const uint32_t ROM_WRITE_TIMEOUT = 1; // 1 ms write at max
  const uint32_t UPDATE_INTERVAL = 100; // 100 ms should be enough
    
  uint32_t last_update_at = millis();
  while(block_size--) {
    uint8_t current_byte = S_FRAME_BUFFER.read_u8();
    uint32_t current_offset = offset++;

    rom_write_unlock_sequence(0xA0);
    rom_write_cycle(current_offset, current_byte);
    
    uint8_t previous_dq6 = rom_read_cycle(current_offset) & 0x40;

    uint32_t started_at = millis();
    for(;;) {
      uint32_t current_time = millis();
      // Periodic update message during block write
      if (current_time - last_update_at > UPDATE_INTERVAL) {
        send_in_progress();
        last_update_at = current_time;
      }
      // Check for timeout
      if (current_time - started_at > ROM_WRITE_TIMEOUT) {
        rom_reset(); // Reset required after error
        send_error(ERROR_WRITE_FAILED);
        return false;
      }
      // Check toggle bit
      uint8_t current_dq6 = rom_read_cycle(current_offset) & 0x40;
      if (previous_dq6 == current_dq6) {
        // 2 consecutive bits equal => finished
        break;
      }
      previous_dq6 = current_dq6;
    }
  }
  return true;
}

// --- Setup / main loop ---

void setup() {
  Serial.begin(DEFAULT_BAUD_RATE);
  // Configure shift register control pins
  pinMode(PIN_SHIFT_DATA_0, OUTPUT);
  pinMode(PIN_SHIFT_DATA_1, OUTPUT);
  pinMode(PIN_SHIFT_DATA_2, OUTPUT);
  pinMode(PIN_SHIFT_CLOCK, OUTPUT);
  pinMode(PIN_SHIFT_LATCH, OUTPUT);
  // Prevent pins to go high after setting mode to OUTPUT
  digitalWrite(PIN_ROM_READ_INV, HIGH);
  digitalWrite(PIN_ROM_WRITE_INV, HIGH);
  digitalWrite(PIN_ROM_CS_INV, HIGH);
  // Configure ROM control pins
  pinMode(PIN_ROM_READ_INV, OUTPUT);
  pinMode(PIN_ROM_WRITE_INV, OUTPUT);
  pinMode(PIN_ROM_CS_INV, OUTPUT);
  // Configure data pins
  set_data_bus_mode(DataBusMode::Input);

  // Configure CRC16 to KERMIT(CCITT) flavour
  S_GLOBAL_CRC.setPolynome(0x1021);
  S_GLOBAL_CRC.setStartXOR(0x0000);
  S_GLOBAL_CRC.setEndXOR(0x0000);
  S_GLOBAL_CRC.setReverseIn(true);
  S_GLOBAL_CRC.setReverseOut(true);
}

void loop() {
  if (read_frame()) {
    process_request();
  }
}
