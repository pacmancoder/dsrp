#include <CRC16.h>

const uint32_t DEFAULT_BAUD_RATE = 115200;

const uint16_t MAX_PACKET_INFO_SIZE = 32;
const uint16_t BLOCK_SIZE = 512;
const uint16_t DATA_BUFFER_SIZE = BLOCK_SIZE + MAX_PACKET_INFO_SIZE;

const uint32_t PACKET_HEADER_SIZE = sizeof(uint16_t);
const uint32_t PACKET_TRAILER_SIZE = sizeof(uint16_t);

const int PIN_SHIFT_DATA = A0;
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

// --- TYPES ---

class PacketBuffer {
private:
  uint8_t m_buffer[DATA_BUFFER_SIZE];
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
};

enum struct PacketId: uint16_t {
  /// Requests
  InitRequest = 0x00AD,
  ReadRequest = 0x01AD,
  WriteRequest = 0x02AD,
  EraseRequest = 0x03AD,
  // Responses
  InitResponse = 0x10AD,
  ReadResponse = 0x11AD,
  WriteResponse = 0x12AD,
  EraseResponse = 0x13AD,
  // Error response
  ErrorResponse = 0xFFAD,
};

template<PacketId> struct __attribute__((packed)) PacketSpec {
  static const uint16_t FIXED_PART_SIZE = 0;
  static const bool VARIABLE = false;
};

template<> struct __attribute__((packed)) PacketSpec<PacketId::InitResponse> {
  static const uint16_t FIXED_PART_SIZE = sizeof(uint16_t); // max_block_size
  static const bool VARIABLE = false;
};

template<> struct __attribute__((packed)) PacketSpec<PacketId::ReadRequest> {
  static const uint16_t FIXED_PART_SIZE = sizeof(uint32_t) // offset
    + sizeof(uint16_t); // read_length
  static const bool VARIABLE = false;
};

template<> struct __attribute__((packed)) PacketSpec<PacketId::ReadResponse> {
  static const uint16_t FIXED_PART_SIZE = sizeof(uint16_t); // variable_payload_length
  static const bool VARIABLE = true;
};

template<> struct __attribute__((packed)) PacketSpec<PacketId::WriteRequest> {
  static const uint16_t FIXED_PART_SIZE = sizeof(uint32_t) // offset
    + sizeof(uint16_t); // variable_payload_length
  static const bool VARIABLE = true;
};

template<> struct __attribute__((packed)) PacketSpec<PacketId::ErrorResponse> {
  static const uint16_t FIXED_PART_SIZE = sizeof(uint8_t); // error_code
  static const bool VARIABLE = false;
};

enum struct ErrorCode: uint8_t {
  // Non-reportable `Success` value, used as return value in functions
  Ok = 0x00,
  
  // Reportable errors
  InvalidCrc = 0x01,
  OffsetOverflow = 0x02,
  LengthOverflow = 0x03,
  EraseRequired = 0x04,
  NotImplemented = 0x05,
  NotSupportedRom = 0x06,

  // Non-reportable errors
  SerialCommunicationFailed = 0x81,
};

bool error_code_is_reportable(ErrorCode code) {
  switch (code) {
    case ErrorCode::SerialCommunicationFailed:
      return false;
    default:
      return true;
  }
}

// --- GLOBALS ---

PacketBuffer S_DATA_BUFFER;
CRC16 S_GLOBAL_CRC;

// --- Programming ---

void set_rom_address(uint32_t addr) {
  // TODO: SPEEDUP via direct writes/reads to/from PORTx/PINx
  const uint8_t ROM_ADDR_BITS = 19;  
  for (int i = ROM_ADDR_BITS - 1; i >= 0; --i) {
    digitalWrite(PIN_SHIFT_DATA, (addr & (uint32_t(1) << i)) ? HIGH : LOW);
    digitalWrite(PIN_SHIFT_CLOCK, HIGH);
    digitalWrite(PIN_SHIFT_CLOCK, LOW);
  }

  digitalWrite(PIN_SHIFT_LATCH, HIGH);
  digitalWrite(PIN_SHIFT_LATCH, LOW);
}

uint8_t read_byte(uint32_t addr) {
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

uint8_t rom_read_block(uint32_t addr, uint16_t len) {
  while (len--) {
    uint8_t b = read_byte(addr++);
    S_DATA_BUFFER.write_u8(b);
  }
}

// --- PACKET PROCESSING ---

template<PacketId PKT>
ErrorCode read_packet() {
  uint16_t packet_size = PACKET_HEADER_SIZE + PacketSpec<PKT>::FIXED_PART_SIZE + PACKET_TRAILER_SIZE;
  
  // Read fixed part (currently buffer contains only packet id
  if (Serial.readBytes(S_DATA_BUFFER.ptr(), PacketSpec<PKT>::FIXED_PART_SIZE) < 0) {
    // Failed to read fixed part
    return ErrorCode::SerialCommunicationFailed;
  }

  // Read variable size
  if (PacketSpec<PKT>::VARIABLE) {
    S_DATA_BUFFER.advance(PacketSpec<PKT>::FIXED_PART_SIZE - sizeof(uint16_t));
    uint16_t variable_part_size = S_DATA_BUFFER.read_u16();
    if (Serial.readBytes(S_DATA_BUFFER.ptr(), variable_part_size) < 0) {
      // Failed to read variable part
      return ErrorCode::SerialCommunicationFailed;
    }
    S_DATA_BUFFER.advance(variable_part_size);
    packet_size += variable_part_size;
  } else {
    S_DATA_BUFFER.advance(PacketSpec<PKT>::FIXED_PART_SIZE);
  }

  // Read CRC16
  if (Serial.readBytes(S_DATA_BUFFER.ptr(), sizeof(uint16_t)) < 0) {
    // Failed to read variable part
    return ErrorCode::SerialCommunicationFailed;
  }
  uint16_t expected_crc = S_DATA_BUFFER.read_u16();

  S_DATA_BUFFER.rewind();
  
  S_GLOBAL_CRC.restart();
  S_GLOBAL_CRC.add(S_DATA_BUFFER.ptr(), packet_size - PACKET_TRAILER_SIZE);
  uint16_t actual_crc = S_GLOBAL_CRC.getCRC();

  if (expected_crc != actual_crc) {
    return ErrorCode::InvalidCrc;
  }

  // Prepare buffer to read packet body
  S_DATA_BUFFER.advance(PACKET_HEADER_SIZE);

  return ErrorCode::Ok;
}

void finalize_packet() {
  uint16_t buffer_bytes = S_DATA_BUFFER.used();
  S_DATA_BUFFER.rewind();

  // Apply CRC16 to all data except trailer (CRC)
  S_GLOBAL_CRC.restart();
  S_GLOBAL_CRC.add(S_DATA_BUFFER.ptr(), buffer_bytes);
  uint16_t crc = S_GLOBAL_CRC.getCRC();
  
  // Write CRC to trailer
  S_DATA_BUFFER.advance(buffer_bytes);
  S_DATA_BUFFER.write_u16(crc);
}

ErrorCode send_packet() {
  finalize_packet();
  
  uint16_t buffer_bytes = S_DATA_BUFFER.used();
  S_DATA_BUFFER.rewind();
  
  Serial.write(S_DATA_BUFFER.ptr(), buffer_bytes);
}

ErrorCode process_init_req() {
  auto result = read_packet<PacketId::InitRequest>();
  if (result != ErrorCode::Ok) {
    return result;
  }

  // InitRequest has no body, nothing to process

  S_DATA_BUFFER.rewind();
  S_DATA_BUFFER.write_u16(static_cast<uint16_t>(PacketId::InitResponse));
  S_DATA_BUFFER.write_u16(BLOCK_SIZE);
  
  send_packet();

  return ErrorCode::Ok;
}

ErrorCode process_read_req() {
  auto result = read_packet<PacketId::ReadRequest>();
  if (result != ErrorCode::Ok) {
    return result;
  }

  // Read request body
  uint32_t offset = S_DATA_BUFFER.read_u32();
  uint16_t bytes_to_read = S_DATA_BUFFER.read_u16();
  
  S_DATA_BUFFER.rewind();
  S_DATA_BUFFER.write_u16(static_cast<uint16_t>(PacketId::ReadResponse));
  S_DATA_BUFFER.write_u16(bytes_to_read);
  rom_read_block(offset, bytes_to_read);
  
  send_packet();

  return ErrorCode::Ok;
}

ErrorCode process_write_req() {
  return ErrorCode::NotImplemented;
}

ErrorCode process_erase_req() {
  return ErrorCode::NotImplemented;
}

void send_error(ErrorCode error_code) {
  S_DATA_BUFFER.rewind();
  S_DATA_BUFFER.write_u16(static_cast<uint16_t>(PacketId::ErrorResponse));
  S_DATA_BUFFER.write_u8(static_cast<uint8_t>(error_code));
  send_packet();
}

void discard_serial_rx() {
  while (Serial.available()) {
    Serial.read();
  }
}

void process_incomming_packet() {
  S_DATA_BUFFER.rewind();
  auto result = Serial.readBytes(S_DATA_BUFFER.ptr(), sizeof(PacketId));
  if (result < 0) {
    discard_serial_rx(); 
    return;
  }
  
  PacketId packet_id = static_cast<PacketId>(S_DATA_BUFFER.read_u16());
  bool valid_packet_id = true;
  ErrorCode error_code = ErrorCode::Ok;
  switch (packet_id) {
    case PacketId::InitRequest:
      error_code = process_init_req();
      break;
    case PacketId::ReadRequest:
      error_code = process_read_req();
      break;
    case PacketId::WriteRequest:
      error_code = process_write_req();
      break;
    case PacketId::EraseRequest:
      error_code = process_erase_req();
      break;
    default:
      valid_packet_id = false;
  }

  if (!valid_packet_id) {
    discard_serial_rx(); 
    return;
  }

  // Send reportable error if present
  if (error_code != ErrorCode::Ok) {
    if (error_code_is_reportable(error_code)) {
      send_error(error_code);
    } else {
      discard_serial_rx(); 
    }
  }
}

// --- Setup / main loop ---

void setup() {
  Serial.begin(DEFAULT_BAUD_RATE);
  // Configure shift register control pins
  pinMode(PIN_SHIFT_DATA, OUTPUT);
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
  pinMode(PIN_ROM_D0, INPUT);
  pinMode(PIN_ROM_D1, INPUT);
  pinMode(PIN_ROM_D2, INPUT);
  pinMode(PIN_ROM_D3, INPUT);
  pinMode(PIN_ROM_D4, INPUT);
  pinMode(PIN_ROM_D5, INPUT);
  pinMode(PIN_ROM_D6, INPUT);
  pinMode(PIN_ROM_D7, INPUT);

  // Configure CRC16 to KERMIT(CCITT) flavour
  S_GLOBAL_CRC.setPolynome(0x1021);
  S_GLOBAL_CRC.setStartXOR(0x0000);
  S_GLOBAL_CRC.setEndXOR(0x0000);
  S_GLOBAL_CRC.setReverseIn(true);
  S_GLOBAL_CRC.setReverseOut(true);
}

void loop() {
  process_incomming_packet();
}
