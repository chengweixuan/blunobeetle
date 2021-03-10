#define ACK 0
#define NAK 1
#define HELLO 2
#define CHECKSUM 0b01011

struct SensorData
{
  uint16_t xAccel;
  uint16_t yAccel;
  uint16_t zAccel;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t row;
};

struct DataPacket
{
  uint8_t startByte;
  uint8_t statusData;
  SensorData sensorData;
  uint8_t CRC;
  uint8_t endByte;
};

struct CommandPacket
{
  uint8_t startByte;
  uint8_t commandData;
  uint8_t endByte;
};

struct CommandPacket makeCommandPacket(uint8_t commandType) {
  uint8_t command = commandType << 5;
  uint8_t checksum = CHECKSUM;
  uint8_t commandData = command + checksum;
  
  CommandPacket commandPacket;
  commandPacket.startByte = 0xAC;
  commandPacket.commandData = commandData;
  commandPacket.endByte = 0xBE;

  return commandPacket;
}

uint8_t getCommandType(uint8_t commandPacket) {
  return commandPacket >> 5;
}

bool isValidCommandPacket(uint8_t commandPacket) {
  uint8_t checksum = CHECKSUM;
  uint8_t mask = 0b00011111;
  uint8_t packetChecksum = commandPacket & mask;
  if (packetChecksum ^ checksum) {
    return false;
  } else {
    return true;
  }
}

uint8_t calculateCRC() {
  uint8_t polynomial = 0b10000111; // not used
  return CHECKSUM; // to be completed
}

struct SensorData getSensorData(uint16_t xAccel, uint16_t yAccel, 
uint16_t zAccel, uint16_t yaw, uint16_t pitch, uint16_t row) {
  SensorData sensorData;
  sensorData.xAccel = xAccel;
  sensorData.yAccel = yAccel;
  sensorData.zAccel = zAccel;
  sensorData.yaw = yaw;
  sensorData.pitch = pitch;
  sensorData.row = row;

  return sensorData;
}

uint16_t getRow(uint16_t row) {
  return row;
}

struct DataPacket makeDataPacket(uint8_t startPosition, uint8_t dancerState, uint8_t leftRightStatus, SensorData sensorData) {
  DataPacket packet;
  packet.startByte = 0xAC;
  packet.statusData = (startPosition << 6) + (dancerState << 2) + leftRightStatus; 
  packet.sensorData = sensorData;
  packet.CRC = calculateCRC();
  packet.endByte = 0xBE;

  return packet;
}
