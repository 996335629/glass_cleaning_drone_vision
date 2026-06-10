#include "communication/protocol.h"
#include <cstring>

static constexpr uint16_t PACKET_HEAD = 0xAA55;
static constexpr uint16_t PACKET_TAIL = 0x55AA;

std::vector<uint8_t> Protocol::packDisplacement(const DisplacementData &data)
{
    std::vector<uint8_t> packet;
    packet.reserve(2 + 1 + 1 + sizeof(DisplacementData) + 2 + 2);

    packet.push_back(static_cast<uint8_t>(PACKET_HEAD >> 8));
    packet.push_back(static_cast<uint8_t>(PACKET_HEAD & 0xFF));
    packet.push_back(static_cast<uint8_t>(sizeof(DisplacementData)));
    packet.push_back(static_cast<uint8_t>(PacketType::DISPLACEMENT));

    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&data);
    packet.insert(packet.end(), ptr, ptr + sizeof(DisplacementData));

    uint16_t crc = calculateCRC16(packet.data() + 2, packet.size() - 2);
    packet.push_back(static_cast<uint8_t>(crc >> 8));
    packet.push_back(static_cast<uint8_t>(crc & 0xFF));

    packet.push_back(static_cast<uint8_t>(PACKET_TAIL >> 8));
    packet.push_back(static_cast<uint8_t>(PACKET_TAIL & 0xFF));

    return packet;
}

std::vector<uint8_t> Protocol::packVelocity(const VelocityData &data)
{
    std::vector<uint8_t> packet;
    packet.reserve(2 + 1 + 1 + sizeof(VelocityData) + 2 + 2);

    packet.push_back(static_cast<uint8_t>(PACKET_HEAD >> 8));
    packet.push_back(static_cast<uint8_t>(PACKET_HEAD & 0xFF));
    packet.push_back(static_cast<uint8_t>(sizeof(VelocityData)));
    packet.push_back(static_cast<uint8_t>(PacketType::VELOCITY));

    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&data);
    packet.insert(packet.end(), ptr, ptr + sizeof(VelocityData));

    uint16_t crc = calculateCRC16(packet.data() + 2, packet.size() - 2);
    packet.push_back(static_cast<uint8_t>(crc >> 8));
    packet.push_back(static_cast<uint8_t>(crc & 0xFF));

    packet.push_back(static_cast<uint8_t>(PACKET_TAIL >> 8));
    packet.push_back(static_cast<uint8_t>(PACKET_TAIL & 0xFF));

    return packet;
}

std::vector<uint8_t> Protocol::packHeartbeat()
{
    std::vector<uint8_t> packet;

    packet.push_back(static_cast<uint8_t>(PACKET_HEAD >> 8));
    packet.push_back(static_cast<uint8_t>(PACKET_HEAD & 0xFF));
    packet.push_back(0);
    packet.push_back(static_cast<uint8_t>(PacketType::HEARTBEAT));

    uint16_t crc = calculateCRC16(packet.data() + 2, packet.size() - 2);
    packet.push_back(static_cast<uint8_t>(crc >> 8));
    packet.push_back(static_cast<uint8_t>(crc & 0xFF));

    packet.push_back(static_cast<uint8_t>(PACKET_TAIL >> 8));
    packet.push_back(static_cast<uint8_t>(PACKET_TAIL & 0xFF));

    return packet;
}

uint16_t Protocol::calculateCRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
