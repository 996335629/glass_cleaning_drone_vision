#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

class Protocol {
public:
    enum class PacketType : uint8_t {
        DISPLACEMENT = 0x01,
        VELOCITY = 0x02,
        STATUS = 0x03,
        HEARTBEAT = 0x04
    };

#pragma pack(push, 1)
    struct DisplacementData {
        float dx;
        float dy;
        float dz;
        float confidence;
        uint32_t timestamp;
        uint8_t status_flags;
    };

    // 速度帧（相机坐标系）
    // vx：向右为正，vy：向下为正，vz：靠近玻璃为正（单位 m/s）
    // 飞控侧自行做坐标系转换
    struct VelocityData {
        float    vx, vy, vz;         // 速度（m/s）
        float    confidence;          // 光流置信度 0.0~1.0
        uint8_t  depth_valid;         // 1=深度可用，0=镜面/失效（速度为上次有效值）
        uint8_t  bar_count;           // 检测到的幕墙竖框数量（0=未检测到）
        float    bar_offset_mm;       // 距最近竖框横向偏移（mm，正=竖框在右）
                                      // bar_count>0 且 depth_valid=1 时有效
        uint32_t timestamp_ms;
    };  // 共 26 bytes
#pragma pack(pop)

    enum StatusFlags : uint8_t {
        NORMAL              = 0x00,
        REFLECTION_DETECTED = 0x01,
        LOW_FEATURES        = 0x02
    };

    static std::vector<uint8_t> packDisplacement(const DisplacementData &data);
    static std::vector<uint8_t> packVelocity(const VelocityData &data);
    static std::vector<uint8_t> packHeartbeat();
    static uint16_t calculateCRC16(const uint8_t *data, size_t length);
};
