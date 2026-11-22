#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace panthera {

constexpr std::size_t kGelloServoCount = 14;

int gello_servo_index(std::uint8_t id) noexcept;
std::uint8_t gello_servo_id(std::size_t index) noexcept;
std::array<std::uint8_t, 8> gello_read_request(std::uint8_t id) noexcept;

struct GelloRawSample {
    std::uint8_t id{};
    std::uint16_t raw{};
    std::int16_t velocity{};
};

struct GelloParser {
    std::array<std::uint8_t, 64> bytes{};
    std::size_t used{};
    std::uint32_t bad{};

    void reset() noexcept;
    bool feed(std::uint8_t byte, GelloRawSample& out) noexcept;
};

struct GelloTracker {
    std::uint16_t raw{};
    std::uint16_t candidate{};
    std::int32_t ticks{};
    bool valid{};
    bool pending{};

    bool update(std::uint16_t raw_pos) noexcept;
};

struct SerialPortInfo {
    std::string port_name;
    std::string description;
    std::uint16_t vendor_id{};
    std::uint16_t product_id{};
    std::string serial_number;
    std::string manufacturer;
};

std::vector<SerialPortInfo> list_serial_ports();

struct GelloConfig {
    std::string port;
    std::uint32_t baud = 1000000;
    // Bit 0 = left arm (IDs 1..7), bit 1 = right arm (IDs 11..17).
    std::uint8_t group_mask = 0x03;
    int servo_timeout_ms = 5;
    int reconnect_interval_ms = 500;
};

struct GelloServoSample {
    std::uint8_t id{};
    std::uint16_t raw{};
    std::int32_t ticks{};
    std::int16_t velocity{};
    double age_s{};
    bool valid{};
};

struct GelloSnapshot {
    bool connected{};
    std::uint32_t scans{};
    std::uint32_t replies{};
    std::uint32_t timeouts{};
    std::uint32_t bad{};
    std::uint32_t jumps{};
    double scan_hz{};
    std::array<GelloServoSample, kGelloServoCount> servos{};
};

class GelloReader {
  public:
    explicit GelloReader(GelloConfig config);
    ~GelloReader();
    GelloReader(const GelloReader&) = delete;
    GelloReader& operator=(const GelloReader&) = delete;

    GelloSnapshot poll() const;
    bool is_open() const;
    void close();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace panthera
