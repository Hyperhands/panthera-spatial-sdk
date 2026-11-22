#include "panthera/gello.hpp"
#include <astrial.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

namespace panthera {
namespace {

using Clock = std::chrono::steady_clock;

std::uint8_t feetech_checksum(const std::uint8_t* p, std::size_t n) noexcept {
    std::uint8_t sum = 0;
    for (std::size_t i = 0; i < n; ++i)
        sum = static_cast<std::uint8_t>(sum + p[i]);
    return static_cast<std::uint8_t>(~sum);
}

std::int16_t signed_magnitude(const std::uint8_t* p) noexcept {
    const unsigned n = static_cast<unsigned>(p[0]) | (static_cast<unsigned>(p[1]) << 8);
    return (n & 0x8000u) ? static_cast<std::int16_t>(-static_cast<int>(n & 0x7fffu))
                         : static_cast<std::int16_t>(n);
}

int wrap_delta(int a, int b) noexcept {
    int d = a - b;
    if (d > 2048)
        d -= 4096;
    if (d < -2048)
        d += 4096;
    return d;
}

} // namespace

int gello_servo_index(std::uint8_t id) noexcept {
    if (id >= 1 && id <= 7)
        return static_cast<int>(id - 1);
    if (id >= 11 && id <= 17)
        return static_cast<int>(id - 4);
    return -1;
}

std::uint8_t gello_servo_id(std::size_t index) noexcept {
    if (index < 7)
        return static_cast<std::uint8_t>(index + 1);
    if (index < 14)
        return static_cast<std::uint8_t>(index + 4);
    return 0;
}

std::array<std::uint8_t, 8> gello_read_request(std::uint8_t id) noexcept {
    std::array<std::uint8_t, 8> out{0xff, 0xff, id, 4, 2, 56, 4, 0};
    out[7] = feetech_checksum(out.data() + 2, 5);
    return out;
}

void GelloParser::reset() noexcept { used = 0; }

bool GelloParser::feed(std::uint8_t byte, GelloRawSample& out) noexcept {
    auto discard = [this](std::size_t n) {
        if (n >= used) {
            used = 0;
            return;
        }
        std::memmove(bytes.data(), bytes.data() + n, used - n);
        used -= n;
    };
    if (used == bytes.size()) {
        discard(1);
        ++bad;
    }
    bytes[used++] = byte;
    while (used >= 2) {
        if (bytes[0] != 0xff || bytes[1] != 0xff) {
            discard(1);
            continue;
        }
        if (used < 4)
            return false;
        const std::size_t length = bytes[3];
        const std::size_t total = length + 4;
        if (bytes[2] == 0xff || length < 2 || length > 32) {
            discard(1);
            ++bad;
            continue;
        }
        if (used < total)
            return false;
        if (feetech_checksum(bytes.data() + 2, total - 3) != bytes[total - 1]) {
            discard(1);
            ++bad;
            continue;
        }
        // Position + velocity replies have LEN=6 and error=0. Read-request
        // echoes have LEN=4, so they cannot masquerade as feedback.
        const bool valid = length == 6 && bytes[4] == 0 && gello_servo_index(bytes[2]) >= 0;
        if (valid) {
            out.id = bytes[2];
            const int pos = signed_magnitude(bytes.data() + 5);
            out.raw = static_cast<std::uint16_t>((pos % 4096 + 4096) % 4096);
            out.velocity = signed_magnitude(bytes.data() + 7);
        } else {
            ++bad;
        }
        discard(total);
        if (valid)
            return true;
    }
    return false;
}

bool GelloTracker::update(std::uint16_t raw_pos) noexcept {
    if (raw_pos >= 4096)
        return false;
    if (!valid) {
        raw = raw_pos;
        ticks = raw_pos;
        valid = true;
        pending = false;
        return true;
    }
    const int d = wrap_delta(raw_pos, raw);
    if (d > 100 || d < -100) {
        const int confirm = wrap_delta(raw_pos, candidate);
        if (!pending || confirm > 20 || confirm < -20) {
            candidate = raw_pos;
            pending = true;
            return false;
        }
    }
    if ((d > 0 && ticks > std::numeric_limits<std::int32_t>::max() - d) ||
        (d < 0 && ticks < std::numeric_limits<std::int32_t>::min() - d))
        return false;
    ticks += d;
    raw = raw_pos;
    pending = false;
    return true;
}

std::vector<SerialPortInfo> list_serial_ports() {
    auto raw_ports = Serial::list_ports();
    std::vector<SerialPortInfo> out;
    out.reserve(raw_ports.size());
    for (auto& p : raw_ports) {
        out.push_back(SerialPortInfo{
            .port_name = std::move(p.port_name),
            .description = std::move(p.description),
            .vendor_id = p.vendor_id,
            .product_id = p.product_id,
            .serial_number = std::move(p.serial_number),
            .manufacturer = std::move(p.manufacturer),
        });
    }
    return out;
}

struct GelloReader::Impl {
    GelloConfig config;
    std::optional<Serial> serial;
    std::atomic<bool> running{false};
    std::thread worker;

    // Synchronizes RX callback with the polling thread.
    std::mutex rx_mutex;
    std::condition_variable rx_cv;
    GelloParser parser;
    std::uint8_t expected_id{};
    bool got_reply{};
    GelloRawSample pending_reply{};

    // Synchronizes published servo state with poll().
    mutable std::mutex state_mutex;
    std::array<GelloTracker, kGelloServoCount> trackers{};
    std::array<std::int16_t, kGelloServoCount> velocities{};
    std::array<Clock::time_point, kGelloServoCount> stamps{};
    std::array<bool, kGelloServoCount> has_stamp{};
    std::uint32_t scans{}, replies{}, timeouts{}, bad{}, jumps{};
    double scan_hz{};
    std::atomic<bool> connected{false};

    explicit Impl(GelloConfig cfg) : config(std::move(cfg)) {
        if (config.port.empty())
            throw std::invalid_argument("GelloConfig.port must not be empty");
        if (config.baud == 0)
            throw std::invalid_argument("GelloConfig.baud must be positive");
        if ((config.group_mask & 0x03u) == 0)
            throw std::invalid_argument("GelloConfig.group_mask must enable at least one group");
        if (config.servo_timeout_ms <= 0)
            throw std::invalid_argument("GelloConfig.servo_timeout_ms must be positive");

        auto opened = Serial::builder()
                          .baud_rate(config.baud)
                          .data_bits(DataBits::Eight)
                          .parity(Parity::None)
                          .stop_bits(StopBits::One)
                          .flow_control(FlowControl::None)
                          .auto_reconnect(true, std::chrono::milliseconds(
                                                    std::max(100, config.reconnect_interval_ms)))
                          .open(config.port);
        if (!opened) {
            throw std::runtime_error("Failed to open GELLO serial port '" + config.port +
                                     "': " + opened.error().message());
        }
        serial.emplace(std::move(opened.value()));
        connected.store(serial->is_open());

        serial->on_disconnect([this](const std::error_code&) {
            connected.store(false);
            rx_cv.notify_all();
        });
        serial->on_reconnect([this]() {
            connected.store(true);
            rx_cv.notify_all();
        });
        serial->on_data([this](std::span<const std::uint8_t> data) {
            std::lock_guard lock(rx_mutex);
            GelloRawSample sample{};
            for (std::uint8_t b : data) {
                if (parser.feed(b, sample) && sample.id == expected_id && !got_reply) {
                    pending_reply = sample;
                    got_reply = true;
                    rx_cv.notify_one();
                }
            }
        });

        running.store(true);
        worker = std::thread([this]() { loop(); });
    }

    ~Impl() { stop(); }

    void stop() {
        running.store(false);
        rx_cv.notify_all();
        if (worker.joinable())
            worker.join();
        if (serial) {
            serial->close();
            serial.reset();
        }
        connected.store(false);
    }

    bool read_one(std::size_t index) {
        const std::uint8_t id = gello_servo_id(index);
        const auto req = gello_read_request(id);
        {
            std::lock_guard lock(rx_mutex);
            parser.reset();
            parser.bad = 0;
            expected_id = id;
            got_reply = false;
        }
        const auto timeout = std::chrono::milliseconds(config.servo_timeout_ms);
        auto wr = serial->write(std::span<const std::uint8_t>(req.data(), req.size()), timeout);
        if (!wr) {
            std::lock_guard lock(state_mutex);
            ++timeouts;
            return false;
        }
        GelloRawSample sample{};
        bool ok = false;
        std::uint32_t bad_bytes = 0;
        {
            std::unique_lock lock(rx_mutex);
            ok = rx_cv.wait_for(lock, timeout, [this]() { return got_reply || !running.load(); });
            if (ok && got_reply)
                sample = pending_reply;
            else
                ok = false;
            bad_bytes = parser.bad;
            expected_id = 0;
        }
        if (!ok) {
            std::lock_guard lock(state_mutex);
            ++timeouts;
            bad += bad_bytes;
            return false;
        }
        const auto now = Clock::now();
        std::lock_guard lock(state_mutex);
        bad += bad_bytes;
        const bool accepted = trackers[index].update(sample.raw);
        if (accepted) {
            velocities[index] = sample.velocity;
            stamps[index] = now;
            has_stamp[index] = true;
            ++replies;
        } else {
            ++jumps;
        }
        return accepted;
    }

    void loop() {
        std::vector<std::size_t> order;
        order.reserve(kGelloServoCount);
        const bool left_on = (config.group_mask & 0x01u) != 0;
        const bool right_on = (config.group_mask & 0x02u) != 0;
        for (std::size_t i = 0; i < 7; ++i) {
            if (left_on)
                order.push_back(i);
            if (right_on)
                order.push_back(7 + i);
        }

        auto rate_t0 = Clock::now();
        std::uint32_t rate_scans = 0;

        while (running.load()) {
            if (!serial || !serial->is_open()) {
                connected.store(false);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            connected.store(true);
            for (std::size_t idx : order) {
                if (!running.load() || !serial->is_open())
                    break;
                if (!read_one(idx)) {
                    bool pending = false;
                    {
                        std::lock_guard lock(state_mutex);
                        pending = trackers[idx].pending;
                    }
                    if (pending && running.load() && serial->is_open())
                        (void)read_one(idx);
                }
            }
            const auto now = Clock::now();
            ++rate_scans;
            const double dt = std::chrono::duration<double>(now - rate_t0).count();
            {
                std::lock_guard lock(state_mutex);
                ++scans;
                if (dt >= 0.5) {
                    scan_hz = rate_scans / dt;
                    rate_t0 = now;
                    rate_scans = 0;
                }
            }
        }
    }

    GelloSnapshot poll() const {
        const auto now = Clock::now();
        GelloSnapshot snap{};
        snap.connected = connected.load();
        std::lock_guard lock(state_mutex);
        snap.scans = scans;
        snap.replies = replies;
        snap.timeouts = timeouts;
        snap.bad = bad;
        snap.jumps = jumps;
        snap.scan_hz = scan_hz;
        for (std::size_t i = 0; i < kGelloServoCount; ++i) {
            auto& s = snap.servos[i];
            s.id = gello_servo_id(i);
            s.raw = trackers[i].raw;
            s.ticks = trackers[i].ticks;
            s.velocity = velocities[i];
            if (has_stamp[i]) {
                s.age_s = std::chrono::duration<double>(now - stamps[i]).count();
                s.valid = trackers[i].valid;
            } else {
                s.age_s = std::numeric_limits<double>::infinity();
                s.valid = false;
            }
        }
        return snap;
    }
};

GelloReader::GelloReader(GelloConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

GelloReader::~GelloReader() = default;

GelloSnapshot GelloReader::poll() const { return impl_->poll(); }

bool GelloReader::is_open() const { return impl_->connected.load(); }

void GelloReader::close() { impl_->stop(); }

} // namespace panthera
