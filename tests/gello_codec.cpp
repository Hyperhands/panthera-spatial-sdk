#include "panthera/gello.hpp"
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace panthera;

namespace {

std::array<std::uint8_t, 10> make_reply(std::uint8_t id, std::uint8_t err, std::int16_t pos,
                                        std::int16_t vel) {
    auto encode_sm = [](std::int16_t v) -> std::uint16_t {
        if (v < 0)
            return static_cast<std::uint16_t>(0x8000u | static_cast<std::uint16_t>(-v));
        return static_cast<std::uint16_t>(v);
    };
    const std::uint16_t p = encode_sm(pos);
    const std::uint16_t v = encode_sm(vel);
    std::array<std::uint8_t, 10> frame{
        0xff,
        0xff,
        id,
        6,
        err,
        static_cast<std::uint8_t>(p & 0xffu),
        static_cast<std::uint8_t>(p >> 8),
        static_cast<std::uint8_t>(v & 0xffu),
        static_cast<std::uint8_t>(v >> 8),
        0,
    };
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < 9; ++i)
        sum = static_cast<std::uint8_t>(sum + frame[i]);
    frame[9] = static_cast<std::uint8_t>(~sum);
    return frame;
}

} // namespace

int main() {
    // 1. ID <-> index mapping
    for (std::size_t i = 0; i < kGelloServoCount; ++i) {
        const auto id = gello_servo_id(i);
        assert(id != 0);
        assert(gello_servo_index(id) == static_cast<int>(i));
    }
    assert(gello_servo_index(0) == -1);
    assert(gello_servo_index(8) == -1);
    assert(gello_servo_index(10) == -1);
    assert(gello_servo_index(18) == -1);

    // 2. Read request framing & checksum
    const auto req = gello_read_request(1);
    assert(req[0] == 0xff && req[1] == 0xff && req[2] == 1 && req[3] == 4 && req[4] == 2 &&
           req[5] == 56 && req[6] == 4);
    std::uint8_t sum = 0;
    for (std::size_t i = 2; i < 7; ++i)
        sum = static_cast<std::uint8_t>(sum + req[i]);
    assert(req[7] == static_cast<std::uint8_t>(~sum));

    // 3. Parser: noise + echo + valid reply + bad checksum + error reply
    GelloParser parser{};
    GelloRawSample out{};
    const auto reply = make_reply(11, 0, 2048, -123);
    std::vector<std::uint8_t> stream{0x00, 0xaa, 0xff};
    stream.insert(stream.end(), req.begin(), req.end()); // echo (LEN=4) must not emit
    stream.insert(stream.end(), reply.begin(), reply.end());
    int parsed = 0;
    for (std::uint8_t b : stream) {
        if (parser.feed(b, out)) {
            ++parsed;
            assert(out.id == 11);
            assert(out.raw == 2048);
            assert(out.velocity == -123);
        }
    }
    assert(parsed == 1);

    // Negative sign-magnitude position wraps cleanly into [0, 4095].
    const auto neg_reply = make_reply(7, 0, -252, 15);
    parsed = 0;
    for (std::uint8_t b : neg_reply) {
        if (parser.feed(b, out)) {
            ++parsed;
            assert(out.id == 7);
            assert(out.raw == 4096 - 252);
            assert(out.velocity == 15);
        }
    }
    assert(parsed == 1);

    // Corrupted checksum rejected.
    auto bad_reply = make_reply(2, 0, 500, 0);
    bad_reply[9] ^= 0x01;
    parsed = 0;
    for (std::uint8_t b : bad_reply)
        parsed += parser.feed(b, out) ? 1 : 0;
    assert(parsed == 0);

    // Non-zero status byte still yields position (matching gello_compare_driver) with vel=0.
    const auto err_reply = make_reply(2, 1, 500, 99);
    parsed = 0;
    for (std::uint8_t b : err_reply) {
        if (parser.feed(b, out)) {
            ++parsed;
            assert(out.id == 2);
            assert(out.raw == 500);
            assert(out.velocity == 0);
        }
    }
    assert(parsed == 1);

    // Sync read request framing & checksum.
    const auto sync_req = gello_sync_read_request({1, 2, 3, 4, 5, 6, 7});
    assert(sync_req.size() == 15);
    assert(sync_req[0] == 0xff && sync_req[1] == 0xff && sync_req[2] == 0xfe &&
           sync_req[3] == 11 && sync_req[4] == 0x82 && sync_req[5] == 56 && sync_req[6] == 4);
    sum = 0;
    for (std::size_t i = 2; i + 1 < sync_req.size(); ++i)
        sum = static_cast<std::uint8_t>(sum + sync_req[i]);
    assert(sync_req.back() == static_cast<std::uint8_t>(~sum));

    // 4. Tracker: multi-turn rollover and fast motion without artificial jump lockouts.
    GelloTracker tracker{};
    assert(tracker.update(4090));
    assert(tracker.ticks == 4090);
    assert(tracker.update(5)); // +11 across 4096 rollover
    assert(tracker.ticks == 4101);
    assert(tracker.update(4095)); // -10 back across rollover
    assert(tracker.ticks == 4091);

    // Fast large move (> 100 ticks) is accepted immediately via shortest-path unwrap.
    assert(tracker.update(3500));
    assert(!tracker.pending);
    assert(tracker.raw == 3500);
    assert(tracker.ticks == 4091 + (3500 - 4095));

    // 5. Serial port enumeration runs without throwing.
    const auto ports = list_serial_ports();
    std::cout << "gello_codec passed (" << ports.size() << " serial ports detected)\n";
    return 0;
}
