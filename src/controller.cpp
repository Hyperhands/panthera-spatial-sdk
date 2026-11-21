#include "panthera/controller.hpp"
#include "control_law.hpp"
#include "session_timer.hpp"
#include <atomic>
#include <mutex>
#include <stdexcept>
namespace panthera {
namespace {
using namespace std::chrono_literals;
using Pair = std::array<Joints, 2>;
JointVector join(const Pair& p) {
    JointVector q;
    for (int i = 0; i < 12; ++i)
        q[i] = p[i / 6][i % 6];
    return q;
}
void split(const JointVector& q, Pair& p) {
    for (int i = 0; i < 12; ++i)
        p[i / 6][i % 6] = static_cast<float>(q[i]);
}
size_t index(Side s) { return s == Side::left ? 0 : 1; }
// Setters serialize writers. The servo tries one versioned snapshot per tick
// and keeps its last complete target if a writer is preempted during publication.
class TargetBuffer {
    std::atomic<std::uint64_t> sequence{};
    std::atomic<TargetType> type{TargetType::joint};
    std::array<std::atomic<double>, 16> matrix{};
    std::array<std::atomic<double>, 6> angles{}, twist{};

  public:
    void store(const ArmTarget& t) {
        ++sequence;
        type.store(t.type);
        for (size_t j = 0; j < 16; ++j)
            matrix[j].store(t.pose.data()[j]);
        for (size_t j = 0; j < 6; ++j) {
            angles[j].store(t.joint[j]);
            twist[j].store(t.twist[j]);
        }
        ++sequence;
    }
    void read_if_new(ArmTarget& target, std::uint64_t& seen) const {
        const auto before = sequence.load();
        if (before == seen || before % 2)
            return;
        ArmTarget copy;
        copy.type = type.load();
        for (size_t j = 0; j < 16; ++j)
            copy.pose.data()[j] = matrix[j].load();
        for (size_t j = 0; j < 6; ++j) {
            copy.joint[j] = angles[j].load();
            copy.twist[j] = twist[j].load();
        }
        if (sequence.load() == before) {
            target = copy;
            seen = before;
        }
    }
};
struct HandTarget {
    Joints q;
    std::uint16_t duration;
};
} // namespace
struct DualArm::Impl {
    detail::ControlLaw law;
    DualSession session;
    detail::SessionTimer timer;
    std::thread worker;
    std::atomic<bool> running{false};
    std::mutex lifecycle, mutex;
    std::array<TargetBuffer, 2> targets;
    std::array<std::optional<HandTarget>, 2> hands;
    ControlState snapshot;
    bool expert{};
    Impl(BoardAddress l, BoardAddress r, ControlConfig config)
        : law(std::move(config)), session(l, r, [](Side side, const Joints& q) {
              auto g = arm_geometry(
                           index(side),
                           Eigen::Map<const Eigen::Matrix<float, 6, 1>>(q.data()).cast<double>())
                           .gravity;
              Joints out;
              std::copy(g.data(), g.data() + 6, out.begin());
              return out;
          }) {}
    void publish() {
        for (auto s : {Side::left, Side::right}) {
            law.state.boards[index(s)] = session.state(s);
            law.state.rpc[index(s)] = session.rpc_timing(s);
        }
        std::lock_guard guard(mutex);
        snapshot = law.state;
    }
    void run() {
        auto deadline = HostClock::now();
        std::array<ArmTarget, 2> goals{};
        std::array<std::uint64_t, 2> seen{};
        for (int s = 0; s < 2; ++s)
            goals[s].joint = law.state.reference.segment<6>(6 * s);
        try {
            while (running.load()) {
                for (int s = 0; s < 2; ++s)
                    targets[s].read_if_new(goals[s], seen[s]);
                std::array<std::optional<HandTarget>, 2> pending;
                {
                    std::unique_lock guard(mutex, std::try_to_lock);
                    if (guard)
                        pending.swap(hands);
                }
                const auto last_reference = law.state.reference;
                auto status = session.control([&](const Pair& measured, Pair& q, Pair& dq) {
                    law.step(join(measured), goals);
                    split(law.state.reference, q);
                    split(law.state.velocity, dq);
                });
                if (status == HoldStatus::frozen) {
                    law.state.reference = last_reference;
                    law.freeze();
                }
                for (auto side : {Side::left, Side::right}) {
                    auto i = index(side);
                    if (pending[i] &&
                        (law.state.frozen ||
                         !session.send_hand(side, pending[i]->q, pending[i]->duration))) {
                        std::lock_guard guard(mutex);
                        if (!hands[i])
                            hands[i] = pending[i];
                    }
                }
                ++law.state.cycles;
                deadline += 2ms;
                auto now = HostClock::now();
                if (deadline <= now) {
                    auto skipped = 1 + (now - deadline) / 2ms;
                    deadline += skipped * 2ms;
                    law.state.missed_cycles += skipped;
                }
                publish();
                timer.until(deadline);
            }
        } catch (const std::exception& e) {
            law.state.error = e.what();
            if (session.active())
                try {
                    session.stop();
                } catch (const std::exception& stop) {
                    law.state.error += ": ";
                    law.state.error += stop.what();
                }
            law.state.enabled = false;
            running = false;
            publish();
        }
    }
    void join_worker() {
        running = false;
        if (worker.joinable())
            worker.join();
    }
    void require_control() const {
        if (!snapshot.enabled || snapshot.expert)
            throw std::logic_error("Normal control is inactive");
    }
};
DualArm::DualArm(BoardAddress l, BoardAddress r, ControlConfig config)
    : impl_(std::make_unique<Impl>(l, r, std::move(config))) {}
DualArm::~DualArm() {
    try {
        softStop();
    } catch (...) {
    }
}
void DualArm::enable(std::chrono::milliseconds timeout) { enable(timeout, nullptr); }
void DualArm::enable(std::chrono::milliseconds timeout, void (*check_interrupt)()) {
    auto& i = *impl_;
    std::lock_guard guard(i.lifecycle);
    if (i.running || i.expert)
        throw std::logic_error("Controller already active");
    i.join_worker();
    i.session.enable_checked(
        timeout, [&](const Pair& p) { i.law.reset(join(p)); }, check_interrupt);
    try {
        {
            std::lock_guard lock(i.mutex);
            i.hands = {};
            for (int side = 0; side < 2; ++side) {
                ArmTarget goal;
                goal.joint = i.law.state.reference.segment<6>(6 * side);
                i.targets[side].store(goal);
            }
        }
        i.law.state.enabled = true;
        i.publish();
        i.running = true;
        i.worker = std::thread([&i] { i.run(); });
    } catch (...) {
        i.running = false;
        i.law.state.enabled = false;
        i.publish();
        i.session.stop();
        throw;
    }
}
void DualArm::setPoseTarget(Side s, const Eigen::Isometry3d& t, const Six& ff) {
    if (!t.matrix().allFinite() || !t.linear().isUnitary(1e-8) || t.linear().determinant() < 0 ||
        !t.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) || !ff.allFinite())
        throw std::invalid_argument("Invalid TCP pose or feedforward twist");
    auto& i = *impl_;
    std::lock_guard guard(i.mutex);
    i.require_control();
    ArmTarget g;
    g.type = TargetType::pose;
    g.pose = t;
    g.twist = ff;
    i.targets[index(s)].store(g);
}
void DualArm::setTwistTarget(Side s, const Six& v) {
    if (!v.allFinite())
        throw std::invalid_argument("Nonfinite twist");
    auto& i = *impl_;
    std::lock_guard guard(i.mutex);
    i.require_control();
    ArmTarget g;
    g.type = TargetType::twist;
    g.twist = v;
    i.targets[index(s)].store(g);
}
void DualArm::setJointTarget(Side s, const Six& q) {
    if (!q.allFinite())
        throw std::invalid_argument("Nonfinite joint target");
    auto& i = *impl_;
    std::lock_guard guard(i.mutex);
    i.require_control();
    ArmTarget g;
    g.type = TargetType::joint;
    g.joint = q;
    // Only the internally seeded hold target can be below nominal J2 zero.
    g.joint[1] = std::max(0., q[1]);
    i.targets[index(s)].store(g);
}
void DualArm::setHandTarget(Side s, const Joints& q, std::uint16_t duration) {
    if (!std::all_of(q.begin(), q.end(), [](auto x) { return std::isfinite(x); }))
        throw std::invalid_argument("Nonfinite hand target");
    auto& i = *impl_;
    std::lock_guard guard(i.mutex);
    i.require_control();
    i.hands[index(s)] = HandTarget{q, duration};
}
ControlState DualArm::state() const {
    std::lock_guard guard(impl_->mutex);
    return impl_->snapshot;
}
void DualArm::softStop() {
    auto& i = *impl_;
    std::lock_guard guard(i.lifecycle);
    i.join_worker();
    i.expert = false;
    i.law.state.enabled = i.law.state.expert = false;
    i.publish();
    i.session.stop();
}
void DualArm::enterExpertMode(ExpertAck ack) {
    if (ack != ExpertAck::IUnderstandNoSafety)
        throw std::invalid_argument("Explicit expert acknowledgement required");
    auto& i = *impl_;
    std::lock_guard guard(i.lifecycle);
    i.join_worker();
    if (!i.session.active())
        throw std::logic_error("Enable first");
    i.expert = i.law.state.expert = true;
    i.publish();
}
void DualArm::sendMit(Side s, const std::array<float, 30>& mit) {
    auto& i = *impl_;
    std::lock_guard guard(i.lifecycle);
    if (!i.expert)
        throw std::logic_error("Expert mode required");
    i.session.raw(s, mit);
}
std::array<std::uint16_t, 2> DualArm::local_ports(Side s) const {
    return impl_->session.local_ports(s);
}
} // namespace panthera
