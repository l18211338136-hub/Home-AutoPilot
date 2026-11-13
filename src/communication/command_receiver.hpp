#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "communication/i_link.hpp"
#include "flight_core/types.hpp"
#include "hal/i_clock.hpp"

namespace homefly
{

// Cross-thread command state, protected by a seqlock in CommandReceiver:
// poll() (low-priority task) is the only writer and latest_setpoint() /
// link_lost() (control loop) are the only readers. The struct must be trivially
// copyable so the seqlock reader can take a consistent snapshot without a mutex.
struct SharedCommandState
{
	Setpoint setpoint{};
	bool received{false};
	uint64_t last_rx_us{0};
};

static_assert(std::is_trivially_copyable<SharedCommandState>::value,
	"SharedCommandState must be trivially copyable for the seqlock snapshot");

// Parses uplink command frames from a ground station (over an ILink, e.g. a UDP
// socket) and exposes them as a latched Setpoint.
//
// Real-time contract (same as the Gazebo SITL transport):
//   - poll() is the ONLY method that does I/O. It must be called from a
//     low-priority task, never from the control loop.
//   - latest_setpoint() is called from the control loop and performs no I/O,
//     no allocation and no blocking: it just reads the latched state.
//
// Arm/Disarm are latched commands: an ARM frame requests arming and keeps the
// request asserted until a DISARM frame arrives. A SETPOINT frame updates the
// attitude/throttle targets and clears any pending disarm request (the pilot is
// clearly still present). On link loss (no frame within link_timeout_us) the
// receiver forces a disarm request, so the vehicle cannot keep flying blind.
//
// Thread safety (see HomeFly Flight Safety & Code Review skill 3.1):
//   poll() and latest_setpoint()/link_lost() run on different threads with no
//   shared mutex. All shared fields live in _state and are guarded by a seqlock
//   (_seq): the writer marks an odd sequence before writing and an even sequence
//   after; readers retry if the sequence changed or a write was in progress.
//   This gives a consistent snapshot of the whole struct with no lock in the
//   1 kHz control loop.
class CommandReceiver
{
public:
	// link_timeout_us == 0 disables the link-loss failsafe.
	CommandReceiver(
		ILink& link,
		IClock& clock,
		uint32_t link_timeout_us
	) noexcept;

	// Drain all currently available datagrams. Call from a low-priority task.
	void poll() noexcept;

	// Latest command state for the control loop. Forces disarm on link loss.
	Setpoint latest_setpoint() const noexcept;

	bool link_lost() const noexcept;

	uint32_t frames_received() const noexcept { return _frames_received.load(std::memory_order_relaxed); }
	uint32_t parse_errors() const noexcept { return _parse_errors.load(std::memory_order_relaxed); }

private:
	// Seqlock helpers. Writer: increment to odd, fence, copy, fence, increment
	// to even. Reader: read seq, if odd spin/yield, fence, copy, fence, re-read
	// seq and retry on mismatch.
	void write_state_(const SharedCommandState& state) noexcept;
	SharedCommandState read_state_() const noexcept;

	ILink& _link;
	IClock& _clock;
	uint32_t _link_timeout_us;
	mutable std::atomic<uint32_t> _seq{0};
	SharedCommandState _state{};
	std::atomic<uint32_t> _frames_received{0};
	std::atomic<uint32_t> _parse_errors{0};
};

} // namespace homefly
