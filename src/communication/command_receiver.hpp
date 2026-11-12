#pragma once

#include <cstdint>

#include "communication/i_link.hpp"
#include "flight_core/types.hpp"
#include "hal/i_clock.hpp"

namespace homefly
{

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

	uint32_t frames_received() const noexcept { return _frames_received; }
	uint32_t parse_errors() const noexcept { return _parse_errors; }

private:
	ILink& _link;
	IClock& _clock;
	uint32_t _link_timeout_us;
	Setpoint _setpoint{};
	uint64_t _last_rx_us{0};
	bool _received{false};
	uint32_t _frames_received{0};
	uint32_t _parse_errors{0};
};

} // namespace homefly
