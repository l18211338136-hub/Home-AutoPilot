#include "communication/command_receiver.hpp"

#include <thread>

#include "communication/control_protocol.hpp"
#include "control/math_types.hpp"

namespace homefly
{

CommandReceiver::CommandReceiver(
	ILink& link,
	IClock& clock,
	uint32_t link_timeout_us
) noexcept
	: _link(link),
	  _clock(clock),
	  _link_timeout_us(link_timeout_us)
{
}

void CommandReceiver::write_state_(const SharedCommandState& state) noexcept
{
	// Mark a write in progress (odd sequence), publish the new state, then mark
	// the write complete (even). Readers retry if they observe an odd sequence
	// or if the sequence changed underneath them.
	const uint32_t before = _seq.load(std::memory_order_relaxed);
	_seq.store(before + 1u, std::memory_order_relaxed);
	std::atomic_thread_fence(std::memory_order_release);
	_state = state;
	std::atomic_thread_fence(std::memory_order_release);
	_seq.store(before + 2u, std::memory_order_relaxed);
}

SharedCommandState CommandReceiver::read_state_() const noexcept
{
	SharedCommandState snapshot{};

	// Single writer / single reader: a torn read is impossible because the
	// writer never blocks and the reader only copies. The seqlock guarantees the
	// copy is a consistent snapshot taken between two matching even sequences.
	for (;;)
	{
		const uint32_t s1 = _seq.load(std::memory_order_acquire);
		if ((s1 & 1u) != 0u)
		{
			std::this_thread::yield();
			continue;
		}
		std::atomic_thread_fence(std::memory_order_acquire);
		snapshot = _state;
		std::atomic_thread_fence(std::memory_order_acquire);
		const uint32_t s2 = _seq.load(std::memory_order_acquire);
		if (s1 == s2)
		{
			break;
		}
	}

	return snapshot;
}

void CommandReceiver::poll() noexcept
{
	uint8_t buffer[64];

	for (;;)
	{
		const size_t received = _link.receive(buffer, sizeof(buffer));
		if (received == 0)
		{
			break;
		}

		ControlFrame frame{};
		if (!parse_control_frame(buffer, static_cast<uint32_t>(received), frame))
		{
			_parse_errors.fetch_add(1u, std::memory_order_relaxed);
			continue;
		}

		_frames_received.fetch_add(1u, std::memory_order_relaxed);

		SharedCommandState next = read_state_();
		next.last_rx_us = _clock.now_us();
		next.received = true;

		switch (static_cast<ControlFrameType>(frame.type))
		{
		case ControlFrameType::Arm:
			next.setpoint.arm_request = true;
			next.setpoint.disarm_request = false;
			break;

		case ControlFrameType::Disarm:
			next.setpoint.disarm_request = true;
			next.setpoint.arm_request = false;
			break;

		case ControlFrameType::Setpoint:
		{
			float throttle = 0.0f;
			float roll = 0.0f;
			float pitch = 0.0f;
			float yaw_rate = 0.0f;
			unpack_setpoint_payload(frame.payload, throttle, roll, pitch, yaw_rate);

			// Reject any non-finite value (NaN/Inf) before it can reach the
			// controller, mixer or PWM output. FNV-1a only detects corruption,
			// not a semantically invalid-but-checksum-valid float
			// (HomeFly Flight Safety & Code Review skill 3.3). Keep the link
			// alive (received / stamp updated) but do NOT touch the latched
			// targets, so a transient bad frame cannot poison motor commands.
			if (!is_finite(throttle) || !is_finite(roll) ||
				!is_finite(pitch) || !is_finite(yaw_rate))
			{
				write_state_(next);
				break;
			}

			// Clamp throttle to [0,1]; roll/pitch/yaw_rate are clamped again by
			// the mixer, but must stay finite here (guaranteed by the check
			// above). The original "< 0 / > 1" clamp is unsafe for NaN because
			// both comparisons are false for NaN.
			if (throttle < 0.0f) throttle = 0.0f;
			else if (throttle > 1.0f) throttle = 1.0f;

			next.setpoint.throttle = throttle;
			next.setpoint.roll = roll;
			next.setpoint.pitch = pitch;
			next.setpoint.yaw_rate = yaw_rate;
			next.setpoint.disarm_request = false;
			break;
		}

		case ControlFrameType::Ping:
		case ControlFrameType::Mode:
		default:
			// Accepted (so the link stays alive) but no state change yet.
			break;
		}

		write_state_(next);
	}
}

Setpoint CommandReceiver::latest_setpoint() const noexcept
{
	SharedCommandState state = read_state_();

	if (_link_timeout_us == 0)
	{
		return state.setpoint;
	}

	// Never received anything, or stale beyond the timeout: fail safe to disarm.
	// Use an explicit flag (not `_last_rx_us == 0`) so a frame that legitimately
	// arrives at t=0 is not mistaken for "never received".
	const bool never_received = !state.received;
	const bool stale = state.received &&
		((_clock.now_us() - state.last_rx_us) > _link_timeout_us);

	if (never_received || stale)
	{
		state.setpoint.arm_request = false;
		state.setpoint.disarm_request = true;
	}

	return state.setpoint;
}

bool CommandReceiver::link_lost() const noexcept
{
	if (_link_timeout_us == 0)
	{
		return false;
	}

	const SharedCommandState state = read_state_();
	if (!state.received)
	{
		return true;
	}
	return (_clock.now_us() - state.last_rx_us) > _link_timeout_us;
}

} // namespace homefly
