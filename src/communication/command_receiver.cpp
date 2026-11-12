#include "communication/command_receiver.hpp"

#include "communication/control_protocol.hpp"

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
			++_parse_errors;
			continue;
		}

		++_frames_received;
		_last_rx_us = _clock.now_us();
		_received = true;

		switch (static_cast<ControlFrameType>(frame.type))
		{
		case ControlFrameType::Arm:
			_setpoint.arm_request = true;
			_setpoint.disarm_request = false;
			break;

		case ControlFrameType::Disarm:
			_setpoint.disarm_request = true;
			_setpoint.arm_request = false;
			break;

		case ControlFrameType::Setpoint:
		{
			float throttle = 0.0f;
			float roll = 0.0f;
			float pitch = 0.0f;
			float yaw_rate = 0.0f;
			unpack_setpoint_payload(frame.payload, throttle, roll, pitch, yaw_rate);

			// Keep targets in sane ranges; the controller re-clamps attitude,
			// but throttle must never exceed [0,1] before it reaches the mixer.
			if (throttle < 0.0f) throttle = 0.0f;
			else if (throttle > 1.0f) throttle = 1.0f;

			_setpoint.throttle = throttle;
			_setpoint.roll = roll;
			_setpoint.pitch = pitch;
			_setpoint.yaw_rate = yaw_rate;
			_setpoint.disarm_request = false;
			break;
		}

		case ControlFrameType::Ping:
		case ControlFrameType::Mode:
		default:
			// Accepted (so the link stays alive) but no state change yet.
			break;
		}
	}
}

Setpoint CommandReceiver::latest_setpoint() const noexcept
{
	Setpoint setpoint = _setpoint;

	if (_link_timeout_us == 0)
	{
		return setpoint;
	}

	// Never received anything, or stale beyond the timeout: fail safe to disarm.
	// Use an explicit flag (not `_last_rx_us == 0`) so a frame that legitimately
	// arrives at t=0 is not mistaken for "never received".
	const bool never_received = !_received;
	const bool stale = _received &&
		((_clock.now_us() - _last_rx_us) > _link_timeout_us);

	if (never_received || stale)
	{
		setpoint.arm_request = false;
		setpoint.disarm_request = true;
	}

	return setpoint;
}

bool CommandReceiver::link_lost() const noexcept
{
	if (_link_timeout_us == 0)
	{
		return false;
	}
	if (!_received)
	{
		return true;
	}
	return (_clock.now_us() - _last_rx_us) > _link_timeout_us;
}

} // namespace homefly
