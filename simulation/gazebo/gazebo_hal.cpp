#include "simulation/gazebo/gazebo_hal.hpp"

#include <algorithm>

#include "control/motor_mixer.hpp"

namespace homefly
{

static_assert(MotorMixer::kMotorCount == kGazeboMotorCount,
	"Gazebo protocol assumes a Quad-X with 4 motors");

uint64_t GazeboClock::now_us() const
{
	return _served_us.load(std::memory_order_acquire);
}

void GazeboClock::set_served_us(
	uint64_t timestamp_us
) noexcept
{
	_served_us.store(timestamp_us, std::memory_order_release);
}

GazeboImu::GazeboImu(
	GazeboTransport& transport,
	GazeboClock& clock
) noexcept
	: _transport(transport),
	  _clock(clock)
{
}

void GazeboImu::set_fault(
	bool fault
) noexcept
{
	_fault = fault;
}

bool GazeboImu::initialize()
{
	_fault = false;
	_initialized = true;
	return true;
}

bool GazeboImu::read(
	ImuData& data
)
{
	GazeboFrame frame{};

	// Always consume one datagram even when faulted or uninitialized. In SITL
	// the bridge publishes exactly one IMU frame per motor frame: skipping
	// the pop would leave the ring full, let the control loop spin at full
	// speed, overflow the motor ring and break lock step with Gazebo.
	if (!_transport.pop_imu(frame))
	{
		return false;
	}

	if (!_initialized || _fault || !gazebo_frame_to_imu(frame, data))
	{
		return false;
	}

	_clock.set_served_us(data.timestamp_us);
	_last_sample = data;
	return true;
}

const ImuData& GazeboImu::last_sample() const noexcept
{
	return _last_sample;
}

GazeboMotor::GazeboMotor(
	GazeboTransport& transport,
	GazeboClock& clock
) noexcept
	: _transport(transport),
	  _clock(clock)
{
}

bool GazeboMotor::initialize()
{
	_armed = false;
	_initialized = true;

	for (float& value : _last_outputs)
	{
		value = 0.0f;
	}

	return true;
}

bool GazeboMotor::arm()
{
	_armed = true;
	return true;
}

void GazeboMotor::disarm()
{
	_armed = false;
	// Do NOT publish here. FlightController::output_safe_state() calls
	// disarm() immediately followed by write(); publishing in both would
	// emit two motor frames in one control cycle and break lock step.
}

bool GazeboMotor::write(
	const float* outputs,
	uint8_t count
)
{
	if (!_initialized || outputs == nullptr || count != MotorMixer::kMotorCount)
	{
		return false;
	}

	if (!_armed)
	{
		publish(nullptr);
		return true;
	}

	float values[MotorMixer::kMotorCount]{};

	for (uint8_t i = 0; i < MotorMixer::kMotorCount; ++i)
	{
		values[i] = std::clamp(outputs[i], 0.0f, 1.0f);
	}

	publish(values);
	return true;
}

bool GazeboMotor::is_armed() const
{
	return _armed;
}

uint8_t GazeboMotor::channel_count() const
{
	return MotorMixer::kMotorCount;
}

bool GazeboMotor::motors_zero() const noexcept
{
	for (const float value : _last_outputs)
	{
		if (value > 1.0e-6f)
		{
			return false;
		}
	}

	return true;
}

void GazeboMotor::publish(
	const float* outputs
) noexcept
{
	float values[MotorMixer::kMotorCount]{};

	if (outputs != nullptr)
	{
		for (uint8_t i = 0; i < MotorMixer::kMotorCount; ++i)
		{
			values[i] = outputs[i];
		}
	}

	for (uint8_t i = 0; i < MotorMixer::kMotorCount; ++i)
	{
		_last_outputs[i] = values[i];
	}

	const uint32_t seq = _seq.fetch_add(1u, std::memory_order_relaxed) + 1u;
	const GazeboFrame frame = gazebo_frame_make_motor(
		seq, _clock.now_us(), values, MotorMixer::kMotorCount);
	_transport.push_motor(frame);
}

} // namespace homefly
