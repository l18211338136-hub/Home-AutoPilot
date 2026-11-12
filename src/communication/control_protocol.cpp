#include "communication/control_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace homefly
{

bool build_control_frame(
	ControlFrameType type,
	const uint8_t* payload,
	uint8_t payload_len,
	uint16_t seq,
	ControlFrame& out
) noexcept
{
	if (payload_len > kControlPayloadSize)
	{
		return false;
	}

	out.magic = kControlFrameMagic;
	out.version = kControlFrameVersion;
	out.type = static_cast<uint8_t>(type);
	out.seq = seq;
	std::memset(out.payload, 0, kControlPayloadSize);
	if (payload != nullptr && payload_len > 0)
	{
		std::memcpy(out.payload, payload, payload_len);
	}

	// Checksum covers every field up to, but not including, the checksum field.
	out.checksum = control_frame_checksum(
		reinterpret_cast<const uint8_t*>(&out),
		static_cast<uint32_t>(offsetof(ControlFrame, checksum)));
	return true;
}

bool parse_control_frame(
	const uint8_t* buf,
	uint32_t len,
	ControlFrame& out
) noexcept
{
	if (buf == nullptr || len != kControlFrameSize)
	{
		return false;
	}

	std::memcpy(&out, buf, kControlFrameSize);

	if (out.magic != kControlFrameMagic)
	{
		return false;
	}
	if (out.version != kControlFrameVersion)
	{
		return false;
	}

	const uint32_t received = out.checksum;
	const uint32_t computed = control_frame_checksum(
		reinterpret_cast<const uint8_t*>(&out),
		static_cast<uint32_t>(offsetof(ControlFrame, checksum)));
	if (received != computed)
	{
		return false;
	}

	switch (static_cast<ControlFrameType>(out.type))
	{
	case ControlFrameType::Arm:
	case ControlFrameType::Disarm:
	case ControlFrameType::Setpoint:
	case ControlFrameType::Ping:
	case ControlFrameType::Mode:
		return true;
	default:
		return false;
	}
}

void pack_setpoint_payload(
	float throttle,
	float roll,
	float pitch,
	float yaw_rate,
	uint8_t payload_out[kControlPayloadSize]
) noexcept
{
	std::memcpy(payload_out + 0, &throttle, sizeof(float));
	std::memcpy(payload_out + 4, &roll, sizeof(float));
	std::memcpy(payload_out + 8, &pitch, sizeof(float));
	std::memcpy(payload_out + 12, &yaw_rate, sizeof(float));
}

void unpack_setpoint_payload(
	const uint8_t payload[kControlPayloadSize],
	float& throttle,
	float& roll,
	float& pitch,
	float& yaw_rate
) noexcept
{
	std::memcpy(&throttle, payload + 0, sizeof(float));
	std::memcpy(&roll, payload + 4, sizeof(float));
	std::memcpy(&pitch, payload + 8, sizeof(float));
	std::memcpy(&yaw_rate, payload + 12, sizeof(float));
}

} // namespace homefly
