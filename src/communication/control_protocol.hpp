#pragma once

#include <cstddef>
#include <cstdint>

namespace homefly
{

// ---------------------------------------------------------------------------
// Minimal, dependency-free uplink protocol for WiFi ground-station control.
// It mirrors the shape of the Gazebo SITL frame (self-describing, little-endian,
// FNV-1a checksum, seq for dedup) but is a separate channel: command frames
// from the GCS, not simulator IMU/motor traffic.
//
// The control loop never touches this directly: CommandReceiver parses frames
// off the ILink in a low-priority task and only exposes a latched Setpoint,
// exactly like the Gazebo SITL transport pattern.
// ---------------------------------------------------------------------------

enum class ControlFrameType : uint8_t
{
	Arm = 1,
	Disarm = 2,
	Setpoint = 3,
	Ping = 4,
	Mode = 5
};

// Fixed 28-byte wire frame. payload is 16 bytes so a Setpoint (4 floats) fits
// exactly; other frame types leave it zeroed.
struct ControlFrame
{
	uint32_t magic{0};
	uint8_t version{0};
	uint8_t type{0};
	uint16_t seq{0};
	uint8_t payload[16]{};
	uint32_t checksum{0};
};

constexpr uint32_t kControlFrameMagic = 0x48464C43u; // "HFLC"
constexpr uint8_t kControlFrameVersion = 1;
constexpr uint32_t kControlFrameSize = 28u;
constexpr uint32_t kControlPayloadSize = 16u;

// FNV-1a (32-bit). Reimplemented locally so communication/ does not depend on
// the logging module.
inline uint32_t control_frame_checksum(
	const uint8_t* data,
	uint32_t size,
	uint32_t seed = 2166136261u
) noexcept
{
	uint32_t hash = seed;
	for (uint32_t i = 0; i < size; ++i)
	{
		hash ^= data[i];
		hash *= 16777619u;
	}
	return hash;
}

// Build a frame. payload may be null when payload_len == 0. Returns false if
// payload_len exceeds the payload capacity.
bool build_control_frame(
	ControlFrameType type,
	const uint8_t* payload,
	uint8_t payload_len,
	uint16_t seq,
	ControlFrame& out
) noexcept;

// Parse a raw datagram into a ControlFrame, validating size, magic, version,
// checksum and a known frame type. Returns false on any mismatch.
bool parse_control_frame(
	const uint8_t* buf,
	uint32_t len,
	ControlFrame& out
) noexcept;

// Pack/unpack the Setpoint payload (4 little-endian floats). Uses memcpy to
// avoid type-punning undefined behaviour.
void pack_setpoint_payload(
	float throttle,
	float roll,
	float pitch,
	float yaw_rate,
	uint8_t payload_out[kControlPayloadSize]
) noexcept;

void unpack_setpoint_payload(
	const uint8_t payload[kControlPayloadSize],
	float& throttle,
	float& roll,
	float& pitch,
	float& yaw_rate
) noexcept;

} // namespace homefly
