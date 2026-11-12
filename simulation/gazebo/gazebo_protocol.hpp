#pragma once

#include <cstdint>

#include "hal/i_imu.hpp"

namespace homefly
{

// ---- HomeFly <-> Gazebo bridge wire protocol (version 1) -----------------
//
// One fixed size (64 B), naturally aligned, little endian UDP datagram in
// both directions. No protobuf, no Gazebo headers, no dynamic allocation:
// the flight controller side must stay buildable on a bare MCU toolchain.
//
// Direction:
//   Gazebo bridge -> HomeFly : type Imu    (gyro/accel + Gazebo sim time)
//   HomeFly -> Gazebo bridge : type Motor  (4 normalized thrusts)
//
// Layout (all fields explicitly initialized, no padding):
//   +0  uint32 magic        "HFGZ" (0x48465A47)
//   +4  uint32 seq          strictly increasing per sender
//   +8  uint64 timestamp_us Gazebo simulation time (microseconds)
//   +16 uint32 checksum     FNV-1a over the whole frame except [16..20)
//   +20 uint8  version      1
//   +21 uint8  type         GazeboFrameType
//   +22 uint16 flags        reserved
//   +24 float  values[8]    Imu: [0..2]=gyro rad/s, [3..5]=accel m/s^2,
//                                 [6]=temperature C, [7]=reserved
//                           Motor: [0..3]=motor[4] normalized [0,1]
//   +56 uint32 reserved
//   +60 uint32 tail         0
//
// Python side: struct.Struct("<IIQIBBH8fII") == 64 bytes.

constexpr uint32_t kGazeboFrameMagic = 0x48465A47u; // "HFGZ"
constexpr uint8_t kGazeboProtocolVersion = 1;
constexpr uint32_t kGazeboFrameSize = 64;
constexpr uint8_t kGazeboMotorCount = 4;

enum class GazeboFrameType : uint8_t
{
	Imu = 1,
	Motor = 2,
	Hello = 3,
	Shutdown = 4
};

struct GazeboFrame
{
	uint32_t magic{kGazeboFrameMagic};
	uint32_t seq{0};
	uint64_t timestamp_us{0};
	uint32_t checksum{0};
	uint8_t version{kGazeboProtocolVersion};
	uint8_t type{0};
	uint16_t flags{0};
	float values[8]{};
	uint32_t reserved{0};
	uint32_t tail{0};
};

static_assert(sizeof(GazeboFrame) == kGazeboFrameSize, "Gazebo frame must be exactly 64 bytes");

// FNV-1a over the raw frame bytes, skipping the checksum field itself.
uint32_t gazebo_frame_checksum(
	const GazeboFrame& frame
) noexcept;

void gazebo_frame_seal(
	GazeboFrame& frame
) noexcept;

// Rejects wrong magic / version / type / checksum / non-finite values.
bool gazebo_frame_validate(
	const GazeboFrame& frame
) noexcept;

GazeboFrame gazebo_frame_make_imu(
	uint32_t seq,
	uint64_t timestamp_us,
	const ImuData& data
) noexcept;

GazeboFrame gazebo_frame_make_motor(
	uint32_t seq,
	uint64_t timestamp_us,
	const float* motors,
	uint8_t count
) noexcept;

bool gazebo_frame_to_imu(
	const GazeboFrame& frame,
	ImuData& data
) noexcept;

bool gazebo_frame_to_motor(
	const GazeboFrame& frame,
	float* motors,
	uint8_t capacity
) noexcept;

} // namespace homefly
