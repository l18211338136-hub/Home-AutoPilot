#pragma once

#include <cstddef>
#include <cstdint>

#include "configuration/flight_config.hpp"
#include "flight_core/types.hpp"

namespace homefly
{

// Individual PreArm gates. See SKILL.md 3.4 (解锁前安全检查).
enum class PreArmCheck : uint8_t
{
	ImuInitialized,    // IMU 初始化完成
	ImuDataValid,     // 传感器数据有效（有限、量级合理、未过期）
	ThrottleAtMinimum,// 油门处于最低位置
	MotorConfigValid, // 电机配置合法
	FlightParamsValid,// 飞控参数合法
	NoSevereFault,    // 不存在未解除的严重故障
	CalibrationOk,    // 陀螺仪/加速度计校准完成（依赖校准子系统）
	BatteryInRange,   // 电池电压在允许范围（依赖电池监测）
	Count
};

// Per-cycle signals the PreArm gate needs. The controller must supply every
// field from a real source; nothing is assumed.
struct PreArmInputs
{
	bool imu_initialized{false};
	bool imu_data_valid{false};
	float throttle{0.0f};
	const FlightConfig* config{nullptr};

	// Calibration: the simplified FC has no calibration subsystem yet, so this
	// is wired as "not required" until one lands. See safety report 遗留风险.
	bool calibration_required{true};
	bool calibration_ok{false};

	// Battery: only enforced when a battery monitor is present.
	bool has_battery{false};
	float battery_voltage{0.0f};
	float battery_min_v{0.0f};
	float battery_max_v{0.0f};

	bool has_severe_fault{false};
};

struct PreArmReport
{
	bool passed[static_cast<size_t>(PreArmCheck::Count)]{};

	bool all_passed() const noexcept;
	PreArmCheck first_failure() const noexcept;
};

// Pure, stateless, no I/O. Called once per control cycle.
class PreArmChecker
{
public:
	static constexpr float kThrottleArmMax = 0.05f;

	PreArmReport check(
		const PreArmInputs& inputs
	) const noexcept;
};

} // namespace homefly
