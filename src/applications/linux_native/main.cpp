#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <thread>

#include "applications/linux_native/simulation_source.hpp"
#include "communication/telemetry.hpp"
#include "configuration/flight_config.hpp"
#include "control/math_types.hpp"
#include "flight_core/flight_controller.hpp"
#include "hal/i_clock.hpp"
#include "platforms/common/logging/flight_data.hpp"
#include "platforms/common/logging/flight_log.hpp"
#include "platforms/common/logging/flight_log_storage.hpp"
#include "platforms/common/logging/log.hpp"
#include "platforms/common/logging/log_backend.hpp"
#include "simulation/sim_link.hpp"

namespace
{

struct Options
{
	uint32_t steps{2000};
	uint32_t throttle_ms{0};
	const char* flight_log_path{"homefly_flight.hfl"};
	bool failsafe{false};
	bool realtime{false};
	bool verbose{false};
	bool no_flight_log{false};
	bool gazebo{false};
	bool help{false};
	uint16_t gazebo_local_port{14550};
	uint16_t gazebo_remote_port{14551};
	uint32_t gazebo_timeout_ms{3000};
	const char* gazebo_host{"127.0.0.1"};
};

Options parse_options(
	int argc,
	char** argv
)
{
	Options options{};

	for (int i = 1; i < argc; ++i)
	{
		if (std::strcmp(argv[i], "--failsafe") == 0)
		{
			options.failsafe = true;
		}
		else if (std::strcmp(argv[i], "--realtime") == 0)
		{
			options.realtime = true;
		}
		else if (std::strcmp(argv[i], "--verbose") == 0)
		{
			options.verbose = true;
		}
		else if (std::strcmp(argv[i], "--no-flight-log") == 0)
		{
			options.no_flight_log = true;
		}
		else if (std::strcmp(argv[i], "--gazebo") == 0)
		{
			options.gazebo = true;
		}
		else if (std::strcmp(argv[i], "--help") == 0)
		{
			options.help = true;
		}
		else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc)
		{
			options.steps = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
		}
		else if (std::strcmp(argv[i], "--throttle") == 0 && i + 1 < argc)
		{
			options.throttle_ms = static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
		}
		else if (std::strcmp(argv[i], "--flight-log") == 0 && i + 1 < argc)
		{
			options.flight_log_path = argv[++i];
		}
		else if (std::strcmp(argv[i], "--gazebo-local-port") == 0 && i + 1 < argc)
		{
			options.gazebo_local_port =
				static_cast<uint16_t>(std::strtoul(argv[++i], nullptr, 10));
		}
		else if (std::strcmp(argv[i], "--gazebo-remote-port") == 0 && i + 1 < argc)
		{
			options.gazebo_remote_port =
				static_cast<uint16_t>(std::strtoul(argv[++i], nullptr, 10));
		}
		else if (std::strcmp(argv[i], "--gazebo-timeout") == 0 && i + 1 < argc)
		{
			options.gazebo_timeout_ms =
				static_cast<uint32_t>(std::strtoul(argv[++i], nullptr, 10));
		}
		else if (std::strcmp(argv[i], "--gazebo-host") == 0 && i + 1 < argc)
		{
			options.gazebo_host = argv[++i];
		}
	}

	return options;
}

void print_usage()
{
	std::printf("HomeFly Linux Native simulation\n");
	std::printf("  --steps N                 control cycles (default 2000 = 2 s at 1 kHz)\n");
	std::printf("  --failsafe                drop the IMU at 40%% of the run and verify protection\n");
	std::printf("  --realtime                pace the loop with the host monotonic clock\n");
	std::printf("  --verbose                 enable DEBUG level logs\n");
	std::printf("  --throttle MS             minimum interval between same (tag, level) logs\n");
	std::printf("  --flight-log PATH         binary flight log file (default homefly_flight.hfl)\n");
	std::printf("  --no-flight-log           disable flight log recording\n");
	std::printf("  --gazebo                  external SITL: IMU/MOTOR over UDP instead of SimPlant\n");
	std::printf("  --gazebo-host IP          bridge host (default 127.0.0.1)\n");
	std::printf("  --gazebo-local-port N     UDP port receiving IMU (default 14550)\n");
	std::printf("  --gazebo-remote-port N    UDP port sending motors (default 14551)\n");
	std::printf("  --gazebo-timeout MS       wait for the next Gazebo IMU frame (default 3000)\n");
	std::printf("  --help                    show this message\n");
}

const char* mode_name(
	homefly::FlightMode mode
)
{
	switch (mode)
	{
	case homefly::FlightMode::Disarmed:
		return "disarmed";
	case homefly::FlightMode::Stabilize:
		return "stabilize";
	case homefly::FlightMode::Failsafe:
		return "failsafe";
	default:
		return "unknown";
	}
}

// Stable fingerprint of the tuning used for this flight. Hashed field by
// field (never over the whole struct, which contains padding bytes).
uint32_t config_fingerprint(
	const homefly::FlightConfig& config
) noexcept
{
	uint32_t hash = 2166136261u;
	const auto add = [&hash](const void* data, size_t size)
	{
		hash = homefly::flight_log_checksum(
			static_cast<const uint8_t*>(data), static_cast<uint32_t>(size), hash);
	};

	add(&config.control_period_us, sizeof(config.control_period_us));
	add(&config.imu_timeout_us, sizeof(config.imu_timeout_us));
	add(&config.overrun_factor, sizeof(config.overrun_factor));
	add(&config.gyro_weight, sizeof(config.gyro_weight));
	add(&config.angle_roll, sizeof(config.angle_roll));
	add(&config.angle_pitch, sizeof(config.angle_pitch));
	add(&config.rate_roll, sizeof(config.rate_roll));
	add(&config.rate_pitch, sizeof(config.rate_pitch));
	add(&config.rate_yaw, sizeof(config.rate_yaw));
	add(&config.max_rate_command, sizeof(config.max_rate_command));
	add(&config.max_angle_command_rad, sizeof(config.max_angle_command_rad));
	add(&config.max_torque_command, sizeof(config.max_torque_command));
	add(&config.motor_output_min, sizeof(config.motor_output_min));
	add(&config.motor_output_max, sizeof(config.motor_output_max));
	return hash;
}

// Used only when flight log recording is disabled, to keep the SPSC ring
// from overflowing. Pure memory, no I/O.
class DiscardSink final : public homefly::ILogSink
{
public:
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override
	{
		(void)data;
		(void)length;
		return true;
	}
};

} // namespace

int main(
	int argc,
	char** argv
)
{
	const Options options = parse_options(argc, argv);

	if (options.help)
	{
		print_usage();
		return 0;
	}

	using namespace homefly;

	const FlightConfig config = default_quad_x_config();
	const float dt_s = static_cast<float>(config.control_period_us) * 1.0e-6f;

	// ---- Composition root: the ONLY place where the simulation source is
	// chosen. FlightCore / logging / telemetry / flight log are identical for
	// the in-process plant and for an external Gazebo SITL session.
	SimulationSourceConfig source_config{};
	source_config.control_period_us = config.control_period_us;
	source_config.steps = options.steps;
	source_config.realtime = options.realtime && !options.gazebo;
	source_config.gazebo = options.gazebo;
	source_config.gazebo_host = options.gazebo_host;
	source_config.gazebo_local_port = options.gazebo_local_port;
	source_config.gazebo_remote_port = options.gazebo_remote_port;
	source_config.gazebo_timeout_ms = options.gazebo_timeout_ms;

	std::unique_ptr<SimulationSource> source = create_simulation_source(source_config);

	if (!source)
	{
		std::fprintf(stderr, "HomeFly: cannot start simulation source (UDP port in use?)\n");
		return 1;
	}

	IImu& imu = source->imu();
	IMotor& motor = source->motor();
	IClock& clock = source->clock();
	(void)imu;
	(void)motor;

	LogConfig log_config{};
	log_config.min_level = options.verbose ? LogLevel::Debug : LogLevel::Info;
	log_config.rate_limit_ms = options.throttle_ms;
	log_init(clock, log_backend_console(), log_config);

	if (options.gazebo && options.realtime)
	{
		HF_LOG_WARN("SYSTEM", "--realtime ignored in --gazebo mode (Gazebo owns simulation time)");
	}

	SimLink link;
	TelemetryReporter telemetry(link);
	FlightController controller(imu, motor, clock, config);
	FlightDataRecorder flight_data;
	FlightLogWriter flight_log;
	DiscardSink discard_sink;

	// ---- Open the platform flight log storage (link time bound). ----
	bool recording = !options.no_flight_log;

	if (recording)
	{
		if (!flight_log_storage_open(options.flight_log_path))
		{
			HF_LOG_ERROR("FLIGHTLOG", "cannot open %s, recording disabled",
				options.flight_log_path);
			recording = false;
		}
		else
		{
			FlightLogHeader header{};
			header.session_id = static_cast<uint64_t>(std::time(nullptr));
			header.start_time_us = 0;
			header.control_period_us = config.control_period_us;
			header.config_hash = config_fingerprint(config);
			std::memcpy(header.platform, "linux_native", sizeof("linux_native"));

			if (!flight_log.begin(flight_log_storage(), header))
			{
				HF_LOG_ERROR("FLIGHTLOG", "header write failed, recording disabled");
				flight_log_storage_close();
				recording = false;
			}
		}
	}

	HF_LOG_INFO("SYSTEM", "HomeFly booting (platform=linux_native, loop=%uHz, steps=%u, scenario=%s, source=%s)",
		static_cast<unsigned>(1000000u / config.control_period_us),
		static_cast<unsigned>(options.steps),
		options.failsafe ? "imu_dropout" : "stabilize",
		source->name());
	HF_LOG_DEBUG("CORE", "control period=%u us, imu timeout=%u us",
		config.control_period_us, config.imu_timeout_us);

	if (!controller.initialize())
	{
		HF_LOG_FATAL("SYSTEM", "FlightController initialization failed");
		log_drain();
		if (recording)
		{
			flight_log.finish(flight_data.dropped());
			flight_log_storage_close();
		}
		source->shutdown();
		log_shutdown();
		return 1;
	}

	HF_LOG_INFO("SENSOR", "simulated IMU initialized");
	HF_LOG_INFO("MOTOR", "simulated motor driver initialized");
	HF_LOG_INFO("FLIGHTLOG", "recording=%s path=%s block_records=%u",
		recording ? "yes" : "no", recording ? options.flight_log_path : "none",
		static_cast<unsigned>(kFlightLogBlockRecords));

	if (options.gazebo)
	{
		HF_LOG_INFO("GAZEBO", "waiting for bridge on udp://%s (imu=%u -> motor=%u), timeout=%u ms",
			options.gazebo_host,
			static_cast<unsigned>(options.gazebo_local_port),
			static_cast<unsigned>(options.gazebo_remote_port),
			static_cast<unsigned>(options.gazebo_timeout_ms));
	}

	// Single threaded startup phase: flushing here is safe and makes boot logs
	// visible before the low priority logging task takes over.
	log_drain();

	// ---- Low priority logging task. ----
	// This is the ONLY place where text log I/O and flight log file writes
	// happen. On a real MCU map this to a FreeRTOS task / idle hook.
	std::atomic<bool> stop_logging{false};
	std::thread logging_task([&stop_logging, &flight_log, &flight_data, &discard_sink]() noexcept
	{
		while (!stop_logging.load(std::memory_order_acquire))
		{
			if (flight_log.active())
			{
				flight_log.pump(flight_data, kFlightLogBlockRecords);
			}
			else
			{
				flight_data.drain_to(discard_sink, kFlightLogBlockRecords);
			}

			log_drain(8u);
			std::this_thread::yield();
		}

		// Final sweep: empty both rings before the file footer is written.
		if (flight_log.active())
		{
			flight_log.pump(flight_data);
		}
		else
		{
			flight_data.drain_to(discard_sink);
		}

		log_drain();
	});

	Setpoint setpoint{};
	setpoint.throttle = 0.5f;
	setpoint.arm_request = true;

	const uint32_t fault_step = options.steps * 2u / 5u;
	FlightMode previous_mode = controller.mode();

	for (uint32_t step = 0; step < options.steps; ++step)
	{
		if (options.failsafe && step == fault_step)
		{
			source->inject_fault(true);
			HF_LOG_WARN("IMU", "injecting sensor dropout at step %u", step);
		}

		if (!source->begin_cycle())
		{
			HF_LOG_ERROR("SYSTEM", "simulation source stopped at step %u (Gazebo bridge timeout?)", step);
			break;
		}

		controller.step(setpoint);
		source->end_cycle(dt_s);
		telemetry.report(controller.state(), controller.output(),
			controller.mode(), controller.failsafe_reason());

		if (controller.mode() != previous_mode)
		{
			HF_LOG_WARN("FLIGHT", "mode %s -> %s (reason=%u)",
				mode_name(previous_mode), mode_name(controller.mode()),
				static_cast<unsigned>(controller.failsafe_reason()));
			previous_mode = controller.mode();
		}

		// Structured high frequency data: one POD copy, no formatting, no I/O.
		const VehicleState& state = controller.state();
		const ImuData sample = source->sensor_sample(state.timestamp_us);
		FlightDataRecord record{};
		record.timestamp_us = state.timestamp_us;
		record.mode = static_cast<uint8_t>(controller.mode());
		record.roll = state.attitude.roll;
		record.pitch = state.attitude.pitch;
		record.yaw = state.attitude.yaw;
		record.gyro[0] = state.rate.x;
		record.gyro[1] = state.rate.y;
		record.gyro[2] = state.rate.z;
		record.accel[0] = sample.accel_x;
		record.accel[1] = sample.accel_y;
		record.accel[2] = sample.accel_z;

		for (uint8_t i = 0; i < MotorMixer::kMotorCount; ++i)
		{
			record.motor[i] = controller.output().motors[i];
		}

		flight_data.push(record);

		// Simulation-only back pressure. A real MCU leaves idle time between
		// 1 kHz control cycles for the low priority logging task; a host
		// simulation runs thousands of cycles in a few milliseconds and the OS
		// may not schedule the drain thread at all. Cap ring occupancy here so
		// results are deterministic instead of depending on host scheduling.
		// This is NOT part of the firmware control loop: on target hardware
		// the low priority task simply consumes during the idle window.
		uint32_t back_pressure_spins = 0;

		while (flight_data.pending() > kFlightDataRingCapacity / 2u &&
			back_pressure_spins++ < 100000u)
		{
			std::this_thread::yield();
		}

		if (step % 200u == 0u)
		{
			HF_LOG_INFO("FLIGHT", "roll=%7.3f pitch=%7.3f yaw=%7.3f deg | mode=%s",
				static_cast<double>(state.attitude.roll * kRadToDeg),
				static_cast<double>(state.attitude.pitch * kRadToDeg),
				static_cast<double>(state.attitude.yaw * kRadToDeg),
				mode_name(controller.mode()));
		}

		// Scheduling yield only (NOT I/O): lets the low priority task drain
		// both rings before they overflow. A real MCU switches tasks here.
		std::this_thread::yield();
	}

	// Wait (bounded) for the low priority task to catch up, then stop it.
	for (uint32_t spin = 0; spin < 1000000u && flight_data.pending() > 0; ++spin)
	{
		std::this_thread::yield();
	}

	stop_logging.store(true, std::memory_order_release);
	logging_task.join();

	// Stop the transport first so its final motor-frame flush is included in
	// the statistics reported below.
	source->shutdown();
	source->report_stats();

	// Single threaded again: drain any remainder (e.g. after a write failure).
	if (flight_log.active())
	{
		flight_log.pump(flight_data);
	}
	else
	{
		flight_data.drain_to(discard_sink);
	}

	log_drain();

	bool log_ok = true;

	if (flight_log.active())
	{
		log_ok = flight_log.finish(flight_data.dropped());
	}

	if (recording)
	{
		flight_log_storage_close();
	}

	const FlightLogStats flight_stats = flight_log.stats();
	const bool all_zero = source->motors_zero();
	const bool completed = source->cycles() == options.steps;

	bool passed = false;

	if (options.failsafe)
	{
		passed = controller.mode() == FlightMode::Failsafe &&
			controller.failsafe_reason() == FailsafeReason::ImuReadFault &&
			all_zero && completed;
	}
	else
	{
		passed = controller.mode() == FlightMode::Stabilize &&
			std::fabs(controller.state().attitude.roll) < 0.05f &&
			std::fabs(controller.state().attitude.pitch) < 0.05f &&
			completed;
	}

	const LogStats stats = log_get_stats();
	HF_LOG_INFO("SYSTEM", "final roll=%.4f pitch=%.4f rad, mode=%s, motors_zero=%s, cycles=%u/%u, telemetry_frames=%zu",
		static_cast<double>(controller.state().attitude.roll),
		static_cast<double>(controller.state().attitude.pitch),
		mode_name(controller.mode()),
		all_zero ? "yes" : "no",
		static_cast<unsigned>(source->cycles()),
		static_cast<unsigned>(options.steps),
		link.sent_frames());
	HF_LOG_INFO("FLIGHTLOG", "path=%s records=%u blocks=%u dropped=%u write_failures=%u checksum=0x%08X footer=%s",
		recording ? options.flight_log_path : "none",
		flight_stats.records_written, flight_stats.blocks_written,
		flight_stats.dropped_records, flight_stats.write_failures,
		static_cast<unsigned>(flight_stats.checksum),
		recording ? (log_ok ? "ok" : "FAILED") : "disabled");
	HF_LOG_INFO("LOGGING", "emitted=%u drained=%u dropped(level=%u rate=%u full=%u) sink_failures=%u",
		stats.emitted, stats.drained, stats.dropped_level, stats.dropped_rate,
		stats.dropped_full, stats.backend_failures);
	HF_LOG_INFO("SYSTEM", "result: %s", passed && log_ok ? "PASS" : "FAIL");

	log_drain();
	log_shutdown();
	return passed && log_ok ? 0 : 1;
}
