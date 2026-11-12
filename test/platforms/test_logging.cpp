#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "homefly_test.hpp"
#include "platforms/common/logging/log.hpp"
#include "simulation/sim_clock.hpp"

using namespace homefly;

namespace
{

class RecordingSink final : public ILogSink
{
public:
	bool write(
		const uint8_t* data,
		uint16_t length
	) noexcept override
	{
		if (_fault || data == nullptr || _count >= kMaxLines)
		{
			return false;
		}

		const size_t copy_length = length < kLogLineCapacity - 1
			? length : kLogLineCapacity - 1;
		std::memcpy(_lines[_count], data, copy_length);
		_lengths[_count] = static_cast<uint16_t>(copy_length);
		++_count;
		return true;
	}

	void flush() noexcept override
	{
		++_flushes;
	}

	void set_fault(
		bool fault
	) noexcept
	{
		_fault = fault;
	}

	uint32_t count() const noexcept
	{
		return _count;
	}

	uint32_t flushes() const noexcept
	{
		return _flushes;
	}

	std::string line(
		uint32_t index
	) const
	{
		return std::string(_lines[index], _lengths[index]);
	}

private:
	static constexpr uint32_t kMaxLines = 256;
	char _lines[kMaxLines][kLogLineCapacity]{};
	uint16_t _lengths[kMaxLines]{};
	uint32_t _count{0};
	uint32_t _flushes{0};
	bool _fault{false};
};

struct LogFixture
{
	SimClock clock;
	RecordingSink sink;

	explicit LogFixture(
		const LogConfig& config = LogConfig{}
	)
	{
		log_init(clock, sink, config);
	}

	~LogFixture()
	{
		log_shutdown();
	}
};

} // namespace

TEST(Logging, LevelFiltering)
{
	LogConfig config{};
	config.min_level = LogLevel::Warn;
	LogFixture fixture(config);

	HF_LOG_DEBUG("TEST", "debug");
	HF_LOG_INFO("TEST", "info");
	HF_LOG_WARN("TEST", "warn");
	HF_LOG_ERROR("TEST", "error");
	HF_LOG_FATAL("TEST", "fatal");
	log_drain();

	const LogStats stats = log_get_stats();
	EXPECT_EQ(stats.emitted, 3u);
	EXPECT_EQ(stats.dropped_level, 2u);
	EXPECT_EQ(fixture.sink.count(), 3u);
}

TEST(Logging, LineFormat)
{
	LogFixture fixture;
	fixture.clock.advance_us(1234000u);
	HF_LOG_INFO("SYSTEM", "HomeFly booting");
	log_drain();

	EXPECT_EQ(fixture.sink.line(0),
		std::string("[00001234] [INFO ] [SYSTEM] HomeFly booting"));
}

TEST(Logging, FormattedValuesAndLevelLabels)
{
	LogFixture fixture;
	HF_LOG_WARN("BATTERY", "Low voltage: %.2fV", 3.45);
	HF_LOG_ERROR("IMU", "Sensor read failed");
	log_drain();

	EXPECT_EQ(fixture.sink.line(0),
		std::string("[00000000] [WARN ] [BATTERY] Low voltage: 3.45V"));
	EXPECT_EQ(fixture.sink.line(1),
		std::string("[00000000] [ERROR] [IMU] Sensor read failed"));
}

TEST(Logging, HighFrequencyThrottling)
{
	LogConfig config{};
	config.min_level = LogLevel::Debug;
	config.rate_limit_ms = 1000u;
	LogFixture fixture(config);

	fixture.clock.advance_us(1000u);
	HF_LOG_INFO("SENSOR", "first");
	HF_LOG_INFO("SENSOR", "too soon");
	fixture.clock.advance_us(2000000u);
	HF_LOG_INFO("SENSOR", "after interval");
	log_drain();

	EXPECT_EQ(fixture.sink.count(), 2u);
	EXPECT_EQ(log_get_stats().dropped_rate, 1u);
}

TEST(Logging, FullRingDropsWithoutBlockingOrIo)
{
	LogConfig config{};
	config.min_level = LogLevel::Debug;
	LogFixture fixture(config);

	for (uint32_t i = 0; i < kLogRingCapacity * 3u; ++i)
	{
		HF_LOG_INFO("FLOOD", "record %u", static_cast<unsigned>(i));
	}

	const LogStats stats = log_get_stats();
	EXPECT_EQ(stats.emitted, kLogRingCapacity);
	EXPECT_TRUE(stats.dropped_full > 0u);
	EXPECT_EQ(fixture.sink.count(), 0u); // no synchronous output happened
}

TEST(Logging, BackendFailureIsCounted)
{
	LogFixture fixture;
	fixture.sink.set_fault(true);
	HF_LOG_ERROR("UART", "write failed");
	EXPECT_EQ(log_drain(), 1u);
	EXPECT_EQ(log_get_stats().backend_failures, 1u);
	EXPECT_EQ(fixture.sink.count(), 0u);
}

TEST(Logging, DrainLimitAndFlush)
{
	LogFixture fixture;

	for (uint32_t i = 0; i < 5u; ++i)
	{
		HF_LOG_INFO("DRAIN", "record %u", static_cast<unsigned>(i));
	}

	EXPECT_EQ(log_drain(2u), 2u);
	EXPECT_EQ(log_drain(0u), 3u);
	EXPECT_EQ(log_drain(0u), 0u);
	EXPECT_EQ(fixture.sink.count(), 5u);
	log_flush();
	EXPECT_EQ(fixture.sink.flushes(), 1u);
}

TEST(Logging, MultipleSinks)
{
	LogFixture fixture;
	RecordingSink second;
	EXPECT_TRUE(log_add_sink(second));
	HF_LOG_INFO("MULTI", "hello");
	log_drain();
	EXPECT_EQ(fixture.sink.count(), 1u);
	EXPECT_EQ(second.count(), 1u);
}

TEST(Logging, SinkCapacityIsBounded)
{
	LogFixture fixture;
	RecordingSink a;
	RecordingSink b;
	RecordingSink c;
	RecordingSink d;
	EXPECT_TRUE(log_add_sink(a));
	EXPECT_TRUE(log_add_sink(b));
	EXPECT_TRUE(log_add_sink(c));
	EXPECT_FALSE(log_add_sink(d)); // console + 3 is the limit
}

TEST(Logging, RuntimeLevelChangeAndStatsReset)
{
	LogFixture fixture;
	log_set_level(LogLevel::Error);
	HF_LOG_INFO("TEST", "filtered");
	EXPECT_EQ(log_get_level(), LogLevel::Error);
	log_set_level(LogLevel::Debug);
	log_reset_stats();
	HF_LOG_DEBUG("TEST", "kept");
	log_drain();
	EXPECT_EQ(log_get_stats().emitted, 1u);
	EXPECT_EQ(log_get_stats().dropped_level, 0u);
}

TEST(Logging, ShutdownDrainsPendingRecords)
{
	SimClock clock;
	RecordingSink sink;
	log_init(clock, sink);
	HF_LOG_INFO("TEST", "tail record");
	EXPECT_EQ(sink.count(), 0u); // still queued: no synchronous I/O

	// A caller that forgets the final log_drain() must not lose the tail.
	log_shutdown();
	EXPECT_EQ(sink.count(), 1u);
	EXPECT_EQ(sink.line(0),
		std::string("[00000000] [INFO ] [TEST] tail record"));
}

TEST(Logging, ShutdownMakesWritesNoOp)
{
	SimClock clock;
	RecordingSink sink;
	log_init(clock, sink);
	log_shutdown();
	HF_LOG_INFO("TEST", "ignored");
	EXPECT_EQ(log_drain(), 0u);
	EXPECT_EQ(sink.count(), 0u);
}

TEST(Logging, MultipleProducersAreSafe)
{
	LogConfig config{};
	config.min_level = LogLevel::Debug;
	LogFixture fixture(config);

	constexpr uint32_t kProducers = 4u;
	constexpr uint32_t kPerProducer = 25u;
	constexpr uint32_t kTotal = kProducers * kPerProducer;

	std::atomic<bool> done{false};
	std::thread drainer([&done]()
	{
		while (!done.load(std::memory_order_acquire))
		{
			log_drain(8u);
			std::this_thread::yield();
		}

		while (log_drain(8u) > 0u)
		{
		}
	});

	std::vector<std::thread> producers;

	for (uint32_t p = 0; p < kProducers; ++p)
	{
		producers.emplace_back([p]()
		{
			for (uint32_t i = 0; i < kPerProducer; ++i)
			{
				HF_LOG_INFO("CONCURRENT", "p%u i%u", p, i);
			}
		});
	}

	for (std::thread& producer : producers)
	{
		producer.join();
	}

	done.store(true, std::memory_order_release);
	drainer.join();

	const LogStats stats = log_get_stats();
	EXPECT_EQ(stats.emitted + stats.dropped_full, kTotal);
	EXPECT_EQ(stats.drained, stats.emitted);
	EXPECT_EQ(fixture.sink.count(), stats.emitted);
}

HOMEFLY_TEST_MAIN()
