#include "communication/command_receiver.hpp"
#include "communication/control_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "communication/i_link.hpp"
#include "hal/i_clock.hpp"
#include "homefly_test.hpp"

namespace
{

// Non-blocking ILink stub: returns one queued frame per receive() call, then 0.
class MockLink final : public homefly::ILink
{
public:
	void queue(const std::vector<uint8_t>& frame)
	{
		_queue.push_back(frame);
	}

	bool send(
		const uint8_t*,
		std::size_t
	) override
	{
		return true;
	}

	std::size_t receive(
		uint8_t* buffer,
		std::size_t capacity
	) override
	{
		if (_queue.empty())
		{
			return 0;
		}
		const std::vector<uint8_t>& frame = _queue.front();
		const std::size_t n = frame.size() <= capacity ? frame.size() : capacity;
		std::memcpy(buffer, frame.data(), n);
		_queue.erase(_queue.begin());
		return n;
	}

private:
	std::vector<std::vector<uint8_t>> _queue;
};

class MockClock final : public homefly::IClock
{
public:
	uint64_t now_us() const override { return _now; }
	void advance(uint64_t us) { _now += us; }

private:
	uint64_t _now{0};
};

std::vector<uint8_t> arm_frame(uint16_t seq = 0)
{
	homefly::ControlFrame f{};
	homefly::build_control_frame(homefly::ControlFrameType::Arm, nullptr, 0, seq, f);
	uint8_t raw[homefly::kControlFrameSize];
	std::memcpy(raw, &f, homefly::kControlFrameSize);
	return {raw, raw + homefly::kControlFrameSize};
}

std::vector<uint8_t> disarm_frame(uint16_t seq = 0)
{
	homefly::ControlFrame f{};
	homefly::build_control_frame(homefly::ControlFrameType::Disarm, nullptr, 0, seq, f);
	uint8_t raw[homefly::kControlFrameSize];
	std::memcpy(raw, &f, homefly::kControlFrameSize);
	return {raw, raw + homefly::kControlFrameSize};
}

std::vector<uint8_t> setpoint_frame(float throttle, float roll, float pitch, float yaw, uint16_t seq = 0)
{
	uint8_t payload[homefly::kControlPayloadSize]{};
	homefly::pack_setpoint_payload(throttle, roll, pitch, yaw, payload);
	homefly::ControlFrame f{};
	homefly::build_control_frame(homefly::ControlFrameType::Setpoint, payload, homefly::kControlPayloadSize, seq, f);
	uint8_t raw[homefly::kControlFrameSize];
	std::memcpy(raw, &f, homefly::kControlFrameSize);
	return {raw, raw + homefly::kControlFrameSize};
}

} // namespace

TEST(CommandReceiver, ArmLatchesUntilDisarm)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 0);

	link.queue(arm_frame());
	receiver.poll();

	homefly::Setpoint sp = receiver.latest_setpoint();
	EXPECT_TRUE(sp.arm_request);
	EXPECT_FALSE(sp.disarm_request);

	// SETPOINT must not clear the arm request.
	link.queue(setpoint_frame(0.6f, 0.0f, 0.0f, 0.0f));
	receiver.poll();
	sp = receiver.latest_setpoint();
	EXPECT_TRUE(sp.arm_request);

	link.queue(disarm_frame());
	receiver.poll();
	sp = receiver.latest_setpoint();
	EXPECT_FALSE(sp.arm_request);
	EXPECT_TRUE(sp.disarm_request);
}

TEST(CommandReceiver, SetpointUpdatesTargets)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 0);

	link.queue(arm_frame());
	link.queue(setpoint_frame(0.7f, 0.05f, -0.05f, 0.3f));
	receiver.poll();

	homefly::Setpoint sp = receiver.latest_setpoint();
	EXPECT_NEAR(sp.throttle, 0.7f, 1e-6f);
	EXPECT_NEAR(sp.roll, 0.05f, 1e-6f);
	EXPECT_NEAR(sp.pitch, -0.05f, 1e-6f);
	EXPECT_NEAR(sp.yaw_rate, 0.3f, 1e-6f);
}

TEST(CommandReceiver, ThrottleClampedToUnitRange)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 0);

	link.queue(setpoint_frame(2.0f, 0.0f, 0.0f, 0.0f));
	receiver.poll();
	EXPECT_NEAR(receiver.latest_setpoint().throttle, 1.0f, 1e-6f);

	link.queue(setpoint_frame(-0.5f, 0.0f, 0.0f, 0.0f));
	receiver.poll();
	EXPECT_NEAR(receiver.latest_setpoint().throttle, 0.0f, 1e-6f);
}

TEST(CommandReceiver, LinkLossForcesDisarm)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 1000u); // 1 ms timeout

	// No frame ever received -> link lost -> disarm requested.
	EXPECT_TRUE(receiver.link_lost());
	homefly::Setpoint sp = receiver.latest_setpoint();
	EXPECT_FALSE(sp.arm_request);
	EXPECT_TRUE(sp.disarm_request);
}

TEST(CommandReceiver, FreshFramePreventsLinkLoss)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 1000u);

	link.queue(arm_frame());
	receiver.poll();
	EXPECT_FALSE(receiver.link_lost());

	clock.advance(500u); // within timeout
	EXPECT_FALSE(receiver.link_lost());
	EXPECT_TRUE(receiver.latest_setpoint().arm_request);

	clock.advance(600u); // now 1100 us since last frame -> stale
	EXPECT_TRUE(receiver.link_lost());
	EXPECT_TRUE(receiver.latest_setpoint().disarm_request);
}

TEST(CommandReceiver, ParseErrorsCounted)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 0);

	// 28 bytes but wrong magic -> parse error, not a frame.
	std::vector<uint8_t> garbage(homefly::kControlFrameSize, 0xAB);
	link.queue(garbage);
	receiver.poll();

	EXPECT_EQ(receiver.frames_received(), 0u);
	EXPECT_EQ(receiver.parse_errors(), 1u);
}

TEST(CommandReceiver, TimeoutDisabledKeepsLatchedState)
{
	MockLink link;
	MockClock clock;
	homefly::CommandReceiver receiver(link, clock, 0); // timeout disabled

	// Never polled, never received: latest_setpoint returns the (default,
	// disarmed) latched state, and link_lost() is always false.
	EXPECT_FALSE(receiver.link_lost());
	EXPECT_FALSE(receiver.latest_setpoint().arm_request);
	EXPECT_FALSE(receiver.latest_setpoint().disarm_request);
}

HOMEFLY_TEST_MAIN()
