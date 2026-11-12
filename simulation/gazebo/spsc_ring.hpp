#pragma once

#include <atomic>
#include <cstdint>

namespace homefly
{

// Lock free single producer / single consumer ring. Capacity must be a power
// of two. push()/pop() never allocate, never block and have bounded runtime,
// so they are safe inside the 1 kHz control loop.
template<typename T, uint32_t Capacity>
class SpscRing
{
	static_assert((Capacity & (Capacity - 1u)) == 0u, "capacity must be a power of two");

public:
	bool push(
		const T& value
	) noexcept
	{
		const uint32_t head = _head.load(std::memory_order_relaxed);
		const uint32_t tail = _tail.load(std::memory_order_acquire);

		if (head - tail >= Capacity)
		{
			_dropped.fetch_add(1u, std::memory_order_relaxed);
			return false;
		}

		_slots[head & (Capacity - 1u)] = value;
		_head.store(head + 1u, std::memory_order_release);
		return true;
	}

	bool pop(
		T& value
	) noexcept
	{
		const uint32_t tail = _tail.load(std::memory_order_relaxed);
		const uint32_t head = _head.load(std::memory_order_acquire);

		if (head == tail)
		{
			return false;
		}

		value = _slots[tail & (Capacity - 1u)];
		_tail.store(tail + 1u, std::memory_order_release);
		return true;
	}

	uint32_t pending() const noexcept
	{
		return _head.load(std::memory_order_acquire) - _tail.load(std::memory_order_acquire);
	}

	uint32_t dropped() const noexcept
	{
		return _dropped.load(std::memory_order_relaxed);
	}

	void reset() noexcept
	{
		_head.store(0u, std::memory_order_relaxed);
		_tail.store(0u, std::memory_order_relaxed);
		_dropped.store(0u, std::memory_order_relaxed);
	}

private:
	T _slots[Capacity]{};
	std::atomic<uint32_t> _head{0};
	std::atomic<uint32_t> _tail{0};
	std::atomic<uint32_t> _dropped{0};
};

} // namespace homefly
