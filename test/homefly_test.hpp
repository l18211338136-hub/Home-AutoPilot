#pragma once

// GoogleTest is used when found by CMake. On offline machines this header
// provides the small subset of the GTest API used by the HomeFly tests, so
// CTest stays fully functional without adding a third-party dependency.

#ifdef HOMEFLY_USE_GTEST

#include <gtest/gtest.h>
#define HOMEFLY_TEST_MAIN()

#else

#include <cmath>
#include <iostream>
#include <vector>

namespace homefly::test
{

struct TestCase
{
	const char* suite;
	const char* name;
	void (*body)();
};

inline std::vector<TestCase>& registry()
{
	static std::vector<TestCase> cases;
	return cases;
}

inline int& failures()
{
	static int count = 0;
	return count;
}

struct Registrar
{
	Registrar(
		const char* suite,
		const char* name,
		void (*body)()
	)
	{
		registry().push_back(TestCase{suite, name, body});
	}
};

inline void report_failure(
	const char* file,
	int line,
	const char* expr
)
{
	++failures();
	std::cerr << "  FAIL " << file << ":" << line << " -> " << expr << '\n';
}

template<typename T>
concept Streamable = requires(std::ostream& stream, const T& value)
{
	stream << value;
};

template<typename A, typename B>
inline void expect_eq(
	const A& lhs,
	const B& rhs,
	const char* expr,
	const char* file,
	int line
)
{
	if (!(lhs == rhs))
	{
		++failures();
		std::cerr << "  FAIL " << file << ":" << line << " -> " << expr;

		// enum class and PODs have no operator<<; still report the failure.
		if constexpr (Streamable<A> && Streamable<B>)
		{
			std::cerr << " (lhs=" << lhs << ", rhs=" << rhs << ")";
		}

		std::cerr << '\n';
	}
}

inline void expect_near(
	float lhs,
	float rhs,
	float tolerance,
	const char* expr,
	const char* file,
	int line
)
{
	if (!(std::fabs(lhs - rhs) <= tolerance))
	{
		++failures();
		std::cerr << "  FAIL " << file << ":" << line << " -> " << expr
			<< " (lhs=" << lhs << ", rhs=" << rhs << ", tol=" << tolerance << ")\n";
	}
}

inline int run_all()
{
	int failed_cases = 0;

	for (const TestCase& test_case : registry())
	{
		const int before = failures();
		std::cout << "[ RUN      ] " << test_case.suite << "." << test_case.name << '\n';
		test_case.body();

		if (failures() != before)
		{
			++failed_cases;
			std::cout << "[  FAILED  ] " << test_case.suite << "." << test_case.name << '\n';
		}
		else
		{
			std::cout << "[       OK ] " << test_case.suite << "." << test_case.name << '\n';
		}
	}

	std::cout << registry().size() << " test(s), " << failed_cases << " failed\n";
	return failed_cases == 0 ? 0 : 1;
}

} // namespace homefly::test

#define TEST(suite, name) \
	static void suite##_##name##_body(); \
	static ::homefly::test::Registrar suite##_##name##_registrar(#suite, #name, &suite##_##name##_body); \
	static void suite##_##name##_body()

#define EXPECT_TRUE(expr) \
	do { if (!(expr)) { ::homefly::test::report_failure(__FILE__, __LINE__, #expr); } } while (false)

#define EXPECT_FALSE(expr) EXPECT_TRUE(!(expr))

#define EXPECT_EQ(lhs, rhs) \
	::homefly::test::expect_eq((lhs), (rhs), #lhs " == " #rhs, __FILE__, __LINE__)

#define EXPECT_NEAR(lhs, rhs, tolerance) \
	::homefly::test::expect_near((lhs), (rhs), (tolerance), #lhs " ~= " #rhs, __FILE__, __LINE__)

#define HOMEFLY_TEST_MAIN() \
	int main() { return ::homefly::test::run_all(); }

#endif // HOMEFLY_USE_GTEST
