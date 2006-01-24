#include "../CppTestHarness.h"

using namespace CppTestHarness;

namespace
{

TEST(CheckCloseTrue)
{
	CHECK_EQUAL(CheckClose(3.001f, 3.0f, 0.1f), true);
}

TEST(CheckCloseFalse)
{
	CHECK_EQUAL(CheckClose(3.12f, 3.0f, 0.1f), false);
}

TEST(CheckArrayEqualTrue)
{
	int const array[3] = { 1, 2, 3 };
	CHECK_EQUAL(CheckArrayEqual(array, array, 3), true);
}

TEST(CheckArrayEqualFalse)
{
	int const array1[3] = { 1, 2, 3	};
	int const array2[3] = { 1, 2, 2 };
	CHECK_EQUAL(CheckArrayEqual(array1, array2, 3), false);
}

TEST(CheckArrayCloseTrue)
{
	float const array1[3] = { 1.0f, 1.5f, 2.0f };
	float const array2[3] = { 1.01f, 1.51f, 2.01f };
	CHECK_EQUAL(CheckArrayClose(array1, array2, 3, 0.02f), true);
}

TEST(CheckArrayCloseFalse)
{
	float const array1[3] = { 1.0f, 1.5f, 2.0f };
	float const array2[3] = { 1.01f, 1.51f, 2.01f };
	CHECK_EQUAL(CheckArrayClose(array1, array2, 3, 0.001f), false);
}

}

