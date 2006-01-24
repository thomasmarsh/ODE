#include "../CppTestHarness.h"
#include "../TestReporter.h"

namespace
{

// vs7.1 warning level 4 release, 'conditional expression is constant'
// gcc doesnt like unknown pragmas, so we only enable them on VC
#ifdef _MSC_VER
#    pragma warning(push)
#    pragma warning(disable:4127)
#endif
TEST(ValidCheckSucceeds)
{
	CHECK(true);
}

TEST(ValidCheckEqualsSucceeds)
{
	CHECK_EQUAL(1, 1);
}
#ifdef _MSC_VER
#    pragma warning(pop)
#endif

TEST(ValidCheckCloseSucceeds)
{
	CHECK_CLOSE(2.0f, 2.001f, 0.01f);
}

TEST(CheckEqualMacroAllowsCharPtrAndStringComparisons)
{
	std::string const str("Hello World");
	CHECK_EQUAL(str, "Hello World");
}

struct SimpleFixture
{
	SimpleFixture()
	{
		constructed = true;
	}

	static bool constructed;
};

bool SimpleFixture::constructed = false;

TEST_FIXTURE(SimpleFixture, DefaultFixtureCtorIsCalled)
{
	CHECK_EQUAL(SimpleFixture::constructed, true);
}

struct SpecializedCtorFixture
{
	SpecializedCtorFixture(int value_ = 0)
		: value(value_)
	{
	}

	int value;
};

TEST_FIXTURE_CTOR(SpecializedCtorFixture, (5), CtorDataGetsPassedToFixture)
{
	CHECK_EQUAL(value, 5);
}

}

