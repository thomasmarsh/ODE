#include "../CppTestHarness.h"

#include "../TestReporter.h"

using namespace CppTestHarness;

namespace
{

class MockTest : public Test
{
public:
	MockTest() 
	{
		++instantiationCount;
	}

	static int instantiationCount;

private:
	virtual void RunImpl(TestResults&) {}
};

int MockTest::instantiationCount = 0;

struct MockReporter : public TestReporter
{
	virtual void ReportFailure(char const*, int, std::string) {}
	virtual void ReportSingleResult(const std::string&, bool) {}
	virtual void ReportSummary(int, int) {}
};

TEST(TypedTestCreatorMakesTestOfCorrectType)
{
	TestLauncher* listHead = 0;
	TypedTestLauncher< MockTest > mockLauncher(&listHead);

	MockReporter mr;
	TestResults tr(mr);
	mockLauncher.Launch(tr);
	CHECK_EQUAL(MockTest::instantiationCount, 1);
}
}

