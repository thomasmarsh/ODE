#include "../CppTestHarness.h"

#include "../TestReporter.h"

class CrashingTest : public CppTestHarness::Test
{
public:
	virtual void RunImpl(CppTestHarness::TestResults&)
	{
		reinterpret_cast< void (*)() >(0)();
	}
};

struct MockTestReporter : public CppTestHarness::TestReporter
{
public:
	virtual void ReportFailure(char const*, int, std::string) {}
	virtual void ReportSingleResult(const std::string&, bool) {}
	virtual void ReportSummary(int, int) {}
};

TEST(CrashingTestsAreReportedAsFailures)
{
	CrashingTest crashingTest;
	MockTestReporter reporter;
	CppTestHarness::TestResults results(reporter);

	crashingTest.Run(results);
	CHECK(results.Failed());
}

