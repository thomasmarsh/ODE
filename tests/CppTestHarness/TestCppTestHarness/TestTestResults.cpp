#include "../CppTestHarness.h"

#include "../TestReporter.h"
#include "../TestResults.h"

using namespace CppTestHarness;

namespace 
{

struct MockTestReporter : public TestReporter
{
public:
	MockTestReporter()
		: failureCount(0)
	{
	}

	virtual void ReportFailure(char const*, int, std::string) 
	{
		++failureCount;
	}
	
	virtual void ReportSingleResult(const std::string&, bool) {}
	virtual void ReportSummary(int, int) {}

	int failureCount;
};

struct MockTestResultsFixture
{
	MockTestResultsFixture()
		: results(reporter)
	{
	}

	MockTestReporter reporter;
	TestResults results;
};

TEST_FIXTURE(MockTestResultsFixture, TestResultsDefaultToSuccess)
{
	CHECK_EQUAL(results.Failed(), false);
}

TEST_FIXTURE(MockTestResultsFixture, TestResultsRecordFailures)
{
	results.ReportFailure("nothing", 0, "expected failure");
	CHECK_EQUAL(results.Failed(), true);
}

TEST_FIXTURE(MockTestResultsFixture, TestResultsReportFailures)
{
	results.ReportFailure("nothing", 0, "expected failure");
	CHECK_EQUAL(reporter.failureCount, 1);
}

}

