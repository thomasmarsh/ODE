#include "../CppTestHarness.h"
#include "../TestReporter.h"

using namespace CppTestHarness;

namespace
{

struct MockTestReporter : public TestReporter
{
public:
	MockTestReporter()
		: failureCount(0)
		, testCount(0)
		, execCount(0)
	{
	}

	virtual void ReportFailure(char const*, int, std::string)
	{
		++failureCount;
	}

	virtual void ReportSummary(int testCount_, int) 
	{
		testCount = testCount_;
	}

	virtual void ReportSingleResult(const std::string&, bool)
	{
		++execCount;
	}

	int failureCount;
	int testCount;
	int execCount;
};

struct MockTest : public Test
{
	MockTest(bool const success_)
		: success(success_)
	{
	}

	virtual void RunImpl(TestResults& testResults_)
	{
		if (!success)
			testResults_.ReportFailure("filename", 0, "message");
	}

	bool success;
};

struct MockTestLauncher : public TestLauncher
{
public:
	MockTestLauncher(TestLauncher** listHead)
		: TestLauncher(listHead)
		, success(true)
	{
	}

	void Launch(TestResults& results) const { MockTest(success).Run(results); }

	bool success;
};

struct TestRunnerFixture
{
	TestRunnerFixture()
		: listHead(0)
	{
		runner.SetTestLauncherListHead(&listHead);
		runner.SetTestReporter(&reporter);
	}

	MockTestReporter reporter;
	TestLauncher* listHead;
	TestRunner runner;
};

TEST_FIXTURE(TestRunnerFixture, FailureCountIsZeroWhenNoTestsAreRun)
{
	CHECK_EQUAL(runner.RunAllTests(), 0);
	CHECK_EQUAL(reporter.testCount, 0);
	CHECK_EQUAL(reporter.execCount, 0) //TODO
}

TEST_FIXTURE(TestRunnerFixture, PassingTestsAreNotReportedAsFailures)
{
	MockTestLauncher launcher(&listHead);
	launcher.success = true;

	CHECK_EQUAL(runner.RunAllTests(), 0);
	CHECK_EQUAL(reporter.failureCount, 0);
	CHECK_EQUAL(reporter.testCount, 1);
}

TEST_FIXTURE(TestRunnerFixture, FinishedTestsReportDone)
{
	MockTestLauncher launcher1(&listHead);
	MockTestLauncher launcher2(&listHead);
	launcher1.success = false;
	launcher2.success = true;

	runner.RunAllTests();
	CHECK_EQUAL(reporter.execCount, 2);
}

TEST_FIXTURE(TestRunnerFixture, TestRunnerCallsReportFailureOncePerFailingTest)
{
	MockTestLauncher launcher1(&listHead);
	MockTestLauncher launcher2(&listHead);
	launcher1.success = false;
	launcher2.success = false;

	CHECK_EQUAL(runner.RunAllTests(), 2);
	CHECK_EQUAL(reporter.failureCount, 2);
}

}

