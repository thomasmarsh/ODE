#include "../CppTestHarness.h"
#include "../HTMLTestReporter.h"

// #define USE_HTML_REPORTER

int main(int, char const *[])
{
#ifdef USE_HTML_REPORTER
	using namespace CppTestHarness;

	TestRunner runner;
	HTMLTestReporter reporter;
	runner.SetTestReporter(&reporter);

	return runner.RunAllTests();
#else
	return CppTestHarness::TestRunner().RunAllTests();
#endif
}

