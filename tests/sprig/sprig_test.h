/*
	Minimal host test harness for the Sprig firmware modules.
 */

#ifndef SPRIG_TEST_H_
#define SPRIG_TEST_H_

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

static int sprigTestChecks;
static int sprigTestFailures;

#define SPRIG_CHECK(name, condition) do { \
	sprigTestChecks++; \
	if(condition) { \
		printf("PASS %s\n", name); \
	} else { \
		sprigTestFailures++; \
		printf("FAIL %s (%s:%d)\n", name, __FILE__, __LINE__); \
	} \
} while(0)

static inline int sprigTestSummary(const char *suite) {
	printf("%s: %d checks, %d failures\n", suite, sprigTestChecks, sprigTestFailures);
	return sprigTestFailures ? 1 : 0;
}

#endif
