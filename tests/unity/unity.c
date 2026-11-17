/* =========================================================================
    Unity - A Test Framework for C
    ThrowTheSwitch.org
    Copyright (c) 2007-25 Mike Karlesky, Mark VanderVoord, Greg Williams
    [Released under MIT License. Please refer to license.txt for details]
============================================================================ */

#include "unity.h"
#include <string.h>
#include <math.h>

struct UNITY_STORAGE_T Unity;

/* setUp and tearDown are provided by each test file */



void UnityBegin(const char* filename) {
    Unity.TestFile = filename;
    Unity.CurrentTestName = NULL;
    Unity.CurrentTestLineNumber = 0;
    Unity.NumberOfTests = 0;
    Unity.TestFailures = 0;
    Unity.TestIgnores = 0;
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;

    printf("\n-----------------------------------------\n");
    printf("Running Tests in: %s\n", filename);
    printf("-----------------------------------------\n");
}

int UnityEnd(void) {
    printf("-----------------------------------------\n");
    printf("%u Tests, %u Failures, %u Ignored\n",
           Unity.NumberOfTests, Unity.TestFailures, Unity.TestIgnores);
    if (Unity.TestFailures == 0) {
        printf("OK\n\n");
    } else {
        printf("FAIL\n\n");
    }
    return (int)Unity.TestFailures;
}

void UnityConcludeTest(void) {
    if (Unity.CurrentTestFailed) {
        Unity.TestFailures++;
    } else if (Unity.CurrentTestIgnored) {
        Unity.TestIgnores++;
    } else {
        printf("  PASS: %s\n", Unity.CurrentTestName);
    }
    Unity.CurrentTestFailed = 0;
    Unity.CurrentTestIgnored = 0;
}



void UnityAssert(int condition, int fail_line, const char* message, UNITY_UINT line) {
    (void)fail_line;
    if (!condition) {
        Unity.CurrentTestFailed = 1;
        printf("  FAIL: %s:%u: %s (Assertion failed: %s)\n",
               Unity.TestFile, line, Unity.CurrentTestName, message ? message : "Condition false");
        TEST_ABORT();
    }
}

void UnityAssertEqualNumber(UNITY_INT expected, UNITY_INT actual, const char* msg, UNITY_UINT line, UNITY_DISPLAY_STYLE_T style) {
    (void)style;
    if (expected != actual) {
        Unity.CurrentTestFailed = 1;
        printf("  FAIL: %s:%u: %s (Expected %ld Was %ld%s%s)\n",
               Unity.TestFile, line, Unity.CurrentTestName, (long)expected, (long)actual,
               msg ? " - " : "", msg ? msg : "");
        TEST_ABORT();
    }
}

void UnityAssertNotEqualNumber(UNITY_INT expected, UNITY_INT actual, const char* msg, UNITY_UINT line, UNITY_DISPLAY_STYLE_T style) {
    (void)style;
    if (expected == actual) {
        Unity.CurrentTestFailed = 1;
        printf("  FAIL: %s:%u: %s (Expected %ld NOT equal to %ld%s%s)\n",
               Unity.TestFile, line, Unity.CurrentTestName, (long)expected, (long)actual,
               msg ? " - " : "", msg ? msg : "");
        TEST_ABORT();
    }
}

void UnityAssertFloatsWithin(UNITY_FLOAT delta, UNITY_FLOAT expected, UNITY_FLOAT actual, const char* msg, UNITY_UINT line) {
    float diff = fabsf(expected - actual);
    if (diff > delta) {
        Unity.CurrentTestFailed = 1;
        printf("  FAIL: %s:%u: %s (Expected %.5f Was %.5f, diff %.5f > delta %.5f%s%s)\n",
               Unity.TestFile, line, Unity.CurrentTestName, expected, actual, diff, delta,
               msg ? " - " : "", msg ? msg : "");
        TEST_ABORT();
    }
}

void UnityAssertEqualString(const char* expected, const char* actual, const char* msg, UNITY_UINT line) {
    if (expected == NULL && actual == NULL) return;
    if (expected == NULL || actual == NULL || strcmp(expected, actual) != 0) {
        Unity.CurrentTestFailed = 1;
        printf("  FAIL: %s:%u: %s (Expected '%s' Was '%s'%s%s)\n",
               Unity.TestFile, line, Unity.CurrentTestName,
               expected ? expected : "NULL", actual ? actual : "NULL",
               msg ? " - " : "", msg ? msg : "");
        TEST_ABORT();
    }
}

void UnityAssertEqualStringLen(const char* expected, const char* actual, UNITY_UINT32 length, const char* msg, UNITY_UINT line) {
    if (expected == NULL && actual == NULL) return;
    if (expected == NULL || actual == NULL || strncmp(expected, actual, length) != 0) {
        Unity.CurrentTestFailed = 1;
        printf("  FAIL: %s:%u: %s (Expected first %u chars '%s' Was '%s'%s%s)\n",
               Unity.TestFile, line, Unity.CurrentTestName, length,
               expected ? expected : "NULL", actual ? actual : "NULL",
               msg ? " - " : "", msg ? msg : "");
        TEST_ABORT();
    }
}
