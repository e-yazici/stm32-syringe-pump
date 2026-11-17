/* =========================================================================
    Unity - A Test Framework for C
    ThrowTheSwitch.org
    Copyright (c) 2007-25 Mike Karlesky, Mark VanderVoord, Greg Williams
    [Released under MIT License. Please refer to license.txt for details]
============================================================================ */

#ifndef UNITY_INTERNALS_H
#define UNITY_INTERNALS_H

#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define UNITY_INT int32_t
#define UNITY_UINT uint32_t
#define UNITY_INT8 int8_t
#define UNITY_UINT8 uint8_t
#define UNITY_INT16 int16_t
#define UNITY_UINT16 uint16_t
#define UNITY_INT32 int32_t
#define UNITY_UINT32 uint32_t
#define UNITY_FLOAT float

typedef enum {
    UNITY_DISPLAY_STYLE_INT = 0,
    UNITY_DISPLAY_STYLE_INT8,
    UNITY_DISPLAY_STYLE_INT16,
    UNITY_DISPLAY_STYLE_INT32,
    UNITY_DISPLAY_STYLE_UINT,
    UNITY_DISPLAY_STYLE_UINT8,
    UNITY_DISPLAY_STYLE_UINT16,
    UNITY_DISPLAY_STYLE_UINT32,
    UNITY_DISPLAY_STYLE_HEX32
} UNITY_DISPLAY_STYLE_T;

struct UNITY_STORAGE_T {
    const char* TestFile;
    const char* CurrentTestName;
    UNITY_UINT CurrentTestLineNumber;
    UNITY_UINT NumberOfTests;
    UNITY_UINT TestFailures;
    UNITY_UINT TestIgnores;
    UNITY_UINT CurrentTestFailed;
    UNITY_UINT CurrentTestIgnored;
    jmp_buf AbortFrame;
};

extern struct UNITY_STORAGE_T Unity;

#define TEST_PROTECT() (setjmp(Unity.AbortFrame) == 0)
#define TEST_ABORT()   longjmp(Unity.AbortFrame, 1)

void UnityBegin(const char* filename);
int  UnityEnd(void);
void UnityConcludeTest(void);

void UnityAssert(int condition, int fail_line, const char* message, UNITY_UINT line);
void UnityAssertEqualNumber(UNITY_INT expected, UNITY_INT actual, const char* msg, UNITY_UINT line, UNITY_DISPLAY_STYLE_T style);
void UnityAssertNotEqualNumber(UNITY_INT expected, UNITY_INT actual, const char* msg, UNITY_UINT line, UNITY_DISPLAY_STYLE_T style);
void UnityAssertFloatsWithin(UNITY_FLOAT delta, UNITY_FLOAT expected, UNITY_FLOAT actual, const char* msg, UNITY_UINT line);
void UnityAssertEqualString(const char* expected, const char* actual, const char* msg, UNITY_UINT line);
void UnityAssertEqualStringLen(const char* expected, const char* actual, UNITY_UINT32 length, const char* msg, UNITY_UINT line);

#ifdef __cplusplus
}
#endif

#endif /* UNITY_INTERNALS_H */
