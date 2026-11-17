/* =========================================================================
    Unity - A Test Framework for C
    ThrowTheSwitch.org
    Copyright (c) 2007-25 Mike Karlesky, Mark VanderVoord, Greg Williams
    [Released under MIT License. Please refer to license.txt for details]
============================================================================ */

#ifndef UNITY_FRAMEWORK_H
#define UNITY_FRAMEWORK_H

#define UNITY_VERSION_MAJOR    2
#define UNITY_VERSION_MINOR    5
#define UNITY_VERSION_BUILD    4

#ifdef __cplusplus
extern "C"
{
#endif

#include "unity_internals.h"

/*-------------------------------------------------------
 * Test Setup / Teardown
 *-------------------------------------------------------*/
void setUp(void);
void tearDown(void);
void suiteSetUp(void);
int suiteTearDown(int num_failures);
void resetTest(void);

/*-------------------------------------------------------
 * Basic Assertions
 *-------------------------------------------------------*/
#define TEST_ASSERT(condition)                                           UnityAssert( (condition), 0, #condition, __LINE__ )
#define TEST_ASSERT_TRUE(condition)                                      TEST_ASSERT(condition)
#define TEST_ASSERT_FALSE(condition)                                     TEST_ASSERT(!(condition))
#define TEST_ASSERT_NULL(pointer)                                        TEST_ASSERT((pointer) == NULL)
#define TEST_ASSERT_NOT_NULL(pointer)                                    TEST_ASSERT((pointer) != NULL)
#define TEST_ASSERT_EMPTY(pointer)                                       TEST_ASSERT((pointer) == NULL || *(pointer) == 0)
#define TEST_ASSERT_NOT_EMPTY(pointer)                                   TEST_ASSERT((pointer) != NULL && *(pointer) != 0)

/*-------------------------------------------------------
 * Integer Assertions
 *-------------------------------------------------------*/
#define TEST_ASSERT_EQUAL_INT(expected, actual)                          UnityAssertEqualNumber((UNITY_INT)(expected), (UNITY_INT)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT)
#define TEST_ASSERT_EQUAL_INT8(expected, actual)                         UnityAssertEqualNumber((UNITY_INT)(UNITY_INT8)(expected), (UNITY_INT)(UNITY_INT8)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT8)
#define TEST_ASSERT_EQUAL_INT16(expected, actual)                        UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)(expected), (UNITY_INT)(UNITY_INT16)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT16)
#define TEST_ASSERT_EQUAL_INT32(expected, actual)                        UnityAssertEqualNumber((UNITY_INT)(UNITY_INT32)(expected), (UNITY_INT)(UNITY_INT32)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT32)
#define TEST_ASSERT_EQUAL_UINT(expected, actual)                         UnityAssertEqualNumber((UNITY_INT)(expected), (UNITY_INT)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT)
#define TEST_ASSERT_EQUAL_UINT8(expected, actual)                        UnityAssertEqualNumber((UNITY_INT)(UNITY_UINT8)(expected), (UNITY_INT)(UNITY_UINT8)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT8)
#define TEST_ASSERT_EQUAL_UINT16(expected, actual)                       UnityAssertEqualNumber((UNITY_INT)(UNITY_UINT16)(expected), (UNITY_INT)(UNITY_UINT16)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT16)
#define TEST_ASSERT_EQUAL_UINT32(expected, actual)                       UnityAssertEqualNumber((UNITY_INT)(UNITY_UINT32)(expected), (UNITY_INT)(UNITY_UINT32)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_UINT32)
#define TEST_ASSERT_EQUAL_HEX32(expected, actual)                        UnityAssertEqualNumber((UNITY_INT)(UNITY_UINT32)(expected), (UNITY_INT)(UNITY_UINT32)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_HEX32)

#define TEST_ASSERT_NOT_EQUAL(expected, actual)                          UnityAssertNotEqualNumber((UNITY_INT)(expected), (UNITY_INT)(actual), NULL, __LINE__, UNITY_DISPLAY_STYLE_INT)
#define TEST_ASSERT_INT_WITHIN(delta, expected, actual)                  TEST_ASSERT((((actual) >= ((expected) - (delta))) && ((actual) <= ((expected) + (delta)))))
#define TEST_ASSERT_UINT32_WITHIN(delta, expected, actual)                TEST_ASSERT((((actual) >= ((expected) - (delta))) && ((actual) <= ((expected) + (delta)))))


/*-------------------------------------------------------
 * Float Assertions
 *-------------------------------------------------------*/
#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual)                UnityAssertFloatsWithin((UNITY_FLOAT)(delta), (UNITY_FLOAT)(expected), (UNITY_FLOAT)(actual), NULL, __LINE__)
#define TEST_ASSERT_EQUAL_FLOAT(expected, actual)                        TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected, actual)

/*-------------------------------------------------------
 * String Assertions
 *-------------------------------------------------------*/
#define TEST_ASSERT_EQUAL_STRING(expected, actual)                       UnityAssertEqualString((const char*)(expected), (const char*)(actual), NULL, __LINE__)
#define TEST_ASSERT_EQUAL_STRING_LEN(expected, actual, len)              UnityAssertEqualStringLen((const char*)(expected), (const char*)(actual), (const UNITY_UINT32)(len), NULL, __LINE__)

/*-------------------------------------------------------
 * Basic Assertions with Messages
 *-------------------------------------------------------*/
#define TEST_ASSERT_MESSAGE(condition, message)                          UnityAssert( (condition), 0, (message), __LINE__ )
#define TEST_ASSERT_TRUE_MESSAGE(condition, message)                     TEST_ASSERT_MESSAGE(condition, message)
#define TEST_ASSERT_FALSE_MESSAGE(condition, message)                    TEST_ASSERT_MESSAGE(!(condition), message)
#define TEST_ASSERT_EQUAL_INT_MESSAGE(expected, actual, message)         UnityAssertEqualNumber((UNITY_INT)(expected), (UNITY_INT)(actual), (message), __LINE__, UNITY_DISPLAY_STYLE_INT)
#define TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected, actual, message)      UnityAssertEqualNumber((UNITY_INT)(expected), (UNITY_INT)(actual), (message), __LINE__, UNITY_DISPLAY_STYLE_UINT32)
#define TEST_ASSERT_FLOAT_WITHIN_MESSAGE(delta, expected, actual, message) UnityAssertFloatsWithin((UNITY_FLOAT)(delta), (UNITY_FLOAT)(expected), (UNITY_FLOAT)(actual), (message), __LINE__)
#define TEST_ASSERT_EQUAL_STRING_MESSAGE(expected, actual, message)      UnityAssertEqualString((const char*)(expected), (const char*)(actual), (message), __LINE__)

/*-------------------------------------------------------
 * Test Execution Macros
 *-------------------------------------------------------*/
#define UNITY_BEGIN() UnityBegin(__FILE__)
#define UNITY_END()   UnityEnd()

#define RUN_TEST(func) \
    Unity.CurrentTestName = #func; \
    Unity.CurrentTestLineNumber = __LINE__; \
    Unity.NumberOfTests++; \
    if (TEST_PROTECT()) { \
        setUp(); \
        func(); \
    } \
    if (TEST_PROTECT()) { \
        tearDown(); \
    } \
    UnityConcludeTest();



#ifdef __cplusplus
}
#endif

#endif /* UNITY_FRAMEWORK_H */
