/**
 * @file    test_syringe_db.c
 * @brief   Unit tests for syringe profile database operations and bounds validation.
 *          Structured in conventional Arrange-Act-Assert (Given-When-Then) style.
 */

#include "unity.h"
#include "syringe_db.h"
#include <string.h>

static SyringeDatabase_t testDb;

void setUp(void) {
    SyringeDB_Format(&testDb);
}

void tearDown(void) {}

void test_initial_db_state(void) {
    /* [GIVEN] A freshly formatted database */
    const uint32_t expected_magic = SYRINGE_DB_MAGIC;
    const uint32_t expected_count = 0;

    /* [WHEN] Validating database header */
    bool actual_is_valid = SyringeDB_IsValid(&testDb);

    /* [THEN] Database is valid with count 0 */
    TEST_ASSERT_TRUE(actual_is_valid);
    TEST_ASSERT_EQUAL_UINT32(expected_magic, testDb.magic);
    TEST_ASSERT_EQUAL_UINT32(expected_count, testDb.count);
}

void test_corrupted_header_detection(void) {
    /* [GIVEN] A database with corrupted magic word */
    testDb.magic = 0xDEADBEEF;

    /* [WHEN] Checking validity */
    bool actual_valid_corrupt_magic = SyringeDB_IsValid(&testDb);

    /* [THEN] Must be rejected */
    TEST_ASSERT_FALSE(actual_valid_corrupt_magic);

    /* [GIVEN] A database with valid magic but count exceeding MAX_SAVED_SYRINGES */
    testDb.magic = SYRINGE_DB_MAGIC;
    testDb.count = MAX_SAVED_SYRINGES + 1;

    /* [WHEN] Checking validity */
    bool actual_valid_overflow_count = SyringeDB_IsValid(&testDb);

    /* [THEN] Must be rejected */
    TEST_ASSERT_FALSE(actual_valid_overflow_count);
}

void test_add_syringes_and_capacity_limit(void) {
    /* [GIVEN] A profile template */
    SyringeProfile_t profile_to_add;
    profile_to_add.maxVolume_ml = 10.0f;
    profile_to_add.innerDiameter_mm = 14.5f;

    /* [WHEN] Adding up to MAX_SAVED_SYRINGES (10) */
    for (uint32_t i = 0; i < MAX_SAVED_SYRINGES; i++) {
        snprintf(profile_to_add.name, SYRINGE_NAME_LEN, "SYR_%u", (unsigned int)i);
        bool actual_add_result = SyringeDB_Add(&testDb, &profile_to_add);
        TEST_ASSERT_TRUE(actual_add_result);
        TEST_ASSERT_EQUAL_UINT32(i + 1, testDb.count);
    }

    /* [WHEN] Attempting to add an 11th syringe */
    snprintf(profile_to_add.name, SYRINGE_NAME_LEN, "OVERFLOW");
    bool actual_overflow_result = SyringeDB_Add(&testDb, &profile_to_add);

    /* [THEN] 11th addition is rejected and count remains at 10 */
    TEST_ASSERT_FALSE(actual_overflow_result);
    TEST_ASSERT_EQUAL_UINT32(MAX_SAVED_SYRINGES, testDb.count);
}

void test_modify_syringe(void) {
    /* [GIVEN] A database containing one profile at index 0 */
    const SyringeProfile_t initial_profile = {"BD 10ML", 10.0f, 14.5f};
    SyringeDB_Add(&testDb, &initial_profile);

    const SyringeProfile_t updated_profile = {"BD 10ML MOD", 12.0f, 15.0f};

    /* [WHEN] Modifying profile at index 0 */
    bool actual_modify_result = SyringeDB_Modify(&testDb, 0, &updated_profile);

    /* [THEN] Profile was updated with new parameters */
    TEST_ASSERT_TRUE(actual_modify_result);

    const SyringeProfile_t *actual_stored = SyringeDB_Get(&testDb, 0);
    TEST_ASSERT_NOT_NULL(actual_stored);
    TEST_ASSERT_EQUAL_STRING("BD 10ML MOD", actual_stored->name);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, actual_stored->maxVolume_ml);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, actual_stored->innerDiameter_mm);

    /* [THEN] Modifying an out-of-bounds index must fail */
    TEST_ASSERT_FALSE(SyringeDB_Modify(&testDb, 5, &updated_profile));
}

void test_delete_syringe_shifts_elements(void) {
    /* [GIVEN] A database with 3 profiles: SYR_0, SYR_1, SYR_2 */
    const SyringeProfile_t s0 = {"SYR_0", 1.0f, 4.0f};
    const SyringeProfile_t s1 = {"SYR_1", 2.0f, 6.0f};
    const SyringeProfile_t s2 = {"SYR_2", 5.0f, 10.0f};

    SyringeDB_Add(&testDb, &s0);
    SyringeDB_Add(&testDb, &s1);
    SyringeDB_Add(&testDb, &s2);
    TEST_ASSERT_EQUAL_UINT32(3, testDb.count);

    /* [WHEN] Deleting the middle syringe (index 1 = SYR_1) */
    bool actual_delete_result = SyringeDB_Delete(&testDb, 1);

    /* [THEN] Deletion succeeds and count decrements to 2 */
    TEST_ASSERT_TRUE(actual_delete_result);
    TEST_ASSERT_EQUAL_UINT32(2, testDb.count);

    /* [THEN] Index 0 remains SYR_0 */
    const SyringeProfile_t *p0 = SyringeDB_Get(&testDb, 0);
    TEST_ASSERT_NOT_NULL(p0);
    TEST_ASSERT_EQUAL_STRING("SYR_0", p0->name);

    /* [THEN] Index 1 was shifted down to become SYR_2 */
    const SyringeProfile_t *p1 = SyringeDB_Get(&testDb, 1);
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_EQUAL_STRING("SYR_2", p1->name);

    /* [THEN] Index 2 is now empty / out of bounds */
    TEST_ASSERT_NULL(SyringeDB_Get(&testDb, 2));
}

void test_find_by_name(void) {
    /* [GIVEN] Profiles "SYR_ALPHA" and "SYR_BETA" in the database */
    const SyringeProfile_t s0 = {"SYR_ALPHA", 5.0f, 10.0f};
    const SyringeProfile_t s1 = {"SYR_BETA", 10.0f, 14.5f};

    SyringeDB_Add(&testDb, &s0);
    SyringeDB_Add(&testDb, &s1);

    /* [WHEN] Searching by name */
    int32_t actual_idx_alpha = SyringeDB_FindByName(&testDb, "SYR_ALPHA");
    int32_t actual_idx_beta = SyringeDB_FindByName(&testDb, "SYR_BETA");
    int32_t actual_idx_missing = SyringeDB_FindByName(&testDb, "NON_EXISTENT");

    /* [THEN] Correct indices returned, -1 for missing */
    TEST_ASSERT_EQUAL_INT32(0, actual_idx_alpha);
    TEST_ASSERT_EQUAL_INT32(1, actual_idx_beta);
    TEST_ASSERT_EQUAL_INT32(-1, actual_idx_missing);
}

void test_null_handling_and_boundary_checks(void) {
    /* [GIVEN] A null database pointer */
    TEST_ASSERT_FALSE(SyringeDB_IsValid(NULL));
    TEST_ASSERT_NULL(SyringeDB_Get(NULL, 0));

    SyringeProfile_t prof = {"SAMPLE", 5.0f, 10.0f};
    TEST_ASSERT_FALSE(SyringeDB_Add(NULL, &prof));
    TEST_ASSERT_FALSE(SyringeDB_Add(&testDb, NULL));
    TEST_ASSERT_FALSE(SyringeDB_Modify(NULL, 0, &prof));
    TEST_ASSERT_FALSE(SyringeDB_Modify(&testDb, 0, NULL));
    TEST_ASSERT_FALSE(SyringeDB_Delete(NULL, 0));
    TEST_ASSERT_EQUAL_INT32(-1, SyringeDB_FindByName(NULL, "SAMPLE"));
    TEST_ASSERT_EQUAL_INT32(-1, SyringeDB_FindByName(&testDb, NULL));

    /* [WHEN] Calling SyringeDB_Get on empty DB */
    TEST_ASSERT_NULL(SyringeDB_Get(&testDb, 0));
    TEST_ASSERT_NULL(SyringeDB_Get(&testDb, 10));

    /* [WHEN] Calling SyringeDB_Get on corrupted DB */
    testDb.magic = 0xBAD0;
    testDb.count = 1;
    TEST_ASSERT_NULL(SyringeDB_Get(&testDb, 0));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_db_state);
    RUN_TEST(test_corrupted_header_detection);
    RUN_TEST(test_add_syringes_and_capacity_limit);
    RUN_TEST(test_modify_syringe);
    RUN_TEST(test_delete_syringe_shifts_elements);
    RUN_TEST(test_find_by_name);
    RUN_TEST(test_null_handling_and_boundary_checks);
    return UNITY_END();
}
