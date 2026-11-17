/**
 * @file    test_tmc2208.c
 * @brief   Unit tests for TMC2208 UART communication verification and checked register reads.
 */

#include "unity.h"
#include "tmc2208.h"
#include <string.h>

extern const uint8_t tmcCRCTable_Poly7Reflected[256];

static uint8_t s_mock_rx_data[8];
static bool s_mock_uart_success = true;
static size_t s_last_write_len = 0;
static size_t s_last_read_len = 0;
static uint8_t s_last_tx_data[8];

static uint8_t calc_test_crc8(const uint8_t *data, uint32_t bytes)
{
    uint8_t result = 0;
    while (bytes--) {
        result = tmcCRCTable_Poly7Reflected[result ^ *data++];
    }
    result = ((result >> 1) & 0x55) | ((result & 0x55) << 1);
    result = ((result >> 2) & 0x33) | ((result & 0x33) << 2);
    result = ((result >> 4) & 0x0F) | ((result & 0x0F) << 4);
    return result;
}

static void prepare_mock_response(uint8_t reg_addr, uint32_t reg_val)
{
    s_mock_rx_data[0] = 0x05;
    s_mock_rx_data[1] = 0xFF;
    s_mock_rx_data[2] = reg_addr & TMC2208_ADDRESS_MASK;
    s_mock_rx_data[3] = (uint8_t)((reg_val >> 24) & 0xFF);
    s_mock_rx_data[4] = (uint8_t)((reg_val >> 16) & 0xFF);
    s_mock_rx_data[5] = (uint8_t)((reg_val >> 8) & 0xFF);
    s_mock_rx_data[6] = (uint8_t)(reg_val & 0xFF);
    s_mock_rx_data[7] = calc_test_crc8(s_mock_rx_data, 7);
}

bool tmc2208_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength)
{
    (void)icID;
    s_last_write_len = writeLength;
    s_last_read_len = readLength;
    if (data != NULL && writeLength > 0) {
        memcpy(s_last_tx_data, data, writeLength < sizeof(s_last_tx_data) ? writeLength : sizeof(s_last_tx_data));
    }

    if (!s_mock_uart_success) {
        return false;
    }

    if (readLength > 0 && data != NULL) {
        memcpy(data, s_mock_rx_data, readLength < sizeof(s_mock_rx_data) ? readLength : sizeof(s_mock_rx_data));
    }

    return true;
}

uint8_t tmc2208_getNodeAddress(uint16_t icID)
{
    return (uint8_t)icID;
}

void setUp(void)
{
    s_mock_uart_success = true;
    s_last_write_len = 0;
    s_last_read_len = 0;
    memset(s_mock_rx_data, 0, sizeof(s_mock_rx_data));
    memset(s_last_tx_data, 0, sizeof(s_last_tx_data));
}

void tearDown(void)
{
}

void test_tmc2208_read_checked_success(void)
{
    const uint32_t expected_ioin = 0x20000000U;
    prepare_mock_response(TMC2208_IOIN, expected_ioin);

    int32_t val = 0;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, &val);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(expected_ioin, (uint32_t)val);
    TEST_ASSERT_EQUAL_UINT32(4, (uint32_t)s_last_write_len);
    TEST_ASSERT_EQUAL_UINT32(8, (uint32_t)s_last_read_len);
    TEST_ASSERT_EQUAL_UINT8(0x05, s_last_tx_data[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, s_last_tx_data[1]);
    TEST_ASSERT_EQUAL_UINT8(TMC2208_IOIN, s_last_tx_data[2]);
}

void test_tmc2208_read_checked_bad_crc(void)
{
    prepare_mock_response(TMC2208_IOIN, 0x20000000U);
    s_mock_rx_data[7] ^= 0x55; /* Invalidate CRC */

    int32_t val = 0x1234;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, &val);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_INT32(0x1234, val);
}

void test_tmc2208_read_checked_bad_sync(void)
{
    prepare_mock_response(TMC2208_IOIN, 0x20000000U);
    s_mock_rx_data[0] = 0x06; /* Invalid sync nibble */

    int32_t val = 0;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, &val);

    TEST_ASSERT_FALSE(ok);
}

void test_tmc2208_read_checked_bad_master_address(void)
{
    prepare_mock_response(TMC2208_IOIN, 0x20000000U);
    s_mock_rx_data[1] = 0xFE; /* Should be 0xFF */

    int32_t val = 0;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, &val);

    TEST_ASSERT_FALSE(ok);
}

void test_tmc2208_read_checked_address_mismatch(void)
{
    prepare_mock_response(TMC2208_GSTAT, 0x00000001U); /* GSTAT (0x01) instead of IOIN (0x06) */

    int32_t val = 0;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, &val);

    TEST_ASSERT_FALSE(ok);
}

void test_tmc2208_read_checked_uart_failure(void)
{
    prepare_mock_response(TMC2208_IOIN, 0x20000000U);
    s_mock_uart_success = false;

    int32_t val = 0;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, &val);

    TEST_ASSERT_FALSE(ok);
}

void test_tmc2208_read_checked_null_output_pointer(void)
{
    prepare_mock_response(TMC2208_IOIN, 0x20000000U);

    bool ok = tmc2208_readRegisterChecked(0, TMC2208_IOIN, NULL);

    TEST_ASSERT_FALSE(ok);
}

void test_tmc2208_legacy_read_register_returns_zero_on_error(void)
{
    prepare_mock_response(TMC2208_IOIN, 0x20000000U);
    s_mock_uart_success = false;

    int32_t val = tmc2208_readRegister(0, TMC2208_IOIN);
    TEST_ASSERT_EQUAL_INT32(0, val);

    s_mock_uart_success = true;
    val = tmc2208_readRegister(0, TMC2208_IOIN);
    TEST_ASSERT_EQUAL_UINT32(0x20000000U, (uint32_t)val);
}

void test_tmc2208_cached_register_read(void)
{
    /* Write-only register TMC2208_TPWMTHRS (0x13) is served from shadow cache */
    uint32_t sample_tpwmthrs = 0x000001F4U;
    (void)tmc2208_cache(0, TMC2208_CACHE_FILL_DEFAULT, TMC2208_TPWMTHRS, &sample_tpwmthrs);

    /* Even if UART is failing, cached write-only register read succeeds */
    s_mock_uart_success = false;

    int32_t val = 0;
    bool ok = tmc2208_readRegisterChecked(0, TMC2208_TPWMTHRS, &val);

    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT32(0x000001F4U, (uint32_t)val);
}

void test_tmc2208_gconf_field_extract_and_update(void)
{
    uint32_t gconf = 0x00000041U;
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_PDN_DISABLE_FIELD,      1U);
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_MSTEP_REG_SELECT_FIELD, 1U);
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_EN_SPREADCYCLE_FIELD,   1U);
    gconf = tmc2208_fieldUpdate(gconf, TMC2208_MULTISTEP_FILT_FIELD,   0U);

    TEST_ASSERT_EQUAL_UINT32(1U, tmc2208_fieldExtract(gconf, TMC2208_PDN_DISABLE_FIELD));
    TEST_ASSERT_EQUAL_UINT32(1U, tmc2208_fieldExtract(gconf, TMC2208_MSTEP_REG_SELECT_FIELD));
    TEST_ASSERT_EQUAL_UINT32(1U, tmc2208_fieldExtract(gconf, TMC2208_EN_SPREADCYCLE_FIELD));
    TEST_ASSERT_EQUAL_UINT32(0U, tmc2208_fieldExtract(gconf, TMC2208_MULTISTEP_FILT_FIELD));
    TEST_ASSERT_EQUAL_UINT32(0x000000C5U, gconf);
}

void test_tmc2208_chopconf_mres_extract_and_update(void)
{
    uint32_t chopconf = 0x10000053U;
    chopconf = tmc2208_fieldUpdate(chopconf, TMC2208_MRES_FIELD,   1U); /* MS128 */
    chopconf = tmc2208_fieldUpdate(chopconf, TMC2208_INTPOL_FIELD, 0U);

    TEST_ASSERT_EQUAL_UINT32(1U, tmc2208_fieldExtract(chopconf, TMC2208_MRES_FIELD));
    TEST_ASSERT_EQUAL_UINT32(0U, tmc2208_fieldExtract(chopconf, TMC2208_INTPOL_FIELD));
    TEST_ASSERT_EQUAL_UINT32(3U, tmc2208_fieldExtract(chopconf, TMC2208_TOFF_FIELD));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_tmc2208_read_checked_success);
    RUN_TEST(test_tmc2208_read_checked_bad_crc);
    RUN_TEST(test_tmc2208_read_checked_bad_sync);
    RUN_TEST(test_tmc2208_read_checked_bad_master_address);
    RUN_TEST(test_tmc2208_read_checked_address_mismatch);
    RUN_TEST(test_tmc2208_read_checked_uart_failure);
    RUN_TEST(test_tmc2208_read_checked_null_output_pointer);
    RUN_TEST(test_tmc2208_legacy_read_register_returns_zero_on_error);
    RUN_TEST(test_tmc2208_cached_register_read);
    RUN_TEST(test_tmc2208_gconf_field_extract_and_update);
    RUN_TEST(test_tmc2208_chopconf_mres_extract_and_update);
    return UNITY_END();
}
