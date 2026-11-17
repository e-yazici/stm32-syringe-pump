/**
 * @file    flash_storage.c
 * @brief   Internal flash-based persistent storage for syringe profiles.
 * @author  eyazici
 * @date    2026
 *
 * @details Implements non-volatile database persistence stored in the last page
 *          of STM32F3 internal flash. Syringe data manipulation is performed
 *          via the unit-tested syringe_db module, and persisted to flash using
 *          page-erase and 64-bit doubleword programming.
 */

#include "flash_storage.h"
#include <string.h>

/**
 * @brief Const pointer to the flash-mapped database.
 */
static SyringeDatabase_t* const dbPtr = (SyringeDatabase_t*)FLASH_DB_ADDR;

/**
 * @brief Static staging buffer for flash write-back operations (8-byte aligned for doubleword flash programming; avoids 248-byte task stack allocation).
 */
static SyringeDatabase_t __attribute__((aligned(8))) tempDb;

/**
 * @brief Helper function to erase flash page and write a database struct.
 */
static bool FlashStorage_WriteBack(const SyringeDatabase_t *db) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_DB_ADDR;
    eraseInit.NbPages = 1;

    uint32_t pageError = 0;
    HAL_FLASHEx_Erase(&eraseInit, &pageError);

    uint32_t numDoubleWords = (sizeof(SyringeDatabase_t) + 7U) / 8U;
    const uint64_t* pData = (const uint64_t*)db;
    uint32_t currentAddr = FLASH_DB_ADDR;

    for (uint32_t i = 0; i < numDoubleWords; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAddr, pData[i]);
        currentAddr += 8U;
    }

    HAL_FLASH_Lock();
    return true;
}

void FlashStorage_Init(void) {
    if (!SyringeDB_IsValid(dbPtr)) {
        /* Flash is uninitialized or corrupted — format it */
        HAL_FLASH_Unlock();

        FLASH_EraseInitTypeDef eraseInit;
        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.PageAddress = FLASH_DB_ADDR;
        eraseInit.NbPages = 1;

        uint32_t pageError = 0;
        HAL_FLASHEx_Erase(&eraseInit, &pageError);

        uint32_t magic = SYRINGE_DB_MAGIC;
        uint32_t count = 0;
        /* Pack magic (bits 31:0) and count (bits 63:32) to program header in a single 64-bit operation */
        uint64_t header = ((uint64_t)count << 32) | magic;

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_DB_ADDR, header);

        HAL_FLASH_Lock();
    }
}

bool FlashStorage_AddSyringe(const SyringeProfile_t *profile) {
    if (profile == NULL || dbPtr->count >= MAX_SAVED_SYRINGES) {
        return false;
    }

    memcpy(&tempDb, dbPtr, sizeof(SyringeDatabase_t));

    if (!SyringeDB_Add(&tempDb, profile)) {
        return false;
    }

    return FlashStorage_WriteBack(&tempDb);
}

bool FlashStorage_ModifySyringe(uint32_t index, const SyringeProfile_t *profile) {
    if (profile == NULL || index >= dbPtr->count) {
        return false;
    }

    memcpy(&tempDb, dbPtr, sizeof(SyringeDatabase_t));

    if (!SyringeDB_Modify(&tempDb, index, profile)) {
        return false;
    }

    return FlashStorage_WriteBack(&tempDb);
}

bool FlashStorage_DeleteSyringe(uint32_t index) {
    if (index >= dbPtr->count) {
        return false;
    }

    memcpy(&tempDb, dbPtr, sizeof(SyringeDatabase_t));

    if (!SyringeDB_Delete(&tempDb, index)) {
        return false;
    }

    return FlashStorage_WriteBack(&tempDb);
}
