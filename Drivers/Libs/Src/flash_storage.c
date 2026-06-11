/**
 * @file    flash_storage.c
 * @brief   Internal flash-based persistent storage for syringe profiles.
 * @author  eyazici
 * @date    2026
 *
 * @details Implements a simple non-volatile database stored in the last page
 *          of STM32F3 internal flash. The database uses a
 *          flat layout: a header with a magic word + count, followed by an
 *          array of SyringeProfile_t records.
 *
 *          Flash memory constraints (STM32F3):
 *          - Page size: 2 KB (2048 bytes)
 *          - Minimum write granularity: 64-bit doubleword
 *          - Pages must be erased (all 0xFF) before programming
 *          - Erased flash reads as 0xFFFFFFFF
 *
 *          The database struct is memory-mapped at FLASH_DB_ADDR for direct
 *          read access (no copy needed for reads). Writes use a read-modify-write
 *          pattern: copy entire DB to RAM, modify, erase page, write back.
 *
 * @note    Flash programming requires HAL_FLASH_Unlock()/Lock() bracketing.
 *          Do not call write functions from ISR context.
 */

#include "flash_storage.h"
#include <string.h>

/** @brief Start address of the database in flash (last page) */
#define FLASH_DB_ADDR 0x0807F800

/** @brief Magic word used to validate that flash contains a formatted database */
#define FLASH_MAGIC_WORD 0xABCD1234

/**
 * @brief Const pointer to the flash-mapped database.
 *
 * @details This pointer is set at compile time to the flash address and provides
 *          direct read-only access to the database without copying. The volatile
 *          nature of flash contents (they survive power cycles but can be erased)
 *          is managed by the Init and AddSyringe, ModifySyringe, DeleteSyringe functions.
 */
static SyringeDatabase_t* const dbPtr = (SyringeDatabase_t*)FLASH_DB_ADDR;

/**
 * @brief  Initialize the flash database, formatting if necessary.
 *
 * @details Checks the magic word and count validity at the database address.
 *          If the magic word doesn't match or the count exceeds the maximum,
 *          the page is erased and a fresh header is written.
 *
 *          The header is packed into a single 64-bit doubleword:
 *            bits [31:0]  = magic word (0xABCD1234)
 *            bits [63:32] = count (0)
 */
void FlashStorage_Init(void) {
    if (dbPtr->magic != FLASH_MAGIC_WORD || dbPtr->count > MAX_SAVED_SYRINGES) {
        /* Flash is uninitialized or corrupted — format it */
        HAL_FLASH_Unlock();
        
        /* Configure page erase for the database page */
        FLASH_EraseInitTypeDef eraseInit;
        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.PageAddress = FLASH_DB_ADDR;
        eraseInit.NbPages = 1;
        
        uint32_t pageError = 0;
        HAL_FLASHEx_Erase(&eraseInit, &pageError);
        
        /*
         * Write the initial header as a single 64-bit doubleword.
         * STM32L4 flash requires doubleword-aligned programming.
         * Pack magic (low 32 bits) and count=0 (high 32 bits) together.
         */
        uint32_t magic = FLASH_MAGIC_WORD;
        uint32_t count = 0;
        uint64_t header = ((uint64_t)count << 32) | magic;
        
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_DB_ADDR, header);
        
        HAL_FLASH_Lock();
    }
}

/**
 * @brief  Get a pointer to the flash-resident database for direct read access.
 * @retval Pointer to the SyringeDatabase_t in flash at FLASH_DB_ADDR.
 */
SyringeDatabase_t* FlashStorage_GetDB(void) {
    return dbPtr;
}

/**
 * @brief  Append a new syringe profile to the persistent database.
 * @param  profile Pointer to the SyringeProfile_t to store.
 * @retval true  if the profile was successfully written.
 * @retval false if the database is full.
 *
 * @details Read-Modify-Write strategy:
 *          1. Copy the entire database struct from flash into a local RAM buffer.
 *          2. Append the new profile at index [count] and increment count.
 *          3. Erase the entire flash page (required before reprogramming).
 *          4. Write the modified struct back to flash in 64-bit doubleword chunks.
 *
 * @warning Erases the entire 2 KB page. Not safe to call from ISR context.
 *          Ensure no other code is reading flash during the erase operation.
 */
bool FlashStorage_AddSyringe(const SyringeProfile_t* profile) {
    if (dbPtr->count >= MAX_SAVED_SYRINGES) {
        return false; /* Database full — no room for another profile */
    }
    
    /* Step 1: Copy current database from flash to a RAM working buffer */
    SyringeDatabase_t tempDb;
    memcpy(&tempDb, dbPtr, sizeof(SyringeDatabase_t));
    
    /* Step 2: Append the new profile and increment the count */
    memcpy(&tempDb.syringes[tempDb.count], profile, sizeof(SyringeProfile_t));
    tempDb.count++;
    
    /* Step 3: Erase the flash page (mandatory before reprogramming) */
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_DB_ADDR;
    eraseInit.NbPages = 1;
    
    uint32_t pageError = 0;
    HAL_FLASHEx_Erase(&eraseInit, &pageError);
    
    /*
     * Step 4: Write the entire struct back to flash in 64-bit chunks.
     * Calculate the number of doublewords needed: round up to cover
     * any trailing bytes (sizeof may not be 8-byte aligned).
     */
    uint32_t numDoubleWords = (sizeof(SyringeDatabase_t) + 7) / 8;
    uint64_t* pData = (uint64_t*)&tempDb;
    uint32_t currentAddr = FLASH_DB_ADDR;
    
    for (uint32_t i = 0; i < numDoubleWords; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAddr, pData[i]);
        currentAddr += 8; /* Advance by 8 bytes (one doubleword) */
    }
    
    HAL_FLASH_Lock();
    return true;
}

bool FlashStorage_ModifySyringe(uint32_t index, const SyringeProfile_t* profile) {
    if (index >= dbPtr->count) {
        return false; /* Invalid index */
    }
    
    SyringeDatabase_t tempDb;
    memcpy(&tempDb, dbPtr, sizeof(SyringeDatabase_t));
    
    memcpy(&tempDb.syringes[index], profile, sizeof(SyringeProfile_t));
    
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_DB_ADDR;
    eraseInit.NbPages = 1;
    
    uint32_t pageError = 0;
    HAL_FLASHEx_Erase(&eraseInit, &pageError);
    
    uint32_t numDoubleWords = (sizeof(SyringeDatabase_t) + 7) / 8;
    uint64_t* pData = (uint64_t*)&tempDb;
    uint32_t currentAddr = FLASH_DB_ADDR;
    
    for (uint32_t i = 0; i < numDoubleWords; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAddr, pData[i]);
        currentAddr += 8;
    }
    
    HAL_FLASH_Lock();
    return true;
}

bool FlashStorage_DeleteSyringe(uint32_t index) {
    if (index >= dbPtr->count) {
        return false; /* Invalid index */
    }
    
    SyringeDatabase_t tempDb;
    memcpy(&tempDb, dbPtr, sizeof(SyringeDatabase_t));
    
    /* Shift subsequent profiles down to fill the gap */
    if (index < tempDb.count - 1) {
        memmove(&tempDb.syringes[index], &tempDb.syringes[index + 1], 
                sizeof(SyringeProfile_t) * (tempDb.count - index - 1));
    }
    
    tempDb.count--;
    
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = FLASH_DB_ADDR;
    eraseInit.NbPages = 1;
    
    uint32_t pageError = 0;
    HAL_FLASHEx_Erase(&eraseInit, &pageError);
    
    uint32_t numDoubleWords = (sizeof(SyringeDatabase_t) + 7) / 8;
    uint64_t* pData = (uint64_t*)&tempDb;
    uint32_t currentAddr = FLASH_DB_ADDR;
    
    for (uint32_t i = 0; i < numDoubleWords; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAddr, pData[i]);
        currentAddr += 8;
    }
    
    HAL_FLASH_Lock();
    return true;
}
