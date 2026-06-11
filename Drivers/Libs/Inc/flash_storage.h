/**
 * @file    flash_storage.h
 * @brief   Non-volatile syringe profile database stored in internal flash.
 * @author  eyazici
 * @date    2026
 *
 * @details Provides a simple key-value style database for storing syringe
 *          profiles (name, volume, diameter) in the last page of the STM32F3
 *          internal flash. Data persists across power cycles and is validated
 *          using a magic word header.
 *
 * @note    The database occupies a single 2 KB flash page (last page).
 *          The STM32F3 flash requires doubleword (64-bit) aligned programming
 *          and page-level erasing before re-writing.
 */

#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include "main.h"
#include <stdbool.h>

#define MAX_SAVED_SYRINGES 10    ///< Maximum number of syringe profiles that can be stored
#define SYRINGE_NAME_LEN   16    ///< Maximum length of a syringe name string (including null terminator)

/**
 * @brief  Single syringe profile with physical dimensions.
 *
 * @details Stores the user-defined parameters needed to calculate linear
 *          displacement and flow rate for a specific syringe model.
 */
typedef struct {
    char name[SYRINGE_NAME_LEN];    ///< Human-readable identifier (null-terminated, e.g. "BD 10ML")
    float maxVolume_ml;             ///< Maximum syringe capacity in milliliters
    float innerDiameter_mm;         ///< Inner barrel diameter in millimeters (used for area calculation)
} SyringeProfile_t;

/**
 * @brief  Flash-resident database header and profile array.
 *
 * @details This structure is memory-mapped directly to the flash page at
 *          FLASH_DB_ADDR. The magic word is checked at startup to determine
 *          whether the flash contains valid data or needs formatting.
 */
typedef struct {
    uint32_t magic; ///< Validation marker; must equal 0xABCD1234 for valid data
    uint32_t count; ///< Number of syringe profiles currently stored (0 to MAX_SAVED_SYRINGES)
    SyringeProfile_t syringes[MAX_SAVED_SYRINGES]; ///< Array of saved syringe profiles
} SyringeDatabase_t;

/**
 * @brief  Initialize the flash storage subsystem.
 *
 * @details Checks the magic word at the database flash address. If the magic
 *          word is missing or the count is out of range, the flash page is
 *          erased and re-initialized with a valid header (magic + count=0).
 *
 * @note   Must be called once at startup before any other FlashStorage function.
 *         Temporarily unlocks flash for programming if formatting is needed.
 */
void FlashStorage_Init(void);

/**
 * @brief  Get a read-only pointer to the flash-resident database.
 * @retval Pointer to the SyringeDatabase_t mapped at FLASH_DB_ADDR.
 *
 * @note   The returned pointer points directly to flash memory. Do not attempt
 *         to write through this pointer — use FlashStorage_AddSyringe() instead.
 */
SyringeDatabase_t* FlashStorage_GetDB(void);

/**
 * @brief  Append a new syringe profile to the database.
 * @param  profile Pointer to the SyringeProfile_t to add.
 * @retval true    Profile was successfully written to flash.
 * @retval false   Database is full (count >= MAX_SAVED_SYRINGES).
 *
 * @details Uses a read-modify-write strategy:
 *          1. Copy entire database from flash to a RAM buffer.
 *          2. Append the new profile and increment the count.
 *          3. Erase the flash page.
 *          4. Write the updated database back in 64-bit doubleword chunks.
 *
 * @warning This function erases and reprograms the entire flash page. Do not
 *          call from ISR context. Ensure no concurrent flash reads during erase.
 */
bool FlashStorage_AddSyringe(const SyringeProfile_t *profile);

/**
 * @brief  Modify an existing syringe profile in the database.
 * @param  index   Index of the syringe profile to modify (0 to count-1).
 * @param  profile Pointer to the updated SyringeProfile_t.
 * @retval true    Profile was successfully updated in flash.
 * @retval false   Invalid index.
 */
bool FlashStorage_ModifySyringe(uint32_t index, const SyringeProfile_t *profile);

/**
 * @brief  Delete a syringe profile from the database.
 * @param  index   Index of the syringe profile to delete (0 to count-1).
 * @retval true    Profile was successfully deleted from flash.
 * @retval false   Invalid index.
 *
 * @details Shifts all subsequent profiles to fill the gap created by the deletion.
 */
bool FlashStorage_DeleteSyringe(uint32_t index);

#endif // FLASH_STORAGE_H
