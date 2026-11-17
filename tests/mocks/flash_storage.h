/*
 * Mock header for host unit tests.
 * IMPORTANT: Keep function prototypes and types synchronized with Drivers/Libs/Inc/flash_storage.h.
 */
#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "syringe_db.h"

void FlashStorage_Init(void);
const SyringeDatabase_t* FlashStorage_GetDB(void);
bool FlashStorage_AddSyringe(const SyringeProfile_t *profile);
bool FlashStorage_ModifySyringe(uint32_t index, const SyringeProfile_t *profile);
bool FlashStorage_DeleteSyringe(uint32_t index);

#endif /* FLASH_STORAGE_H */
