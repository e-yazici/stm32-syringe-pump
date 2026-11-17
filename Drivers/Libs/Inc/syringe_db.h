/**
 * @file    syringe_db.h
 * @brief   Hardware-agnostic syringe profile database logic and manipulation.
 * @author  eyazici
 * @date    2026
 *
 * @details Manages syringe profile records (name, volume, diameter) in a fixed-size
 *          in-memory structure that can be persisted to flash, EEPROM, or tested in RAM.
 */

#ifndef LIBS_INC_SYRINGE_DB_H_
#define LIBS_INC_SYRINGE_DB_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_SAVED_SYRINGES  10
#define SYRINGE_NAME_LEN    16
#define SYRINGE_DB_MAGIC    0xABCD1234U

/**
 * @brief Single syringe profile with physical dimensions.
 */
typedef struct {
    char name[SYRINGE_NAME_LEN];    ///< Human-readable identifier (null-terminated)
    float maxVolume_ml;             ///< Maximum syringe capacity in milliliters
    float innerDiameter_mm;         ///< Inner barrel diameter in millimeters
} SyringeProfile_t;

/**
 * @brief Syringe database header and profile array.
 */
typedef struct {
    uint32_t magic;                                ///< Validation marker; must equal SYRINGE_DB_MAGIC
    uint32_t count;                                ///< Number of syringe profiles stored (0 to MAX_SAVED_SYRINGES)
    SyringeProfile_t syringes[MAX_SAVED_SYRINGES]; ///< Array of saved syringe profiles
} SyringeDatabase_t;

/**
 * @brief  Check whether a database image in memory has valid header and count.
 * @param  db Pointer to SyringeDatabase_t.
 * @retval true if valid, false if corrupted or uninitialized.
 */
static inline bool SyringeDB_IsValid(const SyringeDatabase_t *db) {
    if (db == NULL) {
        return false;
    }
    return (db->magic == SYRINGE_DB_MAGIC && db->count <= MAX_SAVED_SYRINGES);
}

/**
 * @brief  Format / initialize a database structure to default empty state.
 * @param  db Pointer to SyringeDatabase_t.
 */
void SyringeDB_Format(SyringeDatabase_t *db);

/**
 * @brief  Append a new syringe profile to the database.
 * @param  db      Pointer to SyringeDatabase_t.
 * @param  profile Pointer to the SyringeProfile_t to add.
 * @retval true if profile added, false if database is full or params invalid.
 */
bool SyringeDB_Add(SyringeDatabase_t *db, const SyringeProfile_t *profile);

/**
 * @brief  Modify an existing syringe profile at the given index.
 * @param  db      Pointer to SyringeDatabase_t.
 * @param  index   Zero-based index of profile to modify.
 * @param  profile Pointer to new profile data.
 * @retval true if modified, false if index invalid or profile invalid.
 */
bool SyringeDB_Modify(SyringeDatabase_t *db, uint32_t index, const SyringeProfile_t *profile);

/**
 * @brief  Delete a syringe profile at the given index and compact remaining entries.
 * @param  db    Pointer to SyringeDatabase_t.
 * @param  index Zero-based index of profile to delete.
 * @retval true if deleted, false if index invalid or database empty.
 */
bool SyringeDB_Delete(SyringeDatabase_t *db, uint32_t index);

/**
 * @brief  Retrieve a pointer to the syringe profile at the given index.
 * @param  db    Pointer to SyringeDatabase_t.
 * @param  index Zero-based index.
 * @retval Pointer to SyringeProfile_t, or NULL if index out of bounds.
 */
static inline const SyringeProfile_t* SyringeDB_Get(const SyringeDatabase_t *db, uint32_t index) {
    if (!SyringeDB_IsValid(db) || index >= db->count) {
        return NULL;
    }
    return &db->syringes[index];
}

/**
 * @brief  Find a syringe profile by name.
 * @param  db   Pointer to SyringeDatabase_t.
 * @param  name Null-terminated name string.
 * @retval Index (0 to count-1) if found, or -1 if not found.
 */
int32_t SyringeDB_FindByName(const SyringeDatabase_t *db, const char *name);

#ifdef __cplusplus
}
#endif

#endif /* LIBS_INC_SYRINGE_DB_H_ */
