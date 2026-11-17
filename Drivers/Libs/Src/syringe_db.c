/**
 * @file    syringe_db.c
 * @brief   Implementation of hardware-agnostic syringe database operations.
 * @author  eyazici
 * @date    2026
 */

#include "syringe_db.h"
#include <string.h>

void SyringeDB_Format(SyringeDatabase_t *db) {
    if (db == NULL) {
        return;
    }
    memset(db, 0, sizeof(SyringeDatabase_t));
    db->magic = SYRINGE_DB_MAGIC;
    db->count = 0;
}

bool SyringeDB_Add(SyringeDatabase_t *db, const SyringeProfile_t *profile) {
    if (db == NULL || profile == NULL) {
        return false;
    }
    if (!SyringeDB_IsValid(db)) {
        SyringeDB_Format(db);
    }
    if (db->count >= MAX_SAVED_SYRINGES) {
        return false;
    }

    /* Copy profile to end of array */
    memcpy(&db->syringes[db->count], profile, sizeof(SyringeProfile_t));
    /* Guarantee null termination */
    db->syringes[db->count].name[SYRINGE_NAME_LEN - 1] = '\0';
    db->count++;

    return true;
}

bool SyringeDB_Modify(SyringeDatabase_t *db, uint32_t index, const SyringeProfile_t *profile) {
    if (db == NULL || profile == NULL || !SyringeDB_IsValid(db)) {
        return false;
    }
    if (index >= db->count) {
        return false;
    }

    memcpy(&db->syringes[index], profile, sizeof(SyringeProfile_t));
    db->syringes[index].name[SYRINGE_NAME_LEN - 1] = '\0';

    return true;
}

bool SyringeDB_Delete(SyringeDatabase_t *db, uint32_t index) {
    if (db == NULL || !SyringeDB_IsValid(db)) {
        return false;
    }
    if (index >= db->count || db->count == 0) {
        return false;
    }

    /* Shift subsequent profiles left by one */
    for (uint32_t i = index; i < (db->count - 1U); i++) {
        db->syringes[i] = db->syringes[i + 1U];
    }

    /* Clear the last freed slot */
    memset(&db->syringes[db->count - 1U], 0, sizeof(SyringeProfile_t));
    db->count--;

    return true;
}

int32_t SyringeDB_FindByName(const SyringeDatabase_t *db, const char *name) {
    if (db == NULL || !SyringeDB_IsValid(db) || name == NULL) {
        return -1;
    }
    for (uint32_t i = 0; i < db->count; i++) {
        if (strncmp(db->syringes[i].name, name, SYRINGE_NAME_LEN) == 0) {
            return (int32_t)i;
        }
    }
    return -1;
}
