// Copyright (C) 2026 COGIP Robotics association <cogip35@gmail.com>
// This file is subject to the terms and conditions of the GNU Lesser
// General Public License v2.1. See the file LICENSE in the top level
// directory for more details.

/// @ingroup    lib_flash_kv_storage
/// @{
/// @file
/// @brief      FlashKVStorage implementation — FlashDB KVDB singleton
/// @author     Mathis Lécrivain <lecrivain.mathis@gmail.com>

#include "flash_kv_storage/FlashKVStorage.hpp"

#include <cstdio>

#include "fal_cfg.h"
#include "mtd.h"

#ifndef FAL_MTD
#error "FAL_MTD must be defined for FlashKVStorage"
#endif
#ifndef FAL_PART0_LABEL
#error "FAL_PART0_LABEL must be defined in fal_cfg.h for FlashKVStorage"
#endif
#ifndef FAL_PART0_LENGTH
#error "FAL_PART0_LENGTH must be defined in fal_cfg.h for FlashKVStorage"
#endif

extern "C" {
// Initialize FlashDB MTD backend (defined in fal_mtd_port.c, no public header)
void fdb_mtd_init(mtd_dev_t* mtd);
}

namespace cogip {
namespace flash_kv_storage {

void FlashKVStorage::fdb_lock(fdb_db_t db)
{
    mutex_lock(static_cast<mutex_t*>(db->user_data));
}

void FlashKVStorage::fdb_unlock(fdb_db_t db)
{
    mutex_unlock(static_cast<mutex_t*>(db->user_data));
}

FlashKVStorage::FlashKVStorage() : kvdb_{0}, kvdb_mutex_(MUTEX_INIT), initialized_(false) {}

FlashKVStorage& FlashKVStorage::instance()
{
    static FlashKVStorage store;
    return store;
}

int FlashKVStorage::init()
{
    if (initialized_) {
        return 0;
    }

    // Initialize MTD device for FlashDB
    fdb_mtd_init(FAL_MTD);

    // Compute sector size from MTD geometry
    uint32_t sec_size = FAL_MTD->pages_per_sector * FAL_MTD->page_size;
    uint32_t db_size = FAL_PART0_LENGTH;

    // Configure lock/unlock callbacks
    mutex_init(&kvdb_mutex_);
    fdb_kvdb_control(&kvdb_, FDB_KVDB_CTRL_SET_LOCK, reinterpret_cast<void*>(fdb_lock));
    fdb_kvdb_control(&kvdb_, FDB_KVDB_CTRL_SET_UNLOCK, reinterpret_cast<void*>(fdb_unlock));

    // Configure sector and database size
    fdb_kvdb_control(&kvdb_, FDB_KVDB_CTRL_SET_SEC_SIZE, &sec_size);
    fdb_kvdb_control(&kvdb_, FDB_KVDB_CTRL_SET_MAX_SIZE, &db_size);

    // Initialize KVDB with partition label matching FAL_PART0_LABEL
    fdb_err_t result = fdb_kvdb_init(&kvdb_, kDBName, FAL_PART0_LABEL, nullptr, &kvdb_mutex_);
    if (result != FDB_NO_ERR) {
        return -1;
    }

    initialized_ = true;

    return 0;
}

void FlashKVStorage::hash_to_key(uint32_t key_hash, char* buf)
{
    snprintf(buf, 9, "%08x", static_cast<unsigned int>(key_hash));
}

int FlashKVStorage::store_blob(uint32_t key_hash, const void* data, size_t size)
{
    if (!initialized_) {
        return -1;
    }

    char key[9];
    hash_to_key(key_hash, key);

    struct fdb_blob blob;
    fdb_blob_make(&blob, data, size);
    fdb_err_t result = fdb_kv_set_blob(&kvdb_, key, &blob);

    return (result == FDB_NO_ERR) ? 0 : -1;
}

int FlashKVStorage::load_blob(uint32_t key_hash, void* data, size_t size)
{
    if (!initialized_) {
        return -1;
    }

    char key[9];
    hash_to_key(key_hash, key);

    struct fdb_blob blob;
    fdb_blob_make(&blob, data, size);
    fdb_kv_get_blob(&kvdb_, key, &blob);

    return (blob.saved.len == size) ? 0 : -1;
}

int FlashKVStorage::del(uint32_t key_hash)
{
    if (!initialized_) {
        return -1;
    }

    char key[9];
    hash_to_key(key_hash, key);

    fdb_err_t result = fdb_kv_del(&kvdb_, key);

    return (result == FDB_NO_ERR) ? 0 : -1;
}

} // namespace flash_kv_storage
} // namespace cogip

/// @}
