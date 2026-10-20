/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MEMC_MEMC_TT_MIMIR_H_
#define ZEPHYR_INCLUDE_DRIVERS_MEMC_MEMC_TT_MIMIR_H_

#include <zephyr/device.h>

#include "err.h"
#include "fw_params.h"
#include "gddr_common_types.h"
#include "gddr_backend.h"
#include "gddr_types.h"

/**
 * @brief GDDR address-map configuration, applied to the ITN at memc init.
 *
 * Platform-supplied (e.g. FW-handshake scratch registers), like fw_params.
 */
struct memc_tt_mimir_addr_cfg {
	/** First byte of the GDDR window the MEM-tile firewalls allow */
	uint64_t mem_start;
	/** Last byte (inclusive) of the GDDR window the MEM-tile firewalls allow */
	uint64_t mem_end;
	/** ITN interleave mode (noc_interleave_mode_t) */
	uint32_t interleave_mode;
	/** Total GDDR capacity in GB (noc_gddr7_realign_t) */
	uint32_t capacity_gb;
};

/**
 * @brief Install the platform GDDR configuration for the controller.
 *
 * Stores the backend and fw_params in the device's data and copies @p addr_cfg;
 * the device applies them at its own init (tile resets, ITN firewalls and
 * address map, then gddr_init()). Call once, before the memc device initializes.
 *
 * The backend, fw_params and address map are runtime/platform-supplied
 * (host-written params, environment-selected backend), which is why they arrive
 * through this API rather than devicetree. @p params is non-const because this
 * call fills in gddr_tile_mask from the controller's own DT topology (the
 * mem-tiles).
 *
 * @param dev      The memc controller device.
 * @param backend  Caller-owned backend (non-NULL, with a non-NULL get_blob).
 * @param params   fw_params minus gddr_tile_mask
 * @param addr_cfg GDDR address-map configuration (copied; non-NULL)
 * @return 0 on success, -EINVAL if @p dev, @p backend, @p backend->get_blob, @p params or
 *         @p addr_cfg is NULL.
 */
int memc_tt_mimir_set_config(const struct device *dev, const gddr_backend_t *backend,
			     fw_params_t *params, const struct memc_tt_mimir_addr_cfg *addr_cfg);

#endif /* ZEPHYR_INCLUDE_DRIVERS_MEMC_MEMC_TT_MIMIR_H_ */
