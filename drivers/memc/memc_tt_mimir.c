/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tenstorrent Mimir GDDR7 memory-controller driver.
 *
 * One Zephyr device models the whole controller; the tiles it brings up are the
 * status-okay nodes referenced by its mem-tiles. fw_params.gddr_tile_mask is
 * filled from that DT topology.
 *
 * The GDDR bring-up params, backend and address map come from the memc
 * devicetree node, unless the platform installs its own with
 * memc_tt_mimir_set_config() before the device initializes.
 *
 * At init, before gddr_init(), the driver opens the SMC outbound filter, releases
 * the enabled tiles' GDDR/CCE resets, opens their ITN firewalls and programs the
 * ITN address map. The SMN/ITN fabric reset is chip-wide and must already be
 * released by the platform.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/memc/memc_tt_mimir.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <basic_init.h>
#include <itn.h>
#include <noc_init.h>

#include "gddr.h"
#include "gddr_drv.h"

LOG_MODULE_REGISTER(memc_tt_mimir, CONFIG_MEMC_LOG_LEVEL);

#define DT_DRV_COMPAT tenstorrent_mimir_memc

/* gddr_tile_mask = OR of BIT(tile-index) over this controller's status-okay mem-tiles */
#define MEMC_TILE_BIT(node_id, prop, idx)                                                          \
	(DT_NODE_HAS_STATUS(DT_PHANDLE_BY_IDX(node_id, prop, idx), okay)                           \
		 ? BIT(DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, tile_index))                     \
		 : 0)
#define MEMC_GDDR_TILE_MASK DT_INST_FOREACH_PROP_ELEM_SEP(0, mem_tiles, MEMC_TILE_BIT, (|))

struct memc_tt_mimir_tile {
	uint32_t index;
	bool okay;
	uint32_t mem_filter_base;
	uint32_t cce_filter_base;
	uint32_t cce_cfg_filter_base;
};

#define MEMC_TILE_ENTRY(node_id, prop, idx)                                                        \
	{                                                                                          \
		.index = DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, tile_index),                   \
		.okay = DT_NODE_HAS_STATUS(DT_PHANDLE_BY_IDX(node_id, prop, idx), okay),           \
		.mem_filter_base = DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, mem_filter_base),    \
		.cce_filter_base = DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, cce_filter_base),    \
		.cce_cfg_filter_base =                                                             \
			DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, cce_cfg_filter_base),           \
	},

static const struct memc_tt_mimir_tile memc_tiles[] = {
	DT_INST_FOREACH_PROP_ELEM(0, mem_tiles, MEMC_TILE_ENTRY)};

/* Reset domains each GDDR tile needs released, indexed by tile-index */
static const uint32_t memc_tile_reset_mask[] = {
	MIMIR_RST_GDDR0 | MIMIR_RST_CCE0 | MIMIR_RST_CCE0_DEBUG | MIMIR_RST_CCE0_TL1,
	MIMIR_RST_GDDR1 | MIMIR_RST_CCE1 | MIMIR_RST_CCE1_DEBUG | MIMIR_RST_CCE1_TL1,
};

#define RESET_UNIT_SS_RESET_COMPLETE_ADDR 0xC0002060u

/*
 * Bounded poll budget for reset-release completion; on non-convergence fall
 * back to a fixed NOP settle so bring-up still paces (matches the reference SMC
 * bring-up).
 */
#define RESET_COMPLETE_POLL_MAX 500u
#define RESET_SETTLE_SPIN_ITERS 0x40u

#define SMC_OUTBOUND_FILTER_CTRL_0_CONFIG 0xC0016000u
#define SMC_OUTBOUND_FILTER_CTRL_1_CONFIG 0xC0016020u
#define SMC_OUTBOUND_FILTER_CTRL_2_CONFIG 0xC0016040u
#define SMC_OUTBOUND_FILTER_CTRL_3_CONFIG 0xC0016060u
#define FILTER_CONFIG_OPEN                0x01000103u

#define FW_SLOT15_OFFSET 0x1E0u

/* FILTER_CONFIG (smc_cpu_reg.h FILTER_CTRL_FILTER_CONFIG):
 *   read_en[0] write_en[1] addr_mode[4] allow_ns[8] data_bus_width[14:12]
 *   src_id[19:16] group_id[23:20] allow_burst[24]. Reference sets dbw=7,
 *   burst=1, addr_mode=1, r/w=1, src_id/group_id=0.
 */
#define FW_CFG_BASE     (0x1u | 0x2u | 0x10u | 0x7000u | 0x1000000u)
#define FW_CFG_ALLOW_NS (FW_CFG_BASE | 0x100u) /* allow_ns=1 -> 0x01007113 */
#define FW_CFG_SECURE   (FW_CFG_BASE)          /* allow_ns=0 -> 0x01007013 */

/* END_ADDR/START_ADDR are 56-bit fields; full-open end. */
#define FW_ADDR_FULL_OPEN 0x00FFFFFFFFFFFFFFULL

struct memc_tt_mimir_data {
	gddr_config_t cfg;
	struct memc_tt_mimir_addr_cfg addr_cfg;
};

int memc_tt_mimir_set_config(const struct device *dev, const gddr_backend_t *backend,
			     fw_params_t *params, const struct memc_tt_mimir_addr_cfg *addr_cfg)
{
	if (dev == NULL || backend == NULL || backend->get_blob == NULL || params == NULL ||
	    addr_cfg == NULL) {
		return -EINVAL;
	}

	struct memc_tt_mimir_data *data = dev->data;

	params->chiplet.mimir_smc.gddr.gddr_tile_mask = MEMC_GDDR_TILE_MASK;

	data->cfg.params = params;
	data->cfg.backend = backend;
	/* irq_register/irq_ctx left zero: RAS IRQ unarmed, no consumer yet. */
	data->addr_cfg = *addr_cfg;

	return 0;
}

/* Devicetree string enum -> GDDR library constant, e.g. "avery-sim" -> GDDR_VENDOR_AVERY_SIM */
#define MEMC_DT_ENUM(prefix, prop) UTIL_CAT(prefix, DT_INST_STRING_UPPER_TOKEN(0, prop))

#define MEMC_NUM_FSP ARRAY_SIZE(((gddr_fw_params_t *)0)->gddr_fsp)

BUILD_ASSERT(DT_INST_CHILD_NUM(0) >= 1 && DT_INST_CHILD_NUM(0) <= MEMC_NUM_FSP,
	     "memc node must have between 1 and the library's max FSP child nodes");

/* fw_params built from devicetree when the platform does not install its own */
static fw_params_t memc_dt_params;

static grendel_err_t memc_dt_get_blob(void *ctx, gddr_blob_id_e id, void *dst, size_t dst_cap,
				      size_t *out_len)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(dst);
	ARG_UNUSED(dst_cap);

	if (out_len != NULL) {
		*out_len = 0;
	}

	LOG_ERR("get_blob(id=%d) requested but no blob source is wired (EMU expects none)",
		(int)id);
	return GRENDEL_ERR_GDDR7_NOT_SUPPORTED;
}

/* Default backend: no blob source (EMU expects no blobs). */
static const gddr_backend_t memc_dt_backend = {
	.get_blob = memc_dt_get_blob,
	.load_phy_fw = NULL,
	.load_phy_regconfig = NULL,
	.phy_prog_done = NULL,
	.ctx = NULL,
};

/* Apply one FSP child node; FSP order is the child order */
#define MEMC_FSP_INIT(node_id)                                                                     \
	{                                                                                          \
		gddr_fsp_params_t *fsp = &g->gddr_fsp[DT_NODE_CHILD_IDX(node_id)];                 \
                                                                                                   \
		fsp->gddr_freq = DT_PROP(node_id, freq);                                           \
		IF_ENABLED(DT_PROP(node_id, phy_train_en),                                         \
			   (fsp->gddr_phy_train_en = GDDR_ENABLE;))                                \
	}

/* Build fw_params, backend and address map from the memc devicetree node */
static void memc_build_dt_config(struct memc_tt_mimir_data *data)
{
	fw_params_t *fp = &memc_dt_params;
	gddr_fw_params_t *g = &fp->chiplet.mimir_smc.gddr;

	memset(fp, 0, sizeof(*fp));
	fp->magic = FW_PARAMS_MAGIC;
	fp->version = FW_PARAMS_VERSION;
	fp->platform = FW_PARAMS_PLATFORM_MIMIR_SMC;
	fp->chiplet_id = 0;

	/* Every FSP field not set from devicetree is left to the library default */
	for (size_t i = 0; i < ARRAY_SIZE(g->gddr_fsp); i++) {
		gddr_fsp_params_t *fsp = &g->gddr_fsp[i];

		fsp->gddr_ca_lvl_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ca_vref_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_err_lvl_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_rd_lvl_mode = GDDR_LVL_MODE_INHERIT;
		fsp->gddr_rd_coarse_vref_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_rd_fine_vref_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_rd_dfe_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_rd_ctle_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_wr_lvl_mode = GDDR_LVL_MODE_INHERIT;
		fsp->gddr_wr_vref_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_wr_dfe_train = GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_rl = (uint32_t)GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_wl = (uint32_t)GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_crcrl = (uint32_t)GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_crcwl = (uint32_t)GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_capar_err_latency = (uint32_t)GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_rck_mode = (uint32_t)GDDR_OVERRIDE_INHERIT;
		fsp->gddr_ovr_rddata_en_ctrl_delta = (uint32_t)GDDR_OVERRIDE_INHERIT;
	}

	g->gddr_ovr_crc_en = GDDR_OVERRIDE_INHERIT;
	g->gddr_ovr_cabi_en = GDDR_OVERRIDE_INHERIT;
	g->gddr_ovr_capar_en = GDDR_OVERRIDE_INHERIT;
	g->gddr_ovr_bg_interleave = GDDR_OVERRIDE_INHERIT;
	g->gddr_ovr_reorder_en = GDDR_OVERRIDE_INHERIT;
	g->gddr_ovr_clamshell = GDDR_OVERRIDE_INHERIT;
	g->gddr_ovr_phy_wrlat_ctrl_delta = (uint32_t)GDDR_OVERRIDE_INHERIT;

	fp->exec_mode = MEMC_DT_ENUM(FW_PARAMS_EXEC_MODE_, exec_mode);
	g->gddr_phy_type = MEMC_DT_ENUM(GDDR_PHY_TYPE_, phy_type);
	fp->chiplet.mimir_smc.common.channel_capacity =
		MEMC_DT_ENUM(GDDR_CHANNEL_DENSITY_, channel_density);
	g->gddr_vendor = MEMC_DT_ENUM(GDDR_VENDOR_, vendor);
	g->gddr_ctrl_config_method = MEMC_DT_ENUM(GDDR_CTRL_CONFIG_METHOD_, ctrl_config_method);

	g->gddr_num_fsp_en = DT_INST_CHILD_NUM(0);
	DT_INST_FOREACH_CHILD(0, MEMC_FSP_INIT)

	g->gddr_tile_mask = MEMC_GDDR_TILE_MASK;

	data->cfg.params = fp;
	data->cfg.backend = &memc_dt_backend;
	/* irq_register/irq_ctx left zero: RAS IRQ unarmed, no consumer yet. */

	data->addr_cfg = (struct memc_tt_mimir_addr_cfg){
		.mem_start = (uint64_t)DT_INST_PROP(0, mem_start_gb) << 30,
		.mem_end = ((uint64_t)DT_INST_PROP(0, mem_end_gb) << 30) - 1ULL,
		.interleave_mode = DT_INST_PROP(0, interleave_mode),
		.capacity_gb = DT_INST_PROP(0, capacity_gb),
	};
}

/* Wait until the SMC reset unit reports the released domains as complete */
static void wait_reset_release_complete(uint32_t mask)
{
	for (uint32_t i = 0; i < RESET_COMPLETE_POLL_MAX; i++) {
		if ((sys_read32(RESET_UNIT_SS_RESET_COMPLETE_ADDR) & mask) == mask) {
			return;
		}
	}

	LOG_WRN("reset-complete did not converge (mask=0x%08x status=0x%08x); "
		"using fixed settle",
		mask, sys_read32(RESET_UNIT_SS_RESET_COMPLETE_ADDR));

	for (volatile uint32_t i = 0; i < RESET_SETTLE_SPIN_ITERS; i++) {
		__asm__ volatile("nop");
	}
}

/*
 * Open the SMC outbound filter so SoC-space (ITN, GDDR ctrl/PHY) is reachable.
 * Config-only open on CTRL_0..3 (reference setup_smc_outbound_filter()).
 */
static void memc_open_outbound_filter(void)
{
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_0_CONFIG);
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_1_CONFIG);
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_2_CONFIG);
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_3_CONFIG);
}

/* Release cold+warm reset for the GDDR and CCE domains of each enabled tile */
static int memc_release_reset(void)
{
	uint32_t mask = 0;

	ARRAY_FOR_EACH_PTR(memc_tiles, tile) {
		if (!tile->okay) {
			continue;
		}
		if (tile->index >= ARRAY_SIZE(memc_tile_reset_mask)) {
			LOG_ERR("GDDR tile-index %u out of range", tile->index);
			return -EINVAL;
		}
		mask |= memc_tile_reset_mask[tile->index];
	}

	smc_release_reset(mask);
	wait_reset_release_complete(mask);

	return 0;
}

/* Program one firewall filter slot: FILTER_CONFIG, START_ADDR, END_ADDR. */
static inline void fw_prog_slot(uint32_t cfg_addr, uint64_t cfg, uint64_t start, uint64_t end)
{
	sys_write64(cfg, cfg_addr);
	sys_write64(start, cfg_addr + 0x8u);
	sys_write64(end, cfg_addr + 0x10u);
}

/*
 * Program a tile's FIREWALL_0 (allow_ns) and FIREWALL_15 (secure) with a
 * window, mirroring firmware disable_firewall_filters().
 */
static inline void fw_prog_tile(uint32_t base, uint64_t start, uint64_t end)
{
	fw_prog_slot(base, FW_CFG_ALLOW_NS, start, end);
	fw_prog_slot(base + FW_SLOT15_OFFSET, FW_CFG_SECURE, start, end);
}

/*
 * Open each enabled tile's ITN SMN2ITN firewalls: MEM tile to the GDDR window,
 * CCE and CCE_CFG full-open.
 */
static void memc_open_tile_firewalls(const struct memc_tt_mimir_addr_cfg *addr_cfg)
{
	ARRAY_FOR_EACH_PTR(memc_tiles, tile) {
		if (!tile->okay) {
			continue;
		}
		fw_prog_tile(tile->mem_filter_base, addr_cfg->mem_start, addr_cfg->mem_end);
		fw_prog_tile(tile->cce_filter_base, 0x0ULL, FW_ADDR_FULL_OPEN);
		fw_prog_tile(tile->cce_cfg_filter_base, 0x0ULL, FW_ADDR_FULL_OPEN);
	}
}

/* Program the ITN interleave mode and GDDR capacity */
static int memc_cfg_address_map(const struct memc_tt_mimir_addr_cfg *addr_cfg)
{
	grendel_err_t err =
		noc_init_interleave_cfg((noc_interleave_mode_t)addr_cfg->interleave_mode);

	if (err != GRENDEL_ERR_OK) {
		LOG_ERR("ITN interleave mode %u rejected: %d", addr_cfg->interleave_mode, (int)err);
		return -EINVAL;
	}

	itn_gddr7_memory_address_realignment_cfg((noc_gddr7_realign_t)addr_cfg->capacity_gb);

	return 0;
}

static int memc_tt_mimir_init(const struct device *dev)
{
	struct memc_tt_mimir_data *data = dev->data;
	grendel_err_t err;
	int ret;

	/* No platform override installed via memc_tt_mimir_set_config(): use devicetree */
	if (data->cfg.params == NULL) {
		memc_build_dt_config(data);
	}

	if (data->cfg.params == NULL || data->cfg.params->magic != FW_PARAMS_MAGIC ||
	    data->cfg.backend == NULL || data->cfg.backend->get_blob == NULL) {
		LOG_ERR("GDDR config not installed before memc init");
		return -EINVAL;
	}

	memc_open_outbound_filter();

	ret = memc_release_reset();
	if (ret != 0) {
		return ret;
	}

	memc_open_tile_firewalls(&data->addr_cfg);

	ret = memc_cfg_address_map(&data->addr_cfg);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("ITN firewall win[0x%llx..0x%llx], interleave=%u capacity_gb=%u",
		(unsigned long long)data->addr_cfg.mem_start,
		(unsigned long long)data->addr_cfg.mem_end, data->addr_cfg.interleave_mode,
		data->addr_cfg.capacity_gb);

	err = gddr_init(&data->cfg);
	if (err != GRENDEL_ERR_OK) {
		LOG_ERR("gddr_init failed: %s", grendel_err_to_str(err));
		return -EIO;
	}

	LOG_INF("GDDR bring-up complete (tile_mask=0x%x)", (unsigned int)gddr_get_tile_en());
	return 0;
}

static struct memc_tt_mimir_data memc_tt_mimir_data0;

DEVICE_DT_INST_DEFINE(0, memc_tt_mimir_init, NULL, &memc_tt_mimir_data0, NULL, POST_KERNEL,
		      CONFIG_MEMC_TT_MIMIR_INIT_PRIORITY, NULL);
