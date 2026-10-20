/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 *
 * DV stand-in providing the GDDR configuration the memc driver consumes. It
 * builds the fw_params table, selects the backend, and installs fw_params +
 * backend into the memc device via memc_tt_mimir_set_config() before it inits.
 */

#include <string.h>

#include <zephyr/drivers/memc/memc_tt_mimir.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

LOG_MODULE_REGISTER(mimir_gddr_config, CONFIG_LOG_DEFAULT_LEVEL);

#define MIMIR_GDDR_CONFIG_INIT_PRIORITY 80
BUILD_ASSERT(MIMIR_GDDR_CONFIG_INIT_PRIORITY < CONFIG_MEMC_TT_MIMIR_INIT_PRIORITY,
	     "GDDR config must initialize before the memc controller device");

/* FW-handshake scratch regs (written by the DV harness) */
#define SMC_CTRL_SCRATCH_07_ADDR 0xC0010138u /* firewall window    */
#define SMC_CTRL_SCRATCH_14_ADDR 0xC0010170u /* memory_capacity_gb */
#define SMC_CTRL_SCRATCH_15_ADDR 0xC0010178u /* interleave_mode    */

#define SCRATCH07_FW_START_GB(v)        ((uint64_t)((v) & 0x7Fu))        /* [6:0]   */
#define SCRATCH07_FW_END_GB(v)          ((uint64_t)(((v) >> 7) & 0x7Fu)) /* [13:7]  */
#define SCRATCH15_INTERLEAVE_MODE(v)    (((v) >> 2) & 0x7u)              /* [4:2]   */
#define SCRATCH14_MEMORY_CAPACITY_GB(v) (((v) >> 13) & 0x1Fu)            /* [17:13] */

static fw_params_t mimir_fw_params_storage;

static grendel_err_t stub_get_blob(void *ctx, gddr_blob_id_e id, void *dst, size_t dst_cap,
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

/* EMU backend: no blob source (EMU expects no blobs). */
static const gddr_backend_t mimir_gddr_backend = {
	.get_blob = stub_get_blob,
	.load_phy_fw = NULL,
	.load_phy_regconfig = NULL,
	.phy_prog_done = NULL,
	.ctx = NULL,
};

static int mimir_gddr_configure(void)
{
	fw_params_t *fp = &mimir_fw_params_storage;
	gddr_fw_params_t *g = &fp->chiplet.mimir_smc.gddr;
	const gddr_backend_t *backend;

	memset(fp, 0, sizeof(*fp));
	fp->magic = FW_PARAMS_MAGIC;
	fp->version = FW_PARAMS_VERSION;
	fp->platform = FW_PARAMS_PLATFORM_MIMIR_SMC;
	fp->chiplet_id = 0;

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

#if defined(CONFIG_MEMC_TT_MIMIR_ENV_SIM_BEH)
	fp->exec_mode = FW_PARAMS_EXEC_MODE_SIM;
	g->gddr_phy_type = GDDR_PHY_TYPE_BEH;
#else
	fp->exec_mode = FW_PARAMS_EXEC_MODE_EMU;
	g->gddr_phy_type = GDDR_PHY_TYPE_BEH;
#endif
	fp->chiplet.mimir_smc.common.channel_capacity = GDDR_CHANNEL_DENSITY_1_GB;
	g->gddr_vendor = GDDR_VENDOR_AVERY_SIM;
	g->gddr_num_fsp_en = 1;
	g->gddr_fsp[0].gddr_freq = 32;
	g->gddr_fsp[0].gddr_phy_train_en = GDDR_ENABLE;
	g->gddr_ctrl_config_method = GDDR_CTRL_CONFIG_METHOD_COMPILED;

	backend = &mimir_gddr_backend;

	uint32_t s07 = sys_read32(SMC_CTRL_SCRATCH_07_ADDR);
	uint32_t s14 = sys_read32(SMC_CTRL_SCRATCH_14_ADDR);
	uint32_t s15 = sys_read32(SMC_CTRL_SCRATCH_15_ADDR);
	const struct memc_tt_mimir_addr_cfg addr_cfg = {
		.mem_start = SCRATCH07_FW_START_GB(s07) << 30,
		.mem_end = (SCRATCH07_FW_END_GB(s07) << 30) - 1ULL,
		.interleave_mode = SCRATCH15_INTERLEAVE_MODE(s15),
		.capacity_gb = SCRATCH14_MEMORY_CAPACITY_GB(s14),
	};

	return memc_tt_mimir_set_config(DEVICE_DT_GET(DT_NODELABEL(memc)), backend, fp,
					&addr_cfg);
}

SYS_INIT(mimir_gddr_configure, POST_KERNEL, MIMIR_GDDR_CONFIG_INIT_PRIORITY);
