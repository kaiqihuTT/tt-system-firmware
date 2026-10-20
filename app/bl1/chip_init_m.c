/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mimir BL1 chip-wide init: release the SMN/ITN fabric from reset and open the
 * SMC outbound filter so SoC space is reachable. Runs before any driver that
 * touches SoC space (e.g. the memc GDDR bring-up). GDDR/CCE tile resets and
 * firewalls are left to the memc driver.
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

#include <basic_init.h>

LOG_MODULE_REGISTER(chip_init_m, CONFIG_LOG_DEFAULT_LEVEL);

#define RESET_UNIT_SS_RESET_COMPLETE_ADDR 0xC0002060u

/* On-chip fabric shared by every SoC-space consumer (memc, D2D, ...) */
#define FABRIC_RESET_N_MASK ((uint32_t)(MIMIR_RST_SMN | MIMIR_RST_ITN))

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

static int mimir_chip_init(void)
{
	/* 1) Release cold+warm reset for the SMN/ITN fabric. */
	smc_release_reset(FABRIC_RESET_N_MASK);
	wait_reset_release_complete(FABRIC_RESET_N_MASK);

	/*
	 * 2) Open the SMC outbound filter so SoC-space (ITN, GDDR ctrl/PHY) is
	 * reachable. Config-only open on CTRL_0..3 (reference
	 * setup_smc_outbound_filter()).
	 */
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_0_CONFIG);
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_1_CONFIG);
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_2_CONFIG);
	sys_write64(FILTER_CONFIG_OPEN, SMC_OUTBOUND_FILTER_CTRL_3_CONFIG);

	LOG_INF("Mimir chip init: SMN/ITN reset released, outbound filter open");

	return 0;
}

SYS_INIT(mimir_chip_init, POST_KERNEL, 0);
