/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mimir BL1 chip-wide init: release the SMN/ITN fabric from reset. Runs before
 * any driver that touches SoC space (e.g. the memc GDDR bring-up). The SMC
 * outbound filter, GDDR/CCE tile resets and firewalls are left to the memc
 * driver. Also brackets the memc init with boot checkpoints (bl1_phase.h).
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

#include <basic_init.h>

#include "bl1_phase.h"

LOG_MODULE_REGISTER(chip_init_m, CONFIG_LOG_DEFAULT_LEVEL);

/* Checkpoint hooks run just before and just after the memc device init */
BUILD_ASSERT(CONFIG_MEMC_TT_MIMIR_INIT_PRIORITY > 0 && CONFIG_MEMC_TT_MIMIR_INIT_PRIORITY < 99,
	     "memc checkpoint hooks need a free priority on each side of memc init");
#define MEMC_PRE_PRIORITY  UTIL_DEC(CONFIG_MEMC_TT_MIMIR_INIT_PRIORITY)
#define MEMC_POST_PRIORITY UTIL_INC(CONFIG_MEMC_TT_MIMIR_INIT_PRIORITY)

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
	BL1_PHASE(0x01); /* chip init entered */

	/* Release cold+warm reset for the SMN/ITN fabric. */
	smc_release_reset(FABRIC_RESET_N_MASK);
	wait_reset_release_complete(FABRIC_RESET_N_MASK);

	LOG_INF("Mimir chip init: SMN/ITN reset released");
	BL1_PHASE(0x02); /* SMN/ITN reset released */

	return 0;
}

SYS_INIT(mimir_chip_init, POST_KERNEL, 0);

static int memc_pre_init(void)
{
	BL1_PHASE(0x03); /* about to start memc init */

	return 0;
}

SYS_INIT(memc_pre_init, POST_KERNEL, MEMC_PRE_PRIORITY);

static int memc_post_init(void)
{
	if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(memc)))) {
		LOG_ERR("memc init failed");
		BL1_PHASE(BL1_PHASE_FAILED | 0x04); /* memc init failed */

		if (IS_ENABLED(CONFIG_BL1_TB_PASS_BEFORE_MIS)) {
			/* Halt here so the testbench sees the failure, not a pass */
			for (;;) {
			}
		}
		return 0;
	}

	BL1_PHASE(0x04); /* memc init returned */

	return 0;
}

SYS_INIT(memc_post_init, POST_KERNEL, MEMC_POST_PRIORITY);
