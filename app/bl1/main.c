/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include "bl1_phase.h"
#include "mis_launch.h"

LOG_MODULE_REGISTER(main, CONFIG_TT_APP_LOG_LEVEL);

int main(void)
{
	BL1_PHASE(0x05); /* entered main */
	LOG_INF("BL1 - " CONFIG_BOARD_TARGET);

	if (IS_ENABLED(CONFIG_BL1_TB_PASS_BEFORE_MIS)) {
		LOG_INF("BL1 done, signalling testbench pass");
		test_pass(); /* SCRATCH_00 = 0xACAFACA1, releases the TB */
		for (;;) {
		}
	}

	BL1_PHASE(0x06); /* about to launch MIS */
	launch_mis();

	/* Unreachable unless MIS returns, which means the handoff failed and the core's
	 * state (stack, vector table, PMP, ...) can no longer be trusted.
	 */
	k_panic();
	return 0;
}
