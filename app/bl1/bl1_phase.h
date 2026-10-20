/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 * SPDX-License-Identifier: Apache-2.0
 *
 * BL1 boot checkpoints. Each checkpoint overwrites SCRATCH_01 with its phase,
 * so after a hang or failure the last value left there tells the testbench how
 * far BL1 got. A failed step is reported as its phase with BL1_PHASE_FAILED
 * set. On success BL1 finally writes the 0xACAFACA1 pass marker to SCRATCH_00
 * (test_pass()); SCRATCH_00 is cleared before the testbench dump, so the
 * checkpoints are kept out of it.
 *
 * Phases, in execution order (0x01-0x04 are Mimir-only and run before main()):
 *   0x01  chip init entered (first BL1 code to run)
 *   0x02  SMN/ITN fabric reset released
 *   0x03  about to start memc init
 *   0x04  memc init returned (0x84: memc device failed to initialize)
 *   0x05  entered main()
 *   0x06  about to launch MIS
 */

#ifndef BL1_PHASE_H
#define BL1_PHASE_H

#include <soc.h>

#define BL1_PHASE_FAILED 0x80U

#define BL1_PHASE(phase) WRITE_SCRATCH(1, (phase))

#endif /* BL1_PHASE_H */
