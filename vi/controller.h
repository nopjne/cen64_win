//
// vi/controller.h: Video interface controller.
//
// CEN64: Cycle-Accurate Nintendo 64 Simulator.
// Copyright (C) 2014, Tyler J. Stachecki.
//
// This file is subject to the terms and conditions defined in
// 'LICENSE', which is part of this source code package.
//

#ifndef __vi_controller_h__
#define __vi_controller_h__
#include "common.h"

struct bus_controller *bus;

enum vi_register {
#define X(reg) reg,
#include "vi/registers.md"
#undef X
  NUM_VI_REGISTERS
};

struct vi_controller {
  struct bus_controller *bus;
  uint32_t regs[NUM_VI_REGISTERS];
};

int read_vi_regs(void *opaque, uint32_t address, uint32_t *word);
int write_vi_regs(void *opaque, uint32_t address, uint32_t *word);
int vi_init(struct vi_controller *vi, struct bus_controller *bus);

#endif

