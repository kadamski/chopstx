/*
 * entry.c - Entry routine when reset and interrupt vectors.
 *
 * Copyright (C) 2019
 *               Flying Stone Technology
 * Author: NIIBE Yutaka <gniibe@fsij.org>
 *
 * This file is a part of Chopstx, a thread library for embedded.
 *
 * Chopstx is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Chopstx is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <chopstx.h>

#include "board.h"

#include "mcu/clk_gpio_init-gd32vf103.c"

#define C_S_SUB(arg0, arg1, arg2) arg0 #arg1 arg2
#define COMPOSE_STATEMENT(arg0,arg1,arg2)  C_S_SUB (arg0, arg1, arg2)
/*
 *
 */
STATIC_ENTRY __attribute__ ((naked,section(".text.startup.0")))
void
entry (void)
{
  /* Start at 0x00000000 (alias 0x08000000), interrupt masked */
  asm volatile (
	"lui	a0,0x08000\n\t"
	"addi	a0,a0,%pcrel_lo(0f)\n\t"
	"jr	a0\n"		/* Jump to physical address */
    "0:\n"
    ".option push\n"
    ".option norelax\n\t"
	"la	gp,__global_pointer$\n"
    ".option pop\n\t"
	"la	sp,__main_stack_end__\n\t"
	"jal	clock_init\n\t"
	/* Clear BSS section.  */
	"la	a0,_bss_start\n\t"
	"la	a1,_bss_end\n\t"
    "0:\n\t"
	"beq	a0,a1,1f\n"
	"sw	zero,(a0)\n\t"
	"addi	a0,a0,4\n\t"
	"j	0b\n"
    "1:\n\t"
	/* Copy data section.  */
	"la	a0,_textdata\n\t"
	"la	a1,_data\n\t"
	"la	a2,_edata\n"
    "0:\n\t"
	"beq	a1,a2,1f\n\t"
	"lw	t0,(a0)\n\t"
	"sw	t0,(a1)\n\t"
	"addi	a0,a0,4\n\t"
	"addi	a1,a1,4\n\t"
	"j	0b\n"
    "1:\n\t"
	/* Switch to application stack.  */
	"la	sp,__process0_stack_end__\n\t"
	COMPOSE_STATEMENT ("subi	sp, #", CHOPSTX_THREAD_SIZE, "\n\t")
	"jal	chx_init\n\t"
	"jal	chx_systick_init\n\t"
	"jal	gpio_init\n\t"
	/* TBD: msubm??? */
	/* Enable interrupts.  */
	"csrsi	mstatus,8\n\t"
	/* Call main.  */
	"call	main\n\t"
    "4:\n\t"
	"j	4b"
	: /* no output */ : /* no input */ : "memory");
}
