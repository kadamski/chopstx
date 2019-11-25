/*
 * chopstx-riscv32.c - Threads and only threads: Arch specific code
 *                     for RISC-V 32 IMAC (Bumblebee core)
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

/*
 * Define Bumblebee specific CSRs
 */
asm (
	".equ	msubm,0x7c4\n\t" /* No use (not needed to care about) */
	".equ	mtvt,0x307\n\t"
	".equ	mtvt2,0x7ec\n\t"
);

/* Data Memory Barrier.  */
static void
chx_dmb (void)
{
  asm volatile ("fence" : : : "memory");
}

/* No saved registers on the stack.  */

/*
 * Constants for RISC-V.
 */
#define REG_PC    0
#define REG_RA    1
#define REG_SP    2
#define REG_GP    3
#define REG_TP    4
#define REG_A0   10
#define MACHINE_STATUS_INIT 0x00001880 /* Machine-mode, interrupt enabled.  */

/*
 * We keep the TP register to the thread context address.
 * Also, it is kept at the MSCRATCH register.
 */
static struct chx_thread *
chx_running (void)
{
  uint32_t r;

  asm ( "mv	%0,tp"       : "=r" (r));
  if (r == 0)
    return NULL;
  return (struct chx_thread *)(r - 20);
}

static void
chx_set_running (struct chx_thread *tp)
{
  asm (	"addi	tp,%0,20\n\t"
	"csrw	mscratch,tp" : /* no output */ : "r" (tp) : "memory");
}


/*
 * Lower layer architecture specific functions.
 *
 * system tick and interrupt
 */

/*
 * System tick
 *
 * SYSTICK timer model is decreasing counter, where counter value 0
 * means the timer stop.
 *
 * Since RISC-V MTIME is ever increasing counter, we need a bit of
 * glue code here.  Fortunately, we have MSTOP register for Bumblebee
 * core, so, we can use the register to stop the counter.  (If no such
 * register stopping MTIME, we could put possible largest value in
 * MTIMECMP, so that the timer won't be fired.)
 */
/* TIMER registers.  */
struct TIMER {
  volatile uint32_t mtime_lo;
  volatile uint32_t mtime_hi;
  volatile uint32_t mtimecmp_lo;
  volatile uint32_t mtimecmp_hi;
  uint32_t rsv0[1018];
  volatile uint8_t mstop;
  uint8_t rsv1[3];
  volatile uint8_t msip;
  uint8_t rsv2[3];
} __attribute__ ((packed));
static struct TIMER *const TIMER = (struct TIMER *)0xD1000000;

/* In Chopstx, we only use the lower 32-bit of the timer.  */

static void
chx_systick_init_arch (void)
{
  TIMER->mstop |= 1;
  TIMER->mtime_hi = TIMER->mtime_lo = 0;
  TIMER->mtimecmp_lo = 0xffffffff;
  TIMER->mtimecmp_hi = 0;
}

/*
 * @ticks: 0 means to stop the timer, otherwise update the timer
 */
static void
chx_systick_reload (uint32_t ticks)
{
  if (!ticks)
    {
      TIMER->mstop |= 1;
      TIMER->mtimecmp_lo = 0xffffffff;
    }
  else
    {
      TIMER->mtime_lo = 0;
      TIMER->mtimecmp_lo = ticks;
      if ((TIMER->mstop & 1))
	TIMER->mstop &= ~1;
    }
}

static uint32_t
chx_systick_get (void)
{
  int stopped = (TIMER->mstop & 1);

  if (stopped)
    return 0;
  else
    {
      uint32_t now = TIMER->mtime_lo;

      if (TIMER->mtimecmp_lo <= now)
	return 0;

      return TIMER->mtimecmp_lo - now;
    }
}

/* TIMER runs at 1/4 of core clock.  */

static uint32_t
usec_to_ticks (uint32_t usec)
{
  return usec * (MHZ/4);
}

static uint32_t
ticks_to_usec (uint32_t ticks)
{
  return ticks / (MHZ/4);
}

/*
 * Interrupt Handling of (E)CLIC
 */

/* CLIC registers.  */
struct CLIC {
  volatile uint8_t cfg;
  uint8_t rsv[3];
  volatile uint32_t info;
  volatile uint8_t uth;
  volatile uint8_t sth;
  volatile uint8_t hth;
  volatile uint8_t mth;
} __attribute__ ((packed));

static struct CLIC *const CLIC = (struct CLIC *)0xD2000000;

struct CLIC_INT {
  volatile uint8_t ip;
  volatile uint8_t ie;
  volatile uint8_t attr;
  volatile uint8_t ctl;
} __attribute__ ((packed));
static struct CLIC_INT *const CLIC_INT = (struct CLIC_INT *)0xD2001000;

static void
chx_enable_intr (uint8_t irq_num)
{
  CLIC_INT[irq_num].ie |= 1;
}

static void
chx_clr_intr (uint8_t irq_num)
{
  /* Clear pending interrupt is done automatically by the hardware.  */
  (void)irq_num;
}

static int
chx_disable_intr (uint8_t irq_num)
{
  int already_disabled = !(CLIC_INT[irq_num].ie & 1);

  CLIC_INT[irq_num].ie &= ~1;
  return already_disabled;
}

static void
chx_set_intr_prio (uint8_t irq_num)
{
  CLIC_INT[irq_num].attr = 0xc0; /* Level triggered, SHV=0 */
  /* Note: SHV: Selective Hardware Vectoring: off (use common routine).  */

  CLIC_INT[irq_num].ctl = 0xff;
}

#define SWINT_IRQ   3
#define TIMER_IRQ   7
#define MEMERR_IRQ 17           /* Why on earth it's IRQ??? */

static void chx_handle_intr (void);
static void exception_handler (void);

/* Alignment to 64 is needed to enable ECLIC mode.  */
static void __attribute__ ((naked,aligned(64)))
exception_handler (void)
{
  asm volatile (
"0:	j	0b"
	: /* no output */);
}

static void __attribute__ ((naked))
memory_error (void)
{
  asm volatile (
"0:	j	0b"
	: /* no output */);
}

typedef void (*handler)(void);

/* Not used (because we always disable SHV for all interrupts),
 * Just in case, if some interrupt uses SHV=1.
 */
static const handler vector_table[] __attribute__ ((aligned(512))) = {
 0, 0, 0, chx_handle_intr,
 0, 0, 0, chx_handle_intr,
 0, 0, 0, 0,
 0, 0, 0, 0,
 0, memory_error, 0, 0,
};

static void
chx_interrupt_controller_init (void)
{
  asm volatile (
	"csrw	mtvt,%0"
	: /* no output */ : "r" (vector_table) : "memory" );

  asm volatile (
	"csrw	mtvt2,%0\n\t"
	"csrsi	mtvt2,1"
	: /* no output */ : "r" (chx_handle_intr) : "memory");

  asm volatile (
	"csrw	mtvec,%0\n\t"
	"csrsi	mtvec,3"        /* Enable ECLC mode */
	: /* no output */ : "r" (exception_handler) : "memory" );

  CLIC->cfg &= 0xe1;            /* NLBITS = 0 */
  CLIC->mth = 0;

  /* In Bumblebee core, timer interrupt is also handled by CLIC.  */
  chx_set_intr_prio (SWINT_IRQ);
  chx_enable_intr (SWINT_IRQ);
  chx_set_intr_prio (TIMER_IRQ);
  chx_enable_intr (TIMER_IRQ);
  chx_set_intr_prio (MEMERR_IRQ);
  chx_enable_intr (MEMERR_IRQ);
}

/* Just for testing TIMER and ECLIC.  No real use for now.  */
static void
chx_sw_int (int go)
{
  if (go)
    TIMER->msip |= 1;
  else
    TIMER->msip &= ~1;
}

static void
chx_cpu_sched_lock (void)
{
  asm volatile ("csrci	mstatus,8" : : : "memory" );
}

static void
chx_cpu_sched_unlock (void)
{
  asm volatile ("csrsi	mstatus,8" : : : "memory" );
}

static void
chx_init_arch (struct chx_thread *tp)
{
  memset (&tp->tc, 0, sizeof (tp->tc));
  chx_set_running (tp);
}

#define SAVE_CALLEE_SAVE_REGISTERS            \
	"# Save callee save registers\n\t"    \
	"sw	s0,32(sp)\n\t"                \
	"sw	s1,36(sp)\n\t"                \
	"sw	s2,72(sp)\n\t"                \
	"sw	s3,76(sp)\n\t"                \
	"sw	s4,80(sp)\n\t"                \
	"sw	s5,84(sp)\n\t"                \
	"sw	s6,88(sp)\n\t"                \
	"sw	s7,92(sp)\n\t"                \
	"sw	s8,96(sp)\n\t"                \
	"sw	s9,100(sp)\n\t"               \
	"sw	s10,104(sp)\n\t"              \
	"sw	s11,108(sp)\n\t"

#define RESTORE_CALLEE_SAVE_REGISTERS         \
	"# Restore callee save registers\n\t" \
	"lw	s0,32(sp)\n\t"                \
	"lw	s1,36(sp)\n\t"                \
	"lw	s2,72(sp)\n\t"                \
	"lw	s3,76(sp)\n\t"                \
	"lw	s4,80(sp)\n\t"                \
	"lw	s5,84(sp)\n\t"                \
	"lw	s6,88(sp)\n\t"                \
	"lw	s7,92(sp)\n\t"                \
	"lw	s8,96(sp)\n\t"                \
	"lw	s9,100(sp)\n\t"               \
	"lw	s10,104(sp)\n\t"              \
	"lw	s11,108(sp)\n\t"

#define SAVE_OTHER_REGISTERS            \
	"# Save other registers\n\t"    \
	"sw	ra,4(sp)\n\t"           \
	"sw	gp,12(sp)\n\t"          \
	"sw	t0,20(sp)\n\t"          \
	"sw	t1,24(sp)\n\t"          \
	"sw	t2,28(sp)\n\t"          \
	"sw	a0,40(sp)\n\t"          \
	"sw	a1,44(sp)\n\t"          \
	"sw	a2,48(sp)\n\t"          \
	"sw	a3,52(sp)\n\t"          \
	"sw	a4,56(sp)\n\t"          \
	"sw	a5,60(sp)\n\t"          \
	"sw	a6,64(sp)\n\t"          \
	"sw	a7,68(sp)\n\t"          \
	"sw	t3,112(sp)\n\t"         \
	"sw	t4,116(sp)\n\t"         \
	"sw	t5,120(sp)\n\t"         \
	"sw	t6,124(sp)\n\t"

#define RESTORE_OTHER_REGISTERS         \
	"# Restore other registers\n\t" \
	"lw	ra,4(sp)\n\t"           \
	"lw	gp,12(sp)\n\t"          \
	"lw	t0,20(sp)\n\t"          \
	"lw	t1,24(sp)\n\t"          \
	"lw	t2,28(sp)\n\t"          \
	"lw	a0,40(sp)\n\t"          \
	"lw	a1,44(sp)\n\t"          \
	"lw	a2,48(sp)\n\t"          \
	"lw	a3,52(sp)\n\t"          \
	"lw	a4,56(sp)\n\t"          \
	"lw	a5,60(sp)\n\t"          \
	"lw	a6,64(sp)\n\t"          \
	"lw	a7,68(sp)\n\t"          \
	"lw	t3,112(sp)\n\t"         \
	"lw	t4,116(sp)\n\t"         \
	"lw	t5,120(sp)\n\t"         \
	"lw	t6,124(sp)\n\t"
/*
 * MSTATUS register:
 *   31:    SD
 *   16-15: XS (Extra Unit state)
 *   14-13: FS (Floating-point Unit state)
 *   12-11: MPP  (Previous Privilege)
 *   7:     MPIE (Previous Interrupt Enable flag)
 *   3:     MIE  (Interrupt Enable flag)
 */
/*
 * MSUBM register:
 *   9-8 PTYP (Previous Type-of-execution)
 *   7-6 TYP  (Currunt Type-of-execution)
 *   0: Normal, 1: Interrupt, 2: Excep, 3: NMI
 *
 * No need to store PTYP field of MSUBM in MACHINE_STATUS.
 * No need to setup MSUBM for PTYP.
 * Since it's always 0 when this is called.
 *
 * Save MPP..MPIE in MACHINE_STATUS in the thread context.
 */
#define SETUP_MSTATUS_FROM_MACHINE_STATUS                                 \
	"lw	a0,128(sp)\n\t"   /* MPP..MPIE is in MACHINE_STATUS  */   \
	"csrr	a1,mstatus\n\t"                                           \
	"srli	a1,a1,13\n\t"                                             \
	"slli	a1,a1,13\n\t"     /* SD, XS, and FS bits from MSTATUS */  \
	"or	a0,a0,a1\n\t"                                             \
	"csrw	mstatus,a0\n\t"   /* Note: keep MIE=0 */

/* It is good if ISA has a single instruction for this operation.  */
#define CATCH_AN_INTERRUPT_SYNCHRONOUSLY                                     \
	"mv	a0,zero\n"                                                   \
    "0:\n\t"                                                                 \
	/*                                                                   \
	 * Interrupt is masked here, and it executes the WFI                 \
	 * instruction.  When an interrupt occurs, the core is waken         \
	 * up (even if it is masked).                                        \
	 */                                                                  \
	"csrci	mstatus,8\n\t"  /* Interrupt should be masked already.   */  \
				/* Nevertheless, make sure it's masked.  */  \
	"wfi\n\t"                                                            \
	/*                                                                   \
	 * It is good if MCAUSE were updated here, but it is only            \
	 * updated when the control goes into the interrupt service,         \
	 * in a step of the interrupt handling steps of the core.  So,       \
	 * we let it go to chx_handle_intr, by unmasking.                    \
	 */                                                                  \
	"csrsi	mstatus,8\n\t"  /* Unmask interrupts to catch one.  */       \
	/* Just before this line, it is interrupted.  */                     \
	/* And interrupt is masked and a0 is set.     */                     \
	"beqz	a0,0b\n\t"      /* Just in case if not, loop.  */

/*
 * The idle function.
 *
 * NOTE: In this function, interrupt is masked (MIE=0) and interrupt
 * is synchronously handled.
 */
static struct chx_thread * __attribute__ ((used))
chx_idle (void)
{
  extern void chx_prepare_sleep_mode (void);
  register struct chx_thread *tp_next asm ("a0") = NULL;

  while (tp_next == NULL)
    {
      register uint32_t irq_num asm ("a0");

      chx_prepare_sleep_mode ();  /* MCU specific sleep setup */

      asm volatile (
	CATCH_AN_INTERRUPT_SYNCHRONOUSLY
	/*
	 * In chx_handle_intr, a0 is set by the value of MCAUSE.
	 */
	"slli	a0,a0,20\n\t"
	"srli	a0,a0,20"       /* Take lower 12-bit of MCAUSE */
	: "=r" (irq_num));
      /* Note: here, interrupt is masked again.  */

      if (irq_num == SWINT_IRQ)
	chx_sw_int (0);
      else if (irq_num == TIMER_IRQ)
	tp_next = chx_timer_expired ();
      else if (irq_num == MEMERR_IRQ)
	memory_error ();
      else
	tp_next = chx_recv_irq (irq_num);
    }

  return tp_next;
}


static uintptr_t
voluntary_context_switch (struct chx_thread *tp_next)
{
  register uintptr_t result asm ("a0");

  asm volatile (
	/* Here, %0 (a0) points to pointer (struct chx_thread *) to be
	 * switched.  We get the thread context pointer adding the
	 * offset.
	*/
	"# Voluntary context switch\n\t"
	"sw	sp,8(tp)\n\t"
	"mv	sp,tp\n\t"        /* Using SP, we can use C.SWSP instruction */
	"sw	zero,0(sp)\n\t"
	SAVE_CALLEE_SAVE_REGISTERS
	"# Check if going to IDLE function\n\t"
	"bnez	%0,0f\n\t"
	/*
	 * NOTE: Here, for running chx_idle, we can use
	 * __main_stack_end__, because neither any threads, nor
	 * interrupt context use that.	Using __main_stack_end__, we
	 * can minimize stack memory usage of each thread.
	 */
	"# Call the IDLE function, interrupt masked.\n\t"
	"mv	tp,zero\n\t"
	"csrw	mscratch,tp\n\t"
	"la	sp,__main_stack_end__\n\t"
	"call	chx_idle\n"
    ".L_V_CONTEXT_SWITCH_BEGIN:\n"
    "0:\n\t"
	"addi	%0,%0,20\n\t"
	"mv	sp,%0\n\t"
	RESTORE_CALLEE_SAVE_REGISTERS
	/**/
	"csrw	mscratch,sp\n\t"
	"lw	a0,0(sp)\n\t"
	"beqz	a0,1f\n"
    ".L_RETURN_TO_PREEMPTED_THREAD:\n\t"
	"csrw	mepc,a0\n\t"
	SETUP_MSTATUS_FROM_MACHINE_STATUS
	RESTORE_OTHER_REGISTERS
	"lw	tp,16(sp)\n\t"    /* Application is free to other use of TP */
	"lw	sp,8(sp)\n\t"
	"mret\n"
    "1:\n\t"
	"lw	a0,-4(sp)\n\t"  /* Get the result value */
	"mv	tp,sp\n\t"
	"lw	sp,8(sp)\n\t"
	"csrsi	mstatus,8\n"  /* Unmask interrupts.  */
    ".L_V_CONTEXT_SWITCH_FINISH:"
	: "=r" (result)
	: "0" (tp_next)
	: "ra", "t0", "t1", "t2", "t3", "t4", "t5", "t6",
	  "a1", "a2", "a3", "a4", "a5", "a6", "a7",
	  "memory");

  return result;
}

extern void cause_link_time_error_unexpected_size_of_struct_chx_thread (void);

static struct chx_thread *
chopstx_create_arch (uintptr_t stack_addr, size_t stack_size,
		     voidfunc thread_entry, void *arg)
{
  struct chx_thread *tp;
  void *stack;
  register uint32_t gp;

  asm (	"mv	%0,gp" : "=r" (gp));

  if (CHOPSTX_THREAD_SIZE != sizeof (struct chx_thread))
    cause_link_time_error_unexpected_size_of_struct_chx_thread ();

  if (stack_size < sizeof (struct chx_thread))
    chx_fatal (CHOPSTX_ERR_THREAD_CREATE);

  stack = (void *)(stack_addr + stack_size - sizeof (struct chx_thread));
  tp = (struct chx_thread *)stack;
  memset (&tp->tc, 0, sizeof (tp->tc));
  tp->tc.reg[REG_A0] = (uint32_t)arg;
  tp->tc.reg[REG_RA] = (uint32_t)chopstx_exit;
  tp->tc.reg[REG_PC] = (uint32_t)thread_entry;
  tp->tc.reg[REG_GP] = gp;
  tp->tc.reg[REG_TP] = (uint32_t)&tp->tc;
  tp->tc.reg[REG_SP] = (uint32_t)stack;
  tp->tc.machine_status = MACHINE_STATUS_INIT;

  return tp;
}


/*
 * Note: Examine the assembler output carefully, because it has
 * ((naked)) attribute
 */
static void __attribute__ ((naked,aligned(4)))
chx_handle_intr (void)
{
  struct chx_thread *tp_next;
  uint32_t irq_num;

  /*
   * stack setup to __main_stack_end__
   * save registers.
   */
  asm volatile (
	"csrrw	sp,mscratch,sp\n\t" /* SP to MSCRATCH, thread pointer into SP */
	"# Check if it is IDLE\n\t"
	"bnez	sp,0f\n\t"
	/**/
	"csrrw	sp,mscratch,sp\n\t" /* Recover MSCRATCH, the thread pointer */
	"li	a0,0x80\n\t"
	"csrrc	x0,mstatus,a0\n\t"  /* Clear MPIE bit to mask interrupt     */
	"csrr	a0,mcause\n\t"
	"mret\n"
    "0:\n\t"
	"sw	tp,16(sp)\n\t"      /* Application is free to other use of TP */
	SAVE_OTHER_REGISTERS
	"csrr	a0,mepc\n\t"
	"sw	a0,0(sp)\n\t"
	/**/
	"mv	tp,sp\n\t"          /* TP is now the thread pointer */
	"csrrw	sp,mscratch,sp\n\t" /* TP to MSCRATCH, SP_old into SP */
	"sw	sp,8(tp)\n\t"
	"la	sp,__main_stack_end__");

  /*
   * The stack at __main_stack_end__ is shared data between chx_idle
   * and this handler.  When it comes here, there is no active chx_idle,
   * so, it is safe to use the stack.
   *
   * Note that when there are multiple cores, we need this stack for
   * each core.
   */

  asm (	"csrr	%0,mcause\n\t"
	"slli	%0,%0,20\n\t"
	"srli	%0,%0,20"       /* Take lower 12-bit of MCAUSE */
	: "=r" (irq_num));

  tp_next = NULL;
  if (irq_num == SWINT_IRQ)
    chx_sw_int (0);
  else if (irq_num == TIMER_IRQ)
    tp_next = chx_timer_expired ();
  else if (irq_num == MEMERR_IRQ)
    memory_error ();
  else
    tp_next = chx_recv_irq (irq_num);

  if (!tp_next)
    asm volatile (
	"mv	sp,tp\n\t"      /* Using SP, we can use C.SWSP instruction */
	RESTORE_OTHER_REGISTERS
	"lw	tp,16(sp)\n\t"  /* Application is free to other use of TP */
	"lw	sp,8(sp)\n\t"
	"mret");

  tp_next = chx_running_preempted (tp_next);

  asm volatile (
	"# Involuntary context switch\n\t"
	"mv	sp,tp\n\t"      /* Using SP, we can use C.SWSP instruction */
	SAVE_CALLEE_SAVE_REGISTERS
	/*
	 * MACHINE_STATUS = (MSTATUS & 0x00001f80)
	 */
	"csrr	a1,mstatus\n\t"
	"andi	a1,a1,-9\n\t"     /* Clear MIE (bit3) */
	"slli	a1,a1,19\n\t"     /* Clear bit31 to bit13 */
	"srli	a1,a1,19\n\t"     /* MPP..MPIE from MSTATUS  */
	"sw	a1,128(sp)\n"
    ".L_IV_CONTEXT_SWITCH_BEGIN:\n\t"
	"addi	%0,%0,20\n\t"
	"mv	sp,%0\n\t"
	RESTORE_CALLEE_SAVE_REGISTERS
	/**/
	"csrw	mscratch,sp\n\t"
	"lw	a0,0(sp)\n\t"
	"bnez	a0,.L_RETURN_TO_PREEMPTED_THREAD\n\t"
	/**/
	"lw	a0,-4(sp)\n\t"    /* Get the result value */
	"mv	tp,sp\n\t"
	"lw	sp,8(sp)\n\t"
	"la	a1,.L_V_CONTEXT_SWITCH_FINISH\n\t"
	"csrw	mepc,a1\n\t"
	"li	a1,0x188\n\t"     /* Set MPIE and MPP bits */
	"slli	a1,a1,4\n\t"
	"csrrs	x0,mstatus,a1\n\t"/* Prev: Machine mode, enable interrupt */
	"mret"                    /* Return to Prev  */
	: /* no output */ : "r" (tp_next) : "memory");
}
