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
	".equ	msubm,0x7c4\n\t"
	".equ	mtvt,0x307\n\t"   /* Not used. */
	".equ	mtvt2,0x7ec\n\t"
	".equ	wfe,0x810\n\t"    /* Not used (yet). */
	".equ	sleepvalue,0x811" /* Not used (yet). */
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

/*
 * We keep the TP register to the thread context address.
 * Also, it is kept at the MSCRATCH register.
 */
static struct chx_thread *
chx_running (void)
{
  struct chx_thread *tp;

  asm ( "addi	%0,tp,-20"       : "=r" (tp));
  return tp;
}

static void
chx_set_running (struct chx_thread *tp)
{
  asm (	"addi	%0,%0,20\n\t"
        "csrw	mscratch,%0" : /* no output */ : "r" (tp) : "memory");
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
  uint32_t rsv[1018];
  volatile uint32_t mstop;
  volatile uint32_t msip;
};
static struct TIMER *const TIMER = (struct TIMER *)0xD1000000;

/* In Chopstx, we only use the lower 32-bit of the timer.  */

static void
chx_systick_init_arch (void)
{
  TIMER->mstop |= 1;
  TIMER->mtime_hi = TIMER->mtime_lo = 0;
  TIMER->mtimecmp_hi = TIMER->mtimecmp_lo = 0;
}

/*
 * @ticks: 0 means to stop the timer, otherwise update the timer
 */
static void
chx_systick_reload (uint32_t ticks)
{
  uint32_t ticks_old;

  if (!ticks)
    TIMER->mstop |= 1;
  TIMER->mtime_lo = 0;
  ticks_old = TIMER->mtimecmp_lo;
  TIMER->mtimecmp_lo = ticks;
  if (!ticks_old && ticks)
    TIMER->mstop &= ~1;
}

static uint32_t
chx_systick_get (void)
{
  uint32_t now = TIMER->mtime_lo;

  if (TIMER->mtimecmp_lo <= now)
    return 0;

  return TIMER->mtimecmp_lo - now;
}

/* TIMER runs at 1/4 of core clock.  */

static uint32_t
usec_to_ticks (uint32_t usec)
{
  return usec * MHZ / 4;
}

/*
 * Interrupt Handling of (E)CLIC
 */

/* CLIC registers.  */
struct CLIC {
  volatile uint32_t cfg;
  volatile uint32_t info;
  volatile uint8_t uth;
  volatile uint8_t sth;
  volatile uint8_t hth;
  volatile uint8_t mth;
} __attribute__((packed));

static struct CLIC *const CLIC = (struct CLIC *)0xD2000000;

struct CLIC_INT {
  volatile uint8_t ip;
  volatile uint8_t ie;
  volatile uint8_t attr;
  volatile uint8_t ctl;
} __attribute__((packed));
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
  CLIC_INT[irq_num].attr = 0x00; /* Level triggered, rising-edge, SHV=0 */
  /* Note: SHV: Selective Hardware Vectoring: off (use common routine).  */

  CLIC_INT[irq_num].ctl = 0x01; /* FIXME: level=1 */
}

#define TIMER_IRQ 7

static void __attribute__ ((naked)) chx_handle_intr (void);
static void __attribute__ ((naked)) exception_handler (void);

static void
exception_handler (void)
{
   asm volatile (
"0:	j	0b"
        : /* no output */);
}

static void
chx_interrupt_controller_init (void)
{
  /* mtvt2 enable, use common interrupt routine.  */
  const uint32_t mtvt2_value = (uint32_t)chx_handle_intr | 1;
  const uint32_t mtvec_value = (uint32_t)exception_handler;

  asm volatile (
	"csrw	mtvt2,%0"
	: /* no output */ : "r" (mtvt2_value) : "memory");

  asm volatile (
	"csrw	mtvec,%0"
	: /* no output */ : "r" (mtvec_value) : "memory" );

  CLIC->cfg = 0;
  CLIC->mth = 0;

  /* In Bumblebee core, timer interrupt is also handled by CLIC.  */
  chx_set_intr_prio (TIMER_IRQ);
  chx_enable_intr (TIMER_IRQ);
}


static void
chx_cpu_sched_lock (void)
{
  struct chx_thread *r = chx_running ();

  if (r->prio < CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    asm volatile (
	"csrci	mstatus,8"
	: : : "memory" );
}

static void
chx_cpu_sched_unlock (void)
{
  struct chx_thread *r = chx_running ();

  if (r->prio < CHOPSTX_PRIO_INHIBIT_PREEMPTION)
    asm volatile (
	"csrsi	mstatus,8"
	: : : "memory" );
}

static void
chx_init_arch (struct chx_thread *tp)
{
  memset (&tp->tc, 0, sizeof (tp->tc));
  chx_set_running (tp);
}

static uintptr_t
voluntary_context_switch (struct chx_thread *tp_next)
{
  register uintptr_t result asm ("a0");

  asm volatile (
	/* Here, %0 (a0) points to the thread context to be switched.  */
	"# Save registers\n\t"
	"sw	sp,8(tp)\n\t"
	"mv	sp,tp\n\t"      /* Using SP, we can use C.SWSP instruction */
	"sw	zero,0(sp)\n\t"
	"sw	s0,32(sp)\n\t"
	"sw	s1,36(sp)\n\t"
	"sw	s2,72(sp)\n\t"
	"sw	s3,76(sp)\n\t"
	"sw	s4,80(sp)\n\t"
	"sw	s5,84(sp)\n\t"
	"sw	s6,88(sp)\n\t"
	"sw	s7,92(sp)\n\t"
	"sw	s8,96(sp)\n\t"
	"sw	s9,100(sp)\n\t"
	"sw	s10,104(sp)\n\t"
	"sw	s11,108(sp)\n\t"
	"# Check if going to IDLE thread\n\t"
	"bnez	%0,0f\n\t"
	"# Spawn an IDLE thread, interrupt enabled.\n\t"
	"mv	tp,zero\n\t"
	"csrw	mscratch,tp\n\t"
	"la	sp,__main_stack_end__\n\t"
	"csrsi	mstatus,8\n\t"  /* Unmask interrupts.  */
	"j	chx_idle\n"
    "0:\n\t"
	"mv	sp,%0\n\t"
	"# Restore registers\n\t"
	"lw	s0,32(sp)\n\t"
	"lw	s1,36(sp)\n\t"
	"lw	s2,72(sp)\n\t"
	"lw	s3,76(sp)\n\t"
	"lw	s4,80(sp)\n\t"
	"lw	s5,84(sp)\n\t"
	"lw	s6,88(sp)\n\t"
	"lw	s7,92(sp)\n\t"
	"lw	s8,96(sp)\n\t"
	"lw	s9,100(sp)\n\t"
	"lw	s10,104(sp)\n\t"
	"lw	s11,108(sp)\n\t"
	/**/
	"lw	a0,-4(sp)\n\t"  /* Get the result value */
	/**/
	"csrw	mscratch,sp\n\t"
	"lw	a1,0(sp)\n\t"
	"beqz	a1,1f\n\t"
	"# Restore all registers\n\t"
	"csrw	mepc,a0\n\t"
	"lw	ra,4(sp)\n\t"
	"lw	gp,12(sp)\n\t"
	"lw	a3,52(sp)\n\t"
	"lw	a4,56(sp)\n\t"
	"lw	a5,60(sp)\n\t"
	"lw	a6,64(sp)\n\t"
	"lw	a7,68(sp)\n\t"
	"lw	t0,20(sp)\n\t"
	"lw	t1,24(sp)\n\t"
	"lw	t2,28(sp)\n\t"
	"lw	t3,112(sp)\n\t"
	"lw	t4,116(sp)\n\t"
	"lw	t5,120(sp)\n\t"
	"lw	t6,124(sp)\n\t"
	/**/
	"lw	a0,128(sp)\n\t"
	"lui	a1,0x00f00\n\t"
	"and	a2,a0,a1\n\t"
	"srli	a2,a2,14\n\t"
	"csrw	msubm,a2\n\t"
	"not	a1,a1\n\t"
	"and	a0,a0,a1\n\t"
	"csrr	a1,mstatus\n\t"
	"srli	a1,a1,13\n\t"     /* Keep SD, XS, and FS bits */
	"slli	a1,a1,13\n\t"
	"or	a0,a0,a1\n\t"
	"csrw	mstatus,a0\n\t"
	/**/
	"lw	a0,40(sp)\n\t"
	"lw	a1,44(sp)\n\t"
	"lw	a2,48(sp)\n\t"
	"lw	tp,16(sp)\n\t"    /* Application is free to other use of TP */
	"lw	sp,8(sp)\n\t"
	"mret\n"
    "1:\n\t"
	"mv	tp,sp\n\t"
	"lw	sp,8(sp)\n\t"
	"csrsi	mstatus,8\n"  /* Unmask interrupts.  */
    ".L_CONTEXT_SWITCH_FINISH:"
	: "=r" (result)
	: "0" (tp_next)
	: "ra", "t0", "t1", "t2", "t3", "t4", "t5", "t6",
	  "a1", "a2", "a3", "a4", "a5", "a6", "a7",
	  "memory");

  return result;
}

/*
 * chx_sched: switch to another thread.
 *
 * There are two cases:
 *   YIELD=0 (SLEEP): Current RUNNING thread is already connected to
 *                    something (mutex, cond, intr, etc.)
 *   YIELD=1 (YIELD): Current RUNNING thread is active,
 *                    it is needed to be enqueued to READY queue.
 *
 * Returns:
 *       >= 1 on wakeup by others, value means ticks remained for sleep.
 *          0 on normal wakeup (timer expiration, lock acquirement).
 *         -1 on cancellation.
 */
static uintptr_t __attribute__ ((noinline))
chx_sched (uint32_t yield)
{
  struct chx_thread *tp = chx_running ();

  if (yield)
    {
      if (tp->flag_sched_rr)
	chx_timer_dequeue (tp);
      chx_ready_enqueue (tp);
    }

  tp = chx_ready_pop ();
  if (tp && tp->flag_sched_rr)
    {
      chx_spin_lock (&q_timer.lock);
      chx_timer_insert (tp, PREEMPTION_USEC);
      chx_spin_unlock (&q_timer.lock);
    }

  return voluntary_context_switch (tp);
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

  return tp;
}


static struct chx_thread * __attribute__ ((noinline))
chx_recv_irq (uint32_t irq_num)
{
  struct chx_pq *p;
  struct chx_thread *r = chx_running ();

  chx_disable_intr (irq_num);
  chx_spin_lock (&q_intr.lock);
  for (p = q_intr.q.next; p != (struct chx_pq *)&q_intr.q; p = p->next)
    if (p->v == irq_num)
      /* should be one at most.  */
      break;
  chx_spin_unlock (&q_intr.lock);

  if (p)
    {
      struct chx_px *px = (struct chx_px *)p;

      ll_dequeue (p);
      chx_wakeup (p);

      if (r == NULL || (uint16_t)r->prio < px->master->prio)
	{
	  struct chx_thread *tp;

	  tp = chx_ready_pop ();
	  if (tp && tp->flag_sched_rr)
	    {
	      chx_spin_lock (&q_timer.lock);
	      chx_timer_insert (tp, PREEMPTION_USEC);
	      chx_spin_unlock (&q_timer.lock);
	    }

	  return tp;
	}
    }

  return NULL;
}

#define TIMER_IRQ 7
struct chx_thread * chx_timer_expired (void);

/*
 * Note: Examine the assembler output carefully, because it has
 * ((naked)) attribute
 */
static void
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
	"# Check if it is IDLE thread\n\t"
	"bnez	sp,0f\n\t"
	"csrw	mscratch,sp\n\t"    /* Recover MSCRATCH, the thread pointer */
	"mv	tp,zero\n\t"
	"j	1f\n"
    "0:\n\t"
	"sw	tp,16(sp)\n\t"      /* Application is free to other use of TP */
	"sw	ra,4(sp)\n\t"
	"sw	gp,12(sp)\n\t"
	"sw	t0,20(sp)\n\t"
	"sw	t1,24(sp)\n\t"
	"sw	t2,28(sp)\n\t"
	"sw	a0,40(sp)\n\t"
	"sw	a1,44(sp)\n\t"
	"sw	a2,48(sp)\n\t"
	"sw	a3,52(sp)\n\t"
	"sw	a4,56(sp)\n\t"
	"sw	a5,60(sp)\n\t"
	"sw	a6,64(sp)\n\t"
	"sw	a7,68(sp)\n\t"
	"sw	t3,112(sp)\n\t"
	"sw	t4,116(sp)\n\t"
	"sw	t5,120(sp)\n\t"
	"sw	t6,124(sp)\n\t"
	/**/
	"csrr	a0,mepc\n\t"
	"sw	a0,0(sp)\n\t"
	/**/
	"mv	tp,sp\n\t"          /* TP is now the thread pointer */
	"csrrw	sp,mscratch,sp\n\t" /* TP to MSCRATCH, SP_old into SP */
	"sw	sp,8(tp)\n"
	/**/
    "1:\n\t"
	"la	sp,__main_stack_end__");

  asm (	"csrr	%0,mcause\n\t"
	"slli	%0,%0,20\n\t"
	"srli	%0,%0,20"       /* Take lower 12-bit of MCAUSE */
        : "=r" (irq_num));

  if (irq_num == TIMER_IRQ)
    tp_next = chx_timer_expired ();
  else
    tp_next = chx_recv_irq (irq_num);

  if (!tp_next)
    asm volatile (
	"bnez	tp,0f\n\t"
	"# Spawn an IDLE thread.\n\t"
	"la	sp,__main_stack_end__\n\t"
	"la	a0,chx_idle\n\t"
	"csrw	mepc,a0\n\t"
	"mret\n"
    "0:\n\t"
	"mv	sp,tp\n\t"      /* Using SP, we can use C.SWSP instruction */
	"# Restore registers\n\t"
	"lw	ra,4(sp)\n\t"
	"lw	gp,12(sp)\n\t"
	"lw	t0,20(sp)\n\t"
	"lw	t1,24(sp)\n\t"
	"lw	t2,28(sp)\n\t"
	"lw	a0,40(sp)\n\t"
	"lw	a1,44(sp)\n\t"
	"lw	a2,48(sp)\n\t"
	"lw	a3,52(sp)\n\t"
	"lw	a4,56(sp)\n\t"
	"lw	a5,60(sp)\n\t"
	"lw	a6,64(sp)\n\t"
	"lw	a7,68(sp)\n\t"
	"lw	t3,112(sp)\n\t"
	"lw	t4,116(sp)\n\t"
	"lw	t5,120(sp)\n\t"
	"lw	t6,124(sp)\n\t"
	"lw	tp,16(sp)\n\t" /* Application is free to other use of TP */
	"lw	sp,8(sp)\n\t"
	"mret");

  asm volatile (
        "# Involuntary context switch\n\t"
	"mv	sp,tp\n\t"      /* Using SP, we can use C.SWSP instruction */
	"beqz	sp,0f\n\t"
	"# Save registers\n\t"
	"sw	s0,32(sp)\n\t"
	"sw	s1,36(sp)\n\t"
	"sw	s2,72(sp)\n\t"
	"sw	s3,76(sp)\n\t"
	"sw	s4,80(sp)\n\t"
	"sw	s5,84(sp)\n\t"
	"sw	s6,88(sp)\n\t"
	"sw	s7,92(sp)\n\t"
	"sw	s8,96(sp)\n\t"
	"sw	s9,100(sp)\n\t"
	"sw	s10,104(sp)\n\t"
	"sw	s11,108(sp)\n\t"
	"csrr	a0,mstatus\n\t"
	"slli	a0,a0,19\n\t"
	"srli	a0,a0,19\n\t"     /* Clear SD, XS, and FS bits, no save */
	"csrr	a1,msubm\n\t"
	"slli	a1,a1,14\n\t"
	"or	a0,a0,a1\n\t"
	"sw	a0,128(sp)\n"
	/**/
    "0:\n\t"
	"mv	sp,%0\n\t"
	"# Restore registers\n\t"
	"lw	s0,32(sp)\n\t"
	"lw	s1,36(sp)\n\t"
	"lw	s2,72(sp)\n\t"
	"lw	s3,76(sp)\n\t"
	"lw	s4,80(sp)\n\t"
	"lw	s5,84(sp)\n\t"
	"lw	s6,88(sp)\n\t"
	"lw	s7,92(sp)\n\t"
	"lw	s8,96(sp)\n\t"
	"lw	s9,100(sp)\n\t"
	"lw	s10,104(sp)\n\t"
	"lw	s11,108(sp)\n\t"
	/**/
	"csrw	mscratch,sp\n\t"
	"lw	a0,0(sp)\n\t"
	"bnez	a0,1f\n\t"
	/**/
	"lw	a0,-4(sp)\n\t"    /* Get the result value */
	/**/
	"mv	tp,sp\n\t"
	"lw	sp,8(sp)\n\t"
	"la	a1,.L_CONTEXT_SWITCH_FINISH\n\t"
	"csrw	mepc,a1\n\t"
	"csrr	a1,mstatus\n\t"
	"li	a2,0x188\n\t"     /* Set MPIE and MPP... */
	"slli	a2,a2,4\n\t"      /* ... by shifting 4-bit left */
	"or	a1,a1,a2\n\t"
	"csrw	mstatus,a1\n\t"   /* Prev: Machine mode, enable interrupt */
	"csrr	a1,msubm\n\t"
	"andi	a1,a1,0x00c0\n\t"
	"csrw	msubm,a1\n\t"     /* Prev: No-trap */
	"mret\n"                  /* Return to Prev  */
    "1:\n\t"
	"# Restore all registers\n\t"
	"csrw	mepc,a0\n\t"
	"lw	ra,4(sp)\n\t"
	"lw	gp,12(sp)\n\t"
	"lw	a3,52(sp)\n\t"
	"lw	a4,56(sp)\n\t"
	"lw	a5,60(sp)\n\t"
	"lw	a6,64(sp)\n\t"
	"lw	a7,68(sp)\n\t"
	"lw	t0,20(sp)\n\t"
	"lw	t1,24(sp)\n\t"
	"lw	t2,28(sp)\n\t"
	"lw	t3,112(sp)\n\t"
	"lw	t4,116(sp)\n\t"
	"lw	t5,120(sp)\n\t"
	"lw	t6,124(sp)\n\t"
	/**/
	"lw	a0,128(sp)\n\t"
	"lui	a1,0x00f00\n\t"
	"and	a2,a0,a1\n\t"
	"srli	a2,a2,14\n\t"
	"csrw	msubm,a2\n\t"
	"not	a1,a1\n\t"
	"and	a0,a0,a1\n\t"
	"csrr	a1,mstatus\n\t"
	"srli	a1,a1,13\n\t"     /* Keep SD, XS, and FS bits */
	"slli	a1,a1,13\n\t"
	"or	a0,a0,a1\n\t"
	"csrw	mstatus,a0\n\t"
	/**/
	"lw	a0,40(sp)\n\t"
	"lw	a1,44(sp)\n\t"
	"lw	a2,48(sp)\n\t"
	"lw	tp,16(sp)\n\t" /* Application is free to other use of TP */
	"lw	sp,8(sp)\n\t"
	"mret"
	: /* no output */ : "r" (tp_next) : "memory");
}
