/*
 * chopstx-cortex-m.c - Threads and only threads: Arch specific code
 *                      for Cortex-M0/M3/M4
 *
 * Copyright (C) 2013, 2014, 2015, 2016, 2017, 2018, 2019
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

static struct chx_thread *running;

static struct chx_thread *
chx_running (void)
{
  return running;
}

static void
chx_set_running (struct chx_thread *r)
{
  running = r;
}


/* Data Memory Barrier.  */
static void
chx_dmb (void)
{
  asm volatile ("dmb"  : : : "memory");
}

/* Saved registers on the stack.  */
struct chx_stack_regs {
  uint32_t reg[8];	       /* r0, r1, r2, r3, r12, lr, pc, xpsr */
};

/*
 * Constants for ARM.
 */
#define REG_SP 8

#define REG_R0   0
#define REG_LR   5
#define REG_PC   6
#define REG_XPSR 7

#define INITIAL_XPSR 0x01000000	/* T=1 */

/*
 * Exception priority: lower has higher precedence.
 *
 * Cortex-M3
 * =====================================
 * Prio 0x30: svc
 * ---------------------
 * Prio 0x40: thread temporarily inhibiting schedule for critical region
 * ...
 * Prio 0xb0: systick, external interrupt
 * Prio 0xc0: pendsv (not used)
 * =====================================
 *
 * Cortex-M0
 * =====================================
 * Prio 0x00: thread temporarily inhibiting schedule for critical region
 * ...
 * Prio 0x40: systick, external interrupt
 * Prio 0x80: pendsv (not used)
 * Prio 0x80: svc (not used)
 * =====================================
 */

#define CPU_EXCEPTION_PRIORITY_CLEAR         0

#if defined(__ARM_ARCH_6M__)
#define CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED 0x00
/* ... */
#define CPU_EXCEPTION_PRIORITY_SYSTICK       0x40
#define CPU_EXCEPTION_PRIORITY_INTERRUPT     CPU_EXCEPTION_PRIORITY_SYSTICK
#define CPU_EXCEPTION_PRIORITY_PENDSV        CPU_EXCEPTION_PRIORITY_SYSTICK
#define CPU_EXCEPTION_PRIORITY_SVC           0x80 /* No use in this arch */
#elif defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define CPU_EXCEPTION_PRIORITY_SVC           0x30

#define CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED 0x40
/* ... */
#define CPU_EXCEPTION_PRIORITY_SYSTICK       0xb0
#define CPU_EXCEPTION_PRIORITY_INTERRUPT     CPU_EXCEPTION_PRIORITY_SYSTICK
#define CPU_EXCEPTION_PRIORITY_PENDSV        CPU_EXCEPTION_PRIORITY_SYSTICK
#else
#error "no support for this arch"
#endif

/*
 * Lower layer architecture specific functions.
 *
 * system tick and interrupt
 */

/*
 * System tick
 */
/* SysTick registers.  */
struct SYST {
  volatile uint32_t CSR;
  volatile uint32_t RVR;
  volatile uint32_t CVR;
  const uint32_t CALIB;
};
static struct SYST *const SYST = (struct SYST *)0xE000E010;

static void
chx_systick_init_arch (void)
{
  SYST->RVR = 0;
  SYST->CVR = 0;
  SYST->CSR = 7;
}

static void
chx_systick_reload (uint32_t ticks)
{
  SYST->RVR = ticks;
  SYST->CVR = 0;  /* write (any) to clear the counter to reload.  */
  SYST->RVR = 0;
}

static uint32_t
chx_systick_get (void)
{
  return SYST->CVR;
}

static uint32_t usec_to_ticks (uint32_t usec)
{
  return usec * MHZ;
}

static uint32_t
ticks_to_usec (uint32_t ticks)
{
  return ticks / MHZ;
}

/*
 * Interrupt Handling
 */

/* NVIC: Nested Vectored Interrupt Controller.  */
struct NVIC {
  volatile uint32_t ISER[8];
  uint32_t unused1[24];
  volatile uint32_t ICER[8];
  uint32_t unused2[24];
  volatile uint32_t ISPR[8];
  uint32_t unused3[24];
  volatile uint32_t ICPR[8];
  uint32_t unused4[24];
  volatile uint32_t IABR[8];
  uint32_t unused5[56];
  volatile uint32_t IPR[60];
};

static struct NVIC *const NVIC = (struct NVIC *)0xE000E100;
#define NVIC_ISER(n)	(NVIC->ISER[n >> 5])
#define NVIC_ICER(n)	(NVIC->ICER[n >> 5])
#define NVIC_ICPR(n)	(NVIC->ICPR[n >> 5])
#define NVIC_IPR(n)	(NVIC->IPR[n >> 2])


static void
chx_enable_intr (uint8_t irq_num)
{
  NVIC_ISER (irq_num) = 1 << (irq_num & 0x1f);
}

static void
chx_clr_intr (uint8_t irq_num)
{				/* Clear pending interrupt.  */
  NVIC_ICPR (irq_num) = 1 << (irq_num & 0x1f);
}

static int
chx_disable_intr (uint8_t irq_num)
{
  int already_disabled = !!(NVIC_ICER (irq_num) & (1 << (irq_num & 0x1f)));

  NVIC_ICER (irq_num) = 1 << (irq_num & 0x1f);
  return already_disabled;
}

static void
chx_set_intr_prio (uint8_t n)
{
  unsigned int sh = (n & 3) << 3;

  NVIC_IPR (n) = (NVIC_IPR(n) & ~(0xFF << sh))
    | (CPU_EXCEPTION_PRIORITY_INTERRUPT << sh);
}

static volatile uint32_t *const ICSR = (uint32_t *)0xE000ED04;

/* Priority control.  */
static uint32_t *const AIRCR = (uint32_t *)0xE000ED0C;
static uint32_t *const SHPR2 = (uint32_t *)0xE000ED1C;
static uint32_t *const SHPR3 = (uint32_t *)0xE000ED20;

static void
chx_interrupt_controller_init (void)
{
  *AIRCR = 0x05FA0000 | ( 5 << 8); /* PRIGROUP = 5, 2-bit:2-bit. */
  *SHPR2 = (CPU_EXCEPTION_PRIORITY_SVC << 24);
  *SHPR3 = ((CPU_EXCEPTION_PRIORITY_SYSTICK << 24)
	    | (CPU_EXCEPTION_PRIORITY_PENDSV << 16));
}


static void
chx_cpu_sched_lock (void)
{
#if defined(__ARM_ARCH_6M__)
  asm volatile ("cpsid	i" : : : "memory");
#else
  register uint32_t tmp = CPU_EXCEPTION_PRIORITY_INHIBIT_SCHED;
  asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
#endif
}

static void
chx_cpu_sched_unlock (void)
{
#if defined(__ARM_ARCH_6M__)
  asm volatile ("cpsie	i" : : : "memory");
#else
  register uint32_t tmp = CPU_EXCEPTION_PRIORITY_CLEAR;
  asm volatile ("msr	BASEPRI, %0" : : "r" (tmp) : "memory");
#endif
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
  register uintptr_t result asm ("r0");

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
  asm volatile (
	"svc	#0\n\t"
        "add	r1, r0, #16\n\t"
        "ldr	r0, [r1]"       /* Get tp->v */
	: "=r" (result) : "0" (tp_next): "memory");
#else
  register struct chx_thread *tp asm ("r1");

   /*
   * r0:  TP                    scratch
   * r1:  0                     scratch
   * r2:  0                     scratch
   * r3:  0                     scratch
   * r12: 0                     scratch
   * lr   as-is
   * pc:  return address	zero
   * psr: INITIAL_XPSR          scratch
   */
  asm ("mov	%0, lr\n\t"
       "ldr	r2, =.L_CONTEXT_SWITCH_FINISH\n\t"
       "mov	r3, #128\n\t"
       "lsl	r3, #17\n\t"
       "push	{%0, r2, r3}\n\t"
       "mov	%0, #0\n\t"
       "mov	r2, %0\n\t"
       "mov	r3, %0\n\t"
       "push	{%0, r2, r3}\n\t"
       "ldr	r2, =running\n\t"
       "ldr	%0, [r2]"
       "push	{%0, r3}\n\t"
       : "=r" (tp)
       : /* no input */
       : "r2", "r3", "memory");

  /* Save registers onto CHX_THREAD struct.  */
  asm ("add	r1, #20\n\t"
       "stm	r1!, {r4, r5, r6, r7}\n\t"
       "mov	r2, r8\n\t"
       "mov	r3, r9\n\t"
       "mov	r4, r10\n\t"
       "mov	r5, r11\n\t"
       "mov	r6, sp\n\t"
       "stm	r1!, {r2, r3, r4, r5, r6}\n\t"
       "sub	r1, #56"
       : /* no output */
       : "r" (tp)
       : "r2", "r3", "r4", "r5", "r6", "r7", "memory");

  asm volatile (/* Now, r0 points to the thread to be switched.  */
		/* Put it to *running.  */
		"ldr	r1, =running\n\t"
		/* Update running.  */
		"str	r0, [r1]\n\t"
		"cmp	r0, #0\n\t"
		"bne	0f\n\t"

		/* Spawn an IDLE thread.  */
		"ldr	r1, =__main_stack_end__\n\t"
		"mov	sp, r1\n\t"
		"ldr	r0, =chx_idle\n\t" /* PC = idle */
		/* Not unmasking interrupts.  */
		"blx	r0\n"
		/**/
		/* Now, r0 (again) points to the thread to be switched.	 */
		/* Put it to *running.	*/
		"ldr	r1, =running\n\t"
		/* Update running.  */
		"str	r0, [r1]\n\t"
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
                /* FIXME if preempted thread, do "svc", which never return */
#endif
		/* Normal context switch */
	"0:\n\t"
		"add	r0, #20\n\t"
		"ldm	r0!, {r4, r5, r6, r7}\n\t"
		"ldm	r0!, {r1, r2, r3}\n\t"
		"mov	r8, r1\n\t"
		"mov	r9, r2\n\t"
		"mov	r10, r3\n\t"
		"ldm	r0!, {r1, r2}\n\t"
		"mov	r11, r1\n\t"
		"mov	sp, r2\n\t"
		/**/
		/* Unmask interrupts.  */
		"cpsie	i\n\t"
		/*
		  0:  r0
		  4:  r1
		  8:  r2
		  12: r3
		  16: r12
		  20: lr
		  24: pc
		  28: psr
		  32: possibly exists for alignment
		  [28 or 32] <-- pc
		*/
		"ldr	r0, [sp, #28]\n\t"
		"lsl	r1, r0, #23\n\t"
		"bcc	2f\n\t"
		/**/
		"ldr	r2, [sp, #24]\n\t"
		"mov	r1, #1\n\t"
		"orr	r2, r1\n\t"	/* Ensure Thumb-mode */
		"str	r2, [sp, #32]\n\t"
		"msr	APSR_nzcvq, r0\n\t"
		/**/
		"ldr	r0, [sp, #20]\n\t"
		"mov	lr, r0\n\t"
		"ldr	r0, [sp, #16]\n\t"
		"mov	r12, r0\n\t"
		"pop	{r0, r1, r2, r3}\n\t"
		"add	sp, #16\n\t"
		"pop	{pc}\n"
	"2:\n\t"
		"ldr	r2, [sp, #24]\n\t"
		"mov	r1, #1\n\t"
		"orr	r2, r1\n\t"	/* Ensure Thumb-mode */
		"str	r2, [sp, #28]\n\t"
		"msr	APSR_nzcvq, r0\n\t"
		/**/
		"ldr	r0, [sp, #20]\n\t"
		"mov	lr, r0\n\t"
		"ldr	r0, [sp, #16]\n\t"
		"mov	r12, r0\n\t"
		"pop	{r0, r1, r2, r3}\n\t"
		"add	sp, #12\n\t"
		"pop	{pc}\n\t"
	".L_CONTEXT_SWITCH_FINISH:\n\t"
		"add	r1, r0, #16\n\t"
		"ldr	r0, [r1]"       /* Get tp->v */
		: "=r" (result)		/* Return value in R0 */
		: "0" (tp_next)
		: "memory");
#endif
  return (uintptr_t)result;
}

extern void cause_link_time_error_unexpected_size_of_struct_chx_thread (void);

static struct chx_thread *
chopstx_create_arch (uintptr_t stack_addr, size_t stack_size,
		     voidfunc thread_entry, void *arg)
{
  struct chx_thread *tp;
  void *stack;
  struct chx_stack_regs *p;

  if (CHOPSTX_THREAD_SIZE != sizeof(struct chx_thread))
    cause_link_time_error_unexpected_size_of_struct_chx_thread ();

  if (stack_size < sizeof (struct chx_thread) + 8 * sizeof (uint32_t))
    chx_fatal (CHOPSTX_ERR_THREAD_CREATE);

  stack = (void *)(stack_addr + stack_size - sizeof (struct chx_thread)
		   - sizeof (struct chx_stack_regs));
  memset (stack, 0, sizeof (struct chx_stack_regs));
  tp = (struct chx_thread *)(stack + sizeof (struct chx_stack_regs));
  p = (struct chx_stack_regs *)stack;
  p->reg[REG_R0] = (uint32_t)arg;
  p->reg[REG_LR] = (uint32_t)chopstx_exit;
  p->reg[REG_PC] = (uint32_t)thread_entry;
  p->reg[REG_XPSR] = INITIAL_XPSR;

  memset (&tp->tc, 0, sizeof (tp->tc));
  tp->tc.reg[REG_SP] = (uint32_t)stack;

  return tp;
}

/*
 * Lower layer architecture specific exception handling entries.
 *
 */

/*
 * System call: switch to preempted thread.
 */
void __attribute__ ((naked))
svc (void)
{
  register struct chx_thread *tp asm ("r0");
  register uint32_t orig_r0 asm ("r1");

  /* FIXME: Only if it's not idle thread.  */
  /* from idle thread, no need to save registers (and no where to put it) */
  asm ("ldr	r1, =running\n\t"
       "ldr	r0, [r1]\n\t"
       "add	r1, r0, #20\n\t"
       /* Save registers onto CHX_THREAD struct.  */
       "stm	r1!, {r4, r5, r6, r7}\n\t"
       "mov	r2, r8\n\t"
       "mov	r3, r9\n\t"
       "mov	r4, r10\n\t"
       "mov	r5, r11\n\t"
       "mrs	r6, PSP\n\t" /* r13(=SP) in user space.  */
       "stm	r1!, {r2, r3, r4, r5, r6}\n\t"
       "ldr	r1, [r6]\n\t"
       "str	r0, [r6]"
       : "=r" (tp), "=r" (orig_r0)
       : /* no input */
       : "r2", "r3", "r4", "r5", "r6", "memory");

  /* switch to preempted thread.  */
  /* FIXME: For previous/current thread (if not it's idle thread),
   * fix-up PC at [sp, #24] to go back to .L_CONTEXT_SWITCH_FINISH and
   * also fix-up R0 at [sp] to have tp->v
   */
  asm volatile (
	"b	.L_CONTEXT_SWITCH"
	: /* no output */ : "r" (tp) : "memory");
}


void __attribute__ ((naked))
chx_handle_intr (void)
{
  uint32_t irq_num;
  struct chx_thread *tp_next;

  asm volatile (
	"ldr	r1, =running\n\t"
	"ldr	r0, [r1]\n\t"
#if defined(__ARM_ARCH_6M__)
	"cmp	r0, #0\n\t"
	"bne	1f\n\t"
	"cpsid	i\n"
#else
	"cbnz	r0, 1f\n\t"
	"msr	BASEPRI, r0\n\t"
#endif
	/* It is the idle thread.  */
	/*
	 * Check if returning point is about to execute "wfi".	If it
	 * is, skip the instruction, so that it can keep running
	 * correctly.
	 *
	 * The "wfi" instruction is to wait for interrupt, but an
	 * interrupt may occur just before the execution of the
	 * instruction.
	 *
	 * When the core supports stand-by mode into sleep, the system
	 * goes into sleep when it should not (because an interrupt has
	 * already occurred).
	 *
	 * IMHO, this is an essential race condition which an ISA
	 * design should have avoided.
	 */
#if defined(__ARM_ARCH_6M__)
	"mov	r0, #0x30\n\t"
	"mov	r1, #0xbf\n\t"
	"lsl	r1, r1, #8\n\t"
	"orr	r1, r0\n\t"
#else
	"movw	r1, #0xbf30\n\t"
#endif
	"ldr	r0, [sp, #24]\n\t" /* Return PC */
	"ldrh	r0, [r0]\n\t"      /* instruction to be executed next */
	"cmp	r0, r1\n\t"
	"bne	0f\n\t"
	"ldr	r0, [sp, #24]\n\t" /* Return PC */
	"add	r0, #2\n\t"
	"str	r0, [sp, #24]\n\t" /* Update return PC */
    "0:\n\t"
	"mrs	r0, IPSR\n\t"
	"bx	lr\n"
    "1:"
	: : : "r0", "r1");

  asm volatile ("mrs	%0, IPSR\n\t"
		"sub	%0, #16"   /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : /* no input */ : "memory");

  tp_next = NULL;
  if (irq_num == (uint32_t)-1)
    tp_next = chx_timer_expired ();
  else
    tp_next = chx_recv_irq (irq_num);
  if (!tp_next)
    return;

  /* Save registers onto CHX_THREAD struct.  */
  asm volatile (
	"add	r0, #20\n\t"
	"stm	r0!, {r4, r5, r6, r7}\n\t"
	"mov	r2, r8\n\t"
	"mov	r3, r9\n\t"
	"mov	r4, r10\n\t"
	"mov	r5, r11\n\t"
	"mrs	r6, PSP\n\t" /* r13(=SP) in user space.  */
	"stm	r0!, {r2, r3, r4, r5, r6}"
	: :
	  /*
	   * Memory clobber constraint here is not accurate, but this
	   * works????
	   */
	: "r0", "r1", "r2", "r3", "memory");

  tp_next = chx_running_preempted (tp_next);
  asm volatile (
    ".L_CONTEXT_SWITCH:\n\t"
	/* Now, r0 points to the thread to be switched.  */
	/* Put it to *running.  */
	"ldr	r1, =running\n\t"
	/* Update running.  */
	"str	r0, [r1]\n\t"
	/**/
	"add	r0, #20\n\t"
	"ldm	r0!, {r4, r5, r6, r7}\n\t"
#if defined(__ARM_ARCH_6M__)
	"ldm	r0!, {r1, r2, r3}\n\t"
	"mov	r8, r1\n\t"
	"mov	r9, r2\n\t"
	"mov	r10, r3\n\t"
	"ldm	r0!, {r1, r2}\n\t"
	"mov	r11, r1\n\t"
	"msr	PSP, r2\n\t"
#else
	"ldr	r8, [r0], #4\n\t"
	"ldr	r9, [r0], #4\n\t"
	"ldr	r10, [r0], #4\n\t"
	"ldr	r11, [r0], #4\n\t"
	"ldr	r1, [r0], #4\n\t"
	"msr	PSP, r1\n\t"
#endif
	"mov	r0, #0\n\t"
	/* Unmask interrupts.  */
#if defined(__ARM_ARCH_6M__)
	"cpsie	i\n\t"
#else
	"msr	BASEPRI, r0\n\t"
#endif
	/**/
	"sub	r0, #3\n\t" /* EXC_RETURN to a thread with PSP */
	"bx	r0\n"
	: /* no output */ : "r" (tp_next) : "memory");
}

/*
 * In chx_handle_intr, r0 is set by the value of IPSR.
 *
 * Note: IPSR register access is only valid (returning meaningful
 * value (!=0)) in the exception handler.
 */
#if defined(__ARM_ARCH_6M__)
#define CATCH_AN_INTERRUPT_SYNCHRONOUSLY_0      \
	"mov	%0, #13\n\t"                    \
	"cpsie	i\n\t"                          \
	"nop\n\t"                               \
	"cpsid	i"

#define CATCH_AN_INTERRUPT_SYNCHRONOUSLY_1      \
	"mov	%0, #13\n\t"                    \
	"cpsie	i\n\t"                          \
	"wfi\n\t"                               \
	"cpsid	i"
#else
#define CATCH_AN_INTERRUPT_SYNCHRONOUSLY_0      \
	"mov	%0, #13\n\t"                    \
	"mov	r1, #0x40\n\t"                  \
	"msr	BASEPRI, r1\n\t"                \
	"nop\n\t"                               \
	"mov	r1, #0\n\t"                     \
	"msr	BASEPRI, r1"

#define CATCH_AN_INTERRUPT_SYNCHRONOUSLY_1      \
	"mov	%0, #13\n\t"                    \
	"mov	r1, #0x40\n\t"                  \
	"msr	BASEPRI, r1\n\t"                \
	"wfi\n\t"                               \
	"mov	r1, #0\n\t"                     \
	"msr	BASEPRI, r1"
#endif

/*
 * The idle thread.
 *
 * NOTE: In this thread, interrupt is synchronously handled.
 */
static struct chx_thread * __attribute__((used))
chx_idle (void)
{
  register struct chx_thread *tp_next asm ("r0");

  for (;;)
    {
      extern int chx_prepare_sleep_mode (void);
      register uint32_t irq_num asm ("r0");
      int sleep_mode;

      sleep_mode = chx_prepare_sleep_mode ();

      if (sleep_mode == 0)
	asm volatile (
		CATCH_AN_INTERRUPT_SYNCHRONOUSLY_0
		"sub	%0, #16" /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : : "r1", "memory");
      else
	asm volatile (
		CATCH_AN_INTERRUPT_SYNCHRONOUSLY_1
		"sub	%0, #16" /* Exception # - 16 = interrupt number.  */
		: "=r" (irq_num) : : "r1", "memory");

      /* Note: here, interrupt is masked again.  */

      if (irq_num == (uint32_t)-3)
	continue;
      else if (irq_num == (uint32_t)-1)
	{
	  tp_next = chx_timer_expired ();
	  break;
	}
      else
	{
	  tp_next = chx_recv_irq (irq_num);
	  break;
	}
    }

  return tp_next;
}
