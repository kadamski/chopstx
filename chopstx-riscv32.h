/*
 * The thread context: specific to RISC-V 32.
 *
 * Calling convention matters when we implement context switch.
 *
 * We implement a voluntary context switch function which follows
 * standard calling convension.  The function doesn't need to save
 * caller save registers (function arguments and temporaries) and
 * fixed ones (global pointer and thread pointer) to a thread context
 * struct.  Only it needs to care about stack pointer and saved
 * registers.
 *
 * For interrupt handling when it returns directly to original thread,
 * we can avoid saving all registers; It can only save registers used
 * in that handling.  When it retuns to another thread, that is, when
 * it does involuntary context switch (also called "thread
 * preemption"), all registers should be saved.
 *
 * In Chopstx, we don't use nested interrupt of machine.  So, when it
 * a thread is interrupted, the interrupted context is always one of
 * application threads.  We have a dedicated stack for exception
 * handling.
 *
 * Here is RISC-V calling convention.
 *
 * Register   ABI name   description     saver   other usage
 * x0         zero       0, hard-wired   --
 * x1         ra         return address  caller
 * x2         sp         stack pointer   callEE
 * x3         gp         global pointer  --
 * x4         tp         thread pointer  --
 * x5-7       t0-2       temporary       caller
 * x8         s0/fp      saved register  callEE  frame pointer
 * x9         s1         saved register  callEE
 * x10-11     a0-1       argument        caller  return value
 * x12-17     a2-7       argument        caller
 * x18-27     s2-11      saved register  callEE
 * x28-31     t3-6       temporary       caller
 *
 * Simply, we have entries of all registers in the thread context.  We
 * don't use stack to save/restore part of its context.  We don't put
 * any other constraints such that an application should keep gp
 * and/or tp registers; It is free for an application to use registers
 * beyond a conventional usage.
 *
 * We use the first entry, reg[0], to store PC, because x0 is always 0,
 * no need to save/restore it.
 */
struct tcontext {
  uint32_t reg[32];
};

typedef struct tcontext tcontext_t;
