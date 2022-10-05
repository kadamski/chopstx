/*
 * The thread context: specific to ARM Cortex-M0/M3.
 *
 * In this structure, it's only partial information; Other part of the
 * context is on the stack.
 *
 */
struct tcontext {
  uint32_t reg[9];	   /* r4, r5, r6, r7, r8, r9, r10, r11, r13(sp) */
};

typedef struct tcontext tcontext_t;

#define CHOPSTX_THREAD_SIZE 64

#ifdef CHOPSTX_PRIO_BITS
static void chx_smp_kick_cpu (void)
{
}

static void chx_spin_init (struct chx_spinlock *lk)
{
  (void)lk;
}

static void chx_spin_lock (struct chx_spinlock *lk)
{
  (void)lk;
}

static void chx_spin_unlock (struct chx_spinlock *lk)
{
  (void)lk;
}
#endif
