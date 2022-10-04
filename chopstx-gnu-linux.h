#include <ucontext.h>
/*
 * The thread context: specific to GNU/Linux.
 *
 * We use the type ucontext_t, which includes all registers;
 * Note that signal mask is also included in ucontext_t.
 *
 */

typedef ucontext_t tcontext_t;

#ifdef SMP
static void chx_smp_kick_cpu (void);

static void chx_spin_init (struct chx_spinlock *lk)
{
  pthread_spin_init (&lk->lk, PTHREAD_PROCESS_PRIVATE);
}

static void chx_spin_lock (struct chx_spinlock *lk)
{
  pthread_spin_lock (&lk->lk);
}

static void chx_spin_unlock (struct chx_spinlock *lk)
{
  pthread_spin_unlock (&lk->lk);
}
#else
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
