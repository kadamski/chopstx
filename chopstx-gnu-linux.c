/*
 * chopstx-gnu-linux.c - Threads and only threads: Arch specific code
 *                       for GNU/Linux emulation
 *
 * Copyright (C) 2017, 2018, 2019, 2021, 2022 Flying Stone Technology
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

/*
 * NOTE: This code is not portable.  It's intended for GNU/Linux.
 *
 * This implementation depends on the feature of SA_SIGINFO of Linux,
 * which signal handler takes three arguments.  Also, it depends on
 * GNU C library for how to use the makecontext function on 64-bit
 * machine.
 *
 */

#include <unistd.h>
#include <ucontext.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/time.h>

static struct chx_spinlock chx_swapcontext_lock;
static sigset_t ss_fully_blocked;


#ifdef SMP
#include <pthread.h>
#ifndef MAX_CPU
#define MAX_CPU 4
#endif
#define IDLE_STACK_SIZE 2048

static __thread int cpu_id;
static struct chx_thread *running_list[MAX_CPU];
static tcontext_t idle_thread_tc[MAX_CPU];

static volatile struct chx_thread *chx_tp_prev;
static volatile struct chx_thread *chx_tp_next;

struct cpu_info_table {
  pthread_t tid[MAX_CPU];
  int status[MAX_CPU];	      /* 0: idle, 1: running, -1: requested */
};

static struct cpu_info_table cpu_info_table;

static struct chx_thread *
chx_running (void)
{
  return running_list[cpu_id];
}

static void
chx_set_running (struct chx_thread *r)
{
  running_list[cpu_id] = r;
}
#else
static struct chx_thread *running_one;

static struct chx_thread *
chx_running (void)
{
  return running_one;
}

static void
chx_set_running (struct chx_thread *r)
{
  running_one = r;
}
#endif


/* Data Memory Barrier.  */
static void
chx_dmb (void)
{
}


static void preempted_context_switch (struct chx_thread *running,
				      struct chx_thread *tp_next);

#ifdef SMP
static sigset_t ss_cur[MAX_CPU];
#else
static sigset_t ss_cur;
#endif

static void
chx_systick_init_arch (void)
{
  const struct itimerval it = { {0, 0}, {0, 0} };

  setitimer (ITIMER_REAL, &it, 0);
}

static void
chx_systick_reload (uint32_t ticks)
{
  struct itimerval it;

  it.it_value.tv_sec = 0;
  it.it_value.tv_usec = (ticks / MHZ);
  it.it_interval.tv_sec = 0;
  it.it_interval.tv_usec = 0;

  setitimer (ITIMER_REAL, &it, 0);
}

static uint32_t
chx_systick_get (void)
{
  struct itimerval it;
  getitimer (ITIMER_REAL, &it);
  return it.it_value.tv_usec * 72;
}

static uint32_t
usec_to_ticks (uint32_t usec)
{
  return usec * MHZ;
}

static uint32_t
ticks_to_usec (uint32_t ticks)
{
  return ticks / MHZ;
}

static void
chx_enable_intr (uint8_t irq_num)
{
#ifdef SMP
  sigdelset (&ss_cur[cpu_id], irq_num);
#else
  sigdelset (&ss_cur, irq_num);
#endif
}

static void
chx_clr_intr (uint8_t irq_num)
{				/* Clear pending interrupt.  */
  (void)irq_num;
}

static int
chx_disable_intr (uint8_t irq_num)
{
#ifdef SMP
  int already_disabled = sigismember (&ss_cur[cpu_id], irq_num);

  sigaddset (&ss_cur[cpu_id], irq_num);
#else
  int already_disabled = sigismember (&ss_cur, irq_num);

  sigaddset (&ss_cur, irq_num);
#endif
  return already_disabled;
}

static void
chx_set_intr_prio (uint8_t n)
{
  (void)n;
}

static void
chx_interrupt_controller_init (void)
{
}

static void
chx_cpu_sched_lock (void)
{
#ifdef SMP
  pthread_sigmask (SIG_BLOCK, &ss_fully_blocked, &ss_cur[cpu_id]);
#else
  pthread_sigmask (SIG_BLOCK, &ss_fully_blocked, &ss_cur);
#endif
}

static void
chx_cpu_sched_unlock (void)
{
#ifdef SMP
  pthread_sigmask (SIG_SETMASK, &ss_cur[cpu_id], NULL);
#else
  pthread_sigmask (SIG_SETMASK, &ss_cur, NULL);
#endif
}

#ifdef SMP
#define CHX_SIGCPU SIGRTMIN

/* Called with q_ready.lock */
static void
chx_smp_kick_cpu (void)
{
  int i;
  union sigval sigval = { .sival_int = 0 };

  for (i = 0; i < MAX_CPU; i++)
    if (cpu_info_table.status[i] == 0)
      {
	cpu_info_table.status[i] = -1;
	pthread_sigqueue (cpu_info_table.tid[i], CHX_SIGCPU, sigval);
	return;
      }
}
#endif

/* NOTE: Called holding the cpu_sched_lock.  */
static struct chx_thread *
chx_idle (void)
{
  struct chx_thread *tp_next;
  sigset_t set;
  int sig;
#ifdef SMP
  sigset_t setsigcpu;

  sigemptyset (&setsigcpu);
  sigaddset (&setsigcpu, CHX_SIGCPU);
#endif

  chx_spin_lock (&q_ready.lock);
  tp_next = chx_ready_pop ();
  if (tp_next)
    return tp_next;

#ifdef SMP
  cpu_info_table.status[cpu_id] = 0;
#endif

  while (1)
    {
      chx_spin_unlock (&q_ready.lock);
      sigfillset (&set);
      if (sigwait (&set, &sig))
	continue;

      if (sig == SIGALRM)
	{
	  chx_spin_lock (&q_ready.lock);
	  tp_next = chx_timer_expired (NULL);
	}
#ifdef SMP
      else if (sig == CHX_SIGCPU)
	{
	  chx_spin_lock (&q_ready.lock);
	  tp_next = chx_ready_pop ();
	}
#endif
      else
	{
	  chx_spin_lock (&q_ready.lock);
	  tp_next = chx_recv_irq (NULL, (uint32_t)sig);
	  /* Exit when there is no waiter and it's INT or TERM.	 */
	  if (tp_next == NULL
	      && (sig == SIGINT || sig == SIGTERM))
	    exit (1);
	}

      if (tp_next)
	break;

#ifdef SMP
      cpu_info_table.status[cpu_id] = 0;
#endif
    }

#ifdef SMP
  /* Drain SIGCPU signals in the queue if any.  */
  while (1)
    {
      sigpending (&set);
      if (!sigismember (&set, CHX_SIGCPU))
	break;
      sigwait (&setsigcpu, &sig);
    }
  cpu_info_table.status[cpu_id] = 1;

  if (!ll_empty (&q_ready.q))
    chx_smp_kick_cpu ();
#endif
  return tp_next;
}

void
chx_handle_intr (uint32_t irq_num)
{
  struct chx_thread *tp_next;
  struct chx_thread *running = chx_running ();

  chx_spin_lock (&q_ready.lock);
  tp_next = chx_recv_irq (running, irq_num);
  preempted_context_switch (running, tp_next);
}


struct chx_thread main_thread;

void
chx_sigmask (ucontext_t *uc)
{
  /* Modify oldmask to SS_CUR, so that the signal mask will
   * be set to SS_CUR.
   *
   * In user-level, sigset_t is big, but only the first word
   * is used by the kernel.
   */
#ifdef SMP
  memcpy (&uc->uc_sigmask, &ss_cur[cpu_id], sizeof (uint64_t));
#else
  memcpy (&uc->uc_sigmask, &ss_cur, sizeof (uint64_t));
#endif
}

static void
sigalrm_handler (int sig, siginfo_t *siginfo, void *arg)
{
  struct chx_thread *tp_next;
  ucontext_t *uc = arg;
  struct chx_thread *running = chx_running ();

  (void)sig;
  (void)siginfo;

  chx_spin_lock (&q_ready.lock);
  tp_next = chx_timer_expired (running);
  preempted_context_switch (running, tp_next);
  chx_sigmask (uc);
}

#ifdef SMP
static void *
idle_thread_start (void)
{
  struct chx_thread *tp_next;

  if (chx_tp_prev)
    chx_spin_unlock ((struct chx_spinlock *)&chx_tp_prev->lock);
  chx_spin_unlock (&chx_swapcontext_lock);
  chx_spin_unlock (&q_ready.lock);

  tp_next = chx_idle ();
  chx_spin_lock (&chx_swapcontext_lock);
  chx_set_running (tp_next);
  chx_tp_prev = NULL;
  chx_tp_next = tp_next;
  setcontext (&tp_next->tc);

  /* NOTREACHED */
  return NULL;
}

static void *
cpu_start (void *arg)
{
  char stack_for_idle[IDLE_STACK_SIZE];
  int id = (long)arg;
  tcontext_t *tc;

  cpu_id = id;

  tc = &idle_thread_tc[cpu_id];

  chx_cpu_sched_lock ();
  chx_spin_lock (&q_ready.lock);

  getcontext (tc);
  tc->uc_stack.ss_sp = stack_for_idle;
  tc->uc_stack.ss_size = sizeof (stack_for_idle);
  tc->uc_link = NULL;
  makecontext (tc, (void (*)(void))idle_thread_start, 0);

  chx_spin_lock (&chx_swapcontext_lock);
  chx_tp_prev = NULL;
  setcontext (tc);

  /* NOTREACHED */
  return NULL;
}
#endif

#ifdef SMP
static char stack_for_idle0[IDLE_STACK_SIZE];
#endif

static void
chx_init_arch (struct chx_thread *tp)
{
  struct sigaction sa;

  sigfillset (&ss_fully_blocked);
  /*
   * Since call of chx_cpu_sched_lock before chx_init_arch
   * doesn't work, we need to fix it here.
   */
  pthread_sigmask (SIG_BLOCK, &ss_fully_blocked, &ss_cur[cpu_id]);

#ifdef SMP
  long i;

  chx_spin_init (&chx_swapcontext_lock);

  cpu_id = 0;

  for (i = 0; i < MAX_CPU; i++)
    {
      sigemptyset (&ss_cur[i]);
      sigaddset (&ss_cur[i], CHX_SIGCPU);
    }
#else
  sigemptyset (&ss_cur);
#endif

  sa.sa_sigaction = sigalrm_handler;
  sigfillset (&sa.sa_mask);
  sa.sa_flags = SA_SIGINFO|SA_RESTART;
  sigaction (SIGALRM, &sa, NULL); 

#ifdef SMP
  /*
   * Delivery of CHX_SIGCPU is only done synchronously (always caught
   * by sigwait), so, no signal handler setup.
   */

  for (i = 0; i < MAX_CPU; i++)
    {
      pthread_t cpu;

      if (i == 0)
	{
	  tcontext_t *tc = &idle_thread_tc[0];

	  getcontext (tc);
	  tc->uc_stack.ss_sp = stack_for_idle0;
	  tc->uc_stack.ss_size = sizeof (stack_for_idle0);
	  tc->uc_link = NULL;
	  makecontext (tc, (void (*)(void))idle_thread_start, 0);
	  cpu = pthread_self ();
	  cpu_info_table.status[0] = 1;
	}
      else
	{
	  pthread_create (&cpu, NULL, cpu_start, (void *)i);
	  cpu_info_table.status[i] = 1;
	}

      cpu_info_table.tid[i] = cpu;
    }
#else
  getcontext (&tp->tc);
#endif

  chx_set_running (tp);
}


static uintptr_t
chx_swapcontext (struct chx_thread *tp_prev, struct chx_thread *tp_next)
{
  uintptr_t v;

  chx_spin_lock (&chx_swapcontext_lock);
#ifdef SMP
  chx_tp_prev = tp_prev;
  chx_tp_next = tp_next;
#endif
  chx_set_running (tp_next);
  if (tp_next)
    swapcontext (&tp_prev->tc, &tp_next->tc);
  else
    {
#ifdef SMP
      swapcontext (&tp_prev->tc, &idle_thread_tc[cpu_id]);
#else
      tp_next = chx_idle ();
      if (tp_prev != tp_next)
	{
	  chx_set_running (tp_next);
	  swapcontext (&tp_prev->tc, &tp_next->tc);
	}
#endif
    }

  v = chx_tp_next->v;

#ifdef SMP
  if (chx_tp_prev)
    chx_spin_unlock ((struct chx_spinlock *)&chx_tp_prev->lock);
  chx_spin_unlock ((struct chx_spinlock *)&chx_tp_next->lock);
#endif
  chx_spin_unlock (&chx_swapcontext_lock);
  chx_spin_unlock (&q_ready.lock);
  return v;
}

static void
preempted_context_switch (struct chx_thread *running,
			  struct chx_thread *tp_next)
{
  if (tp_next == NULL)
    {
      chx_spin_unlock (&running->lock);
      chx_spin_unlock (&q_ready.lock);
      return;
    }

  /*
   * The swapcontext implementation may reset sigmask in the
   * middle of its execution, unfortunately.  It is best if
   * sigmask restore is done at the end of the routine, but we
   * can't assume that.
   *
   * Thus, there might be a race condition with regards to the
   * user context TCP, if signal mask is cleared and signal comes
   * in.  To avoid this situation, we block signals.
   *
   * We don't need to fill the mask here.  It keeps the condition
   * of blocking signals before&after swapcontext call.  It is
   * done by the signal mask for sigaction, the initial creation
   * of the thread, and the condition of chx_sched function which
   * mandates holding cpu_sched_lock.
   */
  chx_swapcontext (running, tp_next);
}


static uintptr_t
voluntary_context_switch (struct chx_thread *running,
			  struct chx_thread *tp_next)
{
  return chx_swapcontext (running, tp_next);
}

static void __attribute__((__noreturn__))
chx_thread_start (voidfunc thread_entry, void *arg)
{
  void *ret;

#ifdef SMP
  if (chx_tp_prev)
    chx_spin_unlock ((struct chx_spinlock *)&chx_tp_prev->lock);
  chx_spin_unlock ((struct chx_spinlock *)&chx_tp_next->lock);
#endif
  chx_spin_unlock (&chx_swapcontext_lock);
  chx_spin_unlock (&q_ready.lock);

  chx_cpu_sched_unlock ();
  ret = thread_entry (arg);
  chopstx_exit (ret);
}

/* Called with holding cpu_schec_lock.  */
static struct chx_thread *
chopstx_create_arch (uintptr_t stack_addr, size_t stack_size,
		     voidfunc thread_entry, void *arg)
{
  struct chx_thread *tp;
  tp = malloc (sizeof (struct chx_thread));
  if (!tp)
    chx_fatal (CHOPSTX_ERR_THREAD_CREATE);

  /*
   * Calling getcontext with sched_lock held, the context is with
   * signal blocked.  The sigmask will be cleared in chx_thread_start.
   */
  memset (tp, 0, sizeof (struct chx_thread));
  getcontext (&tp->tc);
  tp->tc.uc_stack.ss_sp = (void *)stack_addr;
  tp->tc.uc_stack.ss_size = stack_size;
  tp->tc.uc_link = NULL;

  /*
   * makecontext is hard to use in a portable way, actually.
   *
   * On 64-bit machine, according to the standard, it should be coded
   * to specify int (== 32-bit for LP64 machine) arguments that follow
   * ARGC, so it is not that correct to specify two 64-bit arguments
   * here.  However, GNU C library allows this.
   */
  makecontext (&tp->tc, (void (*)(void))chx_thread_start,
	       2, thread_entry, arg);
  return tp;
}
