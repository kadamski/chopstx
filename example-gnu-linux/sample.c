#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include <chopstx.h>

#ifndef TESTING
#define TESTING 1
#endif

static void
bbl_sort (int array[], int n, int dir)
{
  int i;
  int new;
  do
    {
      new = 0;
      for (i = 1; i < n; i++)
	if ((dir < 0 && array[i-1] > array[i])
	    || (dir > 0 && array[i-1] < array[i]))
          {
            int t = array[i-1];

            array[i-1] = array[i];
            array[i] = t;
            new = i;
          }
      n = new;
    }
  while (n > 1);
}

struct sem {
  chopstx_mutex_t mutex;
  chopstx_cond_t avail;
  int count;
};

static void
sem_init (struct sem *sem, int initial_count)
{
  chopstx_mutex_init (&sem->mutex);
  chopstx_cond_init (&sem->avail);
  sem->count = initial_count;
}

static void
sem_get (struct sem *sem, int count)
{
  chopstx_mutex_lock (&sem->mutex);
  while (sem->count < count)
    chopstx_cond_wait (&sem->avail, &sem->mutex);
  sem->count -= count;
  chopstx_mutex_unlock (&sem->mutex);
}

static void
sem_put (struct sem *sem, int count)
{
  chopstx_mutex_lock (&sem->mutex);
  sem->count += count;
  chopstx_cond_signal (&sem->avail);
  chopstx_mutex_unlock (&sem->mutex);
}


struct xchg {
  chopstx_mutex_t mutex;
  chopstx_cond_t avail;
  int v;
  unsigned int ready;
};

static void
xchg_init (struct xchg *xchg)
{
  chopstx_mutex_init (&xchg->mutex);
  chopstx_cond_init (&xchg->avail);
  xchg->v = 0;
  xchg->ready = 0;
}

static int
xchg_do (struct xchg *xchg, int v)
{
  int r;

  chopstx_mutex_lock (&xchg->mutex);
  if (xchg->ready)
    {
      r = xchg->v;
      xchg->v = v;
      xchg->ready = 0;
      chopstx_cond_signal (&xchg->avail);
    }
  else
    {
      xchg->v = v;
      xchg->ready = 1;
      chopstx_cond_wait (&xchg->avail, &xchg->mutex);
      r = xchg->v;
    }
  chopstx_mutex_unlock (&xchg->mutex);
  return r;
}


static void
print_matrix (int array[], int n)
{
  int i, j;

  for (i = 0; i < n; i++)
    {
      for (j = 0; j < n; j++)
        printf (j != n - 1 ? "%3d ": "%3d", array[i * n + j]);
      if (i != n -1)
        puts ("");
    }
  puts ("");
}

#define N 7
#define N_N (N*N)
#define N_NminusNdiv2 ((N*N-N)/2)
#define ITERATION (32 - 26 +  1)
/* ((sizeof (int) * 8) - __builtin_clz (N_N)) + 1 */

struct worker_args {
  int row;
  int *array;
  struct xchg *xchg;
  struct sem *in;
  struct sem *out;
};

/*
 When N = 7: 

  -   0   1   2   3   4   5
  0   -   6   7   8   9  10
  1   6   -  11  12  13  14
  2   7  11   -  15  16  17
  3   8  12  15   -  18  19
  4   9  13  16  18   -  20
  5  10  14  17  19  20   -
 */

static int
get_port (int row, int col)
{
  if (row == col)
    return -1;

  if (row > col)
    return get_port (col, row);

  return row * (2*N - row - 3) / 2 + col - 1;
}


static void *
worker (void *args0)
{
  struct worker_args *args = args0;
  int row = args->row;
  int *array = args->array;
  int iter;

  iter = 0;
  while (1)
    {
      int i;
      int dir;

      sem_get (args->in, 1);

      if ((iter & 1))
	dir = -1;
      else if ((row & 1))
	dir =  1;
      else
	dir = -1;

      bbl_sort (array, N, dir);
#if TESTING
      chopstx_usec_wait (3);
#endif

      iter++;
      if (iter >= ITERATION)
	break;

      for (i = 0; i < N; i++)
	{
	  int port = get_port (row, i);

	  if (port >= 0)
	    array[i] = xchg_do (&args->xchg[port], array[i]);
	}

      sem_put (args->out, 1);
    }

  sem_put (args->out, 1);

  return NULL;
}

#define PRIO_THD0 2
#define PRIO_THD1 1
#define PRIO_THD2 1
#define PRIO_THD3 1
#define PRIO_THD4 1
#define PRIO_THD5 1
#define PRIO_THD6 1
#define PRIO_THD7 1

static char __process1_stack_base__[4096];
static char __process2_stack_base__[4096];
static char __process3_stack_base__[4096];
static char __process4_stack_base__[4096];
static char __process5_stack_base__[4096];
static char __process6_stack_base__[4096];
static char __process7_stack_base__[4096];

#define STACK_ADDR_THD1 ((uintptr_t)__process1_stack_base__)
#define STACK_ADDR_THD2 ((uintptr_t)__process2_stack_base__)
#define STACK_ADDR_THD3 ((uintptr_t)__process3_stack_base__)
#define STACK_ADDR_THD4 ((uintptr_t)__process4_stack_base__)
#define STACK_ADDR_THD5 ((uintptr_t)__process5_stack_base__)
#define STACK_ADDR_THD6 ((uintptr_t)__process6_stack_base__)
#define STACK_ADDR_THD7 ((uintptr_t)__process7_stack_base__)

#define STACK_SIZE_THD1 (sizeof __process1_stack_base__)
#define STACK_SIZE_THD2 (sizeof __process2_stack_base__)
#define STACK_SIZE_THD3 (sizeof __process3_stack_base__)
#define STACK_SIZE_THD4 (sizeof __process4_stack_base__)
#define STACK_SIZE_THD5 (sizeof __process5_stack_base__)
#define STACK_SIZE_THD6 (sizeof __process6_stack_base__)
#define STACK_SIZE_THD7 (sizeof __process7_stack_base__)

struct thd_table {
  chopstx_prio_t prio;
  uintptr_t stack_addr;
  size_t stack_size;
};

static const struct thd_table thd_table[7] = {
  { PRIO_THD1, STACK_ADDR_THD1, STACK_SIZE_THD1 },
  { PRIO_THD2, STACK_ADDR_THD2, STACK_SIZE_THD2 },
  { PRIO_THD3, STACK_ADDR_THD3, STACK_SIZE_THD3 },
  { PRIO_THD4, STACK_ADDR_THD4, STACK_SIZE_THD4 },
  { PRIO_THD5, STACK_ADDR_THD5, STACK_SIZE_THD5 },
  { PRIO_THD6, STACK_ADDR_THD6, STACK_SIZE_THD6 },
  { PRIO_THD7, STACK_ADDR_THD7, STACK_SIZE_THD7 },
};

static struct xchg xchg[N_NminusNdiv2];

#ifdef GNU_LINUX_EMULATION
#define main emulated_main
#endif

static chopstx_t tid[N];

int
main (int argc, const char *argv[])
{
  struct sem in[N];
  struct sem out;
  struct worker_args args[N];
  int array[N_N] = {
    25, 24, 31, 30, 23, 15, 16,
    26, 20, 21, 46, 45, 40, 36,
    37, 38, 29, 27, 22, 11, 33,
    32, 35, 44, 43, 42, 41,  4,
    10,  3,  1,  9,  8,  7,  5,
    14, 18, 48, 34, 12, 28, 47,
    17, 19,  2, 39,  6,  0, 13,
  };
  int i;
  int iter;

  (void)argc;
  (void)argv;

  sem_init (&out, 0);
  for (i = 0; i < N_NminusNdiv2; i++)
    xchg_init (&xchg[i]);

  print_matrix (array, N);

  for (i = 0; i < N; i++)
    {
      const struct thd_table *thd = &thd_table[i];

      sem_init (&in[i], 0);
      args[i].row = i;
      args[i].array = &array[i*N];
      args[i].xchg = xchg;
      args[i].in = &in[i];
      args[i].out = &out;
      tid[i] = chopstx_create (thd->prio, thd->stack_addr,
			       thd->stack_size,
			       worker, &args[i]);
    }

  iter = 0;
  while (1)
    {
      for (i = 0; i < N; i++)
        sem_put (&in[i], 1);

      sem_get (&out, N);

      iter++;
      if (iter == ITERATION)
	break;
    }

  for (i = 0; i < N; i++)
    chopstx_join (tid[i], NULL);

  puts ("");
  print_matrix (array, N);
  return 0;
}
