#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include <chopstx.h>

static int
increasing (int a, int b)
{
  return a < b ? 1 : 0;
}

static int
decreasing (int a , int b)
{
  return a > b ? 1 : 0;
}

static void
bbl_sort (int array[], int n,
	  int (*cmp) (int, int))
{
  int i;
  int new;
  do
    {
      new = 0;
      for (i = 1; i < n; i++)
        if (cmp (array[i-1], array[i]))
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
  chopstx_mutex_unlock(&sem->mutex);
}

static void
sem_put (struct sem *sem, int count)
{
  chopstx_mutex_lock (&sem->mutex);
  sem->count += count;
  chopstx_cond_signal (&sem->avail);
  chopstx_mutex_unlock (&sem->mutex);
}

static void
transpose (int array[], int n)
{
  int i, j;

  for (i = 0; i < n; i++)
    for (j = i + 1; j < n; j++)
      {
        int t = array[i * n + j];
        array[i * n + j] = array[j * n + i];
        array[j * n + i] = t;
      }
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

struct worker_args {
  int row;
  int n;
  int *array;
  struct sem *in;
  struct sem *out;
};

#define N 7
#define N_N (N*N)
#define ITERATION (32 - 26 +  1)
/* ((sizeof (int) * 8) - __builtin_clz (N_N)) + 1 */

static void *
worker (void *args0)
{
  struct worker_args *args = args0;
  int row = args->row;
  int n = args->n;
  int *array = args->array;
  int iter;

  for (iter = 0; iter < ITERATION; iter++)
    {
      int (*cmp) (int, int);

      sem_get (args->in, 1);

      if ((iter & 1))
	cmp = decreasing;
      else if ((row & 1))
	cmp = increasing;
      else
	cmp = decreasing;

      bbl_sort (&array[row * n], n, cmp);

      sem_put (args->out, 1);
    }

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

#ifdef GNU_LINUX_EMULATION
#define main emulated_main
#endif

int
main (int argc, const char *argv[])
{
  chopstx_t tid[N_N];
  struct sem in[N_N];
  struct sem out;
  struct worker_args args[N_N];
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

  print_matrix (array, N);

  for (i = 0; i < N; i++)
    {
      const struct thd_table *thd = &thd_table[i];

      sem_init (&in[i], 0);
      args[i].row = i;
      args[i].array = array;
      args[i].n = N;
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

      puts ("");
      print_matrix (array, N);

      iter++;
      if (iter == ITERATION)
	break;

      transpose (array, N);
    }

  for (i = 0; i < N; i++)
    chopstx_join (tid[i], NULL);

  return 0;
}
