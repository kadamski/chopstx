/*
 * ackbtn-gnu-linux.c - Acknowledge button support for GNU/Linux
 *
 * Copyright (C) 2021  Free Software Initiative of Japan
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

#include <pthread.h>

#include <unistd.h>

#include <stdint.h>
#include <string.h>
#include <chopstx.h>

#include <stdio.h>
#include <signal.h>
#include <sys/eventfd.h>
#include <poll.h>
#include <errno.h>
#include <stdlib.h>

static pthread_t tid_main;
static pthread_t tid_ui;

static pthread_mutex_t mutex;
static pthread_cond_t cond;
static int enabled;
static int event_fd;

static void
ackbtn_intr (int signum, siginfo_t *siginfo, void *arg)
{
  extern void chx_sigmask (ucontext_t *uc);
  extern void chx_handle_intr (int signum);

  ucontext_t *uc = arg;
  (void)signum;
  (void)siginfo;
  chx_handle_intr (SIGUSR2);
  chx_sigmask (uc);
}


#define ACKBTN_ACKED   (1 << 0)
#define ACKBTN_TIMEOUT (1 << 2)

static void *
user_interaction (void *arg)
{
  struct pollfd pollfds[2];

  (void)arg;

  pollfds[0].fd = 0;		/* standard input */
  pollfds[0].events = POLLIN;

  pollfds[1].fd = event_fd;
  pollfds[1].events = POLLIN;

  fputs ("User interaction thread for AckBtn started.\n", stdout);

  while (1)
    {
      unsigned int acked_or_timeout = 0;
      char buf[256];

      pthread_mutex_lock (&mutex);
      while (!enabled)
	pthread_cond_wait (&cond, &mutex);
      pthread_mutex_unlock (&mutex);

      /* Consume all input if any.  */
      while (1)
	{
	  int r;

	  pollfds[0].revents = 0;

	  r = poll (pollfds, 1, 0);
	  if (r < 0)
	    {
	      if (errno == EINTR)
		continue;

	      perror ("poll");
	      exit (1);
	    }

	  if (r == 0)
	    break;

	  if ((pollfds[0].revents & POLLIN))
	    read (0, buf, sizeof buf);
	}

      fputs ("Type RET to acknowledge (or wait for timeout) > ", stdout);
      fflush (stdout);

      while (!acked_or_timeout)
	{
	  pollfds[0].revents = 0;
	  pollfds[1].revents = 0;

	  if (poll (pollfds, 2, -1) < 0)
	    {
	      if (errno == EINTR)
		continue;

	      perror ("poll");
	      exit (1);
	    }

	  if ((pollfds[0].revents & POLLIN))
	    {
	      read (0, buf, sizeof buf);
	      acked_or_timeout |= ACKBTN_ACKED;
	    }

	  if ((pollfds[1].revents & POLLIN))
	    acked_or_timeout |= ACKBTN_TIMEOUT;
	}

      pthread_mutex_lock (&mutex);
      if ((acked_or_timeout & ACKBTN_ACKED))
	{
	  if ((acked_or_timeout & ACKBTN_TIMEOUT))
	    /* No output of newline, as it follows timeout message.  */
	    fputs ("Input ignored", stdout);
	  else
	    {
	      pthread_kill (tid_main, SIGUSR2);
	      fputs ("Acknowledged\n", stdout);
	    }
	}

      if ((acked_or_timeout & ACKBTN_TIMEOUT))
	fputs ("\nTimeout\n", stdout);

      while (enabled)
	pthread_cond_wait (&cond, &mutex);

      read (event_fd, buf, sizeof (uint64_t));
      pthread_mutex_unlock (&mutex);
    }
}

void
ackbtn_init (chopstx_intr_t *intr)
{
  int r;
  sigset_t sigset;
  struct sigaction act;

  event_fd = eventfd (0, EFD_CLOEXEC);
  if (event_fd < 0)
    {
      perror ("eventfd");
      exit (1);
    }

  pthread_mutex_init (&mutex, NULL);
  pthread_cond_init (&cond, NULL);
  enabled = 0;

  sigemptyset (&sigset);
  sigaddset (&sigset, SIGUSR2);
  sigaddset (&sigset, SIGALRM);

  tid_main = pthread_self ();

  /* Launch the thread for user interaction.  */
  pthread_sigmask (SIG_BLOCK, &sigset, NULL);

  r = pthread_create (&tid_ui, NULL, user_interaction, NULL);
  if (r)
    {
      fprintf (stderr, "ackbtn_init: %s\n", strerror (r));
      exit (1);
    }

  act.sa_sigaction = ackbtn_intr;
  sigfillset (&act.sa_mask);
  act.sa_flags = SA_SIGINFO|SA_RESTART;
  sigaction (SIGUSR2, &act, NULL);

  pthread_sigmask (SIG_UNBLOCK, &sigset, NULL);

  chopstx_claim_irq (intr, SIGUSR2);
}

void
ackbtn_enable (void)
{
  pthread_mutex_lock (&mutex);
  enabled = 1;
  pthread_cond_signal (&cond);
  pthread_mutex_unlock (&mutex);
}

void
ackbtn_disable (void)
{
  const uint64_t l = 1;

  pthread_mutex_lock (&mutex);
  enabled = 0;
  write (event_fd, &l, sizeof (l));
  pthread_cond_signal (&cond);
  pthread_mutex_unlock (&mutex);
}
