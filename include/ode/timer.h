/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001 Russell L. Smith.            *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of the GNU Lesser General Public            *
 * License as published by the Free Software Foundation; either          *
 * version 2.1 of the License, or (at your option) any later version.    *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU      *
 * Lesser General Public License for more details.                       *
 *                                                                       *
 * You should have received a copy of the GNU Lesser General Public      *
 * License along with this library (see the file LICENSE.TXT); if not,   *
 * write to the Free Software Foundation, Inc., 59 Temple Place,         *
 * Suite 330, Boston, MA 02111-1307 USA.                                 *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_TIMER_H_
#define _ODE_TIMER_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


/* stop watch objects */

typedef struct dStopwatch {
  double time;			/* total clock count */
  unsigned long cc[2];		/* clock count since last `start' */
} dStopwatch;

void dStopwatchReset (dStopwatch *);
void dStopwatchStart (dStopwatch *);
void dStopwatchStop  (dStopwatch *);
double dStopwatchTime (dStopwatch *);	/* returns total time in secs */


/* code timers */

void dTimerStart (const char *description);	/* pass a static string here */
void dTimerNow (const char *description);	/* pass a static string here */
void dTimerEnd();

/* print out a timer report. if `average' is nonzero, print out the average
 * time for each slot (this is only meaningful if the same start-now-end
 * calls are being made repeatedly.
 */
void dTimerReport (FILE *fout, int average);


/* resolution */

/* returns the timer ticks per second implied by the timing hardware or API.
 * the actual timer resolution may not be this great.
 */
double dTimerTicksPerSecond();

/* returns an estimate of the actual timer resolution, in seconds. this may
 * be greater than 1/ticks_per_second.
 */
double dTimerResolution();


#ifdef __cplusplus
}
#endif

#endif
