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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "ode/error.h"


static dMessageFunction *error_function = 0;
static dMessageFunction *debug_function = 0;
static dMessageFunction *message_function = 0;


extern "C" void dSetErrorHandler (dMessageFunction *fn)
{
  error_function = fn;
}


extern "C" void dSetDebugHandler (dMessageFunction *fn)
{
  debug_function = fn;
}


extern "C" void dSetMessageHandler (dMessageFunction *fn)
{
  message_function = fn;
}


extern "C" dMessageFunction *dGetErrorHandler()
{
  return error_function;
}


extern "C" dMessageFunction *dGetDebugHandler()
{
  return debug_function;
}


extern "C" dMessageFunction *dGetMessageHandler()
{
  return message_function;
}


static void printMessage (int num, const char *msg1, const char *msg2,
			  va_list ap)
{
  fflush (stderr);
  fflush (stdout);
  if (num) fprintf (stderr,"\n%s %d: ",msg1,num);
  else fprintf (stderr,"\n%s: ",msg1);
  vfprintf (stderr,msg2,ap);
  fprintf (stderr,"\n");
  fflush (stderr);
}

//****************************************************************************
// unix

#ifndef WIN32

extern "C" void dError (int num, const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  if (error_function) error_function (num,msg,ap);
  else printMessage (num,"ODE Error",msg,ap);
  exit (1);
}


extern "C" void dDebug (int num, const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  if (debug_function) debug_function (num,msg,ap);
  else printMessage (num,"ODE INTERNAL ERROR",msg,ap);
  // *((char *)0) = 0;   ... commit SEGVicide
  abort();
}


extern "C" void dMessage (int num, const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  if (message_function) message_function (num,msg,ap);
  else printMessage (num,"ODE Message",msg,ap);
}

#endif

//****************************************************************************
// windows

#ifdef WIN32

#include "windows.h"


extern "C" void dError (int num, const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  if (error_function) error_function (num,msg,ap);
  else {
    char s[1000],title[100];
    _snprintf (title,sizeof(title),"ODE Error %d",num);
    _vsnprintf (s,sizeof(s),msg,ap);
    s[sizeof(s)-1] = 0;
    MessageBox(0,s,title,MB_OK | MB_ICONWARNING);
  }
  exit (1);
}


extern "C" void dDebug (int num, const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  if (debug_function) debug_function (num,msg,ap);
  else {
    char s[1000],title[100];
    _snprintf (title,sizeof(title),"ODE INTERNAL ERROR %d",num);
    _vsnprintf (s,sizeof(s),msg,ap);
    s[sizeof(s)-1] = 0;
    MessageBox(0,s,title,MB_OK | MB_ICONSTOP);
  }
  abort();
}


extern "C" void dMessage (int num, const char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  if (message_function) message_function (num,msg,ap);
  else printMessage (num,"ODE Message",msg,ap);
}


#endif
