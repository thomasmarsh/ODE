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

/* this comes from the `reuse' library. copy any changes back to the source */

#ifndef _ODE_ERROR_H_
#define _ODE_ERROR_H_

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

/* all user defined error functions have this type. error and debug functions
 * should not return.
 */
typedef void dMessageFunction (int errnum, char *msg, va_list ap);

/* set a new error, debug or warning handler. if fn is 0, the default handlers
 * are used.
 */
void dSetErrorHandler (dMessageFunction *fn);
void dSetDebugHandler (dMessageFunction *fn);
void dSetMessageHandler (dMessageFunction *fn);

/* generate a fatal error, debug trap or a message. */
void dError (int num, char *msg, ...);
void dDebug (int num, char *msg, ...);
void dMessage (int num, char *msg, ...);


#ifdef __cplusplus
}
#endif

#endif
