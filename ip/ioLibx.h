/* ioLibx.h -- ioLib.h header file extension 
*
* Version: "%W%    %G% TSL"
*
* Copyright (c) 1994 The Svedberg Laboratory.
*/

#pragma ident "%W%    %G% TSL"

/*
modification history
--------------------
940422,LT	Written
*/

/*
DESCRIPTION
Use this header file instead of ioLib.h to include definitions used for
the TSL extension of the tty ioctl function.
*/

#ifndef __INCioLibxh
#define __INCioLibxh

#ifdef __cplusplus
extern "C" {
#endif

#include "ioLib.h"

#define FIODATABITS     100              /* Set serial data size */
#define FIOPARITY       101              /* Set serial parity */
#define FIOFMODE		102              /* Set FIFO mode on/off (0/1) */

#ifdef __cplusplus
}
#endif

#endif /* __INCioLibxh */
