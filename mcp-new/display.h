/******************************************************************************/
/*
Copyright (C) Fermi National Accelerator Laboratory
Filename: display.h
Revision: 1.1
Date and Time: 04/04/96 13:46:48
*/
/******************************************************************************/
/*
 display.h
 Display Screen Functions
 Jeff Utterback
 originated 3/3/94

These functions can be used with a Vt220 terminal to position the cursor,
erase the screen etc.

*/
/******************************************************************************/
#ifndef Display_H
#define Display_H
/******************************************************************************/
/* DEFINES */
/******************************************************************************/
/* TYPES */
/******************************************************************************/
/* GLOBAL VARS */
/******************************************************************************/
/* FUNCTION PROTOTYPES */
/******************************************************************************/
void EraseDisplayAll();
void EraseDisplayRest();
void CursPos(int X,int Y);
void SaveCursPos();
void RestoreCursPos();
char GetCharNoEcho();
/******************************************************************************/
#endif     
