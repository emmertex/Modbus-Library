/*
    Modbus.c - a Modbus TCP Slave Library for Netcruzer (PIC24)
    EMMERTEX - Andrew Frahn
    https://github.com/emmertex/Modbus-Library/

    Ported from Siamects variation of Mudbus.cpp by Dee Wykoff
    Arduino Library - http://gitorious.org/mudbus



    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#define MbDebug   //extra debugging, see Modbus.c
#include <stdbool.h>
#include <stdint.h>
#ifndef Modbus_h
#define Modbus_h

#define MB_C 64         //Holding Coils (commonly 0x)
#define MB_I 16         //Input Coils (commonly 1x)
#define MB_IR 32        //Input Registers (commonly 3x)
#define MB_HR 64        //Holding Registers (commonly 4x)
#define MB_PORT 502     //Modbus Port (default is 502)


// Global Variables  ///////////////////////////////////
#if !defined(THIS_IS_MODBUS_C)
    extern BOOL MBC[MB_C];
    extern BOOL MBI[MB_I];
    extern WORD MBIR[MB_IR];
    extern WORD MBR[MB_HR];
#endif


// Function Prototypes  ///////////////////////////////////
void MBRun();
void MFSetFC(WORD fc);
void MBPopulateSendBuffer(BYTE *SendBuffer, WORD NoOfBytes);
void MBbuffer_restore();
void MBbuffer_save();
int word(BYTE a, BYTE b);
void MB_wFloat(float f, int i);
float MB_rFloat(int i);



/* Speculations on Modbus message structure:
**********************************************
**********Master(PC) request frames***********
00 ID high              0
01 ID low               1
02 Protocol high        0
03 Protocol low         0
04 Message length high  0
05 Message length low   6 (6 bytes after this)
06 Slave number         1
07 Function code
08 Start address high   maybe 0
09 Start address low    maybe 0
10 Length high          maybe 125 or Data high if write
11 Length low           maybe 125 or Data low if write
**********************************************
**********Slave response frames******
00 ID high              echo  /           0
01 ID low               echo  /  slave ID 1
02 Protocol high        echo
03 Protocol low         echo
04 Message length high  echo
05 Message length low   num bytes after this
06 Slave number         echo
07 Function code        echo for OK, 80h+FC for Exception
08 Start address high   num bytes of data for OK, else Exception Code
09 Data high
10 Data low
**********************************************
*/

/* Exception Codes
**********************************************
01 Illegal Function         Implemented
02 Illegal Data Address     Implemented
03 Illegal Data Value       Not Needed? Application Specific?
04 Failure in Device        Not Needed. Application Specific.
All others are program or special function related
**********************************************
*/



#endif	/* MODBUS_H */

