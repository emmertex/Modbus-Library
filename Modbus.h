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

//#define MbDebug

    // Removed - Arduino Libraries
//#include "Arduino.h"
#include <stdbool.h>
#include <stdint.h>
#ifndef Modbus_h
#define Modbus_h

/*
Below are the limits of the number of accessible registers, coils and inputs.

Observe that there are no checking if you try to access anything outside the limits.
If you for example try to read the holding register on address 65 you will simply get
the value of a memory location used for something else.

If you try to write to anything outside the limits, disaster will strike...
DON'T DO THAT!!!

*/

#define MB_N_C_0x 8
#define MB_N_I_1x 8
#define MB_N_IR_3x 64
#define MB_N_HR_4x 64
#define MB_PORT 502


enum MB_FC {
    MB_FC_NONE                        = 0,
    MB_FC_READ_COILS_0x               = 1,
    MB_FC_READ_INPUTS_1x              = 2,
    MB_FC_READ_REGISTERS_4x           = 3,
    MB_FC_READ_INPUT_REGISTERS_3x     = 4,
    MB_FC_WRITE_COIL_0x               = 5,
    MB_FC_WRITE_REGISTER_4x           = 6,
    MB_FC_WRITE_MULTIPLE_COILS_0x     = 15,
    MB_FC_WRITE_MULTIPLE_REGISTERS_4x = 16
};

    void MBRun();
    void MFSetFC(WORD fc);
    void MBPopulateSendBuffer(BYTE *SendBuffer, WORD NoOfBytes);
    void MBbuffer_restore();
    void MBbuffer_save();
    int word(BYTE a, BYTE b);



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
**********Slave(Arduino) response frames******
00 ID high              echo  /           0
01 ID low               echo  /  slave ID 1
02 Protocol high        echo
03 Protocol low         echo
04 Message length high  echo
05 Message length low   num bytes after this
06 Slave number         echo
07 Function code        echo
08 Start address high   num bytes of data
09 Data high
10 Data low
**********************************************
*/


#endif	/* MODBUS_H */

