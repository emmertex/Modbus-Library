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

#include "TCPIPConfig.h"
#include "TCPIP Stack/TCPIP.h"
#include <HardwareProfile.h>
#include "nz_debug.h"
#include "Modbus.h"

// Porting Defines (Arduino to XC8)
 
#define lowByte(x)     ((unsigned char)((x)&0xFF))
#define highByte(x)    ((unsigned char)(((x)>>8)&0xFF))
#define bitRead(value,bit) (((value) >> (bit)) & 0x01)
#define bitSet(value,bit) ((value) |= (1ul << (bit)))
#define bitClear(value,bit) ((value) &= ~(1ul <<(bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value,bit) : bitClear(value,bit))
int FC;
#define MbDebug
#define MbRunsDebug

#if !defined(DEBUG_CONF_MODBUS)
  #define DEBUG_CONF_MODBUS     4
  #endif
  #define MY_DEBUG_LEVEL   DEBUG_CONF_MODBUS


long unsigned int getTick16bit_1ms();
void MBSetFC(int fc);

void MBRun()
{
    Runs = 1 + Runs * (Runs < 999);
/*
    //****************** Read from socket ****************
    EthernetClient client = MbServer.available();
    if(client.available())
    {
        Reads = 1 + Reads * (Reads < 999);
        int i = 0;
        while(client.available())
        {
            ByteReceiveArray[i] = client.read();
            i++;

        }
        TotalMessageLength = i;
        NoOfBytesToSend = 0;
        MessageStart = 0;
#ifdef MbDebug
        for (i=0; i<TotalMessageLength; i++) {
            //DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, ByteReceiveArray[i],HEX);
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, ByteReceiveArray[i]);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " ");
        }
        DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " received\n");
        DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "MessageLength = ");
        DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, TotalMessageLength);
        DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
        MBSetFC(ByteReceiveArray[7]);  //Byte 7 of request is FC
        JustReceivedOne = true;
        if(!Active)
        {
            Active = true;
            PreviousActivityTime = getTick16bit_1ms();
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb active\n");
#endif
        }
    }
    */

    if(getTick16bit_1ms() > (PreviousActivityTime + 60000))
    {
        if(Active)
        {
            Active = false;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb not active\n");
#endif
        }
    }

    int Start, WordDataLength, ByteDataLength, CoilDataLength, MessageLength, i, j;

    while (FC != 0) {
        //**1**************** Read Coils **********************
        if(FC == MB_FC_READ_COILS_0x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_READ_COILS_0x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, CoilDataLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            for(i = 0; i < ByteDataLength ; i++)
            {

                for(j = 0; j < 8; j++)
                {
                    bitWrite(ByteReceiveArray[9 + i + MessageStart], j, MBC[Start + i * 8 + j]);
                }
            }
            MessageLength = ByteDataLength + 9;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
        }

        //**2**************** Read descrete Inputs **********************
        else if(FC == MB_FC_READ_INPUTS_1x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_READ_INPUTS_1x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, CoilDataLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            int i,j;
            for(i = 0; i < ByteDataLength ; i++)
            {
                for(j = 0; j < 8; j++)
                {
                    bitWrite(ByteReceiveArray[9 + i + MessageStart], j, MBI[Start + i * 8 + j]);
                }
            }
            MessageLength = ByteDataLength + 9;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
        }

        //**3**************** Read Holding Registers ******************
        else if(FC == MB_FC_READ_REGISTERS_4x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = WordDataLength * 2;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_READ_REGISTERS_4x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, WordDataLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            int i;
            for(i = 0; i < WordDataLength; i++)
            {
                ByteReceiveArray[ 9 + i * 2 + MessageStart] = highByte(MBR[Start + i]);
                ByteReceiveArray[10 + i * 2 + MessageStart] =  lowByte(MBR[Start + i]);
            }
            MessageLength = ByteDataLength + 9;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
        }

        //**4**************** Read Input Registers ******************
        else if(FC == MB_FC_READ_INPUT_REGISTERS_3x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = WordDataLength * 2;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_READ_INPUT_REGISTERS_3x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, WordDataLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            int i;
            for(i = 0; i < WordDataLength; i++)
            {
                ByteReceiveArray[ 9 + i * 2 + MessageStart] = highByte(MBIR[Start + i]);
                ByteReceiveArray[10 + i * 2 + MessageStart] =  lowByte(MBIR[Start + i]);
            }
            MessageLength = ByteDataLength + 9;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
        }



        //**5**************** Write Coil **********************
        else if(FC == MB_FC_WRITE_COIL_0x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            MBC[Start] = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]) > 0;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_WRITE_COIL_0x C");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MBC[Start]);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**6**************** Write Single Register ******************
        else if(FC == MB_FC_WRITE_REGISTER_4x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            MBR[Start] = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_WRITE_REGISTER_4x R");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MBR[Start]);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }


        //**15**************** Write Multiple Coils **********************
        else if(FC == MB_FC_WRITE_MULTIPLE_COILS_0x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_WRITE_MULTIPLE_COILS_0x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, CoilDataLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            int i,j;
            for(i = 0; i < ByteDataLength ; i++)
            {
                for(j = 0; j < 8; j++)
                {
                    MBC[Start + i * 8 + j] = bitRead( ByteReceiveArray[13 + i + MessageStart], j);
                }
            }
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
        }


        //**16**************** Write Multiple Registers ******************
        else if(FC == MB_FC_WRITE_MULTIPLE_REGISTERS_4x)
        {
            Start = word(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = word(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            ByteDataLength = WordDataLength * 2;
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " MB_FC_WRITE_MULTIPLE_REGISTERS_4x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, WordDataLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            int i;
            for(i = 0; i < WordDataLength; i++)
            {
                MBR[Start + i] =  word(ByteReceiveArray[ 13 + i * 2 + MessageStart],ByteReceiveArray[14 + i * 2 + MessageStart]);
            }
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
            }


        if (JustReceivedOne) {
            
            MessageStart = MessageStart + 6 + ByteReceiveArray[5 + MessageStart];
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n Next start = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MessageStart);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif
            if (MessageStart+5<TotalMessageLength) MBSetFC(ByteReceiveArray[7 + MessageStart]);
            else {
                JustReceivedOne = false;
                FC = MB_FC_NONE;

#ifdef MbDebug
                for (i=0; i<NoOfBytesToSend; i++) {
                    //DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, ByteSendArray[i],HEX);
                    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, ByteSendArray[i]);
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " ");
                }
#endif
 ///////              client.write(ByteSendArray,NoOfBytesToSend);
#ifdef MbDebug
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " sent\n");
#endif
                NoOfBytesToSend = 0;
                MessageStart = 0;
            }
#ifdef MbDebug
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "TotalMessageLength = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, TotalMessageLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n MessageStart = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MessageStart);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " FC = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, FC);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");


#endif
        }

    }

#ifdef MbRunsDebug
    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb runs: ");
    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Runs);
    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "  reads: ");
    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Reads);
    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "  writes: ");
    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Writes);
    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\n");
#endif

}

void MBbuffer_save()
{
    int i;
    i=0;
    while(i<160)
    {
        SaveArray[i] = ByteReceiveArray[i];
        i++;
    }
}
void MBbuffer_restore()
{
    int i;
    i=0;
    while(i<160)
    {
        ByteReceiveArray[i] = SaveArray[i];
        i++;
    }
}


void MBPopulateSendBuffer(uint8_t *SendBuffer, int NoOfBytes)
{
    int i;
    i=0;
    while(i<NoOfBytes)
    {
        ByteSendArray[NoOfBytesToSend] = SendBuffer[i];
        NoOfBytesToSend++;
        i++;

    }
}
void MBSetFC(int fc)
{

// Read coils (FC 1) 0x
    if(fc == 1) FC = MB_FC_READ_COILS_0x;

// Read input discretes (FC 2) 1x
    else if(fc == 2) FC = MB_FC_READ_INPUTS_1x;

// Read multiple registers (FC 3) 4x
    else if(fc == 3) FC = MB_FC_READ_REGISTERS_4x;

// Read input registers (FC 4) 3x
    else if(fc == 4) FC = MB_FC_READ_INPUT_REGISTERS_3x;

// Write coil (FC 5) 0x
    else if(fc == 5) FC = MB_FC_WRITE_COIL_0x;

// Write single register (FC 6) 4x
    else if(fc == 6) FC = MB_FC_WRITE_REGISTER_4x;

// Read exception status (FC 7) we skip this one

// Force multiple coils (FC 15) 0x
    else if(fc == 15) FC = MB_FC_WRITE_MULTIPLE_COILS_0x;

// Write multiple registers (FC 16) 4x
    else if(fc == 16) FC = MB_FC_WRITE_MULTIPLE_REGISTERS_4x;

// Read general reference (FC 20)  we skip this one

// Write general reference (FC 21)  we skip this one

// Mask write register (FC 22)  we skip this one

// Read/write registers (FC 23)  we skip this one

// Read FIFO queue (FC 24)  we skip this one
    else {
       DEBUG_PUT_STR(DEBUG_LEVEL_ERROR, " FC not supported: ");
       DEBUG_PUT_WORD(DEBUG_LEVEL_ERROR, fc);
       DEBUG_PUT_STR(DEBUG_LEVEL_ERROR, "\n");

    }
}



// Add Arduino Function

int word(uint8_t high, uint8_t low)
{
    int result;
    result = (high * 256) + low;
    return result;
}