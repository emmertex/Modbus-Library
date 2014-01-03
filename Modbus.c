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
#define THIS_IS_MODBUS_C

#include "TCPIPConfig.h"
#include "TCPIP Stack/TCPIP.h"
#include <HardwareProfile.h>
#include "Modbus.h"
#include "nz_tick.h"

//Add debugging to this file. To disable all debugging:
// - Change "#else" part below to: "#define MY_DEBUG_LEVEL  DEBUG_LEVEL_OFF"
#if defined(DEBUG_LEVEL_ALLOFF)
    #define MY_DEBUG_LEVEL  0                   //Disable debugging if "DEBUG_LEVEL_ALLOFF" is defined
#else
    #define MY_DEBUG_LEVEL  DEBUG_LEVEL_INFO    //Set debug level. All debug messages with equal or higher priority will be outputted
#endif
#include "nz_debug.h"                           //Required for debugging. This include MUST be after "#define MY_DEBUG_LEVEL ..."!

// Defines  /////////////////////////////////////
#define lowByte(x)     ((BYTE)((x)&0xFF))
#define highByte(x)    ((BYTE)(((x)>>8)&0xFF))
#define lowWord(x)  ((DWORD)((x)&0xFFFF))
#define highWord(x) (((DWORD)(((x)>>16)&0xFFFF)))
#define wordstoDWord(hw,lw) ((((DWORD)(hw&0xFFFF))<<16) | ((DWORD)lw))
#define bitRead(value,bit) (((value) >> (bit)) & 0x01)
#define bitSet(value,bit) ((value) |= (1ul << (bit)))
#define bitClear(value,bit) ((value) &= ~(1ul <<(bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value,bit) : bitClear(value,bit))
#define bytesToWord(hb,lb) ( (((WORD)(hb&0xFF))<<8) | ((WORD)lb) )
#define Float_CD_AB
//#define Float_AB_CD

// Global Variables  ///////////////////////////////////
BOOL MBC[MB_C];
BOOL MBI[MB_I];
WORD MBIR[MB_IR];
WORD MBR[MB_HR];


// Variables  ///////////////////////////////////
WORD FC;
BOOL Active = FALSE;
BOOL JustReceivedOne;
WORD Runs, Reads, Writes, TotalMessageLength, MessageStart, NoOfBytesToSend;
BYTE ByteReceiveArray[160];
BYTE ByteSendArray[160];
BYTE SaveArray[160];
WORD PreviousActivityTime = 0;


// Function Prototypes  /////////////////////////
void MBSetFC(WORD fc);

void MBRun()
{
    WORD Start, WordDataLength, ByteDataLength, CoilDataLength, MessageLength, i, j;
    static TCP_SOCKET MySocket;
    #if (MY_DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
        static BOOL oldConnectionState = FALSE;
    #endif
    WORD wMaxGet;
	typedef enum
    {
    	SM_HOME = 0,
    	SM_LISTENING,
        SM_CLOSING,
    } TCPServerState;
    static TCPServerState smMB = SM_HOME;

    Runs = 1 + Runs * (Runs < 999);

	switch(smMB)
    {
    	case SM_HOME:
            // Allocate a socket for this server to listen and accept connections on
        	MySocket = TCPOpen(0, TCP_OPEN_SERVER, MB_PORT, TCP_PURPOSE_DEFAULT);
        	if(MySocket == INVALID_SOCKET)
            	return;

            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB Opened Socket");
        	smMB = SM_LISTENING;
        	break;

    	case SM_LISTENING:
            // See if anyone is connected to us
        	if(!TCPIsConnected(MySocket)) {
                #if (MY_DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
                if (oldConnectionState == TRUE) {
                    oldConnectionState = FALSE;
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB Disconnected");
                }
                #endif
            	return;
            }

            #if (MY_DEBUG_LEVEL >= DEBUG_LEVEL_INFO)
            if (oldConnectionState == FALSE) {
                oldConnectionState = TRUE;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB Connected");
            }
            #endif

            if ( (wMaxGet=TCPIsGetReady(MySocket) > 0) ) {
                Reads = 1 + Reads * (Reads < 999);
                

                // Transfer the data out of the TCP RX FIFO and into our local processing buffer.
            	TotalMessageLength = TCPGetArray(MySocket, ByteReceiveArray, sizeof(ByteReceiveArray));
                NoOfBytesToSend = 0;
                MessageStart = 0;
        #ifdef MbDebug
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nReceived: ");
                WORD i = 0;
                for (i=0; i<TotalMessageLength; i++) {
                    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, ByteReceiveArray[i]);
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " ");
                }
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMessageLength = ");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, TotalMessageLength);
        #endif
                MBSetFC(ByteReceiveArray[7]);  //Byte 7 of request is FC
                JustReceivedOne = TRUE;
                if(!Active)
                {
                    Active = TRUE;
                    PreviousActivityTime = getTick16bit_1ms();   //Get 16-bit, 1ms tick
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb active\n");
                }
            }

            // No need to perform any flush.  TCP data in TX FIFO will automatically transmit itself after
            //it accumulates for a while.  If you want to decrease latency (at the expense of wasting network
            //bandwidth on TCP overhead), perform and explicit flush via the TCPFlush() API.
            //TCPFlush();

        	break;

//        case SM_RESPONDING:
//            smMB = SM_LISTENING;
//            break;

    	case SM_CLOSING:
            // Close the socket connection.
            TCPClose(MySocket);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB Closed Socket");
        	smMB = SM_HOME;
        	break;
    }

    if (Active == FALSE) {
        return;
    }

    if(getTick16bit_1ms() > (PreviousActivityTime + 60000))
    {
        if(Active)
        {
            Active = FALSE;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb not active\n");
        }
    }

    while (FC != 0) {
        //**1**************** Read Coils **********************
        if(FC == MB_FC_READ_COILS_0x)
        {
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            if((Start+CoilDataLength > MB_C)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_COILS_0x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start+CoilDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_C);
                break;
            }
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_COILS_0x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, CoilDataLength);
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
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            if((Start+CoilDataLength > MB_I)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_INPUTS_1x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start+CoilDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_I);
                break;
            }
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_INPUTS_1x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, CoilDataLength);
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            WORD i,j;
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
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            if((Start+WordDataLength > MB_HR)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_REGISTERS_4x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start+WordDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_HR);
                break;
            }
            ByteDataLength = WordDataLength * 2;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_REGISTERS_4x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, WordDataLength);
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            WORD i;
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
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            if((Start+WordDataLength > MB_IR)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_INPUT_REGISTERS_3x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start+WordDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_IR);
                break;
            }
            ByteDataLength = WordDataLength * 2;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_READ_INPUT_REGISTERS_3x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, WordDataLength);
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = ByteDataLength + 3; //number of bytes after this one.
            ByteReceiveArray[8 + MessageStart] = ByteDataLength;     //number of bytes after this one (or number of bytes of data).
            WORD i;
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
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            if((Start > MB_C)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_COIL_0x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Alocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_C);
                break;
            }
            MBC[Start] = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]) > 0;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_COIL_0x C");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MBC[Start]);
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**6**************** Write Single Register ******************
        else if(FC == MB_FC_WRITE_REGISTER_4x)
        {
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            if((Start > MB_HR)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_REGISTER_4x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_HR);
                break;
            }
            MBR[Start] = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_REGISTER_4x R");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MBR[Start]);
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
        }

        //**15**************** Write Multiple Coils **********************
        else if(FC == MB_FC_WRITE_MULTIPLE_COILS_0x)
        {
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            CoilDataLength = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            if((Start+CoilDataLength > MB_C)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_MULTIPLE_COILS_0x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start+CoilDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_C);
                break;
            }
            ByteDataLength = CoilDataLength / 8;
            if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;
            CoilDataLength = ByteDataLength * 8;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_MULTIPLE_COILS_0x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, CoilDataLength);
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            WORD i,j;
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
            Start = bytesToWord(ByteReceiveArray[8 + MessageStart],ByteReceiveArray[9 + MessageStart]);
            WordDataLength = bytesToWord(ByteReceiveArray[10 + MessageStart],ByteReceiveArray[11 + MessageStart]);
            if((Start+WordDataLength > MB_HR)) {
                FC = 0;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_MULTIPLE_REGISTERS_4x Invalid Request   End of Request=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start+WordDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   Last Allocated Address=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_HR);
                break;
            }
            ByteDataLength = WordDataLength * 2;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB_FC_WRITE_MULTIPLE_REGISTERS_4x S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, WordDataLength);
            MBbuffer_save();
            ByteReceiveArray[5 + MessageStart] = 6; //number of bytes after this one.
            WORD i;
            for(i = 0; i < WordDataLength; i++)
            {
                MBR[Start + i] =  bytesToWord(ByteReceiveArray[ 13 + i * 2 + MessageStart],ByteReceiveArray[14 + i * 2 + MessageStart]);
            }
            MessageLength = 12;
            MBPopulateSendBuffer(&ByteReceiveArray[MessageStart], MessageLength);
            Writes = 1 + Writes * (Writes < 999);
            FC = MB_FC_NONE;
            MBbuffer_restore();
        }

        if (JustReceivedOne) {
            MessageStart = MessageStart + 6 + ByteReceiveArray[5 + MessageStart];
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nNext start = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MessageStart);
            if (MessageStart+5<TotalMessageLength)
                MBSetFC(ByteReceiveArray[7 + MessageStart]);
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
                TCPPutArray(MySocket, ByteSendArray, NoOfBytesToSend);
                TCPFlush(MySocket);
//                smMB = SM_RESPONDING;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nSent");
                NoOfBytesToSend = 0;
                MessageStart = 0;
            }
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nTotalMessageLength = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, TotalMessageLength);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMessageStart = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MessageStart);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " FC = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, FC);
        }   // if (JustReceivedOne)
    }   // while (FC != 0)

//    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMb runs: ");
//    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Runs);
//    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "  reads: ");
//    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Reads);
//    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "  writes: ");
//    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, Writes);
}

void MBbuffer_save()
{
    WORD i;
    i=0;
    while(i<160)
    {
        SaveArray[i] = ByteReceiveArray[i];
        i++;
    }
}
void MBbuffer_restore()
{
    WORD i;
    i=0;
    while(i<160)
    {
        ByteReceiveArray[i] = SaveArray[i];
        i++;
    }
}


void MBPopulateSendBuffer(BYTE *SendBuffer, WORD NoOfBytes)
{
    WORD i;
    i=0;
    while(i<NoOfBytes)
    {
        ByteSendArray[NoOfBytesToSend] = SendBuffer[i];
        NoOfBytesToSend++;
        i++;

    }
}
void MBSetFC(WORD fc)
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

void MB_wFloat(float f, int i){
    DWORD x = *(DWORD*)&f;   //Place Float (as data, not value) into DWORD var.
    #ifdef Float_CD_AB
        MBR[i] = lowWord(x);      //Split DWORD into WORD
        MBR[i+1] = highWord(x);
    #endif

    #ifdef Float_AB_CD
        MBR[i+1] = lowWord(x);
        MBR[i] = highWord(x);
    #endif
}

float MB_rFloat(int i){
    DWORD x;
    #ifdef Float_CD_AB
        x = wordstoDWord(MBR[i+1],MBR[i]);   //Join 2 WORDs into one DWORD
    #endif

    #ifdef Float_AB_CD
        x = wordstoDWord(MBR[i],MBR[i+1]);
    #endif
    return *(float*)&x;     //Place DWORD (as data, not value) into float var.
}
