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
#define Float_CD_AB    //else it is Float_CD_AB

// Global Variables  ///////////////////////////////////
BOOL MBC[MB_C];
BOOL MBI[MB_I];
WORD MBIR[MB_IR];
WORD MBR[MB_HR];


// Variables  ///////////////////////////////////
BYTE FC;
BOOL active = FALSE;
BOOL justReceivedOne;
WORD runs, reads, writes, totalmessageLength, messagestart, noOfBytesToSend;
BYTE byteReceiveArray[160];
BYTE byteSendArray[160];
BYTE saveArray[160];
WORD previousActivityTime = 0;
BYTE exception;


// Function Prototypes  /////////////////////////
void MBSetFC(WORD fc);

void MBRun()
{
    WORD start, wordDataLength, byteDataLength, coilDataLength, messageLength, i, j;
    static TCP_SOCKET mySocket;
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

    runs = 1 + runs * (runs < 999);

	switch(smMB)
    {
    	case SM_HOME:   // Allocate a socket for this server to listen and accept connections on
        	mySocket = TCPOpen(0, TCP_OPEN_SERVER, MB_PORT, TCP_PURPOSE_DEFAULT);
        	if(mySocket == INVALID_SOCKET)
            	return;

            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB Opened Socket");
        	smMB = SM_LISTENING;
        	break;

    	case SM_LISTENING:
            // See if anyone is connected to us
        	if(!TCPIsConnected(mySocket)) {
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

            if ( (wMaxGet=TCPIsGetReady(mySocket) > 0) ) {
                reads = 1 + reads * (reads < 999);
                

                // Transfer the data out of the TCP RX FIFO and into our local processing buffer.
            	totalmessageLength = TCPGetArray(mySocket, byteReceiveArray, sizeof(byteReceiveArray));
                noOfBytesToSend = 0;
                messagestart = 0;
        #ifdef MbDebug
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nReceived: ");
                WORD i = 0;
                for (i=0; i<totalmessageLength; i++) {
                    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, byteReceiveArray[i]);
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " ");
                }
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nmessageLength = ");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, totalmessageLength);
        #endif
                MBSetFC(byteReceiveArray[7]);  //Byte 7 of request is FC
                justReceivedOne = TRUE;
                if(!active)
                {
                    active = TRUE;
                    previousActivityTime = getTick16bit_1ms();   //Get 16-bit, 1ms tick
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb active\n");
                }
            }

        	break;

    	case SM_CLOSING:        // Close the socket connection.
            TCPClose(mySocket);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nMB Closed Socket");
        	smMB = SM_HOME;
        	break;
    }

    if (active == FALSE) {
        return;
    }

    if(getTick16bit_1ms() > (previousActivityTime + 60000))
    {
        if(active)
        {
            active = FALSE;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "Mb not active\n");
        }
    }

    while (FC != 0) {
        //**1**************** Read Coils **********************
        if(FC == 1)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            coilDataLength = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            if((start+coilDataLength > MB_C)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:1 - Read Coils - Error  EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start+coilDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "   L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_C);
                break;
            }
            byteDataLength = coilDataLength / 8;
            if(byteDataLength * 8 < coilDataLength) byteDataLength++;
            coilDataLength = byteDataLength * 8;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:1 - Read Coils S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, coilDataLength);
            MBbuffer_save();
            byteReceiveArray[5 + messagestart] = byteDataLength + 3; //number of bytes after this one.
            byteReceiveArray[8 + messagestart] = byteDataLength;     //number of bytes after this one (or number of bytes of data).
            for(i = 0; i < byteDataLength ; i++)
            {

                for(j = 0; j < 8; j++)
                {
                    bitWrite(byteReceiveArray[9 + i + messagestart], j, MBC[start + i * 8 + j]);
                }
            }
            messageLength = byteDataLength + 9;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
            MBbuffer_restore();
        }

        //**2**************** Read descrete Inputs **********************
        else if(FC == 2)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            coilDataLength = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            if((start+coilDataLength > MB_I)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:2 - Read Inputs EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start+coilDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_I);
                break;
            }
            byteDataLength = coilDataLength / 8;
            if(byteDataLength * 8 < coilDataLength) byteDataLength++;
            coilDataLength = byteDataLength * 8;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:2 - Read Inputs S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, coilDataLength);
            MBbuffer_save();
            byteReceiveArray[5 + messagestart] = byteDataLength + 3; //number of bytes after this one.
            byteReceiveArray[8 + messagestart] = byteDataLength;     //number of bytes after this one (or number of bytes of data).
            WORD i,j;
            for(i = 0; i < byteDataLength ; i++)
            {
                for(j = 0; j < 8; j++)
                {
                    bitWrite(byteReceiveArray[9 + i + messagestart], j, MBI[start + i * 8 + j]);
                }
            }
            messageLength = byteDataLength + 9;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
            MBbuffer_restore();
        }

        //**3**************** Read Holding Registers ******************
        else if(FC == 3)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            wordDataLength = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            if((start+wordDataLength > MB_HR)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:3 Read Holding Registers - EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start+wordDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_HR);
                break;
            }
            byteDataLength = wordDataLength * 2;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:3 Read Holding Registers S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, wordDataLength);
            MBbuffer_save();
            byteReceiveArray[5 + messagestart] = byteDataLength + 3; //number of bytes after this one.
            byteReceiveArray[8 + messagestart] = byteDataLength;     //number of bytes after this one (or number of bytes of data).
            WORD i;
            for(i = 0; i < wordDataLength; i++)
            {
                byteReceiveArray[ 9 + i * 2 + messagestart] = highByte(MBR[start + i]);
                byteReceiveArray[10 + i * 2 + messagestart] =  lowByte(MBR[start + i]);
            }
            messageLength = byteDataLength + 9;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
            MBbuffer_restore();
        }

        //**4**************** Read Input Registers ******************
        else if(FC == 4)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            wordDataLength = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            if((start+wordDataLength > MB_IR)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:4 Read Input Registers EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start+wordDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_IR);
                break;
            }
            byteDataLength = wordDataLength * 2;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:4 Read Input Registers S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, wordDataLength);
            MBbuffer_save();
            byteReceiveArray[5 + messagestart] = byteDataLength + 3; //number of bytes after this one.
            byteReceiveArray[8 + messagestart] = byteDataLength;     //number of bytes after this one (or number of bytes of data).
            WORD i;
            for(i = 0; i < wordDataLength; i++)
            {
                byteReceiveArray[ 9 + i * 2 + messagestart] = highByte(MBIR[start + i]);
                byteReceiveArray[10 + i * 2 + messagestart] =  lowByte(MBIR[start + i]);
            }
            messageLength = byteDataLength + 9;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
            MBbuffer_restore();
        }

        //**5**************** Write Coil **********************
        else if(FC == 5)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            if((start > MB_C)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:5 Write Coil EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_C);
                break;
            }
            MBC[start] = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]) > 0;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:5 Write Coil C");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MBC[start]);
            byteReceiveArray[5 + messagestart] = 6; //number of bytes after this one.
            messageLength = 12;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
        }

        //**6**************** Write Single Register ******************
        else if(FC == 6)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            if((start > MB_HR)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:6 Write Single Registers EOR==");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_HR);
                break;
            }
            MBR[start] = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:6 Write Single Registers R");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MBR[start]);
            byteReceiveArray[5 + messagestart] = 6; //number of bytes after this one.
            messageLength = 12;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
        }

        //**15**************** Write Multiple Coils **********************
        else if(FC == 15)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            coilDataLength = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            if((start+coilDataLength > MB_C)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:15 Write Multiple Coils EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start+coilDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_C);
                break;
            }
            byteDataLength = coilDataLength / 8;
            if(byteDataLength * 8 < coilDataLength) byteDataLength++;
            coilDataLength = byteDataLength * 8;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:15 Write Multiple Coils S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, coilDataLength);
            MBbuffer_save();
            byteReceiveArray[5 + messagestart] = 6; //number of bytes after this one.
            WORD i,j;
            for(i = 0; i < byteDataLength ; i++)
            {
                for(j = 0; j < 8; j++)
                {
                    MBC[start + i * 8 + j] = bitRead( byteReceiveArray[13 + i + messagestart], j);
                }
            }
            messageLength = 12;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
            MBbuffer_restore();
        }

        //**16**************** Write Multiple Registers ******************
        else if(FC == 16)
        {
            start = bytesToWord(byteReceiveArray[8 + messagestart],byteReceiveArray[9 + messagestart]);
            wordDataLength = bytesToWord(byteReceiveArray[10 + messagestart],byteReceiveArray[11 + messagestart]);
            if((start+wordDataLength > MB_HR)) {
                FC += 128;
                exception = 2;
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:16 Write Multiple Registers EOR=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start+wordDataLength);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
                DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, MB_HR);
                break;
            }
            byteDataLength = wordDataLength * 2;
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nFC:16 Write Multiple Registers S=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, start);
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " L=");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, wordDataLength);
            MBbuffer_save();
            byteReceiveArray[5 + messagestart] = 6; //number of bytes after this one.
            WORD i;
            for(i = 0; i < wordDataLength; i++)
            {
                MBR[start + i] =  bytesToWord(byteReceiveArray[ 13 + i * 2 + messagestart],byteReceiveArray[14 + i * 2 + messagestart]);
            }
            messageLength = 12;
            MBPopulateSendBuffer(&byteReceiveArray[messagestart], messageLength);
            writes = 1 + writes * (writes < 999);
            FC = 0;
            MBbuffer_restore();
        }

        if (FC > 128) {
            //80h + FC = exception
            MBbuffer_save();
            byteReceiveArray[7 + messagestart] += 128;  //Turn FC echo into exception
            byteReceiveArray[5 + messagestart] = 3;     //Number of bytes after this one
            byteReceiveArray[8 + messagestart] = exception; //exception Code
            MBPopulateSendBuffer(&byteReceiveArray[messagestart],9);
            FC = 0;
            MBbuffer_restore();
        }

        if (justReceivedOne) {
            messagestart = messagestart + 6 + byteReceiveArray[5 + messagestart];
            DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nNext start = ");
            DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, messagestart);
            if (messagestart+5<totalmessageLength)
                MBSetFC(byteReceiveArray[7 + messagestart]);
            else {
                justReceivedOne = false;
                FC = 0;
#ifdef MbDebug
                for (i=0; i<noOfBytesToSend; i++) {
                    DEBUG_PUT_WORD(DEBUG_LEVEL_INFO, byteSendArray[i]);
                    DEBUG_PUT_STR(DEBUG_LEVEL_INFO, " ");
                }
#endif
                TCPPutArray(mySocket, byteSendArray, noOfBytesToSend);
                TCPFlush(mySocket);
                DEBUG_PUT_STR(DEBUG_LEVEL_INFO, "\nSent");
                noOfBytesToSend = 0;
                messagestart = 0;
            }
        }  
    } 
}

void MBbuffer_save() 
{
    WORD i;
    for(i=0;i<160;i++){
        saveArray[i] = byteReceiveArray[i];
    }
}
void MBbuffer_restore() 
{
    WORD i;
    for(i=0;i<160;i++){
        byteReceiveArray[i] = saveArray[i];
    }
}


void MBPopulateSendBuffer(BYTE *SendBuffer, WORD NoOfBytes) 
{
    WORD i;
    for(i=0;i<NoOfBytes;i++){
        byteSendArray[noOfBytesToSend] = SendBuffer[i];
        noOfBytesToSend++;
    }
}
void MBSetFC(WORD fc)
{
    if(fc == 1) FC = 1;         // Read coils (FC 1) 0x
    else if(fc == 2) FC = 2;    // Read input discretes (FC 2) 1x
    else if(fc == 3) FC = 3;    // Read multiple registers (FC 3) 4x
    else if(fc == 4) FC = 4;    // Read input registers (FC 4) 3x
    else if(fc == 5) FC = 5;    // Write coil (FC 5) 0x
    else if(fc == 6) FC = 6;    // Write single register (FC 6) 4x
    else if(fc == 15) FC = 15;  // Force multiple coils (FC 15) 0x
    else if(fc == 16) FC = 16;  // Write multiple registers (FC 16) 4x
                                // Read general reference (FC 20)
                                // Write general reference (FC 21) 
                                // Mask write register (FC 22) 
                                // Read/write registers (FC 23) 
                                // Read FIFO queue (FC 24)
    // 7, 8, 11, 12, 17 are all for Serial Only
    else {
        exception = 1;
        FC = fc + 128;
        DEBUG_PUT_STR(DEBUG_LEVEL_ERROR, " FC not supported: ");
        DEBUG_PUT_WORD(DEBUG_LEVEL_ERROR, fc);
        DEBUG_PUT_STR(DEBUG_LEVEL_ERROR, "\n");
    }
}

void MB_wFloat(float f, int i){
    DWORD x = *(DWORD*)&f;      //Place Float (as data, not value) into DWORD var.
    #ifdef Float_CD_AB
        MBR[i] = lowWord(x);    //Split DWORD into WORD
        MBR[i+1] = highWord(x);
    #else
        MBR[i+1] = lowWord(x);
        MBR[i] = highWord(x);
    #endif
}

float MB_rFloat(int i){
    DWORD x;
    #ifdef Float_CD_AB
        x = wordstoDWord(MBR[i+1],MBR[i]);   //Join 2 WORDs into one DWORD
    #else
        x = wordstoDWord(MBR[i],MBR[i+1]);
    #endif
    return *(float*)&x;     //Place DWORD (as data, not value) into float var.
}
