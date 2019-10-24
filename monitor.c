/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*******************************************************************************
 *
 *  monitor.c - c file for serial command monitor. Interprets and sends
 *            - data to/from the host PC
 *
 ******************************************************************************/

#include "uart.h"
#include "monitor.h"
#include "utility.h"

#define RW_CMD              0x80
#define TRANSFER_SIZE_MASK  0x3f
#define BYTE_MASK           0xff
#define RW_MASK             0x40

#define ADDR_SIZE           2

// RW CMD TYPE
#define READ                1
#define WRITE               0
// Override these depends on target:
// TODO Replace hard-coded number
// CMD_BUFFER_SIZE =  5 + sizeOfMauIn8BitByte * 63
#define CMD_BUFFER_SIZE     68 // 1 + 4 + 63 = 68

unsigned char           gInCmdBuffer[CMD_BUFFER_SIZE];
unsigned short          gInCmdBufferIdx = 0;
volatile unsigned short gInCmdSkipCount;

void ClearBufferRelatedParam ();

// Override these depends on target
void
WriteByteToCOM (unsigned char c)
{
    uartTxByte (c & BYTE_MASK);
}

int
WriteToCmdBuffer (unsigned char  *buf,
                  unsigned short *bufIdx,
                  unsigned char   d)
{
    if ((*bufIdx) < CMD_BUFFER_SIZE) {
        buf[*bufIdx] = d & BYTE_MASK;
        (*bufIdx)++;
        return 0;
    }

    return 1;
}

int
WriteByteToInCmdBuffer (unsigned char d)
{
    return WriteToCmdBuffer (gInCmdBuffer, &gInCmdBufferIdx, d);
}

int
GetTransferSize () // Transfer size refer to the words to read/write of a
                   // given cmd, not the number of bytes for the whole cmd
                   // packet
{
    return (gInCmdBuffer[0] & TRANSFER_SIZE_MASK);
}

int
VerifyInputCmdHeaders ()
{
    return ((gInCmdBuffer[0] & RW_CMD) == RW_CMD) ? 0 : 1;
}

int
GetRWFlag () // Equivalent to endianness on the MAU in transmission
{
    return ((gInCmdBuffer[0] & RW_MASK) == RW_MASK) ? 1 : 0;
}

// TODO Replace with shorter addresses
unsigned char
GetInCmdAddress () // Returns a pointer to internal memory
{
    unsigned char addr = 0;
    int addressSize = ADDR_SIZE; // Always use 32bit address
    for (int i = 1; i <= addressSize; i++) {
        addr |= (unsigned long)(gInCmdBuffer[i] << 8 *
                                (addressSize - i)); // Big endian
    }

    return addr;
}

unsigned char
GetWriteCmdDataMAU (int idx)
{
    unsigned char startIdx  = 1 + ADDR_SIZE;

    unsigned char val       = 0;
    int byteOffset          = idx;

    val = gInCmdBuffer[startIdx + byteOffset];

    return val;
}

void
ClearBufferRelatedParam ()
{
    gInCmdSkipCount = 0;
    gInCmdBufferIdx = 0;
}

void
MemAccessCmd (int RW)
{
    unsigned short  MAUsToRead  = 0;
    unsigned char   dataChar    = 0;
    unsigned char   addr        = GetInCmdAddress ();

    WriteByteToCOM (gInCmdBuffer[0]);

    MAUsToRead = GetTransferSize ();
    for (unsigned short i = 0; i < MAUsToRead; i++) {
        switch (RW) {
        case READ:          // TODO Modify here to assign variables
            dataChar = *(addr + i);
            WriteByteToCOM (dataChar);
            break;
        case WRITE:
        default:
            dataChar = GetWriteCmdDataMAU (i);
            *(addr + i) = dataChar;
            break;
        }
    }
}

int
ProcessCommand ()
{
    if (VerifyInputCmdHeaders ())
        return 1;
    else
        MemAccessCmd (GetRWFlag ());

    return 0;
}

void
receivedDataCommand (unsigned char d) // Only lower byte will be used even if
                                      // MAU is bigger than 1 byte
{
    WriteByteToInCmdBuffer (d);

    if (gInCmdSkipCount > 0) {
        gInCmdSkipCount--;
        return;
    }

    if (gInCmdBufferIdx > 0 && gInCmdSkipCount == 0) {
        // Wrong input header, clear cmd buffer
        if (VerifyInputCmdHeaders ()) {
            ClearBufferRelatedParam ();
            return;
        }

        if (gInCmdBufferIdx == 1) {
            if (GetRWFlag () == WRITE)
                gInCmdSkipCount = ADDR_SIZE - 1 + GetTransferSize ();
            else
                gInCmdSkipCount = ADDR_SIZE - 1;
        } else {
            ProcessCommand ();
            ClearBufferRelatedParam ();
        }
        return;
    }
}