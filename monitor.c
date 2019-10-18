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

#define RW_CMD              0x80

#define EXTENSION_BYTE      0x07

// RW CMD TYPE
#define READ                1
#define WRITE               0
// ENDIANNESS
#define BIG_ENDIAN          1
#define LITTLE_ENDIAN       0
// ADDRESSIBLE SIZE
#define ADDRESSIBLE_32_BIT  1
#define ADDRESSIBLE_16_BIT  0
// Override these depends on target:
// CMD_BUFFER_SIZE =  5 + sizeOfMauIn8BitByte * 63
#define CMD_BUFFER_SIZE     68 // 1 + 4 + 63 = 68

unsigned char           gInCmdBuffer[CMD_BUFFER_SIZE];
unsigned short          gInCmdBufferIdx = 0;
volatile unsigned short gInCmdSkipCount;

void ClearBufferRelatedParam ();

// Override these depends on target
int
GetTargetEndianness ()
{
    return LITTLE_ENDIAN;
}
// Override these depends on target
void
Write8bitByteToCOM (unsigned char c)
{
    uartTxByte (c & 0xff);
}

int
GetSizeOfMAUIn8bitByte ()
{
    unsigned char maxMAUValue = (unsigned char)(-1);
    switch (maxMAUValue) {
    case 0xff:
        return 1;

    case 0xffff:
        return 2;

    default:
        return 0;
    }
}

int
WriteToCmdBuffer (unsigned char  *buf,
                  unsigned short *bufIdx,
                  unsigned char   d)
{
    if ((*bufIdx) < CMD_BUFFER_SIZE) {
        buf[*bufIdx] = d & 0xff;
        (*bufIdx)++;
        return 0;
    }

    return 1;
}

void
ResetInCmdBuffer ()
{
    gInCmdBufferIdx = 0;
}

int
WriteByteToInCmdBuffer (unsigned char d)
{
    return WriteToCmdBuffer (gInCmdBuffer, &gInCmdBufferIdx, d);
}

int
GetTransferSizeInMAU () // Transfer size refer to the words to read/write of a
                        // given cmd, not the number of bytes for the whole cmd
                        // packet
{
    return (gInCmdBuffer[0] & 0x3f);
}

int
VerifyInputCmdHeaders ()
{
    return ((gInCmdBuffer[0] & 0x80) == 0x80) ? 0 : 1;
}

int
GetInputCmdType ()
{
    return (gInCmdBuffer[0] & 0x80);
}

int
GetRWFlag () // Equivalent to endianness on the MAU in transmission
{
    int ret = ((gInCmdBuffer[0] >> 6) & 0x1);
    return ret;
}

unsigned char
*GetInCmdAddress () // Returns a pointer to internal memory
{
    unsigned char *addr         = 0;
    unsigned long  addr_value   = 0;
    int addressSize = 4; // Always use 32bit address
    for (int i = 0; i < addressSize; i++) {
        addr_value |= (unsigned long)(gInCmdBuffer[1 + i] << 8 *
                                      (addressSize - 1 - i)); // Big endian
    }

    addr = (unsigned char*) addr_value;
    return addr;
}

void
WriteMAUToCOM (unsigned char d)
{
    int MAUSize = GetSizeOfMAUIn8bitByte ();

    switch (MAUSize) {
    case 1:
        Write8bitByteToCOM (d);
        break;

    case 2:
        unsigned char MAU[2];
        MAU[0] = (unsigned char)(d & 0xff);
        MAU[1] = (unsigned char)(d >> 8);
        if (GetTargetEndianness () == LITTLE_ENDIAN) {
            Write8bitByteToCOM (MAU[0]);
            Write8bitByteToCOM (MAU[1]);
        } else {
            Write8bitByteToCOM (MAU[1]);
            Write8bitByteToCOM (MAU[0]);
        }
        break;

    default: // Only handles 8bit, 16bit MAU
        break;
    }
}

unsigned char
GetWriteCmdDataMAU (int idx)
{
    unsigned char startIdx  = 1 + 4;

    unsigned char val       = 0;
    int MAUSize = GetSizeOfMAUIn8bitByte ();
    int byteOffset = idx*MAUSize;

    switch (MAUSize) {
    case 1:
        val = gInCmdBuffer[startIdx + byteOffset];
        break;

    case 2:
        if (GetTargetEndianness () == LITTLE_ENDIAN)
            val = (gInCmdBuffer[startIdx + byteOffset + 1] << 8 ) |
                  gInCmdBuffer[startIdx + byteOffset];
        else
            val = (gInCmdBuffer[startIdx + byteOffset] |
                   gInCmdBuffer[startIdx + byteOffset + 1] << 8);
        break;

    default: // Only handles 8bit, 16bit MAU
        break;
    }

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
    unsigned char  *addr        = GetInCmdAddress ();

    for (unsigned short j = 0; j < 1; j++)
        Write8bitByteToCOM (gInCmdBuffer[j]);

    MAUsToRead = GetTransferSizeInMAU ();
    for (unsigned short i = 0; i < MAUsToRead; i++) {
        // TODO Replace with switch
        if (RW == READ) {
            dataChar = *(addr + i);
            WriteMAUToCOM (dataChar);
        } else { // WRITE
            dataChar = GetWriteCmdDataMAU (i);
            *(addr + i) = dataChar;
        }
    }
}

int
ProcessCommand ()
{
    // TODO Remove redundant checks
    if (VerifyInputCmdHeaders ())
        return 1;

    switch (GetInputCmdType ()) {
    case RW_CMD:
        MemAccessCmd (GetRWFlag ());
        break;

    default:
        return 1;
    }

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

    if (gInCmdBufferIdx > 0 && gInCmdSkipCount == 0) { // Wrong input header,
                                                       // clear cmd buffer
        if (VerifyInputCmdHeaders ()) {
            ClearBufferRelatedParam ();
            return;
        }

        if (gInCmdBufferIdx == 1) {
            if (GetRWFlag () == WRITE) {
                gInCmdSkipCount = 4 - 1 + GetTransferSizeInMAU () *
                                  GetSizeOfMAUIn8bitByte ();
            } else
                gInCmdSkipCount = 4 - 1;
        } else {
            ProcessCommand ();
            ClearBufferRelatedParam ();
        }
        return;
    }
}