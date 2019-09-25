/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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
 *  registers.h - DRV8711 Registers
 *  Micael Jarniac
 *  LaunchPad Stepper
 *  25/09/2019
 *
 ******************************************************************************/

#ifndef REGISTERS_H_
#define REGISTERS_H_

// CTRL register
struct CTRL_Register
{
    unsigned int Address;   // Bits 14-12
    unsigned int DTIME;     // Bits 11-10
    unsigned int ISGAIN;    // Bits 9-8
    unsigned int EXSTALL;   // Bit 7
    unsigned int MODE;      // Bits 6-3
    unsigned int RSTEP;     // Bit 2
    unsigned int RDIR;      // Bit 1
    unsigned int ENBL;      // Bit 0
};

// TORQUE register
struct TORQUE_Register
{
    unsigned int Address;   // Bits 14-12
    /* Reserved */          // Bit 11
    unsigned int SIMPLTH;   // Bits 10-8
    unsigned int TORQUE;    // Bits 7-0
};

// OFF register
struct OFF_Register
{
    unsigned int Address;   // Bits 14-12
    /* Reserved */          // Bits 11-9
    unsigned int PWMMODE;   // Bit 8
    unsigned int TOFF;      // Bits 7-0
};

// BLANK register
struct BLANK_Register
{
    unsigned int Address;   // Bits 14-12
    /* Reserved */          // Bits 11-9
    unsigned int ABT;       // Bit 8
    unsigned int TBLANK;    // Bits 7-0
};

// DECAY register
struct DECAY_Register
{
    unsigned int Address;   // Bits 14-12
    /* Reserved */          // Bit 11
    unsigned int DECMOD;    // Bits 10-8
    unsigned int TDECAY;    // Bits 7-0
};

// STALL register
struct STALL_Register
{
    unsigned int Address;   // Bits 14-12
    unsigned int VDIV;      // Bits 11-10
    unsigned int SDCNT;     // Bits 9-8
    unsigned int SDTHR;     // Bits 7-0
};

// DRIVE register
struct DRIVE_Register
{
    unsigned int Address;   // Bits 14-12
    unsigned int IDRIVEP;   // Bits 11-10
    unsigned int IDRIVEN;   // Bits 9-8
    unsigned int TDRIVEP;   // Bits 7-6
    unsigned int TDRIVEN;   // Bits 5-4
    unsigned int OCPDEG;    // Bits 3-2
    unsigned int OCPTH;     // Bits 1-0
};

// STATUS register
struct STATUS_Register
{
    unsigned int Address;   // Bits 14-12
    /* Reserved */          // Bits 11-8
    unsigned int STDLAT;    // Bit 7
    unsigned int STD;       // Bit 6
    unsigned int UVLO;      // Bit 5
    unsigned int BPDF;      // Bit 4
    unsigned int APDF;      // Bit 3
    unsigned int BOCP;      // Bit 2
    unsigned int AOCP;      // Bit 1
    unsigned int OTS;       // Bit 0
};

#endif /* REGISTERS_H_ */