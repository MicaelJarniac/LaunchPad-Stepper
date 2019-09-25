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
 *  utility.c - Definition file for utility functions
 *  Micael Jarniac
 *  LaunchPad Stepper
 *  25/09/2019
 *
 ******************************************************************************/

#include "msp430.h"
#include "utility.h"
#include "monitor.h"
#include "uart.h"
#include "registers.h"

/******************************************************************************/

// Define global variables

// GUI variables
float           G_FIRMWARE_VERSION      = 1.0;
float           G_FULL_SCALE_CURRENT    = 1.0;
int             G_TORQUE_OLD            = 0;
int             G_ISGAIN_OLD            = 0;
boolean         G_BYPASS_INDEXER        = false;
boolean         G_BYPASS_INDEXER_OLD    = false;
boolean         G_WRITE_ALL_REG         = false;
boolean         G_READ_ALL_REG          = false;
boolean         G_RESET_FAULTS          = false;
boolean         G_MANUAL_WRITE          = false;
unsigned int    G_WRITE_ADDR            = 0;
unsigned int    G_WRITE_DATA            = 0;
boolean         G_MANUAL_READ           = false;
unsigned int    G_READ_ADDR             = 0;
unsigned int    G_READ_DATA             = 0;

// Stepper motion profile
unsigned int    G_START_STOP_SPEED      = 0;
unsigned int    G_TARGET_SPEED          = 0;
unsigned int    G_ACCEL_RATE            = 0;
unsigned int    G_TOTAL_NUM_STEPS       = 0;
unsigned int    G_STEPS_TO_ACCEL        = 0;
MotorState      G_MOTOR_STATE           = HOLD;
boolean         G_SPEED_PROFILE         = false;
boolean         G_SPEED_PROFILE_LOCK    = false;
boolean         G_STEP_PROFILE          = false;
boolean         G_STEP_PROFILE_LOCK     = false;

// Motor status
unsigned int    G_CUR_NUM_STEPS         = 0;
unsigned int    G_CUR_SPEED             = 0;
unsigned int    G_CUR_SPEED_TEMP        = 0;
unsigned int    G_SPEED_INCR            = 0;
boolean         G_ACCEL_FLAG            = false;

// Holding values for timer A1 CCR registers
unsigned int    G_TA1CCR0_TEMP          = 0;
unsigned int    G_TA1CCR1_TEMP          = 0;
boolean         G_LOAD_CCR_VALS         = false;

// DRV8711 GPIO
gpio G_nSLEEP           = low;
gpio G_RESET            = low;
gpio G_STEP_AIN1        = low;
gpio G_DIR_AIN2         = low;
gpio G_BIN2             = low;
gpio G_BIN1             = low;
gpio G_nFAULT           = low;
gpio G_nSTALL           = low;

// DRV8711 registers
struct CTRL_Register    G_CTRL_REG;
struct TORQUE_Register  G_TORQUE_REG;
struct OFF_Register     G_OFF_REG;
struct BLANK_Register   G_BLANK_REG;
struct DECAY_Register   G_DECAY_REG;
struct STALL_Register   G_STALL_REG;
struct DRIVE_Register   G_DRIVE_REG;
struct STATUS_Register  G_STATUS_REG;

/******************************************************************************/

// Function definitions
void
Initialize ()
{
    // Setup CLKs

    // Stop watchdog timer
    WDTCTL   = WDTPW | WDTHOLD;
    // Set DCO to 16MHz
    DCOCTL   = 0x00;
    DCOCTL   = CALDCO_16MHZ;
    BCSCTL1  = CALBC1_16MHZ;
    // Set SMCLK to 2MHz
    BCSCTL2 |= DIVS_3;
    // ACLK = VLO
    BCSCTL3 |= LFXT1S_2;

    // Configure port directions and peripherals as needed

    // Configure GPIO
    P1SEL  &= ~(POT | nSLEEP);
    P1SEL2 &= ~(POT | nSLEEP);

    P1DIR  |=  (POT | nSLEEP);
    P1OUT  |=  (POT | nSLEEP);

    P2SEL  &= ~(RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1 | nFAULT | nSTALL);
    P2SEL2 &= ~(RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1 | nFAULT | nSTALL);

    P2DIR  |=  (RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
    P2OUT  &= ~(RESET | STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);

    P2DIR  &= ~(nFAULT | nSTALL);
    P2REN  |=  (nFAULT | nSTALL);

    P3DIR  |=  (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3OUT  &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);

    // Configure SPI
    // Recommended USCI initialization/re-configure process
    // 1. Set UCSWRST (BIS.B #UCSWRST,&UCxCTL1)
    // 2. Initialize all USCI registers with UCSWRST=1 (including UCxCTL1)
    // 3. Configure ports
    // 4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCxCTL1)
    // 5. Enable interrupts (optional) via UCxRXIE and/or UCxTXIE

    // 1)
    UCB0CTL1 = UCSWRST;

    // 2)
    P2DIR  |=  CS;
    P2OUT  &= ~CS;
    P1SEL  |=  SCLK | SDATO | SDATI;
    P1SEL2 |=  SCLK | SDATO | SDATI;

    // 3) 3-pin, 8-bit SPI master
    UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCSYNC;
    UCB0CTL1 |= UCSSEL_2;   // SMCLK

    // 4)
    UCB0CTL1 &= ~UCSWRST;
    // End SPI configure

    // UART initialization
    uartInit ();

    // Enables LPM interrupts
    __bis_SR_register (GIE);

    // GUI composer monitor initialization
    ClearBufferRelatedParam ();

    // Set default GPIO settings
    G_nSLEEP    = low;
    G_RESET     = low;
    G_STEP_AIN1 = low;
    G_DIR_AIN2  = low;
    G_BIN2      = low;
    G_BIN1      = low;
    G_nFAULT    = high;
    G_nSTALL    = high;
    UpdateGPIO ();

    // Setup pin for timer output
    P2DIR  |=  STEP_AIN1;
    P2SEL  |=  STEP_AIN1;
    P2SEL2 &= ~STEP_AIN1;

    // Load default speed profile values
    G_START_STOP_SPEED      = DEFAULT_START_STOP_SPEED;
    G_TARGET_SPEED          = DEFAULT_TARGET_SPEED;
    G_ACCEL_RATE            = DEFAULT_ACCEL_RATE;
    G_TOTAL_NUM_STEPS       = DEFAULT_NUM_STEPS;

    // Set default register settings
    // CTRL register
    G_CTRL_REG.Address      = 0x00;
    G_CTRL_REG.DTIME        = 0x03;
    G_CTRL_REG.ISGAIN       = 0x03;
    G_CTRL_REG.EXSTALL      = 0x00;
    G_CTRL_REG.MODE         = 0x03;
    G_CTRL_REG.RSTEP        = 0x00;
    G_CTRL_REG.RDIR         = 0x00;
    G_CTRL_REG.ENBL         = 0x01;

    // TORQUE register
    G_TORQUE_REG.Address    = 0x01;
    G_TORQUE_REG.SIMPLTH    = 0x00;
    G_TORQUE_REG.TORQUE     = 0xBA;

    // OFF register
    G_OFF_REG.Address       = 0x02;
    G_OFF_REG.PWMMODE       = 0x00;
    G_OFF_REG.TOFF          = 0x30;

    // BLANK register
    G_BLANK_REG.Address     = 0x03;
    G_BLANK_REG.ABT         = 0x01;
    G_BLANK_REG.TBLANK      = 0x08;

    // DECAY register.
    G_DECAY_REG.Address     = 0x04;
    G_DECAY_REG.DECMOD      = 0x03;
    G_DECAY_REG.TDECAY      = 0x10;

    // STALL register
    G_STALL_REG.Address     = 0x05;
    G_STALL_REG.VDIV        = 0x03;
    G_STALL_REG.SDCNT       = 0x03;
    G_STALL_REG.SDTHR       = 0x40;

    // DRIVE register
    G_DRIVE_REG.Address     = 0x06;
    G_DRIVE_REG.IDRIVEP     = 0x00;
    G_DRIVE_REG.IDRIVEN     = 0x00;
    G_DRIVE_REG.TDRIVEP     = 0x01;
    G_DRIVE_REG.TDRIVEN     = 0x01;
    G_DRIVE_REG.OCPDEG      = 0x01;
    G_DRIVE_REG.OCPTH       = 0x01;

    // STATUS register
    G_STATUS_REG.Address    = 0x07;
    G_STATUS_REG.STDLAT     = 0x00;
    G_STATUS_REG.STD        = 0x00;
    G_STATUS_REG.UVLO       = 0x00;
    G_STATUS_REG.BPDF       = 0x00;
    G_STATUS_REG.APDF       = 0x00;
    G_STATUS_REG.BOCP       = 0x00;
    G_STATUS_REG.AOCP       = 0x00;
    G_STATUS_REG.OTS        = 0x00;
    WriteAllRegisters ();
}

void
UpdateGPIO ()
{
    // Update the GPIO pins from global variables
    // nSLEEP
    if (G_nSLEEP == high)
        P1OUT |=  nSLEEP;
    else
        P1OUT &= ~nSLEEP;

    // RESET
    if (G_RESET == high)
        P2OUT |=  RESET;
    else
        P2OUT &= ~RESET;

    // DC motor mode
    if (G_BYPASS_INDEXER == true) {
        // Change from indexer to DC motor mode
        if (G_BYPASS_INDEXER_OLD == false) {
            // Stop watchdog interval timer
            WDTCTL =  WDTPW | WDTHOLD;
            IE1   &= ~WDTIE;
            // Disable timer
            TA1CTL =  TASSEL_2 | MC_0 | TACLR;
            G_CUR_SPEED             = 0;
            // Reset state machine
            G_SPEED_PROFILE         = false;
            G_STEP_PROFILE          = false;
            G_SPEED_PROFILE_LOCK    = false;
            G_STEP_PROFILE_LOCK     = false;
            G_MOTOR_STATE           = HOLD;

            // Configure pins for IN/IN control and set all outputs low
            P2SEL  &= ~(STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
            P2SEL2 &= ~(STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
            P2OUT  &= ~(STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);
            P2DIR  |=  (STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);

            G_STEP_AIN1 = low;
            G_DIR_AIN2  = low;
            G_BIN2      = low;
            G_BIN1      = low;

            // Change DRV8711 mode
            G_OFF_REG.PWMMODE = 1;
            WriteAllRegisters ();

            G_BYPASS_INDEXER_OLD = true;
        }

        // STEP_AIN1
        if (G_STEP_AIN1 == high)
            P2OUT |=  STEP_AIN1;
        else
            P2OUT &= ~STEP_AIN1;

        // DIR_AIN2
        if (G_DIR_AIN2 == high)
            P2OUT |=  DIR_AIN2;
        else
            P2OUT &= ~DIR_AIN2;

        // BIN1
        if (G_BIN1 == high)
            P2OUT |=  BIN1;
        else
            P2OUT &= ~BIN1;

        // BIN2
        if (G_BIN2 == high)
            P2OUT |=  BIN2;
        else
            P2OUT &= ~BIN2;
    } else {
        // Stepper motor mode
        // Change from DC motor to indexer mode
        if (G_BYPASS_INDEXER_OLD == true) {
            // Reset state machine
            G_SPEED_PROFILE         = false;
            G_STEP_PROFILE          = false;
            G_SPEED_PROFILE_LOCK    = false;
            G_STEP_PROFILE_LOCK     = false;
            G_MOTOR_STATE           = HOLD;

            // Set all outputs low
            P2OUT &= ~(STEP_AIN1 | DIR_AIN2 | BIN2 | BIN1);

            G_STEP_AIN1 = low;
            G_DIR_AIN2  = low;
            G_BIN2      = low;
            G_BIN1      = low;

            // Setup pin for timer output
            P2DIR  |=  STEP_AIN1;
            P2SEL  |=  STEP_AIN1;
            P2SEL2 &= ~STEP_AIN1;

            // Change DRV8711 mode
            G_OFF_REG.PWMMODE = 0;
            WriteAllRegisters ();

            G_BYPASS_INDEXER_OLD = false;
        }

        // STEP_AIN1 (send 1 step)
        if (G_STEP_AIN1 == high) {
            // Set the timer output low
            TA1CCTL1 =  OUTMOD_0;
            P2OUT   &= ~STEP_AIN1;
            // Configure pin for GPIO
            P2SEL   &= ~STEP_AIN1;
            P2SEL2  &= ~STEP_AIN1;
            P2DIR   |=  STEP_AIN1;
            P2OUT   &= ~STEP_AIN1;
            // Send 1 pulse
            P2OUT   |=  STEP_AIN1;
            int i = 0;
            for (i = 0; i < 1000; i++)
                __delay_cycles (1000);
            P2OUT   &= ~STEP_AIN1;
            // Setup pin for timer output
            P2DIR   |=  STEP_AIN1;
            P2SEL   |=  STEP_AIN1;
            P2SEL2  &= ~STEP_AIN1;
            // Reset STEP_AIN1
            G_STEP_AIN1 = low;
        }

        // DIR_AIN2
        if (G_DIR_AIN2 == high)
            P2OUT |=  DIR_AIN2;
        else
            P2OUT &= ~DIR_AIN2;
    }

    // nFAULT
    if ((nFAULT & P2IN) == 0)
        G_nFAULT = low;
    else
        G_nFAULT = high;

    // nSTALL
    if ((nSTALL & P2IN) == 0)
        G_nSTALL = low;
    else
        G_nSTALL = high;
}

void
UpdateDRV8711Registers ()
{
    // Write all registers
    if (G_WRITE_ALL_REG == true) {
        WriteAllRegisters ();
        G_WRITE_ALL_REG = false;
    }

    // Read all registers
    if (G_READ_ALL_REG == true) {
        ReadAllRegisters ();
        G_READ_ALL_REG = false;
    }

    // Reset FAULTS
    if (G_RESET_FAULTS == true) {
        SPI_DRV8711_ReadWrite (0x70, 0x00);
        G_STATUS_REG.STDLAT = 0x00;
        G_STATUS_REG.STD    = 0x00;
        G_STATUS_REG.UVLO   = 0x00;
        G_STATUS_REG.BPDF   = 0x00;
        G_STATUS_REG.APDF   = 0x00;
        G_STATUS_REG.BOCP   = 0x00;
        G_STATUS_REG.AOCP   = 0x00;
        G_STATUS_REG.OTS    = 0x00;
        G_RESET_FAULTS      = false;
    }

    // Manual SPI write
    if (G_MANUAL_WRITE == true) {
        unsigned char byteHi = REGWRITE | (G_WRITE_ADDR << 4) | (G_WRITE_DATA >> 8);
        unsigned char byteLo = G_WRITE_DATA;
        SPI_DRV8711_ReadWrite (byteHi, byteLo);
        G_MANUAL_WRITE = false;
    }

    // Manual SPI read
    if (G_MANUAL_READ == true) {
        unsigned char byte = REGREAD | (G_READ_ADDR << 4);
        G_READ_DATA = SPI_DRV8711_ReadWrite (byte, 0x00) & 0x0FFF;
        G_MANUAL_READ = false;
    }
}

void
WriteAllRegisters ()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write CTRL register
    dataHi = REGWRITE | (G_CTRL_REG.Address << 4) | (G_CTRL_REG.DTIME << 2) | (G_CTRL_REG.ISGAIN);
    dataLo = (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write TORQUE register
    dataHi = REGWRITE | (G_TORQUE_REG.Address << 4) | (G_TORQUE_REG.SIMPLTH);
    dataLo = G_TORQUE_REG.TORQUE;
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write OFF register
    dataHi = REGWRITE | (G_OFF_REG.Address << 4) | (G_OFF_REG.PWMMODE);
    dataLo = G_OFF_REG.TOFF;
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write BLANK register
    dataHi = REGWRITE | (G_BLANK_REG.Address << 4) | (G_BLANK_REG.ABT);
    dataLo = G_BLANK_REG.TBLANK;
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write DECAY register
    dataHi = REGWRITE | (G_DECAY_REG.Address << 4) | (G_DECAY_REG.DECMOD);
    dataLo = G_DECAY_REG.TDECAY;
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write STALL register
    dataHi = REGWRITE | (G_STALL_REG.Address << 4) | (G_STALL_REG.VDIV << 2) | (G_STALL_REG.SDCNT);
    dataLo = G_STALL_REG.SDTHR;
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write DRIVE register
    dataHi = REGWRITE | (G_DRIVE_REG.Address << 4) | (G_DRIVE_REG.IDRIVEP << 2) | (G_DRIVE_REG.IDRIVEN);
    dataLo = (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
    SPI_DRV8711_ReadWrite (dataHi, dataLo);

    // Write STATUS register
    dataHi = REGWRITE | (G_STATUS_REG.Address << 4);
    dataLo = (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
    SPI_DRV8711_ReadWrite (dataHi, dataLo);
}

void
ReadAllRegisters ()
{
    unsigned char       dataHi      = 0x00;
    const unsigned char dataLo      = 0x00;
    unsigned int        readData    = 0x00;

    // Read CTRL register
    dataHi      = REGREAD | (G_CTRL_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_CTRL_REG.DTIME        = ((readData >> 10) & 0x0003);
    G_CTRL_REG.ISGAIN       = ((readData >> 8) & 0x0003);
    G_CTRL_REG.EXSTALL      = ((readData >> 7) & 0x0001);
    G_CTRL_REG.MODE         = ((readData >> 3) & 0x000F);
    G_CTRL_REG.RSTEP        = ((readData >> 2) & 0x0001);
    G_CTRL_REG.RDIR         = ((readData >> 1) & 0x0001);
    G_CTRL_REG.ENBL         = ((readData >> 0) & 0x0001);

    // Read TORQUE register
    dataHi      = REGREAD | (G_TORQUE_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_TORQUE_REG.SIMPLTH    = ((readData >> 8) & 0x0007);
    G_TORQUE_REG.TORQUE     = ((readData >> 0) & 0x00FF);

    // Read OFF register
    dataHi      = REGREAD | (G_OFF_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_OFF_REG.PWMMODE       = ((readData >> 8) & 0x0001);
    G_OFF_REG.TOFF          = ((readData >> 0) & 0x00FF);

    // Read BLANK register
    dataHi      = REGREAD | (G_BLANK_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_BLANK_REG.ABT         = ((readData >> 8) & 0x0001);
    G_BLANK_REG.TBLANK      = ((readData >> 0) & 0x00FF);

    // Read DECAY register
    dataHi      = REGREAD | (G_DECAY_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_DECAY_REG.DECMOD      = ((readData >> 8) & 0x0007);
    G_DECAY_REG.TDECAY      = ((readData >> 0) & 0x00FF);

    // Read STALL register
    dataHi      = REGREAD | (G_STALL_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_STALL_REG.VDIV        = ((readData >> 10) & 0x0003);
    G_STALL_REG.SDCNT       = ((readData >> 8) & 0x0003);
    G_STALL_REG.SDTHR       = ((readData >> 0) & 0x00FF);

    // Read DRIVE register
    dataHi      = REGREAD | (G_DRIVE_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_DRIVE_REG.IDRIVEP     = ((readData >> 10) & 0x0003);
    G_DRIVE_REG.IDRIVEN     = ((readData >> 8) & 0x0003);
    G_DRIVE_REG.TDRIVEP     = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.TDRIVEN     = ((readData >> 4) & 0x0003);
    G_DRIVE_REG.OCPDEG      = ((readData >> 2) & 0x0003);
    G_DRIVE_REG.OCPTH       = ((readData >> 0) & 0x0003);

    // Read STATUS register
    dataHi      = REGREAD | (G_STATUS_REG.Address << 4);
    readData    = SPI_DRV8711_ReadWrite (dataHi, dataLo);
    G_STATUS_REG.STDLAT     = ((readData >> 7) & 0x0001);
    G_STATUS_REG.STD        = ((readData >> 6) & 0x0001);
    G_STATUS_REG.UVLO       = ((readData >> 5) & 0x0001);
    G_STATUS_REG.BPDF       = ((readData >> 4) & 0x0001);
    G_STATUS_REG.APDF       = ((readData >> 3) & 0x0001);
    G_STATUS_REG.BOCP       = ((readData >> 2) & 0x0001);
    G_STATUS_REG.AOCP       = ((readData >> 1) & 0x0001);
    G_STATUS_REG.OTS        = ((readData >> 0) & 0x0001);
}

void
UpdateFullScaleCurrent ()
{
    // Calculate the 100% chopping current level
    if ((G_CTRL_REG.ISGAIN != G_ISGAIN_OLD) || (G_TORQUE_REG.TORQUE != G_TORQUE_OLD)) {
        // Parse ISGAIN
        unsigned int temp_isgain;
        if (G_CTRL_REG.ISGAIN == 0)
            temp_isgain = 5;
        else if (G_CTRL_REG.ISGAIN == 1)
            temp_isgain = 10;
        else if (G_CTRL_REG.ISGAIN == 2)
            temp_isgain = 20;
        else if (G_CTRL_REG.ISGAIN == 3)
            temp_isgain = 40;
        else
            temp_isgain = 5;

        // Calculate floating point value
        G_FULL_SCALE_CURRENT = (2.75 * (float)G_TORQUE_REG.TORQUE) / (256 * (float)temp_isgain * 0.05);
        G_ISGAIN_OLD = G_CTRL_REG.ISGAIN;
        G_TORQUE_OLD = G_TORQUE_REG.TORQUE;
    }
}

void
UpdateStepperMotionProfile ()
{
    if (G_BYPASS_INDEXER == false) {
        // Motion profile state machine
        // Speed profile motor start
        if ((G_SPEED_PROFILE == true) && (G_SPEED_PROFILE_LOCK == false)) {
            G_MOTOR_STATE = SPD_START;
        }
        // Speed profile motor accelerating
        else if ((G_CUR_SPEED < G_TARGET_SPEED) && (G_SPEED_PROFILE == true) && (G_SPEED_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = SPD_ACCEL;
        }
        // Speed profile motor stable
        else if ((G_CUR_SPEED == G_TARGET_SPEED) && (G_SPEED_PROFILE == true) && (G_SPEED_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = SPD_STABLE;
        }
        // Speed profile motor decelerating
        else if ((G_CUR_SPEED > G_START_STOP_SPEED) && (G_SPEED_PROFILE == false) && (G_SPEED_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = SPD_DECEL;
        }
        // Speed profile motor stop
        else if ((G_CUR_SPEED == G_START_STOP_SPEED) && (G_SPEED_PROFILE == false) && (G_SPEED_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = SPD_STOP;
        }
        // Step profile motor start
        else if ((G_STEP_PROFILE == true) && (G_STEP_PROFILE_LOCK == false)) {
            G_MOTOR_STATE = STP_START;
        }
        // Step profile motor accelerating
        else if ((G_CUR_NUM_STEPS < G_STEPS_TO_ACCEL) && (G_STEP_PROFILE == true) && (G_STEP_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = STP_ACCEL;
        }
        // Step profile motor stable
        else if ((G_CUR_NUM_STEPS < (G_TOTAL_NUM_STEPS - G_STEPS_TO_ACCEL)) && (G_STEP_PROFILE == true) && (G_STEP_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = STP_STABLE;
        }
        // Step profile motor decelerating
        else if ((G_CUR_NUM_STEPS >= (G_TOTAL_NUM_STEPS - G_STEPS_TO_ACCEL)) && (G_CUR_NUM_STEPS < G_TOTAL_NUM_STEPS) && (G_STEP_PROFILE == true) && (G_STEP_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = STP_DECEL;
        }
        // Step profile motor stop
        else if ((G_CUR_NUM_STEPS >= G_TOTAL_NUM_STEPS) && (G_STEP_PROFILE == true) && (G_STEP_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = STP_STOP;
        }
        // Step profile force motor stop
        else if ((G_STEP_PROFILE == false) && (G_STEP_PROFILE_LOCK == true)) {
            G_MOTOR_STATE = STP_STOP;
        }
        // Motor hold
        else {
            G_MOTOR_STATE = HOLD;
        }

        // Speed profile
        SpeedProfile ();
        // Step profile
        StepProfile ();
    }
}

void
SpeedProfile ()
{
    // Motor start
    if (G_MOTOR_STATE == SPD_START) {
        G_SPEED_PROFILE_LOCK = true;
        // Set the start speed (cannot overflow the timer register)
        G_CUR_NUM_STEPS = 0;
        // Value check
        if (G_START_STOP_SPEED > G_TARGET_SPEED)
            G_START_STOP_SPEED = G_TARGET_SPEED;
        G_CUR_SPEED = G_START_STOP_SPEED;
        TA1CCR0 = (SMCLK_MHz*1000000)/G_START_STOP_SPEED;
        TA1CCR1 = ((SMCLK_MHz*1000000)/G_START_STOP_SPEED) >> 1;

        // Determine the acceleration increment
        // Watchdog timer fires every 16.384ms, ~61.035 interupts/s (accel rate cannot be less than 64)
        // Round accelerat rate to increment of 64
        G_ACCEL_RATE = (G_ACCEL_RATE >> 6) << 6;
        G_SPEED_INCR = G_ACCEL_RATE >> 6;

        // Set the timer output low
        TA1CCTL1 =  OUTMOD_0;
        P2OUT   &= ~STEP_AIN1;
        // Setup the output and interrupt
        TA1CCTL0 =  CCIE;
        TA1CCTL1 =  OUTMOD_3 | CCIE;
        // Start the timer
        TA1CTL   =  TASSEL_2 | MC_1 | TACLR;

        // Setup watchdog timer as an interval timer for accel/decel updates
        WDTCTL   =  WDT_MDLY_32;
        IE1     |=  WDTIE;
    }

    // Motor accelerate
    else if ((G_MOTOR_STATE == SPD_ACCEL) && (G_ACCEL_FLAG == true)) {
        G_ACCEL_FLAG = false;
        // Increase speed
        if ((G_CUR_SPEED + G_SPEED_INCR) < G_TARGET_SPEED) {
            // Calcute next speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_CUR_SPEED + G_SPEED_INCR;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP) >> 1;
            G_LOAD_CCR_VALS = true;
        }
        // Load target speed
        else {
            // Calcute target speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_TARGET_SPEED;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_TARGET_SPEED;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_TARGET_SPEED) >> 1;
            G_LOAD_CCR_VALS = true;
        }
    }

    // Motor decelerate
    else if ((G_MOTOR_STATE == SPD_DECEL) && (G_ACCEL_FLAG == true)) {
        G_ACCEL_FLAG = false;
        // Decrease speed
        if (((G_CUR_SPEED - G_SPEED_INCR) > G_START_STOP_SPEED) && (G_CUR_SPEED > G_SPEED_INCR)) {
            // Calcute next speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_CUR_SPEED - G_SPEED_INCR;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP) >> 1;
            G_LOAD_CCR_VALS = true;
        }
        // Load stop speed
        else {
            // Calcute stop speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_START_STOP_SPEED;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_START_STOP_SPEED;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_START_STOP_SPEED) >> 1;
            G_LOAD_CCR_VALS = true;
        }
    }

    // Motor stop
    else if (G_MOTOR_STATE == SPD_STOP) {
        // Stop watchdog interval timer
        WDTCTL =  WDTPW | WDTHOLD;
        IE1   &= ~WDTIE;
        // Disable timer
        TA1CTL =  TASSEL_2 | MC_0 | TACLR;
        G_CUR_SPEED = 0;
        G_SPEED_PROFILE_LOCK = false;
    }

    // Catch all
    else {}
}

void
StepProfile ()
{
    // Motor start
    if (G_MOTOR_STATE == STP_START) {
        G_STEP_PROFILE_LOCK = true;
        // Set the start speed (cannot overflow the timer register)
        G_CUR_NUM_STEPS = 0;
        // Value check
        if (G_START_STOP_SPEED > G_TARGET_SPEED)
            G_START_STOP_SPEED = G_TARGET_SPEED;
        G_CUR_SPEED = G_START_STOP_SPEED;
        TA1CCR0 = (SMCLK_MHz*1000000)/G_START_STOP_SPEED;
        TA1CCR1 = ((SMCLK_MHz*1000000)/G_START_STOP_SPEED) >> 1;

        // Determine the acceleration increment
        // Watchdog timer fires every 16.384ms, ~61.035 interupts/s (accel rate cannot be less than 64)
        // Round accelerat rate to increment of 64
        G_ACCEL_RATE = (G_ACCEL_RATE >> 6) << 6;
        G_SPEED_INCR = G_ACCEL_RATE >> 6;

        // Determine the number of steps to accel/decel
        // TODO explain this
        float time = (float)(G_TARGET_SPEED - G_START_STOP_SPEED)/(float)G_ACCEL_RATE;
        G_STEPS_TO_ACCEL = ((G_ACCEL_RATE >> 1) * (time * time)) + (G_START_STOP_SPEED * time);
        if (G_STEPS_TO_ACCEL > (G_TOTAL_NUM_STEPS >> 1))
            G_STEPS_TO_ACCEL = G_TOTAL_NUM_STEPS >> 1;

        // Set the timer output low
        TA1CCTL1 =  OUTMOD_0;
        P2OUT   &= ~STEP_AIN1;
        // Setup the output and interrupt
        TA1CCTL0 =  CCIE;
        TA1CCTL1 =  OUTMOD_3 | CCIE;
        // Start the timer
        TA1CTL   =  TASSEL_2 | MC_1 | TACLR;

        // Setup watchdog timer as an interval timer for accel/decel updates
        WDTCTL   =  WDT_MDLY_32;
        IE1     |=  WDTIE;
    }

    // Motor accelerate
    else if ((G_MOTOR_STATE == STP_ACCEL) && (G_ACCEL_FLAG == true)) {
        G_ACCEL_FLAG = false;
        // Increase speed
        if ((G_CUR_SPEED + G_SPEED_INCR) < G_TARGET_SPEED) {
            // Calcute next speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_CUR_SPEED + G_SPEED_INCR;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP) >> 1;
            G_LOAD_CCR_VALS = true;
        }
        // Load target speed
        else {
            // Calcute target speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_TARGET_SPEED;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_TARGET_SPEED;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_TARGET_SPEED) >> 1;
            G_LOAD_CCR_VALS = true;
        }
    }

    // Motor decelerate
    else if ((G_MOTOR_STATE == STP_DECEL) && (G_ACCEL_FLAG == true)) {
        G_ACCEL_FLAG = false;
        // Decrease speed
        if (((G_CUR_SPEED - G_SPEED_INCR) > G_START_STOP_SPEED) && (G_CUR_SPEED > G_SPEED_INCR)) {
            // Calcute next speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_CUR_SPEED - G_SPEED_INCR;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_CUR_SPEED_TEMP) >> 1;
            G_LOAD_CCR_VALS = true;
        }
        // Load stop speed
        else {
            // Calcute stop speed value (cannot overflow the timer register)
            G_CUR_SPEED_TEMP = G_START_STOP_SPEED;
            G_TA1CCR0_TEMP = (SMCLK_MHz*1000000)/G_START_STOP_SPEED;
            G_TA1CCR1_TEMP = ((SMCLK_MHz*1000000)/G_START_STOP_SPEED) >> 1;
            G_LOAD_CCR_VALS = true;
        }
    }

    // Motor stop
    else if (G_MOTOR_STATE == STP_STOP) {
        // Stop watchdog interval timer
        WDTCTL =  WDTPW | WDTHOLD;
        IE1   &= ~WDTIE;
        // Disable timer
        TA1CTL =  TASSEL_2 | MC_0 | TACLR;
        // End step profle
        G_CUR_SPEED = 0;
        G_STEP_PROFILE = false;
        G_STEP_PROFILE_LOCK = false;
    }

    // Catch all
    else {}
}

unsigned int
SPI_DRV8711_ReadWrite (unsigned char dataHi,
                       unsigned char dataLo)
{
    unsigned int readData = 0;

    P2OUT |= CS;

    UCB0TXBUF = dataHi;
    while (UCB0STAT & BUSY);
    readData |= (UCB0RXBUF << 8);

    UCB0TXBUF = dataLo;
    while (UCB0STAT & BUSY);
    readData |= UCB0RXBUF;

    P2OUT &= ~CS;

    return readData;
}