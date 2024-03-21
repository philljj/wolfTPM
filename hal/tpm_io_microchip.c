/* tpm_io_microchip.c
 *
 * Copyright (C) 2006-2023 wolfSSL Inc.
 *
 * This file is part of wolfTPM.
 *
 * wolfTPM is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfTPM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */

/* This example shows IO interfaces for Microchip micro-controllers using
 * MPLAB X and Harmony
 */

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include <wolftpm/tpm2.h>
#include <wolftpm/tpm2_tis.h>
#include "tpm_io.h"

/******************************************************************************/
/* --- BEGIN IO Callback Logic -- */
/******************************************************************************/

/* Included via tpm_io.c if WOLFTPM_INCLUDE_IO_FILE is defined */
#ifdef WOLFTPM_INCLUDE_IO_FILE

#if ! (defined(WOLFTPM_LINUX_DEV) || \
       defined(WOLFTPM_SWTPM) ||     \
       defined(WOLFTPM_WINAPI) )

/* Use the max speed by default - see tpm2_types.h for chip specific max values */
#ifndef TPM2_SPI_HZ
    #define TPM2_SPI_HZ TPM2_SPI_MAX_HZ
#endif

#if defined(WOLFTPM_MICROCHIP_HARMONY)

#include "configuration.h"
#include "definitions.h"


#ifdef WOLFTPM_I2C /* Microchip Harmony Hal I2C */
    /* Implementation based on parabit (pb) i2c driver. */
    #include "pb/pb_i2c.h"

    #ifndef TPM_I2C_TRIES
    #define TPM_I2C_TRIES 10
    #endif
    #ifndef TPM2_I2C_ADDR
    #define TPM2_I2C_ADDR I2C_ST33_TPM
    #endif

#if 0
    /* todo: use this wait function? */
    /* Wait for time_ms using Micochip Harmony SYS_TIME API. */
    static void microchip_wait(uint32_t time_ms)
    {
        /* Microchip Harmony example from documentation.
         * SYS_TIME_DelayMS will internally create the timer,
         * and SYS_TIME_DelayIsComplete will delete it when
         * the delay has completed. */
        SYS_TIME_HANDLE timer = SYS_TIME_HANDLE_INVALID;

        if (SYS_TIME_DelayMS(time_ms, &timer) != SYS_TIME_SUCCESS) {
            printf("error: microchip_wait: SYS_TIME_DelayMS failed\n");
        }
        else if(SYS_TIME_DelayIsComplete(timer) != true) {
            /* Loop until delay is complete. */
            while (SYS_TIME_DelayIsComplete(timer) == false);
        }

        return;
    }
#endif

    static int i2c_read(word32 reg, byte* data, int len)
    {
        int                             ret = TPM_RC_FAILURE;
        byte                            buf[1];
        pb_i2c_transfer_seq_type_def    seq;
        pb_i2c_transfer_return_type_def result = i2cTransferNack;
        int                             retries = 0;

        /* TIS layer should never provide a buffer larger than this,
           but double check for good coding practice */
        if (len > MAX_SPI_FRAMESIZE) {
            printf("error: i2c_read: len too large: %d\n", len);
            return BAD_FUNC_ARG;
        }

        buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */

        seq.addr = TPM2_I2C_ADDR << 1;
        seq.flags = I2C_FLAG_WRITE;    /* Write using buf[0] */
        seq.buf[0].data = buf;
        seq.buf[0].len = sizeof(buf);
        result = pb_i2c_bb_transfer(&seq);

        while(result == i2cTransferNack)
        {
            if (retries > TPM_I2C_TRIES) {
                printf("error: pb_i2c_bb_transfer Nack retries exhausted\n");
                return -1;
            }

            pb_sw_delay_us(0xFFFF);
            result = pb_i2c_bb_transfer(&seq);
            retries++;
        }

        if (result == i2cTransferDone) {
            #ifdef WOLFTPM_DEBUG_I2C
            printf("info: pb_i2c_bb_transfer read success after %d retries\n",
                   retries);
            #endif
        }
        else {
            printf("error: pb_i2c_bb_transfer failed: %d\n", result);
            return -1;
        }

        retries = 0;

        // Requesting Read
        seq.addr = TPM2_I2C_ADDR << 1;
        seq.flags = I2C_FLAG_READ;    /* Read using buf[0] */
        seq.buf[0].data = data;
        seq.buf[0].len = len;
        result = pb_i2c_bb_transfer(&seq);

        while(result == i2cTransferNack)
        {
            if (retries > TPM_I2C_TRIES) {
                printf("error: read pb_i2c_bb_transfer Nack retries exhausted\n");
                return -1;
            }

            pb_sw_delay_us(0xFFFF);
            result = pb_i2c_bb_transfer(&seq);
            retries++;
        }

        if (result == i2cTransferDone) {
            #ifdef WOLFTPM_DEBUG_I2C
            printf("info: pb_i2c_bb_transfer write success after %d retries\n",
                   retries);
            #endif
            ret = TPM_RC_SUCCESS;
        }
        else {
            printf("error: pb_i2c_bb_transfer failed: %d\n", result);
            return -1;
        }

        return ret;
    }

    static int i2c_write(word32 reg, byte* data, int len)
    {
        int                             ret = TPM_RC_FAILURE;
        byte                            buf[MAX_SPI_FRAMESIZE + 1];
        pb_i2c_transfer_seq_type_def    seq;
        pb_i2c_transfer_return_type_def result = i2cTransferNack;
        int                             retries = 0;

        /* TIS layer should never provide a buffer larger than this,
           but double check for good coding practice */
        if (len > MAX_SPI_FRAMESIZE) {
            printf("error: i2c_write: len too large: %d\n", len);
            return BAD_FUNC_ARG;
        }

        /* Build packet with TPM register and data */
        buf[0] = (reg & 0xFF); /* convert to simple 8-bit address for I2C */
        XMEMCPY(buf + 1, data, len);

        seq.addr = TPM2_I2C_ADDR << 1;
        seq.flags = I2C_FLAG_WRITE;    /* Write using buf[0] */
        seq.buf[0].data = buf;
        seq.buf[0].len = len + 1;
        result = pb_i2c_bb_transfer(&seq);

        while(result == i2cTransferNack)
        {
            if (retries > TPM_I2C_TRIES) {
                printf("error: pb_i2c_bb_transfer Nack retries exhausted\n");
                return -1;
            }

            pb_sw_delay_us(0xFFFF);
            result = pb_i2c_bb_transfer(&seq);
            retries++;
        }

        if (result == i2cTransferDone) {
            #ifdef WOLFTPM_DEBUG_I2C
            printf("info: pb_i2c_bb_transfer read success after %d retries\n",
                   retries);
            #endif
            ret = TPM_RC_SUCCESS;
        }
        else {
            printf("error: pb_i2c_bb_transfer failed: %d\n", result);
            return -1;
        }

        return ret;
    }

    int TPM2_IoCb_MicrochipHarmony_I2C(TPM2_CTX* ctx, int isRead, word32 addr,
        byte* buf, word16 size, void* userCtx)
    {
        int ret = TPM_RC_FAILURE;

        (void)userCtx;
        (void)ctx;

        pb_i2c_bb_init();

        if (isRead) {
            ret = i2c_read(addr, buf, size);
        }
        else {
            ret = i2c_write(addr, buf, size);
        }

        return ret;
    }
#else /* Microchip Harmony Hal SPI */

#ifdef WOLFTPM_CHECK_WAIT_STATE
    #error This driver does not support check wait state yet
#endif

/* TPM Chip Select Pin (default PC5) */
#ifndef TPM_SPI_PIN
#define SYS_PORT_PIN_PC5
#endif

int TPM2_IoCb_Microchip_SPI(TPM2_CTX* ctx, const byte* txBuf, byte* rxBuf,
    word16 xferSz, void* userCtx)
{
    int ret = TPM_RC_FAILURE;
    DRV_HANDLE handle = DRV_HANDLE_INVALID;
    DRV_SPI_TRANSFER_SETUP setup;

    /* Setup SPI */
    handle = DRV_SPI_Open(DRV_SPI_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (handle == DRV_HANDLE_INVALID) {
        return TPM_RC_FAILURE;
    }

    memset(&setup, 0, sizeof(setup));
    setup.baudRateInHz = TPM2_SPI_HZ;
    setup.clockPhase = DRV_SPI_CLOCK_PHASE_VALID_TRAILING_EDGE;
    setup.clockPolarity = DRV_SPI_CLOCK_POLARITY_IDLE_LOW;
    setup.dataBits = DRV_SPI_DATA_BITS_8;
    setup.chipSelect = TPM_SPI_PIN;
    setup.csPolarity = DRV_SPI_CS_POLARITY_ACTIVE_LOW;
    DRV_SPI_TransferSetup(handle, &setup);

    /* Send Entire Message blocking - no wait states */
    if (DRV_SPI_WriteReadTransfer(handle, (byte*)txBuf, xferSz, rxBuf,
                                                              xferSz) == true) {
        ret = TPM_RC_SUCCESS
    }

    (void)ctx;
    (void)userCtx;

    DRV_SPI_Close(handle);
    handle = DRV_HANDLE_INVALID;

    return ret;
}

#endif /* WOLFTPM_I2C */
#endif /* WOLFTPM_MICROCHIP_HARMONY */
#endif /* !(WOLFTPM_LINUX_DEV || WOLFTPM_SWTPM || WOLFTPM_WINAPI) */
#endif /* WOLFTPM_INCLUDE_IO_FILE */

/******************************************************************************/
/* --- END IO Callback Logic -- */
/******************************************************************************/
