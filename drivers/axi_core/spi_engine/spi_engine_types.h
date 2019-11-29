/*******************************************************************************
 *   @file   spi_engine_types.h
 *   @brief  Header file of SPI Engine types.
 *   @author Sergiu Cuciurean sergiu.cuciurean@analog.com
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef SPI_ENGINE_TYPES_H
#define SPI_ENGINE_TYPES_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

typedef struct spi_init_param_extra {
	uint32_t 	spi_clk_hz;
	uint32_t 	spi_baseaddr;
	uint8_t		cs_delay;
	uint8_t 	spi_offload_rx_support_en;
	uint32_t	spi_offload_rx_dma_baseaddr;
	uint8_t		spi_offload_tx_support_en;
	uint32_t	spi_offload_tx_dma_baseaddr;
	uint32_t	spi_clk_hz_reg_access;
} spi_init_param_extra;

typedef struct spi_eng_init_param {
	uint32_t	max_speed_hz;
	uint8_t		chip_select;
	enum spi_mode	mode;
	struct spi_init_param_extra 	extra;
} spi_eng_init_param;

typedef struct spi_desc_extra {
	uint32_t 	spi_clk_hz;
	uint32_t	spi_baseaddr;
	uint8_t		cs_delay;
	uint8_t 	spi_offload_rx_support_en;
	uint32_t	spi_offload_rx_dma_baseaddr;
	uint8_t		spi_offload_tx_support_en;
	uint32_t	spi_offload_tx_dma_baseaddr;
	uint32_t	rx_dma_startaddr;
	uint32_t	tx_dma_startaddr;
	uint32_t	rx_length;
	uint32_t	tx_length;
	uint32_t	clk_div;
	uint8_t		offload_configured;
	uint8_t		data_width;
	uint8_t 	max_data_width;
} spi_desc_extra;

typedef struct spi_eng_desc {
	uint32_t	max_speed_hz;
	uint8_t		chip_select;
	enum spi_mode	mode;
	struct spi_desc_extra	extra;
} spi_eng_desc;

typedef struct spi_eng_msg {
	uint32_t	tx_buf_addr;
	uint32_t	rx_buf_addr;
	uint32_t	*rx_buf;
	uint32_t	*tx_buf;
	uint32_t	*spi_msg_cmds;
	uint8_t		msg_cmd_len;
} spi_eng_msg;

typedef struct spi_eng_transfer_fifo {
	uint32_t	cmd_fifo_len;
	uint16_t	cmd_fifo[];
} spi_eng_transfer_fifo;

#endif // SPI_ENGINE_TYPES_H
