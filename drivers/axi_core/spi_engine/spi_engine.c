// /*******************************************************************************
//  *   @file   spi_engine.c
//  *   @brief  Core implementation of the SPI Engine Driver.
//  *   @author Sergiu Cuciurean (sergiu.cuciurean@analog.com)
// ********************************************************************************
//  * Copyright 2019(c) Analog Devices, Inc.
//  *
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *  - Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *  - Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  *  - Neither the name of Analog Devices, Inc. nor the names of its
//  *    contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *  - The use of this software may or may not infringe the patent rights
//  *    of one or more patent holders.  This license does not release you
//  *    from the requirement that you obtain separate licenses from these
//  *    patent holders to use this software.
//  *  - Use of the software either in source or binary form, must be run
//  *    on or directly connected to an Analog Devices Inc. component.
//  *
//  * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
//  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
//  * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//  * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  ******************************************************************************/

#ifndef _SPI_ENGINE_
#define _SPI_ENGINE_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sleep.h>

#include "axi_io.h"
#include "debug.h"
#include "error.h"
#include "spi.h"
#include "spi_engine.h"

// /******************************************************************************/
// /********************** Macros and Constants Definitions **********************/
// /******************************************************************************/

// /**
//  * @brief Macro used to compute the size of an array
//  * 
//  */
// #define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

// /******************************************************************************/
// /***************************** Static variables *******************************/
// /******************************************************************************/

// static uint8_t _sync_id = 0x55;

// /******************************************************************************/
// /************************** Functions Implementation **************************/
// /******************************************************************************/

 /**
  * @brief Write SPI Engine's axi registers
  *
  * @param desc Decriptor containing SPI Engine's parameters
  * @param reg_addr The address of the SPI Engine's axi register where the data
  * 	will be written
  * @param reg_data Data that will be written
  * @return int32_t This function allways returns SUCCESS
  */
 int32_t spi_engine_write(spi_engine_desc *desc,
 		      uint32_t reg_addr,
 		      uint32_t reg_data)
 {
 	dev_dbg(desc, "reg_addr = 0x%X; reg_data = 0x%X", reg_addr, reg_data);
 	axi_io_write(desc->spi_engine_baseaddr, reg_addr, reg_data);

 	return SUCCESS;
 }

 /**
  * @brief Read SPI Engine's axi registers
  *
  * @param desc Decriptor containing SPI Engine's parameters
  * @param reg_addr The address of the SPI Engine's axi register from where the
  * 	data where the data will be read
  * @param reg_data Pointer where the read that will be stored
  * @return int32_t This function allways returns SUCCESS
  */
 int32_t spi_engine_read(spi_engine_desc *desc,
 		     uint32_t reg_addr,
 		     uint32_t *reg_data)
 {
 	axi_io_read(desc->spi_engine_baseaddr, reg_addr, reg_data);
 	dev_dbg(desc, "reg_addr = 0x%X; reg_data = 0x%X", reg_addr, *reg_data);

 	return SUCCESS;
 }

// /**
//  * @brief Write SPI Engine's DMA axi registers
//  * 
//  * @param desc Decriptor containing SPI Engine's parameters
//  * @param reg_addr The address of the SPI Engine's DMA axi register where the
//  * 	data will be written
//  * @param reg_data Data that will be written
//  * @return int32_t This function allways returns SUCCESS
//  */
// int32_t spi_engine_dma_write(spi_engine_desc *desc,
// 			  uint32_t reg_addr,
// 			  uint32_t reg_data)
// {
// 	dev_dbg(desc, "reg_addr = 0x%X; reg_data = 0x%X", reg_addr, reg_data);
// 	axi_io_write(desc->tx_dma_baseaddr, reg_addr, reg_data);

// 	return SUCCESS;
// }

// /**
//  * @brief Read SPI Engine's DMA axi registers
//  * 
//  * @param desc Decriptor containing SPI Engine's parameters
//  * @param reg_addr The address of the SPI Engine's DMA axi register from where
//  * 	the data where the data will be read
//  * @param reg_data Pointer where the read that will be stored
//  * @return int32_t This function allways returns SUCCESS
//  */
// int32_t spi_engine_dma_read(spi_engine_desc *desc,
// 			 uint32_t reg_addr,
// 			 uint32_t *reg_data)
// {
// 	axi_io_read(desc->rx_dma_baseaddr, reg_addr, reg_data);
// 	dev_dbg(desc, "reg_addr = 0x%X; reg_data = 0x%X", reg_addr, *reg_data);

// 	return SUCCESS;
// }

// /**
//  * @brief Set width of the transfered word over SPI
//  * 
//  * @param desc Decriptor containing SPI interface parameters
//  * @param data_wdith The desired data width
//  * 	The supported values are:
//  * 		- 8
//  * 		- 16
//  * 		- 24
//  * 		- 32
//  */
// int32_t spi_engine_set_transfer_wdith(spi_desc *desc,
// 				 uint8_t data_wdith)
// {
// 	spi_engine_desc	*desc_extra;

// 	desc_extra = desc->extra;

// 	dev_dbg(desc, "data_wdith = 0x%x",data_wdith);

// 	if (data_wdith > desc_extra->max_data_width) {
// 		desc_extra->data_width = desc_extra->max_data_width;
// 	} else if ((data_wdith % 8) == 0) {
// 		desc_extra->data_width = data_wdith;
// 	} else {
// 		return FAILURE;
// 	}

// 	return SUCCESS;
// }

// /**
//  * @brief Set the number of words transfered in a single transaction
//  * 
//  * @param desc Decriptor containing SPI Engine's parameters
//  * @param bytes_number The number of bytes to be cnoverted
//  * @return uint8_t A number of words in which bytes_number can be grouped
//  */
// uint8_t spi_get_words_number(spi_engine_desc *desc,
// 			     uint8_t bytes_number)
// {
// 	uint8_t xfer_word_len;
// 	uint8_t words_number;
	
// 	xfer_word_len = desc->data_width / 8;
// 	words_number = bytes_number / xfer_word_len;

// 	if ((bytes_number % xfer_word_len) != 0)
// 		words_number++;

// 	dev_dbg(desc, "words_number = 0x%X",words_number);

// 	return words_number;
// }

// /**
//  * @brief Get the word lenght in bytes
//  * 
//  * @param desc Decriptor containing SPI Engine's parameters
//  * @return uint8_t Number of bytes that fit in one word
//  */
// uint8_t spi_get_word_lenght(spi_engine_desc *desc)
// {
// 	uint8_t words_number;

// 	words_number = desc->data_width / 8;

// 	dev_dbg(desc, "words_lenght = 0x%X",words_number);

// 	return words_number;
// }

// /*******************************************************************************
//  *
//  * @name	spi_check_dma_config
//  *
//  * @brief	Check if both tx and rx channels of the DMA are enabled while
//  *		offload mode is cnofigured.
//  *
//  * @param
//  *		desc		- Spi engine descriptor extra parameters
//  *		rx		- Offload flag for rx channel
//  *		tx		- Offload flag for tx channel
//  *
//  * @return			- SUCCESS if both channels are configured
//  *				- FAILURE if contrary
//  *
//  ******************************************************************************/
// int32_t spi_check_dma_config(spi_engine_desc *desc,
// 			     uint8_t rx,
// 			     uint8_t tx)
// {
// 	if(desc->offload_configured &&
// 	  ((rx && !(desc->offload_config & OFFLOAD_RX_EN)) ||
// 	  (tx && !(desc->offload_config & OFFLOAD_TX_EN))))
// 		return FAILURE;
	
// 	return SUCCESS;
// }

// /**
//  * @brief  Compute the prescaler used to set the sleep period.
//  * 
//  * @param desc Decriptor containing SPI interface parameters
//  * @param sleep_time_ns The amount of time where the transfer hangs
//  * @param sleep_div Clock prescaler
//  * @return int32_t This function allways returns SUCCESS
//  */
// int32_t spi_get_sleep_div(spi_desc *desc,
// 			  uint32_t sleep_time_ns,
// 			  uint32_t *sleep_div)
// {
// 	spi_engine_desc	*eng_desc;

// 	eng_desc = desc->extra;

// 	/*
// 	 * Engine Wiki:
// 	 *
// 	 * The frequency of the SCLK signal is derived from the
// 	 * module clock frequency using the following formula:
// 	 * f_sclk = f_clk / ((div + 1) * 2)
// 	 */

// 	*sleep_div = (desc->max_speed_hz / 1000000 * sleep_time_ns / 1000) /
// 		     ((eng_desc->clk_div + 1) * 2) - 1;

// #if DEBUG_LEVEL == 1
// 	dev_dbg("sleep_div = 0x%X",sleep_div);
// #endif

// 	return SUCCESS;
// }

// /*******************************************************************************
//  *
//  * @name	spi_engine_program_add_cmd
//  *
//  * @brief	Add a command to the CMD_FIFO buffer
//  *
//  * @param
//  *		xfer		- Fifo transfer structure
//  *		cmd		- The command that will be added to the buffer
//  *
//  * @return	void
//  *
//  ******************************************************************************/
// void spi_engine_program_add_cmd(spi_engine_transfer_fifo *xfer,
// 			     uint16_t cmd)
// {
// 	xfer->cmd_fifo[xfer->cmd_fifo_len] = cmd;
// 	xfer->cmd_fifo_len++;
// }

// int32_t spi_engine_queue_new_cmd(spi_engine_cmd_queue **fifo,
// 			    uint32_t cmd)
// {
// 	spi_engine_cmd_queue *local_fifo;

// 	local_fifo = (spi_engine_cmd_queue*)malloc(sizeof(*local_fifo));

// 	if(!local_fifo)
// 		return FAILURE;

// 	local_fifo->cmd = cmd;
// 	local_fifo->next = NULL;
// 	*fifo = local_fifo;

// 	return SUCCESS;
// }

// void spi_engine_queue_add_cmd(spi_engine_cmd_queue **fifo,
// 			 uint32_t cmd)
// {
// 	spi_engine_cmd_queue *to_add = NULL;
	
// 	// Create a new element
// 	spi_engine_queue_new_cmd(&to_add, cmd);
// 	// Interchange the addresses
// 	to_add->next = *fifo;
// 	*fifo = to_add;
// }

// int32_t spi_engine_queue_get_cmd(spi_engine_cmd_queue **fifo,
// 			     uint32_t *cmd)
// {
// 	if ((*fifo)->next)
// 	{
// 		*cmd = (*fifo)->cmd;
// 		*fifo = (*fifo)->next;
// 	}
// 	else
// 	{
// 		return FAILURE;
// 	}
	
// 	return SUCCESS;
// }

// int32_t spi_engine_lifo_get_cmd(spi_engine_cmd_queue **fifo,
// 			     uint32_t *cmd)
// {
// 	spi_engine_cmd_queue *local_fifo;
// 	spi_engine_cmd_queue *last_fifo = NULL;

// 	local_fifo = *fifo;
// 	while (local_fifo->next != NULL)
// 	{
// 		// Get the last element
// 		last_fifo = local_fifo;
// 		local_fifo = local_fifo->next;
// 	}
// 	*cmd = local_fifo->cmd;
// 	if(last_fifo->next != NULL)
// 	{
// 		// Remove the last element
// 		free(last_fifo->next);
// 		last_fifo->next = NULL;
// 	}
// 	else
// 	{
// 		// Delete the fifo
// 		free(last_fifo);
// 	}
	
// 	return SUCCESS;
// }

// int32_t spi_engine_queue_free(spi_engine_cmd_buffer **fifo)
// {
// 	if ((*fifo)->next)
// 		spi_engine_queue_free(&(*fifo)->next);
// 	free(*fifo);
// 	*fifo = NULL;

// 	return SUCCESS;
// }

// /*******************************************************************************
//  *
//  * @name	spi_engine_gen_transfer
//  *
//  * @brief	Initiate an spi transfer.
//  *
//  * @param
//  *		desc		- Spi engine descriptor extra parameters
//  *		xfer		- Fifo transfer structure
//  *		write		- Write transaction flag
//  *		read		- Read transaction flag
//  *		bytes_number	- Number of bytes to transfer
//  *
//  * @return			- This function allways returns SUCCESS
//  *
//  ******************************************************************************/
// int32_t spi_engine_gen_transfer(spi_engine_desc *desc,
// 			     spi_engine_transfer_fifo *xfer,
// 			     bool write,
// 			     bool read,
// 			     uint8_t bytes_number)
// {
// 	uint8_t words_number;

// 	words_number = spi_get_words_number(desc, bytes_number) - 1;

// 	/*
// 	 * Engine Wiki:
// 	 *
// 	 * https://wiki.analog.com/resources/fpga/peripherals/spi_engine
// 	 * 
// 	 * The words number is zero based
// 	 */

// 	spi_engine_program_add_cmd(xfer,
// 				SPI_ENGINE_CMD_TRANSFER(write, read, words_number));

// 	return SUCCESS;
// }

// /*******************************************************************************
//  *
//  * @name	spi_engine_gen_cs
//  *
//  * @brief	Change the state of the chip select port
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		xfer		- Fifo transfer structure
//  *		assert		- Switch HIGH/LOW the chip select
//  *
//  * @return	void
//  *
//  ******************************************************************************/
// void spi_engine_gen_cs(spi_desc *desc,
// 		    spi_engine_transfer_fifo *xfer,
// 		    bool assert)
// {
// 	uint8_t		mask = 0xff;
// 	spi_engine_desc	*eng_desc;

// 	eng_desc = desc->extra;

// 	/* Switch the state only of the selected chip select */
// 	if (!assert)
// 		mask ^= BIT(desc->chip_select);

// 	spi_engine_program_add_cmd(xfer,
// 				SPI_ENGINE_CMD_ASSERT(eng_desc->cs_delay, mask));
// }

// /*******************************************************************************
//  *
//  * @name	spi_gen_sleep_ns
//  *
//  * @brief	Add a delay bewtheen the engine commands
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		xfer		- Fifo transfer structure
//  *		sleep_time_ns	- Number of nanoseconds
//  *
//  * @return	void
//  *
//  ******************************************************************************/
// void spi_gen_sleep_ns(spi_desc *desc,
// 		      spi_engine_transfer_fifo *xfer,
// 		      uint32_t sleep_time_ns)
// {
// 	uint32_t sleep_div;

// 	spi_get_sleep_div(desc, sleep_time_ns, &sleep_div);
// 	/* Wait for the device to do the conversion */
// 	spi_engine_program_add_cmd(xfer,
// 				SPI_ENGINE_CMD_SLEEP(sleep_div));
// }

// /*******************************************************************************
//  *
//  * @name	spi_engine_add_user_cmd
//  *
//  * @brief	Spi engine command interpreter
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		xfer		- Fifo transfer structure
//  *		cmd		- Command to send to the engine
//  *
//  * @return	void
//  *
//  ******************************************************************************/
// void spi_engine_add_user_cmd(spi_desc *desc,
// 			  spi_engine_transfer_fifo *xfer,
// 			  uint32_t cmd)
// {
// 	uint32_t	cmd_msk = (0xF << 28);
// 	uint32_t	param_msk = cmd & (~cmd_msk);
// 	uint32_t	command;
// 	uint16_t	param;
// 	spi_engine_desc	*desc_extra;

// 	desc_extra = desc->extra;

// 	command = (cmd & cmd_msk);
// 	param = (cmd & param_msk);

// 	switch(command) {
// 	case CS_DEASSERT:
// 		/* Reet the CS */
// 		spi_engine_gen_cs(desc, xfer, false);
// 		break;

// 	case CS_ASSERT:
// 		/* Set the CS */
// 		spi_engine_gen_cs(desc, xfer, true);
// 		break;

// 	case SLEEP_CMD:
// 		/* Sleep */
// 		spi_gen_sleep_ns(desc, xfer, param);
// 		break;

// 	case TRANSFER_R_CMD:
// 		if(spi_check_dma_config(desc_extra, 1, 0) == SUCCESS) {
// 			/* Read */
// 			spi_engine_gen_transfer(desc_extra,
// 					     xfer, false, true, param);
// 			desc_extra->rx_length = param;
// 		} else {
// 			printf("%s: DMA Rx not configured.\n", __func__);
// 			desc_extra->rx_length = 0;
// 		}

// 		break;

// 	case TRANSFER_W_CMD:
// 		if(spi_check_dma_config(desc_extra, 0, 1) == SUCCESS) {
// 			/* Write */
// 			spi_engine_gen_transfer(desc_extra,
// 					     xfer, true, false, param);
// 			desc_extra->tx_length = param;
// 		} else {
// 			printf("%s: DMA Tx not configured.\n", __func__);
// 			desc_extra->tx_length = 0;
// 		}
// 		break;

// 	case TRANSFER_R_W_CMD:
// 		if(spi_check_dma_config(desc_extra, 1, 1) == SUCCESS) {
// 			/* Read and write */
// 			spi_engine_gen_transfer(desc_extra,
// 					     xfer, true, true, param);
// 			desc_extra->tx_length = param;
// 			desc_extra->rx_length = param;
// 		} else {
// 			printf("%s: DMA Rx and Tx not configured.\n", __func__);
// 			desc_extra->tx_length = 0;
// 			desc_extra->rx_length = 0;
// 		}
// 		break;

// 	default:
// 		break;
// 	}
// }

// /*******************************************************************************
//  *
//  * @name	spi_engine_compile_message
//  *
//  * @brief	Prepare the command queue before sending it to the engine
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		msg		- Structure used to store the messages
//  *		xfer		- Fifo transfer structure
//  *
//  * @return			- This function allways returns SUCCESS
//  *
//  ******************************************************************************/
// int32_t spi_engine_compile_message(spi_desc *desc,
// 				spi_engine_msg *msg,
// 				spi_engine_transfer_fifo *xfer)
// {
// 	uint32_t	i;
// 	uint32_t	n;
// 	spi_engine_desc	*desc_extra;

// 	desc_extra = desc->extra;

// 	n = msg->msg_cmd_len;

// 	/* Configure the prescaler */
// 	spi_engine_program_add_cmd(xfer,
// 				SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_REG_CLK_DIV,
// 						desc_extra->clk_div));
// 	/*
// 	 * Configure the spi mode :
// 	 *	- 3 wire
// 	 *	- CPOL
// 	 *	- CPHA
// 	 */
// 	spi_engine_program_add_cmd(xfer,
// 				SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_REG_CONFIG,
// 						desc->mode));

// 	/* Set the data transfer length */
// 	spi_engine_program_add_cmd(xfer,
// 				SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_DATA_TRANSFER_LEN,
// 						desc_extra->data_width));

// 	/* Add the rest of the commands */
// 	for (i = 0; i < n; i++)
// 		spi_engine_add_user_cmd(desc, xfer, msg->spi_msg_cmds[i]);

// 	/* Add a sync command to signal that the transfer has finished */
// 	spi_engine_program_add_cmd(xfer, SPI_ENGINE_CMD_SYNC(_sync_id));

// 	return SUCCESS;
// }

// /*******************************************************************************
//  *
//  * @name	spi_engine_transfer_message
//  *
//  * @brief	Initiate a spi transfer
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		msg		- Structure used to store the messages
//  *
//  * @return			- SUCCESS if the transfer finished
//  *				- FAILURE if the memory allocation failed
//  *
//  ******************************************************************************/
// int32_t spi_engine_transfer_message(spi_desc *desc,
// 				 spi_engine_msg *msg)
// {
// 	uint32_t	size;
// 	uint32_t	i;
// 	uint32_t	data;
// 	uint32_t	sync_id;
// 	spi_engine_desc	*desc_extra;
// 	spi_engine_transfer_fifo *xfer;

// 	desc_extra = desc->extra;

// 	size = sizeof(*xfer->cmd_fifo) * (msg->msg_cmd_len + 3);

// 	xfer = (spi_engine_transfer_fifo *)malloc(sizeof(*xfer) + size);
// 	if (!xfer)
// 		return FAILURE;

// 	xfer->cmd_fifo_len = 0;

// 	spi_engine_compile_message(desc, msg, xfer);

// 	/* Write the command fifo buffer */
// 	for (i = 0; i < xfer->cmd_fifo_len; i++)
// 		spi_engine_write(desc_extra,
// 			      SPI_ENGINE_REG_CMD_FIFO, xfer->cmd_fifo[i]);

// 	/* Write a number of tx_length WORDS on the SDO line */
// 	for(i = 0; i < desc_extra->tx_length; i++)
// 		spi_engine_write(desc_extra,
// 			      SPI_ENGINE_REG_SDO_DATA_FIFO, msg->tx_buf[i]);

// 	 /* Wait for all the transactions to finish */
// 	 do {
// 	 	spi_engine_read(desc_extra, SPI_ENGINE_REG_SYNC_ID, &sync_id);
// 	 }
// 	 /* Wait for the end sync signal */
// 	 while(sync_id != _sync_id);
// 	 _sync_id = ~_sync_id;

// 	/* Read a number of rx_length WORDS from the SDI line and store them */
// 	for(i = 0; i < desc_extra->rx_length; i++) {
// 		spi_engine_read(desc_extra,
// 			     SPI_ENGINE_REG_SDI_DATA_FIFO, &data);
// 		msg->rx_buf[i] = data;
// 	}

// 	free(xfer);

// 	return SUCCESS;
// }

// /*******************************************************************************
//  *
//  * @name	spi_init
//  *
//  * @brief	Initialize the spi engine
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		param		- Structure containing the spi init parameters
//  *
//  * @return			- SUCCESS if the transfer finished
//  *				- FAILURE if the memory allocation failed
//  *
//  ******************************************************************************/
int32_t spi_engine_init(struct spi_desc **desc,
			const struct spi_init_param *param)
{
	uint32_t			data_width;
	struct spi_desc			*local_desc;
	struct spi_engine_desc		*eng_desc;
	struct spi_engine_init_param	*spi_engine_init;

	local_desc = (struct spi_desc *)malloc(sizeof(*local_desc));
	eng_desc = (struct spi_engine_desc*)malloc(sizeof(*eng_desc));

	if (!desc || !eng_desc)
		return FAILURE;

	spi_engine_init = param->extra;

	local_desc->max_speed_hz = param->max_speed_hz;
	local_desc->chip_select = param->chip_select;
	local_desc->mode = param->mode;
	local_desc->extra = eng_desc;

	eng_desc->clk_div = param->max_speed_hz /
		(2 * SPI_ENGINE_MAX_SPEED_HZ) - 1;
	eng_desc->spi_engine_baseaddr = spi_engine_init->spi_engine_baseaddr;
	//TODO: add this to offload init
	//eng_desc->offload_config = spi_engine_init->offload_config;

	/* Perform a reset */
	spi_engine_write(eng_desc, SPI_ENGINE_REG_RESET, 0x01);
	usleep(1000);
	spi_engine_write(eng_desc, SPI_ENGINE_REG_RESET, 0x00);

	/* Get current data width */
	spi_engine_read(eng_desc, SPI_ENGINE_REG_DATA_WIDTH, &data_width);
	printf("%d",data_width);
	eng_desc->max_data_width = data_width;
	eng_desc->data_width = data_width;

	*desc = local_desc;

	return SUCCESS;
}

// /*******************************************************************************
//  *
//  * @name	spi_write_and_read
//  *
//  * @brief	Write/read on the spi interface
//  *
//  * @param
//  *		desc		- Spi engine descriptor
//  *		data		- Pointer to data buffer
//  *		bytes_number	- Number of bytes to transfer
//  *

//  * @return			- SUCCESS if the transfer finished
//  *				- FAILURE if the memory allocation
//  *				or transfer failed
//  *
//  ******************************************************************************/
// int32_t spi_engine_write_and_read(spi_desc *desc,
// 			   uint8_t *data,
// 			   uint8_t bytes_number)
// {
// 	uint8_t 	i;
// 	uint8_t 	xfer_word_len;
// 	uint8_t 	xfer_words_number;
// 	uint32_t	spi_engine_msg_cmds[4];
// 	int32_t 	ret;
// 	spi_engine_msg	*msg;
// 	spi_engine_desc	*desc_extra;

// 	desc_extra = desc->extra;

// 	xfer_words_number = spi_get_words_number(desc_extra, bytes_number);

// 	/* Make sure the CS is HIGH before starting a transaction */
// 	spi_engine_msg_cmds[0] = CS_ASSERT;
// 	spi_engine_msg_cmds[1] = CS_DEASSERT;
// 	spi_engine_msg_cmds[2] = TRANSFER_BYTES_R_W(xfer_words_number);
// 	spi_engine_msg_cmds[3] = CS_ASSERT;

// 	msg = (spi_engine_msg *)malloc(sizeof(*msg));
// 	if (!msg)
// 		return FAILURE;

// 	msg->spi_msg_cmds = spi_engine_msg_cmds;
// 	msg->msg_cmd_len = ARRAY_SIZE(spi_engine_msg_cmds);

// 	msg->tx_buf =(uint32_t*)malloc(xfer_words_number * sizeof(msg->tx_buf[0]));
// 	msg->rx_buf =(uint32_t*)malloc(xfer_words_number * sizeof(msg->rx_buf[0]));

// 	/* Init the rx and tx buffers with 0s */
// 	for (i = 0; i < xfer_words_number; i++) {
// 		msg->tx_buf[i] = 0;
// 		msg->rx_buf[i] = 0;
// 	}

// 	xfer_word_len = spi_get_word_lenght(desc_extra);

// 	/* Pack the bytes into engine WORDS */
// 	for (i = 0; i < bytes_number; i++)
// 		msg->tx_buf[i / xfer_word_len] |=
// 			data[i] << (desc_extra->data_width -
// 				    (i % xfer_word_len + 1) * 8);

// 	ret = spi_engine_transfer_message(desc, msg);

// 	/* Skip the first byte ( dummy read byte ) */
// 	for (i = 1; i < bytes_number; i++)
// 		data[i - 1] = msg->rx_buf[(i) / xfer_word_len] >>
// 			      (desc_extra->data_width - ((i) % xfer_word_len + 1) * 8);

// 	free(msg->tx_buf);
// 	free(msg->rx_buf);
// 	free(msg);

// 	return ret;
// }

// /*******************************************************************************
//  *
//  * @name	spi_remove
//  *
//  * @brief	Free the resources allocated by spi_init().
//  *
//  * @param
//  *		desc		- Spi engine descriptor extra parameters
//  *
//  * @return			- This function allways returns SUCCESS
//  *
//  ******************************************************************************/
// int32_t spi_engine_remove(spi_desc *desc)
// {
// 	free(desc->extra);
// 	free(desc);

// 	return SUCCESS;
// } 

#endif //_SPI_ENGINE_
