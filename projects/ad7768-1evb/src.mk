################################################################################
#									       #
#     Shared variables:							       #
#	- PROJECT							       #
#	- DRIVERS							       #
#	- INCLUDE							       #
#	- PLATFORM_DRIVERS						       #
#	- NO-OS								       #
#									       #
################################################################################

SRCS := $(DRIVERS)/devices/adc/ad7768-1/ad77681.c		\
	$(DRIVERS)/axi_core/spi_engine/spi_engine_core.c	\
	$(DRIVERS)/axi_core/spi_engine/spi_engine_fifo.c	\
	$(DRIVERS)/axi_core/spi_engine/spi_engine_offload.c	\
	$(PLATFORM_DRIVERS)/axi_io.c				
	
# Include paths
INCS := $(DRIVERS)/devices/adc/ad7768-1/ad77681.h		\
	$(DRIVERS)/axi_core/spi_engine/spi_engine_core.h	\
	$(DRIVERS)/axi_core/spi_engine/spi_engine_offload.h	\
	$(DRIVERS)/axi_core/spi_engine/spi_engine_types.h	\
	$(INCLUDE)/axi_io.h					\
	$(INCLUDE)/spi.h					\
	$(INCLUDE)/error.h					\
	$(PLATFORM_DRIVERS)/spi_extra.h				