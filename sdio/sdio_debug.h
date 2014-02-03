/*----------------------------------------------------------------------*/
#ifndef _SDIO_DEBUG_H
#define _SDIO_DEBUG_H

#include <stdio.h>
#include <inttypes.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery_sdio_sd.h"

extern SD_CardInfo	SDCardInfo;

void SD_NVIC_Configuration(void);
void print_RCC_Clocks(void);
void print_all_SD( uint32_t size );
void SD_test( void );
void SD_print_error( SD_Error SDError );
SD_Error SD_PrintCardStatus( void );
SD_Error SD_PrintInfo( void );
void SD_PrintState( uint8_t SDCardState );
void print_adr_HEX_str( const uint32_t adr, const uint8_t * Buff );

#endif /* _SDIO_DEBUG_H */
