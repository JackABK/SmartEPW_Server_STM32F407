#include "sdio_debug.h"

void SD_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void print_RCC_Clocks(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
//  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
	
	printf("SYSCLK = %u\n", RCC_Clocks.SYSCLK_Frequency);
	printf("HCLK = %u\n", RCC_Clocks.HCLK_Frequency);
	printf("PCLK1 = %u\n", RCC_Clocks.PCLK1_Frequency);
	printf("PCLK2 = %u\n", RCC_Clocks.PCLK2_Frequency);
}

#define PRINT_ROW		32	/* строка */
#define PRINT_COL		16	/* столбец */
#define PRINT_BUF_SIZE	PRINT_ROW * PRINT_COL	/* размер буфера */


//=============================================================================
// 00	NUL	^@	Пусто (конец строки)
// 01	SOH	^A	Начало заголовка
// 02	STX	^B	Начало текста
// 03	EOT	^C	Конец текста
// 04	ENQ	^D	Конец передачи
// 06	ACK	^F	Подтверждение
// 07	BEL	^G	Звонок
// 08	BS	^H	Шаг назад
// 09	HT	^I	Горизонтальная табуляция
// 0A	LF	^J	Перевод строки
// 0B	VT	^K	Вертикальная табуляция
// 0C	FF	^L	Перевод страницы
// 0D	CR	^M	Возврат каретки
// 0E	SO	^N	Выдвинуть
// 0F	SI	^O	Сдвинуть
// 10	DLE	^P	Оставить канал данных
// 11	DC1/XON	^Q	Управление устройством 1
// 12	DC2	^R	Управление устройством 2
// 13	DC3/XOFF	^S	Управление устройством 3
// 14	DC4	^T	Управление устройством 4
// 15	NAK	^U	Отрицательное подтверждение
// 16	SYN	^V	Синхронизация
// 17	ETB	^W	Конец блока передачи
// 18	CAN	^X	Отмена
// 19	EM	^Y	Конец носителя
// 1A	SUB	^Z	Замена
// 1B	ESC	^[	Escape
// 1C	FS	^\	Разделитель файлов
// 1D	GS	^]	Разделитель групп
// 1E	RS	^^	Разделитель записей
// 1F	US	^_	Разделитель полей
// 20	SP	 	Пробел
// 7F	DEL	^?	Удаление
//=============================================================================

void print_adr_HEX_str( const uint32_t ReadAddr, const uint8_t * Buff )
{
	uint32_t i = 0;
	uint32_t j = 0;
	uint16_t adr = 0;
	uint8_t Byte = Buff[0];
	
	for(i=0;i<PRINT_ROW;++i)
	{
		printf("\n");		
		// print adr
		printf("0x%08X | ", ReadAddr + i*PRINT_COL );
		
		// print HEX
		for(j=0;j<PRINT_COL;++j)
		{
			Byte = Buff[adr++];
			printf("%02X ",Byte);
		}
		
		printf("| ");
		
		adr -= PRINT_COL;
		// print STR
		for(j=0;j<PRINT_COL;++j)
		{
			Byte = Buff[adr++];
			if(Byte > 0x20)
				printf("%c",Byte);
			else
				printf("%c",'.');
		}
		printf(" |");
	}
}

void print_all_SD( uint32_t size )
{
	uint32_t ReadAddr = 0;
	SD_Error SDError;
	uint8_t Buff[PRINT_BUF_SIZE];
	
	while(size--)
	{
		//printf("k = %i\n",k);
		//SDError = SD_ReadMultiBlocksFIXED(Buff, ReadAddr, PRINT_BUF_SIZE, 1);
		SDError = SD_ReadMultiBlocks(Buff, ReadAddr, PRINT_BUF_SIZE, 1);
		if(SDError != SD_OK )
			SD_print_error( SDError );
		
		#ifdef SD_DMA_MODE
		while(SD_GetStatus() != SD_TRANSFER_OK);
		#endif
		
		print_adr_HEX_str( ReadAddr*PRINT_BUF_SIZE, Buff );
		printf(" ReadAddr = %i",ReadAddr++);
	}
}

void SD_print_error( SD_Error SDError )
{
		const char *SDErrorStr[] = {
		"SD_OK",
		"SD_CMD_CRC_FAIL",                    /*= (1), !< Command response received (but CRC check failed) */
		"SD_DATA_CRC_FAIL",                   /*= (2), !< Data bock sent/received (CRC check Failed) */
		"SD_CMD_RSP_TIMEOUT",                 /*= (3), !< Command response timeout */
		"SD_DATA_TIMEOUT",                    /*= (4), !< Data time out */
		"SD_TX_UNDERRUN",                     /*= (5), !< Transmit FIFO under-run */
		"SD_RX_OVERRUN",                      /*= (6), !< Receive FIFO over-run */
		"SD_START_BIT_ERR",                   /*= (7), !< Start bit not detected on all data signals in widE bus mode */
		"SD_CMD_OUT_OF_RANGE",                /*= (8), !< CMD's argument was out of range.*/
		"SD_ADDR_MISALIGNED",                 /*= (9), !< Misaligned address */
		"SD_BLOCK_LEN_ERR",                   /*= (10), !< Transferred block length is not allowed for the card or the number of transferred bytes does not match the block length */
		"SD_ERASE_SEQ_ERR",                   /*= (11), !< An error in the sequence of erase command occurs.*/
		"SD_BAD_ERASE_PARAM",                 /*= (12), !< An Invalid selection for erase groups */
		"SD_WRITE_PROT_VIOLATION",            /*= (13), !< Attempt to program a write protect block */
		"SD_LOCK_UNLOCK_FAILED",              /*= (14), !< Sequence or password error has been detected in unlock command or if there was an attempt to access a locked card */
		"SD_COM_CRC_FAILED",                  /*= (15), !< CRC check of the previous command failed */
		"SD_ILLEGAL_CMD",                     /*= (16), !< Command is not legal for the card state */
		"SD_CARD_ECC_FAILED",                 /*= (17), !< Card internal ECC was applied but failed to correct the data */
		"SD_CC_ERROR",                        /*= (18), !< Internal card controller error */
		"SD_GENERAL_UNKNOWN_ERROR",           /*= (19), !< General or Unknown error */
		"SD_STREAM_READ_UNDERRUN",            /*= (20), !< The card could not sustain data transfer in stream read operation. */
		"SD_STREAM_WRITE_OVERRUN",            /*= (21), !< The card could not sustain data programming in stream mode */
		"SD_CID_CSD_OVERWRITE",               /*= (22), !< CID/CSD overwrite error */
		"SD_WP_ERASE_SKIP",                   /*= (23), !< only partial address space was erased */
		"SD_CARD_ECC_DISABLED",               /*= (24), !< Command has been executed without using internal ECC */
		"SD_ERASE_RESET",                     /*= (25), !< Erase sequence was cleared before executing because an out of erase sequence command was received */
		"SD_AKE_SEQ_ERROR",                   /*= (26), !< Error in sequence of authentication. */
		"SD_INVALID_VOLTRANGE",               /*= (27),*/
		"SD_ADDR_OUT_OF_RANGE",               /*= (28),*/
		"SD_SWITCH_ERROR",                    /*= (29),*/
		"SD_SDIO_DISABLED",                   /*= (30),*/
		"SD_SDIO_FUNCTION_BUSY",              /*= (31),*/
		"SD_SDIO_FUNCTION_FAILED",            /*= (32),*/
		"SD_SDIO_UNKNOWN_FUNCTION",           /*= (33),*/
		"SD_INTERNAL_ERROR",
		"SD_NOT_CONFIGURED",
		"SD_REQUEST_PENDING",
		"SD_REQUEST_NOT_APPLICABLE",
		"SD_INVALID_PARAMETER",
		"SD_UNSUPPORTED_FEATURE",
		"SD_UNSUPPORTED_HW",
		"SD_ERROR"
	};// 	printf("!SDError: %d - %s \n", SDError, SDErrorStr[SDError] );
	
	printf("\t!SDError: %d - %s \n", SDError, SDErrorStr[SDError] );
}
void SD_PrintState( uint8_t SDCardState )
{
	const char *SDCardStateStr[] = {
		"SD_CARD_READY",
		"SD_CARD_IDENTIFICATION",
		"SD_CARD_STANDBY",
		"SD_CARD_TRANSFER",
		"SD_CARD_SENDING",
		"SD_CARD_RECEIVING",
		"SD_CARD_PROGRAMMING",
		"SD_CARD_DISCONNECTED",
		"SD_CARD_ERROR"
	};
	if(SDCardState < 8)
		printf("\tSDCardState: %d - %s\n",SDCardState,SDCardStateStr[SDCardState]);
	else 
		printf("\tSDCardState: %d - %s\n",SDCardState,SDCardStateStr[8]);
	
}

SD_Error SD_PrintInfo( void )
{
	SD_Error	SDError;

	const char *CardTypes[] = {
		"STD_CAPACITY_SD_CARD_V1_1",
		"STD_CAPACITY_SD_CARD_V2_0",
		"HIGH_CAPACITY_SD_CARD",
		"MULTIMEDIA_CARD",
		"SECURE_DIGITAL_IO_CARD",
		"HIGH_SPEED_MULTIMEDIA_CARD",
		"SECURE_DIGITAL_IO_COMBO_CARD",
		"HIGH_CAPACITY_MMC_CARD"
    };
	
	printf("Card info:\n");
	if ((SDError = SD_GetCardInfo(&SDCardInfo)) != SD_OK)
	{
		printf("\tConfigure SD card - FAILED. SD_GetCardInfo()\n");
		SD_print_error(SDError);
		return SDError;
	}
	
	if(SDError == SD_OK)
	{
		SD_PrintState(SD_GetState());
		
		printf("\tBlockSize: %d\n", 			SDCardInfo.CardBlockSize);
		printf("\tCapacity: %"PRIu64" Byte", 	SDCardInfo.CardCapacity);
		printf(" (%i MiB)\n", (uint32_t)(		SDCardInfo.CardCapacity/1024/1024));
		printf("\tType: %s\n", 					CardTypes[SDCardInfo.CardType]);
		printf("\tRCA: %d\n", 					SDCardInfo.RCA);
		printf("\tCSD.DeviceSize: %d\n", 		SDCardInfo.SD_csd.DeviceSize);
		printf("\tCSD.DeviceSizeMul: %d\n", 	SDCardInfo.SD_csd.DeviceSizeMul);
		printf("\tCID:\n");
		printf("\t\tManufacturerID: %u\n",	SDCardInfo.SD_cid.ManufacturerID);
		printf("\t\tProdName1: %u\n",		SDCardInfo.SD_cid.ProdName1);
		printf("\t\tProdName2: %u\n",		SDCardInfo.SD_cid.ProdName2);
		printf("\t\tProdRev: %u\n",			SDCardInfo.SD_cid.ProdRev);
		printf("\t\tProdSN: %u\n",			SDCardInfo.SD_cid.ProdSN);
		printf("\t\tManufactDate: %u\n",	SDCardInfo.SD_cid.ManufactDate);
		printf("\t\tCID_CRC: %u\n",			SDCardInfo.SD_cid.CID_CRC);
		
		SD_PrintState(SD_GetState());
	}
	return SD_OK;
}

SD_Error SD_PrintCardStatus( void )
{
	SD_Error		SDError;
	SD_CardStatus	SDCardStatus;
	
	printf("CardStatus:\n");
	if ((SDError = SD_GetCardStatus(&SDCardStatus)) != SD_OK)
	{
		printf("\tConfigure SD card - FAILED. SD_GetCardStatus()\n");
		SD_print_error(SDError);
		return SDError;
	}
	
	if(SDError == SD_OK)
	{
		printf("\tDAT_BUS_WIDTH = %i\n",	SDCardStatus.DAT_BUS_WIDTH);	//   __IO uint8_t DAT_BUS_WIDTH;
		printf("\tSECURED_MODE = %i\n",		SDCardStatus.SECURED_MODE);		//   __IO uint8_t SECURED_MODE;
		printf("\tSD_CARD_TYPE = %i\n",		SDCardStatus.SD_CARD_TYPE);		//   __IO uint16_t SD_CARD_TYPE;
		printf("\tSIZE_OF_PROTECTED_AREA = %i\n",SDCardStatus.SIZE_OF_PROTECTED_AREA);//   __IO uint32_t SIZE_OF_PROTECTED_AREA;
		printf("\tSPEED_CLASS = %i\n",		SDCardStatus.SPEED_CLASS);		//   __IO uint8_t SPEED_CLASS;
		printf("\tPERFORMANCE_MOVE = %i\n",	SDCardStatus.PERFORMANCE_MOVE);	//   __IO uint8_t PERFORMANCE_MOVE;
		printf("\tAU_SIZE = %i\n",			SDCardStatus.AU_SIZE);			//   __IO uint8_t AU_SIZE;
		printf("\tERASE_SIZE = %i\n",		SDCardStatus.ERASE_SIZE);		//   __IO uint16_t ERASE_SIZE;
		printf("\tERASE_TIMEOUT = %i\n",	SDCardStatus.ERASE_TIMEOUT);	//   __IO uint8_t ERASE_TIMEOUT;
		printf("\tERASE_OFFSET = %i\n",		SDCardStatus.ERASE_OFFSET);		//   __IO uint8_t ERASE_OFFSET;
	}
	return SD_OK;
}

void SD_test( void )
{	
	static uint32_t sdioclk = ((HSE_VALUE/8)*336)/6;
	SD_Error		SDError;
	
	printf("Configure SD card...\n");
	
	print_RCC_Clocks();
	printf("SDIOCLK = ((HSE_VALUE / PLL_M) * PLL_N) / PLLQ = %u\n", sdioclk);
//	printf("For init: SDIO_CK = SDIOCLK / (SDIO_INIT_CLK_DIV + 2) = %i\n",(sdioclk/(SDIO_INIT_CLK_DIV + 2)));
//	printf("For transfer: SDIO_CK = SDIOCLK / (SDIO_TRANSFER_CLK_DIV + 2) = %i\n",(sdioclk/(SDIO_TRANSFER_CLK_DIV + 2)));
	
	#ifdef SD_DMA_MODE
	printf("SD_DMA_MODE\n");
	#endif
	
	#ifdef SD_POLLING_MODE
	printf("SD_POLLING_MODE\n");
	#endif
	
	if ((SDError = SD_Init()) != SD_OK)
	{
		printf("\tConfigure SD card - FAILED. SD_Init()\n");
		SD_print_error(SDError);
	}
	
	SD_PrintInfo();
	SD_PrintCardStatus();
    printf("Configure SD card - OK.\n");
	
	if ((SDError = SD_GetCardInfo(&SDCardInfo)) != SD_OK)
	{
		printf("\tConfigure SD card - FAILED. SD_GetCardInfo()\n");
		SD_print_error(SDError);
	}	
	printf("Run print all SD in Hex. %u bloks.\n",SDCardInfo.SD_csd.DeviceSize);
	print_all_SD(SDCardInfo.SD_csd.DeviceSize);
}
//******************************************************************************

