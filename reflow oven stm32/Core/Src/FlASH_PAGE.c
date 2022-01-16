/*
 * FlASH_PAGE.c
 *
 *  Created on: 08-May-2020
 *      Author: controllerstech
 */

#include "FLASH_PAGE.h"
#include "string.h"
#include "stdio.h"


static uint32_t GetPage(uint32_t Address)
{
  for (uint32_t indx=0; indx<128; indx++)
  {
	  if((Address < (0x08000000 + (1024 *(indx+1))) ) && (Address >= (0x08000000 + 1024*indx)))
	  {
		  return (0x08000000 + 1024*indx);
	  }
  }

  return -1;
}

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *DATA_32, uint32_t NumberWords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	int numberofwords = NumberWords;//(strlen(DATA_32)/4) + ((strlen(DATA_32) % 4) != 0);

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	  uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords*4;
	  uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.PageAddress = StartPage;
	   EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		  return HAL_FLASH_GetError ();
	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, DATA_32[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();

	   return 0;
}


void Flash_Read_Data (uint32_t StartPageAddress, __IO uint32_t * DATA_32)
{
	while (1)
	{
		*DATA_32 = *(__IO uint32_t *)StartPageAddress;
		if (*DATA_32 == 0xffffffff)
		{
			*DATA_32 = '\0';
			break;
		}
		StartPageAddress += 4;
		DATA_32++;
	}
}

void Convert_To_Str (const char *data, char *str)
{
	int numberofbytes = ((strlen(data)/4) + ((strlen(data) % 4) != 0)) *4;

	for (int i=0; i<numberofbytes; i++)
	{
		str[i] = data[i/4]>>(8*(i%4));
	}
}

