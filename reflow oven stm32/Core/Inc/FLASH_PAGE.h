/*
 * FLASH_PAGE.h
 *
 *  Created on: 08-May-2020
 *      Author: controllerstech
 */

#ifndef INC_FLASH_PAGE_H_
#define INC_FLASH_PAGE_H_

#include "stm32f1xx_hal.h"


uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t * DATA_32, uint32_t NumberWords);
void Flash_Read_Data (uint32_t StartPageAddress, __IO uint32_t * DATA_32);
void Convert_To_Str (uint32_t *data, char *str);



/********************  FLASH_Error_Codes   ***********************//*
HAL_FLASH_ERROR_NONE      0x00U  // No error
HAL_FLASH_ERROR_PROG      0x01U  // Programming error
HAL_FLASH_ERROR_WRP       0x02U  // Write protection error
HAL_FLASH_ERROR_OPTV      0x04U  // Option validity error
*/


#endif /* INC_FLASH_PAGE_H_ */
