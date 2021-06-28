/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM3
#include "arm_math.h"
#include "FLASH_PAGE.h"
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	float32_t KP ;
	float32_t Ki ;
	float32_t KD ;
	float32_t firstHeatUpRate;
	uint32_t SoakTempeture ;
	uint32_t SoakTime ;
	float32_t secondHeatUpRate;
	uint32_t ReflowTempeture ;
	uint32_t ReflowTime;
}ReflowTemplate;

uint8_t data[2];
uint8_t ReflowCurve[4000];
float temp;
float duty;
uint32_t data_flash[] = {400, 200, 0, 50, 900, 2};
arm_pid_instance_f32 PID;
ReflowTemplate ReflowParameters;
uint8_t ReflowEnable = 0;
uint16_t ReflowIndex = 0;
float32_t debug = 0;
uint8_t Cmd_End[3] = {0xFF, 0xFF, 0xFF};
uint8_t UART_Recieved_Data[5]={'p','0','x','x','x'};
uint8_t UART_Recieved_Flag = 0;
char input[20];
uint16_t PhaseIndex[5]={0};
char ConsoleMSG[20];
uint8_t TempDrawEnable = 0;
uint32_t TempDrawCounter = 0;


//ReflowTemplate ReflowDebug;

//char test2[5]={'1','.','2','3','4'};
uint8_t test2[5] = {'p','0','x','x','x'};

//void FlashReflowParameters(){
////Flash_Write_Data(0x0801FC00, (uint32_t*)ReflowParameters, 9);
//
//int i;
//uint8_t *data_p = &ReflowParameters;
//uint64_t flash_address = 0x0801FC00;//put first flash address here
//HAL_FLASH_Unlock();
//for ( i = 0; i < sizeof(ReflowTemplate); i++, data_p++, flash_address++ )
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, *data_p);
//HAL_FLASH_Lock();
//}

void SaveParameters(){
	Flash_Write_Data(0x0801FC00, &ReflowParameters, 9);
}






void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart1, UART_Recieved_Data, 5);
	UART_Recieved_Flag =1;
}

void NEXTION_SendString (char *ID, char *string){
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%s\"", ID, string);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);

}

void NEXTION_SendFloat (char *ID, float32_t number){
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%.2f\"", ID, number);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void NEXTION_SenduInt (char *ID, uint32_t number){
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%lu\"", ID, number);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void NEXTION_CMD (char *string){
	HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void UpdatePage(){
	NEXTION_SendFloat("t1",ReflowParameters.firstHeatUpRate);
	NEXTION_SenduInt("t2", ReflowParameters.SoakTempeture);
	NEXTION_SenduInt("t3", ReflowParameters.SoakTime);
	NEXTION_SendFloat("t4",ReflowParameters.secondHeatUpRate);
	NEXTION_SenduInt("t5", ReflowParameters.ReflowTempeture);
	NEXTION_SenduInt("t6", ReflowParameters.ReflowTime);
}

void NextionDrawDot(uint32_t x, uint32_t y){

	char buf[50];
	int len = sprintf(buf, "cirs %lu,%lu,2,BLUE", x, y);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void NextionDrawTemp(uint32_t x, uint32_t y){

	char buf[50];
	int len = sprintf(buf, "cirs %lu,%lu,2,RED", x, y);
	HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}



float32_t HandleKeyPad() {

	//clear Input
	for (int i = 0; i < 20; i++) {
		input[i] = 0;
	}

	uint8_t index = 0;

	NEXTION_SendString("t0", "");
	UART_Recieved_Flag = 0;

	while (strncmp((char*) UART_Recieved_Data, "enter", 5) != 0) {

		if(strncmp((char*) UART_Recieved_Data, "abbre", 5) == 0)
				return 9999;
		if(strncmp((char*) UART_Recieved_Data, "kback", 5) == 0)
				return 8888;


		if (UART_Recieved_Flag == 1) {
			input[index] = UART_Recieved_Data[4];
			UART_Recieved_Flag = 0;
			index++;
			NEXTION_SendString("t0", input);
		}
	}
	return atof(input);
}


void Update_Page_3() {
	uint8_t defaultUart[5] = {'p','3','x','x','x'};
	for(int i=0;i<5;i++){
		UART_Recieved_Data[i]=defaultUart[i];
	}

	NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);
	NEXTION_SenduInt("t1", ReflowParameters.SoakTime);
	NEXTION_SenduInt("t2", ReflowParameters.SoakTempeture);
	NEXTION_SendFloat("t3", ReflowParameters.secondHeatUpRate);
	NEXTION_SenduInt("t4", ReflowParameters.ReflowTime);
	NEXTION_SenduInt("t5", ReflowParameters.ReflowTempeture);

}

void Update_Page_0() {
	uint8_t defaultUart[5] = {'p','0','x','x','x'};
	for(int i=0;i<5;i++){
		UART_Recieved_Data[i]=defaultUart[i];
	}

	float32_t dx = 0.625/4; //275px / 400s
	float32_t dy = 0.8333; //200px / 240 Grad
	uint32_t OffsetX = 35;
	uint32_t OffsetY = 230;

		//Reflow Aktuelle Temperatur anzeigen:
		if (ReflowEnable == 1){
			TempDrawEnable = 1;

		}

		if(TempDrawEnable == 1){
			NextionDrawTemp(OffsetX + (uint32_t)((float32_t)(TempDrawCounter)*dx), OffsetY - (uint32_t)((float32_t)(temp)*dy));

		if(ReflowCurve[TempDrawCounter] == 0 ){
			TempDrawEnable = 0;
		}


		}



	NEXTION_SendFloat("t0", temp);
	NEXTION_SendFloat("t1", ReflowParameters.firstHeatUpRate);
	NEXTION_SenduInt("t3", ReflowParameters.SoakTime);
	NEXTION_SenduInt("t2", ReflowParameters.SoakTempeture);
	NEXTION_SendFloat("t4", ReflowParameters.secondHeatUpRate);
	NEXTION_SenduInt("t6", ReflowParameters.ReflowTime);
	NEXTION_SenduInt("t5", ReflowParameters.ReflowTempeture);
	NEXTION_SendString("g1", ConsoleMSG);

}

void Update_Page_2() {
	uint8_t defaultUart[5] = {'p','2','x','x','x'};
	for(int i=0;i<5;i++){
		UART_Recieved_Data[i]=defaultUart[i];
	}

	NEXTION_SendFloat("t0", ReflowParameters.KP);
	NEXTION_SendFloat("t1", ReflowParameters.Ki);
	NEXTION_SendFloat("t2", ReflowParameters.KD);
}



void HandleGui(){
	//###################Page0##########################

	if(strncmp((char *)UART_Recieved_Data, "p0xxx", 5) == 0){
			Update_Page_0();
			}

	if(strncmp((char *)UART_Recieved_Data, "p0b02", 5) == 0){
			Update_Page_3();
			}

	if(strncmp((char *)UART_Recieved_Data, "p0b00", 5) == 0){
			startReflow();
			Update_Page_0();
			}

	if(strncmp((char *)UART_Recieved_Data, "p0b01", 5) == 0){
			stopReflow();
			Update_Page_0();
			}

	if(strncmp((char *)UART_Recieved_Data, "p0b02", 5) == 0){
			Update_Page_3();
			}


	//###################Page2##########################

	  if(strncmp((char *)UART_Recieved_Data, "p2xxx", 5) == 0){
		Update_Page_2();
		}

		  if(strncmp((char *)UART_Recieved_Data, "p2b00", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.KP;
			  }
	    ReflowParameters.KP = Output;
	    PID.Kp = ReflowParameters.KP;
		arm_pid_init_f32(&PID, 1);
		Update_Page_2();
		NEXTION_CMD("page 2");

		}

		  if(strncmp((char *)UART_Recieved_Data, "p2b01", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.Ki;
			  }
			  ReflowParameters.Ki = Output;
			  	    PID.Ki = ReflowParameters.Ki;
			arm_pid_init_f32(&PID, 1);
			Update_Page_2();
			NEXTION_CMD("page 2");

		}

		  if(strncmp((char *)UART_Recieved_Data, "p2b02", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.KD;
			  }
			  ReflowParameters.KD = Output;
			 			  	    PID.Kd = ReflowParameters.KD;
			arm_pid_init_f32(&PID, 1);
			Update_Page_2();
			NEXTION_CMD("page 2");

		}

		  if(strncmp((char *)UART_Recieved_Data, "p2b03", 5) == 0){
			Update_Page_3();
		}







	//###################Page 3########################

	  if(strncmp((char *)UART_Recieved_Data, "p3xxx", 5) == 0){
		Update_Page_3();
		}

		  if(strncmp((char *)UART_Recieved_Data, "p3b00", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.firstHeatUpRate;
			  }

			  if(Output < 0.2)
				  Output = 0.2;

		ReflowParameters.firstHeatUpRate = Output;
		Update_Page_3();
		NEXTION_CMD("page 3");
		calculateReflowCurve();

		}
		//NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);

		  if(strncmp((char *)UART_Recieved_Data, "p3b01", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.SoakTime;
			  }
			ReflowParameters.SoakTime = Output;
			Update_Page_3();
			NEXTION_CMD("page 3");
			calculateReflowCurve();

		}
		//NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);

		  if(strncmp((char *)UART_Recieved_Data, "p3b02", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.SoakTempeture;
			  }
			ReflowParameters.SoakTempeture = Output;
			Update_Page_3();
			NEXTION_CMD("page 3");
			calculateReflowCurve();

		}
		//NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);

		  if(strncmp((char *)UART_Recieved_Data, "p3b03", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.secondHeatUpRate;
			  }
			  if(Output < 0.2)
						  Output = 0.2;
			ReflowParameters.secondHeatUpRate = Output;
			Update_Page_3();
			NEXTION_CMD("page 3");
			calculateReflowCurve();

		}


		//NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);

		  if(strncmp((char *)UART_Recieved_Data, "p3b04", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.ReflowTime;
			  }
			ReflowParameters.ReflowTime = Output;
			Update_Page_3();
			NEXTION_CMD("page 3");
			calculateReflowCurve();

		}
		//NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);

		  if(strncmp((char *)UART_Recieved_Data, "p3b05", 5) == 0){
			  float32_t Output =0;
			  Output = HandleKeyPad();
			  while(Output == 9999){
					uint8_t defaultUart[5] = {'x','x','x','x','x'};
					for(int i=0;i<5;i++){
						UART_Recieved_Data[i]=defaultUart[i];
					}
				  Output = HandleKeyPad();
			  }
			  if(Output == 8888){
				 Output = ReflowParameters.ReflowTempeture;
			  }
			ReflowParameters.ReflowTempeture = Output;
			Update_Page_3();
			NEXTION_CMD("page 3");
			calculateReflowCurve();

		}

		  if(strncmp((char *)UART_Recieved_Data, "p3b06", 5) == 0){
			Update_Page_2();
		}

		  if(strncmp((char *)UART_Recieved_Data, "p3b07", 5) == 0){
			Update_Page_0();
			Draw_Reflow_Curve();
			SaveParameters();
		}
		//NEXTION_SendFloat("t0", ReflowParameters.firstHeatUpRate);


//	NEXTION_SendFloat("t0",ReflowParameters.firstHeatUpRate );
//	NEXTION_SenduInt("t1",ReflowParameters.SoakTime );
//	NEXTION_SenduInt("t2",ReflowParameters.SoakTempeture );
//	NEXTION_SendFloat("t3",ReflowParameters.secondHeatUpRate );
//	NEXTION_SenduInt("t4",ReflowParameters.ReflowTime );
//	NEXTION_SenduInt("t5",ReflowParameters.ReflowTempeture );
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	float32_t dx = 0.625/4; //275px / 880s / 500ms
	float32_t dy = 0.8333; //200px / 240 Grad
	uint32_t OffsetX = 35;
	uint32_t OffsetY = 230;

	TempDrawCounter++;

	if (htim == &htim4) {
		//Thermocouple alle 500ms auslesen:
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
		HAL_SPI_Receive(&hspi1, data, 2, 100);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
		temp = ((((uint16_t) data[1] << 8) | data[2]) >> 3) * 0.249;

		//Reflow Prozess Einleiten:
		if (ReflowEnable == 1) {
			//NextionDrawTemp(OffsetX + (uint32_t)((float32_t)(ReflowIndex)*dx), OffsetY - (uint32_t)((float32_t)(temp)*dy));

			if(ReflowIndex == PhaseIndex[0])
				sprintf(ConsoleMSG,"HEAT UP");
			if(ReflowIndex == PhaseIndex[1])
				sprintf(ConsoleMSG,"SOAK");
			if(ReflowIndex == PhaseIndex[2])
				sprintf(ConsoleMSG,"HEAT UP");
			if(ReflowIndex == PhaseIndex[3])
				sprintf(ConsoleMSG,"REFLOW");
			if(ReflowIndex == PhaseIndex[4])
				sprintf(ConsoleMSG,"COOL DOWN");







			//Regelabweichung
			float pid_error =  ReflowCurve[ReflowIndex] - temp;
			//Stellgroesse
			duty =  arm_pid_f32(&PID, pid_error);

			//Stellgrößenbegrenzung und Anti-Wind-UP (update 27.03.2021)
			if (duty > 1000) {
				duty = 1000;
				PID.Ki = 0;
			} else if (duty < 0) {
				duty = 0;
			}
			else{
				PID.Ki = ReflowParameters.Ki;
			}

			//Dutycycle Anpassen
			htim1.Instance->CCR1 = (uint16_t)duty;

			ReflowIndex++;
			//Abbruchbedingung
			if (ReflowIndex == PhaseIndex[4]) {
				sprintf(ConsoleMSG,"FINISHED");
				ReflowEnable = 0;
			}

		} else {
			ReflowIndex = 0;
			//Dutycycle = 0
			htim1.Instance->CCR1 = 0;
		}

	}

}

void setReflowParameters(){
	ReflowParameters.KP = 10;
	ReflowParameters.Ki = 10;
	ReflowParameters.KD = 0;
	ReflowParameters.firstHeatUpRate = 1.2;
	ReflowParameters.SoakTempeture = 100;
	ReflowParameters.SoakTime = 100;
	ReflowParameters.secondHeatUpRate = 1.2;
	ReflowParameters.ReflowTempeture = 210;
	ReflowParameters.ReflowTime =100;
	sprintf(ConsoleMSG,"IDLE");
}

void calculateReflowCurve(){
	for(int i =0;i<4000;i++){
		ReflowCurve[i]=0;
	}

	int index = 0;
	float timestep = 0.5;
	//First Heat Up:
	while (24 + timestep * ReflowParameters.firstHeatUpRate <= ReflowParameters.SoakTempeture) {
	ReflowCurve[index] = 24 + timestep * ReflowParameters.firstHeatUpRate;
	index++;
	timestep = timestep + 0.5;
	}
	PhaseIndex[1]=index;

	//Soak
	int Soakduration = 2*ReflowParameters.SoakTime;

	for(int i=0;i<Soakduration;i++){
		ReflowCurve[index+i]=ReflowParameters.SoakTempeture;
	}


	//Second Heat Up:
	index = index + Soakduration;
	PhaseIndex[2]=index;
	timestep = 0.5;
	while (ReflowParameters.SoakTempeture + timestep * ReflowParameters.secondHeatUpRate <= ReflowParameters.ReflowTempeture) {
	ReflowCurve[index] = ReflowParameters.SoakTempeture + (uint8_t)timestep * ReflowParameters.secondHeatUpRate;
	index++;
	timestep = timestep + 0.5;
	}
	PhaseIndex[3]=index;

	//Reflow
	int Reflowduration = 2*ReflowParameters.ReflowTime;

	for(int i=0;i<Reflowduration;i++){
		ReflowCurve[index+i]=ReflowParameters.ReflowTempeture;
	}

	index = index + Reflowduration;
	ReflowCurve[index]=0;
	PhaseIndex[4]=index;

	//Cooldown
	timestep = 0.5;
	while (ReflowParameters.ReflowTempeture - timestep * 1.8 >= 24) {
	ReflowCurve[index] = ReflowParameters.ReflowTempeture - timestep * 1.8;
	index++;
	timestep = timestep + 0.5;
	}




}

void Draw_Reflow_Curve(){
	float32_t dx = 0.625 / 4; //275px / 880s / 500 ms
	float32_t dy = 0.8333; //200px / 240 Grad
	uint32_t OffsetX = 35;
	uint32_t OffsetY = 230;
	uint32_t index = 0;



	while(ReflowCurve[index] != 0){

		NextionDrawDot(OffsetX + (uint32_t)((float32_t)(index)*dx), OffsetY - (uint32_t)((float32_t)(ReflowCurve[index])*dy));
		index= index + 4;

		if(strncmp((char *)UART_Recieved_Data, "p0b02", 5) == 0)
			break;
	}
}



void startReflow(){
ReflowEnable = 1;
NEXTION_CMD("page 0");
Draw_Reflow_Curve();
TempDrawCounter = 0;
Update_Page_0();


}

void stopReflow(){
	if(ReflowEnable ==1){
	ReflowEnable = 0;
	TempDrawEnable = 0;
	sprintf(ConsoleMSG,"STOPPED");
	Update_Page_0();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	setReflowParameters();
	Flash_Read_Data(0x0801FC00, &ReflowParameters);
	calculateReflowCurve();




	PID.Kp = ReflowParameters.KP;
	PID.Ki = ReflowParameters.Ki;
	PID.Kd = ReflowParameters.KD;

	arm_pid_init_f32(&PID, 1);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  htim1.Instance->CCR1 = 10;

  //startReflow();
	HAL_Delay(2000);
  HAL_UART_Receive_IT(&huart1, UART_Recieved_Data, 5);
 sprintf(ConsoleMSG,"IDLE");
 Update_Page_0();
  Draw_Reflow_Curve();
  HAL_UART_Receive_IT(&huart1, UART_Recieved_Data, 5);
 // SaveParameters();



  //HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //char k[] = "1.2";
	  //debug = atof(k);

	  //debug = ReflowCurve[ReflowIndex];
//	  if(UART_Recieved_Data[3] == '1'){
//	  NEXTION_SendFloat("t0",33);
//	  debug = 5;
//	  }
//	  else{
//      NEXTION_SendFloat("t0",66);
//      debug = 3;
//	  }
	  HandleGui();
	  HAL_Delay(500);


	  if(strncmp((char *)UART_Recieved_Data, "p0xxx", 5) == 0){


		  debug = 5;

	  }

	  //NextionDrawDot(200,200);



	  //debug = HandleKeyPad();
	  //debug = atof(input);

	  //UpdatePage();



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
