
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


int zuraas_count=4; 

#define abs(x) ((x)>0?(x):-(x))
int s_error,error,last_error,L,R,PID,P,I,D,position;

int cal_sum[6];
int cal_min[6];
int cal_max[6];
int L_angle=45;
int R_angle=125;

float	kp=0.07;
float	kd=0.3;
float	ki=0.01;

int max_speed=350;//350
int pos,pos_sen=0;
int min_speed=0;
int def_speed;
int t=0;
int _lastValue=0; 
int R_en,L_en,e;
int sd[6];
int flg=0;
int line_color=1; // 0 = BLACK LINE FOLLOWER 
            // 1 = WHITE LINE FOLLOWER
char buff[ 50 ];
int f=0;
uint16_t adc_value;
double volt=0;
int flag=0;
uint8_t rec_data[4];
int sum_lenght=29813;
int over_lenght=54934;
long d=0;
int err=0;
int tr[10];
int threshold=600;
int Hiz=0;
int Hde=0;
int geo=0;
int h_zuraas = 0; //Tsagaan zuraasnii too
bool p_state = false; // tooluur omno ni toolson esehiih shalgan
int h_zuraas1 = 0; //Tsagaan zuraasnii too
bool p_state1 = false; // tooluur omno ni toolson esehiih shalgan

int trSensor_max[10];
int trSensor_min[10];
int trSensor_med[10];



int Stop_s=0; int gg=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/




uint32_t adc_buf[10];
unsigned int s[8];
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



#ifdef __GNUC__   

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif


PUTCHAR_PROTOTYPE
{
	
	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1, 0xFFFF);
	return ch;
	
}






void Motor(int R, int L){
	if(R>0){
		  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	}else
	if(R<0){
//      L=L*(-1);	   
		  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
	}else{
			HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	}
	
	
	
	if(L>0){
		  HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	}else
	if(L<0){
//		  R=R*(-1);
		  HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	}else
	{
			HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	}

	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	TIM3->CCR1 = abs(R);
	TIM3->CCR2 = abs(L);
	
	
	
}
double map(double x, double in_min, double in_max, double out_min, double out_max)
{
	
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//void servo(int a){
//	
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//		
//	htim2.Instance->CCR1=map(a,0,180,25,125);
//	  

//}
void readCalibrated(){
	 for(int i=0;i<6;i++)
    {
//			  unsigned int calmin,calmax;
//        unsigned int denominator;
//				int adc_buf_10_bit[6];
			
			//s[i]=map(adc_buf[i],cal_min[i],cal_max[i],0,1000);
//			
//			calmax = cal_max[i];
//      calmin = cal_min[i];
//			
//			 denominator = calmax - calmin;

//        signed int x = 0;
//        if(denominator != 0)
//            x = (((signed long)adc_buf_10_bit[i]) - calmin)
//                * 1000 / denominator;
//        if(x < 0)
//            x = 0;
//        else if(x > 1000)
//            x = 1000;
    if(line_color==0){
			sd[i] = (cal_max[i]+cal_min[i])/2;
				if(adc_buf[i]>sd[i]){
					s[i]=0;
				}else{
					s[i]=1;
				}
		}
				else
				if(line_color==1){
				sd[i] = (cal_max[i]+cal_min[i])/2;
				if(adc_buf[i]>sd[i]){
					s[i]=1;
				}else{
					s[i]=0;
				}
			}
				
			

    
			}
		
	
		}
	


void detect_geo(){
		for(int j=9;j>5;j--){
			if(adc_buf[j]>trSensor_med[j]){
			tr[j]=1;	
		}else{
			tr[j]=0;
		}
	}
//	
	
		if(tr[9]==1 && tr[8]==1)Hiz=1; else Hiz=0;
		if(tr[7]&&tr[6])Hde=1; else Hde=0;
		//printf("%d %d %d %d\n", adc_buf[9],adc_buf[8], adc_buf[7],adc_buf[6]);
	//	printf(" %d\n",Stop_s);
	

	
////		if(Stop_s==0){
////		flg=0;
////		}

	
	
	
		  if  ( Hiz  ==  0  &&  Hde  ==  0 )  { 
    geo  =  0 ; 
  } 
	  if ( Hiz  ==  1  &&  Hde  ==  0 )  { 
		
	geo  =  1 ; 
HAL_GPIO_WritePin(Hiz_test_GPIO_Port,Hiz_test_Pin,GPIO_PIN_RESET);

	
		 HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_SET);
	
	}else{
	 HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_RESET);
	 
HAL_GPIO_WritePin(Hiz_test_GPIO_Port,Hiz_test_Pin,GPIO_PIN_SET);
	}
	
  if  ( Hiz  ==  0  &&  Hde  ==  1 )  { 
    geo  =  2 ; 
  
HAL_GPIO_WritePin(Hde_test_GPIO_Port,Hde_test_Pin,GPIO_PIN_RESET);
	
	} else{
	
HAL_GPIO_WritePin(Hde_test_GPIO_Port,Hde_test_Pin,GPIO_PIN_SET);
	
	}
  if  ( Hiz  ==  1  &&  Hde  ==  1 ) { 
    geo  =  3 ; 
  }


				
	
		
			
	

}

void bat_voltage(){
			HAL_ADC_Start_IT(&hadc2);
		adc_value = HAL_ADC_GetValue(&hadc2);
 volt=map(adc_value,0,4095,0,8)-0.23;
printf("BATTERY VOLTAGE = %2f\n",volt);
	if(volt>7.6){
	
			HAL_GPIO_WritePin(BAT_LED_GPIO_Port,BAT_LED_Pin,GPIO_PIN_RESET);
	}else if(volt<7.6 && adc_value>7.4){
	
		HAL_GPIO_WritePin(BAT_LED_GPIO_Port,BAT_LED_Pin,GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(BAT_LED_GPIO_Port,BAT_LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);		
	}else if(adc_value<7.45){
	
		HAL_GPIO_WritePin(BAT_LED_GPIO_Port,BAT_LED_Pin,GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(BAT_LED_GPIO_Port,BAT_LED_Pin,GPIO_PIN_RESET);
		HAL_Delay(50);		
	}
	
HAL_ADC_Stop_IT(&hadc2);

}


void buzzer(int count){
		for(int i=0;i<count;i++){
			 HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_SET);
			HAL_Delay(50);
			 HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_RESET);
			HAL_Delay(50);
		}
		
	}
	
void buzzer_notes(int count, int time){
		for(int i=0;i<count;i++){
			 HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_SET);
			HAL_Delay(time);
			 HAL_GPIO_WritePin(BUZZ_GPIO_Port,BUZZ_Pin,GPIO_PIN_RESET);
			HAL_Delay(time);
		}
		
	}
void shuluun(){
	
	max_speed=600;  //600
  min_speed=0;
			kp=0.02; //0.02
			kd=0.8;  //0.8
			ki=0; //0.01
}
void race(){
	   max_speed=400;//350
     min_speed=0;
			kp=0.04;
			kd=0.55;
			//ki=0.01;
	
}
void line(){
readCalibrated();
//TIM4->CNT=0;TIM1->CNT=0;
    def_speed=((max_speed+min_speed)/2);
	



	
	

		if (s[0] ==0 && s[1] ==0 && s[2] ==0 && s[3] ==0 && s[4] ==0 && s[5] ==0) {
    error = 2500;
  } else {
   		position=((s[0] * 0)+(s[1] * 1000)+(s[2] * 2000)+(s[3] * 3000)+(s[4] * 4000)+(s[5] * 5000))/(s[0]+s[1]+s[2]+s[3]+s[4]+s[5]);
  }


	
	  error=position-2500;
		
//printf("%d\n", error);
//	for(int i=0;i<6;i++){
//		
//  printf("%d ", s[i]);
//	}
//printf("\n");
	
		P=kp*error;
		I=ki*(I+error);
		D=kd*(error-last_error);
		
		PID=P+I+D;
		
		last_error=error;
		L=def_speed-PID;
	  R=def_speed+PID;
		
		if(L>max_speed)L=max_speed;
		if(R>max_speed)R=max_speed;
		if(L<min_speed)L=L;
		if(R<min_speed)R=R;
		
//printf("%d  %d \n", L,R);
//	 	HAL_Delay(100);
	Motor(R,L);
	}

	
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
//	HAL_UART_Receive_IT(&huart3,rec_data,4);
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(MODE_GPIO_Port,MODE_Pin,GPIO_PIN_SET);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
//	servo(50);
//	
do{
	Motor(0,0);
	//printf("ON UART\n");
	//bat_voltage();
	}
	while(HAL_GPIO_ReadPin(ADJ__BUTTON_GPIO_Port,ADJ__BUTTON_Pin)!=0);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
		for(int j=0;j<10;j++){
				
		for(int i=9;i>5;i--){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
		  trSensor_max[i]=adc_buf[i];
	   	 
		}
		HAL_Delay(1);
			
	}
		for(int i=9;i>5;i--){
	printf("%d ",trSensor_max[i]);

		}
			printf("\n"); 
HAL_Delay(500);

	  printf("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n ");
buzzer(1);
while(HAL_GPIO_ReadPin(ADJ__BUTTON_GPIO_Port,ADJ__BUTTON_Pin)!=0);
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
		for(int j=0;j<10;j++){
				
		for(int i=9;i>5;i--){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
		  trSensor_min[i]=adc_buf[i];
	   	 
		}
		HAL_Delay(1);
			
	}
		for(int i=9;i>5;i--){
	printf("%d ",trSensor_min[i]);

		}
			printf("\n"); 
HAL_Delay(500);

	  printf("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n ");
buzzer(1);
for(int i=9;i>5;i--){
trSensor_med[i]=(trSensor_max[i]+trSensor_min[i])/2;
printf("%d ",trSensor_med[i]);
}
printf("\n"); 
buzzer(1);
while(HAL_GPIO_ReadPin(ADJ__BUTTON_GPIO_Port,ADJ__BUTTON_Pin)!=0);
	
//servo(85);
//	HAL_Delay(1000);
//servo(130);
//	HAL_Delay(1000);
		do{
		detect_geo();
		
}

while(HAL_GPIO_ReadPin(START_GPIO_Port,START_Pin)!=0);
buzzer(1);
//Motor(512,512);
//HAL_Delay(1000);
//Motor(0,0);
		for(int j=0;j<10;j++){
				
		for(int i=0;i<6;i++){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
		  cal_max[i]=adc_buf[i];
	   	 
		}
		HAL_Delay(1);
			
	}
		for(int i=0;i<6;i++){
	printf("%d ",cal_max[i]);

		}
			printf("\n"); 
HAL_Delay(500);

	  printf("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n ");
//		Motor(400,400);
//		HAL_Delay(500);
//		Motor(0,0);
		
while(HAL_GPIO_ReadPin(START_GPIO_Port,START_Pin)!=0);
		buzzer(1);
for(int j=0;j<10;j++){
				
		for(int i=0;i<6;i++){
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 10);
		  cal_min[i]=adc_buf[i];

		}
		HAL_Delay(1);

	}
	for(int i=0;i<6;i++){
	printf("%d ",cal_min[i]);

		}
			printf("\n"); 
HAL_Delay(500);
		
		
				
		hurd:
	do{
		if(HAL_GPIO_ReadPin(ADJ__BUTTON_GPIO_Port,ADJ__BUTTON_Pin)==0||t==1){
			t=1;
			
			HAL_GPIO_WritePin(MODE_GPIO_Port,MODE_Pin,GPIO_PIN_RESET);
      race();
	//		shuluun();
			flg=1;
		buzzer_notes(1,50);
		}else{
			HAL_GPIO_WritePin(MODE_GPIO_Port,MODE_Pin,GPIO_PIN_SET);
		}
	}
	while(HAL_GPIO_ReadPin(START_GPIO_Port,START_Pin)!=0);
		buzzer(2);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf,10);
			 // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf,6);

		
		
		
//		do{
		HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);

//    	max_speed=200;
//	   R_en=TIM4->CNT;
//  	 L_en=TIM1->CNT;
//     L_en=60000-L_en;
////		 
//		e=(L_en+R_en)/2;	 
////		 if(e<sum_lenght){
//			// d=d+e;
//line();		
//	//d=d/1000;
//		// }
//		//29813
//		//printf("%d\n",d/1000);
//		//HAL_Delay(1);
//	}while(e<sum_lenght);

//Motor(150,150);

////while(1);
//		max_speed=450;
		
	
//	HAL_Delay(200);
	
	

//		TIM4->CNT=0;
//		TIM1->CNT=0;
  while (1)
  {
	
//	detect_geo();
//		
//		if(h_zuraas == zuraas_count) { Motor(20,20); h_zuraas=0;  buzzer_notes(30,5); HAL_Delay(500); Motor(0,0); goto hurd; }

//if(( geo == 2 )&& p_state == 0) {h_zuraas++; p_state = 1; HAL_Delay(50); }
//  else if(geo != 2) p_state = 0; 
	
	//		int R_en=TIM4->CNT;
//  	int L_en=TIM1->CNT;
//     L_en=60000-L_en;
//		
//		int e=(L_en+R_en)/2;
		//Motor(-30,-30);

	//	e=0;
  	//printf("%d %d\n",L_en,R_en);
//		line();
		
//		do{
//			
//readCalibrated();
		
		
		
	line();	
		
	
HAL_UART_Receive(&huart3,rec_data,4,1);

//	if(rec_data[0]==49){
//		do{
//		//printf("OFF\n");
//		Motor(0,0);
//	//	buzzer(3);
//		HAL_UART_Receive(&huart3,rec_data,4,1);
//		}while(rec_data[0]!=50);
//	}
	switch(rec_data[0]){
	case 49:
	kp=kp+0.01;
	printf("kp = %f\n",kp);	
	break;
	case 50:
	kp=kp-0.01;
	printf("kp = %f\n",kp);	
	break;
	
	}
	
	
	
//	}
//	if(rec_data[0]==51){
//		Motor(400,400);
//	}else if(rec_data[0]==52){
//		Motor(-400,-400);
//	}else if(rec_data[0]==53){
//		Motor(-400,400);
//	}else if(rec_data[0]==54){
//		Motor(400,-400);
//	}else{
//		Motor(0,0);
//		
//	}
	//if(


//HAL_Delay(100);
	
//		if(err>0)err=err*(-1);
//		if(err<0)err=err*(-1);
//if(HAL_GPIO_ReadPin(ADJ__BUTTON_GPIO_Port,ADJ__BUTTON_Pin)==0){
//	Motor(0,0);
//}else{
//	 R_en=TIM4->CNT;
//  	 L_en=TIM1->CNT;
//     //L_en=60000-L_en;
//    err=30000-R_en;
////int	err1=30000-L_en;
//		//printf("%d\n",err1);
//  	Motor(0,err);
//}

////		 
//		e=(L_en+R_en)/2;	 
////		 if(e<sum_lenght){
//			 d=d+e;

//			 //d=d/1000;
//		// }
//		//29813
//		printf("%d\n",d/1000);
//		//HAL_Delay(1);
//		if(d/1000>sum_lenght){
//Motor(0,0);
//while(1);
//		}
//		main();
		
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1280;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 512;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 256;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Hde_test_Pin|BUZZ_Pin|AIN2_Pin|AIN1_Pin 
                          |BIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BAT_LED_Pin|Hiz_test_Pin|MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Hde_test_Pin BUZZ_Pin AIN2_Pin AIN1_Pin 
                           BIN1_Pin */
  GPIO_InitStruct.Pin = Hde_test_Pin|BUZZ_Pin|AIN2_Pin|AIN1_Pin 
                          |BIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : START_Pin ADJ__BUTTON_Pin */
  GPIO_InitStruct.Pin = START_Pin|ADJ__BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : STOP_SENSOR_Pin */
  GPIO_InitStruct.Pin = STOP_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(STOP_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BIN2_Pin */
  GPIO_InitStruct.Pin = BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BIN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_LED_Pin Hiz_test_Pin MODE_Pin */
  GPIO_InitStruct.Pin = BAT_LED_Pin|Hiz_test_Pin|MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */






	
	
	
	
	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
