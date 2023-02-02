/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define G_acc (0.004) // accelerometer gain in normal mode
#define G_mag (1.5) // magnetometer gain in normal mode

#define LSM303AGR_ACC_ADDR (0x19<<1) // Accelerometer address (notre adresse est sur 7 bits, on ne perd pas l'information en faisant le décalage)
#define LSM303AGR_WHO_AM_I_A (0x0F) // Accelerometer Who am I Register

#define LSM303AGR_CTRL_REG1_A (0x20) // CTRL register 1
#define LSM303AGR_CTRL_REG5_A (0x24) // CTRL register 5
#define LSM303AGR_OUT_X_L_A (0x28) // Acceleration Registers

#define LSM303AGR_MAG_ADDR (0x1E<<1) // Magnetometer address (notre adresse est sur 7 bits, on ne perd pas l'information en faisant le décalage)
#define LSM303AGR_WHO_AM_I_M (0x4F) // Magnetometer Who am I Register

#define LSM303AGR_CFG_REG_A_M (0x60) // magnetometer cfg reg A (reboot)
#define LSM303AGR_OUTX_L_REG_M (0x68) // Magnetic field registers

// Les décalages sur les adresses sont faits en vue d'utiliser des bibliotèques

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char acc_data_write[6];
char acc_data_read[6];
char mag_data_write[6];
char mag_data_read[6];

float acc_B[3][3]={
		{3.906e-3,9.209e-8,7.720e-6},
		{-7.766e-6,3.976e-3,-1.535e-8},
		{4.623e-5,4.715e-5,3.953e-3}

};
float acc_offset[3]={9.5,-8,-3};

float mag_offX = 0 ,mag_offY = 0,mag_offZ = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void powerup_sensor(void); // power-up sensor and set basics parameters

void get_data(void); // read acceleration in acc_data_read[0:5] and magnetic field in mag_data_read[0:5]

void soft_reset(void); // reset sensor internal parameters ("factory reset")

void read_registers(void);// read all accelerometer and magnetometer registers and print it on serial

void mesure_angle(float aXm, float aYm, float aZm, float mXm, float mYm, float mZm, float* ret);

void read_registers(void);

void calibrer(void);

float radToDeg(float radvalue);

int16_t fusionbinaire(uint8_t LSB, uint8_t MSB);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void __io_putchar(uint8_t ch){
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

int16_t fusionbinaire(uint8_t LSB, uint8_t MSB){
    int16_t res=0x0000;
    if(MSB & (1<<7)){
        MSB = (uint8_t)((~MSB)+1);
        res |= LSB>>6;
        res |= (MSB<<2);
        res = (int16_t)((~res)+1);
    }else{
        res |= LSB>>6;
        res |= (MSB<<2);
    }
    return res;
}

void calibrer(void){
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = 8000;
	float minX=0,minZ=0,maxZ=0,maxX=0,minY=0,maxY=0,val1=0,val2=0,val3=0;
	mag_data_write[0] = LSM303AGR_OUTX_L_REG_M | (1<<7);
	HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)mag_data_write,1,6000);
	HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)mag_data_read,6,6000);
	maxX=fusionbinaire(mag_data_read[0],mag_data_read[1]);
	maxY=fusionbinaire(mag_data_read[2],mag_data_read[3]);
	maxZ=fusionbinaire(mag_data_read[4],mag_data_read[5]);
	minX=maxX;
	minY=maxY;
	minZ=maxZ;
	printf("\033[35mDébut de calibration (8 secondes) (Effectuez des cercles avec le niveau pour le calibrer)\n\r");
	while((HAL_GetTick() - tickstart) < wait)
	  {
		mag_data_write[0] = LSM303AGR_OUTX_L_REG_M | (1<<7);
		HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)mag_data_write,1,6000);
		HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)mag_data_read,6,6000);
		val1=fusionbinaire(mag_data_read[0],mag_data_read[1]);
		val2=fusionbinaire(mag_data_read[2],mag_data_read[3]);
		val3=fusionbinaire(mag_data_read[4],mag_data_read[5]);
		if (val1>maxX) maxX = val1;
		if (val1<minX) minX = val1;
		if (val2>maxY) maxY = val2;
		if (val2<minY) minY = val2;
		if (val3>maxZ) maxZ = val3;
		if (val3<minZ) minZ = val3;
	  }
	mag_offX=(maxX+minX)/2;
	mag_offY=(maxY+minY)/2;
	mag_offZ=(maxZ+minZ)/2;
	//printf("\033[32mFin de la calibration ! offsetX= %fdigit offsetY= %fdigit offsetZ= %fdigit\033[0m\n\r",mag_offX,mag_offY,mag_offZ);
	printf("\033[32mFin de la calibration ! offsetX= %fmG offsetY= %fmG offsetZ= %fmG\033[0m\n\r",mag_offX*G_mag,mag_offY*G_mag,mag_offZ*G_mag);
	HAL_Delay(2000);
}

void get_data(void){
	 acc_data_write[0] = LSM303AGR_OUT_X_L_A | (1<<7);
	 HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,1,6000);
	 HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_read,6,6000);

	 mag_data_write[0] = LSM303AGR_OUTX_L_REG_M | (1<<7);
	 HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)mag_data_write,1,6000);
	 HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)mag_data_read,6,6000);
}
void powerup_sensor(void){
	// Turn on accelerometer
	HAL_StatusTypeDef ret;
	// Accelerometer test
	acc_data_write[0] = LSM303AGR_WHO_AM_I_A;

	ret = HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,1,2000);
	if (ret != HAL_OK) {printf("FAILED TX1: RETURN ERROR: %d",ret);}

	HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_read,1,1000);
	printf("RECEIVED ACCELEROMETER 'WHO AM I' WITH ID : (0x%.2x)\n\r",acc_data_read[0]);

	acc_data_write[0] = LSM303AGR_WHO_AM_I_M;

	ret = HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)acc_data_write,1,2000);
	if (ret != HAL_OK) {printf("FAILED TX2: RETURN ERROR: %d",ret);}

	HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)acc_data_read,1,1000);
	printf("RECEIVED MAGNETO 'WHO AM I' WITH ID : (0x%.2x)\n\r",acc_data_read[0]);
	printf("Powerup [OK]\n\r");
	printf("\n\r");

}
float radToDeg(float radvalue){
	return radvalue * 180/M_PI;
}

void mesure_angle(float aXm, float aYm, float aZm, float mXm, float mYm, float mZm, float* ret){

	//ACCELERO
	float theta = atan(aYm/aXm);
        float psi =  atan((-aZm)/sqrt(aYm*aYm+aXm*aXm));
       // MAGNETO
        float phi = atan((mXm*sin(theta)-mYm*cos(theta))/(mZm*cos(psi)+mYm*sin(theta)*sin(psi)+mXm*cos(theta)*sin(psi)));
        float delta = acos(sqrt((mYm*aZm-mZm*aYm)*(mYm*aZm-mZm*aYm)+(mZm*aXm-mXm*aZm)*(mZm*aXm-mXm*aZm)+(mXm*aYm-mYm*aXm)*(mXm*aYm-mYm*aXm))/((sqrt(mXm*mXm+mYm*mYm+mZm*mZm)*sqrt(aXm*aXm+aYm*aYm+aZm*aZm))));
        ret[0] = radToDeg(theta);
        ret[1] = radToDeg(psi);
		ret[2] = radToDeg(phi);
		ret[3] = radToDeg(delta);
}

void soft_reset(void){
	acc_data_write[0]=LSM303AGR_CTRL_REG5_A | (1<<7);
	acc_data_write[1] = 0x40; //Reboot memory content -> en binaire 0b10000000
	HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,2,1000);
	HAL_Delay(100);
	acc_data_write[0] = LSM303AGR_CTRL_REG1_A;
	acc_data_write[1] = 0x27; //10 Hz, ENABLE X-AXIS,Y-AXIS,Z-AXIS
	HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_ACC_ADDR,(uint8_t*)acc_data_write,2,1000);
	printf("SOFT RESET [OK]\n\r");

	//MAGNETO RESET
	acc_data_write[0]=LSM303AGR_CFG_REG_A_M | (1<<7);
	acc_data_write[1] = 0b10000000;
		HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)acc_data_write,2,1000);
}

void read_registers(void){
	acc_data_write[0] = LSM303AGR_CFG_REG_A_M | (1 << 7);
	HAL_I2C_Master_Transmit(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)acc_data_write,1,6000);

	HAL_I2C_Master_Receive(&hi2c1,LSM303AGR_MAG_ADDR,(uint8_t*)acc_data_read,6,6000);

	for(int i=0;i<6;i++){
		printf("CTRL %d : 0x%.2x\n\r",i+1,acc_data_read[i]);
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
 int16_t aXm,aYm,aZm;
 int16_t mXm,mYm,mZm;
 float aXmR,aYmR,aZmR;
 float mXmR,mYmR,mZmR;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("\033[2J\033[33m\r---- PROJET NIVEAU ----\n\r\033[0m");
  soft_reset();
  powerup_sensor();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);
  calibrer();
  HAL_Delay(2000);
  while (1)
  {
	  HAL_Delay(200);
	 get_data();
	aXm=fusionbinaire(acc_data_read[0],acc_data_read[1]);
	aYm=fusionbinaire(acc_data_read[2],acc_data_read[3]);
	aZm=fusionbinaire(acc_data_read[4],acc_data_read[5]);




	aXmR=(acc_B[0][0]*(aXm-acc_offset[0]) + acc_B[0][1]*(aYm-acc_offset[1]) + acc_B[0][2]*(aZm-acc_offset[2]));
	aYmR=(acc_B[1][0]*(aXm-acc_offset[0]) + acc_B[1][1]*(aYm-acc_offset[1]) + acc_B[1][2]*(aZm-acc_offset[2]));
	aZmR=(acc_B[2][0]*(aXm-acc_offset[0]) + acc_B[2][1]*(aYm-acc_offset[1]) + acc_B[2][2]*(aZm-acc_offset[2]));


	mXm=fusionbinaire(mag_data_read[0],mag_data_read[1]);
	mYm=fusionbinaire(mag_data_read[2],mag_data_read[3]);
	mZm=fusionbinaire(mag_data_read[4],mag_data_read[5]);


	mXmR=( mXm - mag_offX)*G_mag;
	mYmR= (mYm - mag_offY)*G_mag;
	mZmR = (mZm - mag_offZ)*G_mag;

	printf("\033[2J");
	printf("\033[Hax = %d\t ay = %d\t az=%d\r\n",aXm,aYm,aZm);
	printf("aXm = %3.2fg\t aYm = %3.2fg\t aZm=%3.2fg\r\n",(float)aXm*G_acc,(float)aYm*G_acc,(float)aZm*G_acc);
	printf("aXmR = %3.2fg\t aYmR = %3.2fg\t aZmR=%3.2fg\r\n",aXmR,aYmR,aZmR);
	printf("\n\r");
	printf("mx = %d\t my = %d\t mz=%d\r\n",mXm,mYm,mZm);
	printf("mXm = %3.2fg\t mYm = %3.2fg\t mZm=%3.2fg\r\n",(float)mXm*G_mag,(float)mYm*G_mag,(float)mZm*G_mag);
	printf("mXmR = %3.2fg\t mYmR = %3.2fg\t mZmR=%3.2fg\r\n",mXmR,mYmR,mZmR);
	printf("\n\r");
	float ret[4];
	mesure_angle(aXmR,aYmR,aZmR,mXmR,mYmR,mZmR,(float*)ret);
	printf("Θ = %3.2f\n\rΨ=%3.2f\n\rφ= %3.2f\n\rδ= %3.2f\n\r ",ret[0],ret[1],ret[2],ret[3]);
	//read_registers();
	//printf("VALUE : %d \t HEX: %.4X\n\r",fusionbinaire(0x80,0x9F),fusionbinaire(0x80,0x9F));
	//printf("mx=%5d my=%5d mz=%5d\t mx = %3.2fmG\t my = %3.2fmG\t mz=%3.2fmG\r\n", mXm,mYm,mZm,((float) mXm)*G_mag,((float)mYm)*G_mag,((float)mZm)*G_mag);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
