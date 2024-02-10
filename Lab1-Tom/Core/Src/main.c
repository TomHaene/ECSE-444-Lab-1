/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float q;
	float r;
	float x;
	float p;
	float k;

}kalman_state;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM_MATH_CM4


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t count = 0;
float diffArray[5];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern int kalmanfilter_single_asm(int *ptr, float measurement); //The ASM subroutine

extern void calculateDifference(float* inputArray, float* outputArray, float* differenceArray, int size);
extern void calculateStats(const float* differenceArray, int size, float *average, float *stdDev);
extern float calculateCorrelation(const float* x, const float* y, int size);
extern void convolution(const float* x, const float* h, int N, float result[]);



extern void compute_error_dsp(float *reference_lst, float *tracked_lst, float *error, size_t length);
extern float compute_mean_dsp(float *array, size_t length);
float compute_stddev_dsp(float *array, size_t length, int is_sample);
float compute_correlation_dsp(float *x, float *y, int size);
void compute_convolution_dsp(const float *x, int N, const float *h, int M, float *result);




/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int kalmanfilter_ASM(float* inputArray, float* outputArray, kalman_state* myState, int size){
	int result;
	for (uint32_t i = 0; i < size ; i++){
		  	result = kalmanfilter_single_asm(myState, inputArray[i]);  // Kalman algorithm in assembly
		  	if (result == 1){
		  		return 1; //Error code
		  	}
		  	outputArray[i] = myState->x;
	      }

	return 0;
}




int Kalmanfilter_C(float* InputArray, float* OutputArray, kalman_state* kstate, int Length){
	//Since I have the pointer to the struct I need to use the arrow operator to get fields
		 // Guard against null pointers.
		  if (InputArray == NULL || OutputArray == NULL || kstate == NULL || Length <= 0){
		    return -1;
		  }

		  uint32_t fpscrValue;
		  uint32_t bitmask = 0x0000000F;
		  uint32_t resetBitmask = 0xFFFFFFE0;
		  uint32_t result;

		  for (int i = 0; i < Length; i++){

		    kstate->p = kstate->p + kstate->q;
		    kstate->k = kstate->p / (kstate->p + kstate->r);
		    kstate->x = kstate->x + kstate->k * (InputArray[i] - kstate->x);
		    kstate->p = (1 - kstate->k) * kstate->p;
		    OutputArray[i] = kstate->x;


			fpscrValue = __get_FPSCR();
		    result = fpscrValue & bitmask;

		    //Check if the result is non-zero
		    //I also allow bit 4 to be set because that can happen in C but doesn't mean there was
		    //a exception
		    if (result != 0 && result != 16) {

		    	return 1;
		    }
		  }
		  //After each iteration, set the relevant bits of the FPSCR to zero
		  result = fpscrValue & resetBitmask;
		  __set_FPSCR(result);
		  return 0;
	}



// CMSIS DSP implementation.
int Kalmanfilter_DSP(float *InputArray, float *OutputArray, kalman_state *kstate, int Length){
  // Guard against null pointers.
  if (InputArray == NULL || OutputArray == NULL || kstate == NULL || Length <= 0){
    return -1;
  }
  uint32_t fpscrValue;
  uint32_t bitmask = 0x0000000F;
  uint32_t resetBitmask = 0xFFFFFFE0;
  uint32_t fpscr_result;
  for (int i = 0; i < Length; i++)
  {
    // Use single-element arrays to hold scalar values
    float32_t p_array[1] = {kstate->p};
    float32_t q_array[1] = {kstate->q};
    float32_t r_array[1] = {kstate->r};
    float32_t result[1];
    // p = p + q (scalar addition using single-element arrays)
    p_array[0] = kstate->p;
    q_array[0] = kstate->q;
    r_array[0] = kstate->r;
    arm_add_f32(p_array, q_array, result, 1);
    kstate->p = result[0]; // Update p with the result

    // TODO? k = p / (p + r), since no division in CMSIS-DSP
    kstate->k = kstate->p / (kstate->p + kstate->r);

    // Calculate x = x + k * (InputArray[i] - x) using CMSIS-DSP
    float32_t input_minus_x_array[1] = {InputArray[i] - kstate->x};
    float32_t k_array[1] = {kstate->k};

    p_array[0] = kstate->p;
    q_array[0] = kstate->q;
    r_array[0] = kstate->r;
    arm_mult_f32(k_array, input_minus_x_array, result, 1); // k * (input - x)
    kstate->x += result[0];                                // Update x with the result

    // Update error covariance: p = (1 - k) * p using CMSIS-DSP
    float32_t one_minus_k_array[1] = {1.0f - kstate->k};
    p_array[0] = kstate->p;
    q_array[0] = kstate->q;
    r_array[0] = kstate->r;
    arm_mult_f32(one_minus_k_array, p_array, result, 1); // (1 - k) * p
    kstate->p = result[0];                               // Update p with the result
    // Store the updated estimate in the output array
    OutputArray[i] = kstate->x;
    fpscrValue = __get_FPSCR();
    fpscr_result = fpscrValue & bitmask;

    //Check if the result is non-zero
    //I also allow bit 4 to be set because that can happen in C but doesn't mean there was
    //a exception
    if (fpscr_result != 0 && fpscr_result != 16) {
    	return 1;
    }
  }
  fpscr_result = fpscrValue & resetBitmask;
 __set_FPSCR(fpscr_result);
  // Success.
  return 0;
}


void initialize_output(float* arr, int n){
	for (int i =0; i<n; i++){
		arr[i] = 0.0;
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
  /* USER CODE BEGIN 2 */

  /*
  float TEST_ARRAY[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
    	                    10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
    	                    9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
    	                    9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
    	                    10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
    	                    10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
    	                    10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
    	                    10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
    	                    9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
    	                    10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
    	                    10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
    	                    10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
    	                    10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
    	                    10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
    	                    9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
    	                    10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
    	                    9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
    	                    10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
    	                    9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
    	                    10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
    	                    9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
    	                    10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
    	                    9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
    	                    9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
    	                    10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
    	                    9.5799256668};
*/

  //int size = sizeof(TEST_ARRAY) / sizeof(TEST_ARRAY[0]);


  float TEST_ARRAY[] = {0,2,3,5,4,5,6,4,5,4,5,5,5};
  int size = sizeof(TEST_ARRAY) / sizeof(TEST_ARRAY[0]);


  kalman_state ASM_state;
  ASM_state.q = 0.1;
  ASM_state.r = 0.1;
  ASM_state.p = 0.1;
  ASM_state.x = 5.0;
  ASM_state.k = 0.0;
  float asm_output[size];
  initialize_output(asm_output, size);

  kalman_state C_state;
  C_state.q = 0.1;
  C_state.r = 0.1;
  C_state.p = 0.1;
  C_state.x = 5.0;
  C_state.k = 0.0;
  float c_output[size];
  initialize_output(c_output, size);


  kalman_state CMSIS_state;
  CMSIS_state.q = 0.1;
  CMSIS_state.r = 0.1;
  CMSIS_state.p = 0.1;
  CMSIS_state.x = 5.0;
  CMSIS_state.k = 0.0;
  float cmsis_output[size];
  initialize_output(cmsis_output, size);


  	float asm_average_usingc;
  	float asm_stdDev_usingc;
  	float asm_diffArray_usingc[size];

	float c_average_usingc;
  	float c_stdDev_usingc;
  	float c_diffArray_usingc[size];


	float cmsis_average_usingc;
  	float cmsis_stdDev_usingc;
  	float cmsis_diffArray_usingc[size];


  	kalman_state testState;
  	testState.q = 0.0;
  	testState.r = 0.0;
  	testState.p = 0.0;
  	testState.x = 5.0;
  	testState.k = 0.0;

  	ITM_Port32(31) = 1;
  	kalmanfilter_ASM(TEST_ARRAY , asm_output, &ASM_state, size);
  	ITM_Port32(31) = 2;

  	//ASM run time: 3.214200ms - 3.097717ms = 0.116483ms

  	ITM_Port32(31) = 3;
  	Kalmanfilter_C(TEST_ARRAY, c_output, &C_state, size);
  	ITM_Port32(31) = 4;

  	//C run time: 3.334383ms - 3.214300ms = 0.120083ms

  	ITM_Port32(31) = 5;
  	Kalmanfilter_DSP(TEST_ARRAY, cmsis_output, &CMSIS_state, size);
  	ITM_Port32(31) = 6;

  	//CMSIS run time: 3.576242ms - 3.334483ms = 0.241759ms



//Doing analysis using plain C--------------------------------(Below)

//Calculating tracking difference using plain C stats function
  	calculateDifference(TEST_ARRAY, asm_output, asm_diffArray_usingc, size);
	calculateDifference(TEST_ARRAY, c_output, c_diffArray_usingc, size);
	calculateDifference(TEST_ARRAY, cmsis_output, cmsis_diffArray_usingc, size);


//Calculating average and std deviation using plain C stats function
  	calculateStats(asm_diffArray_usingc, size, &asm_average_usingc, &asm_stdDev_usingc);
  	calculateStats(c_diffArray_usingc, size, &c_average_usingc, &c_stdDev_usingc);
  	calculateStats(cmsis_diffArray_usingc, size, &cmsis_average_usingc, &cmsis_stdDev_usingc);


//Calculating correlation using plain C stats function
  	float asm_correlation_usingc = calculateCorrelation(TEST_ARRAY, asm_output, size);
  	float c_correlation_usingc = calculateCorrelation(TEST_ARRAY, c_output, size);
  	float cmsis_correlation_usingc = calculateCorrelation(TEST_ARRAY, cmsis_output, size);


//Calculating convolution using plain C stats function

  	float asm_convolution_vector[(2* size) -1];
  	float c_convolution_vector[(2* size) -1];
  	float cmsis_convolution_vector[(2* size) -1];

  	convolution(TEST_ARRAY, asm_output, size, asm_convolution_vector);
  	convolution(TEST_ARRAY, c_output, size, c_convolution_vector);
  	convolution(TEST_ARRAY, cmsis_output, size, cmsis_convolution_vector);

//Doing analysis using plain C--------------------------------(Above)

//Doing analysis using CMSIS-DSP--------------------------------(Below)




  	float asm_average_usingcmsis;
  	float asm_stdDev_usingcmsis;
  	float asm_diffArray_usingcmsis[size];
  	float asm_convolution_vector_usingcmsis[(2*size) -1];


  	float c_average_usingcmsis;
  	float c_stdDev_usingcmsis;
  	float c_diffArray_usingcmsis[size];
	float c_convolution_vector_usingcmsis[(2*size) - 1];



  	float cmsis_average_usingcmsis;
  	float cmsis_stdDev_usingcmsis;
  	float cmsis_diffArray_usingcmsis[size];
	float cmsis_convolution_vector_usingcmsis[(2*size)- 1];



  	//Calculating tracking difference using CMSIS stats function
  	compute_error_dsp(TEST_ARRAY, asm_output, asm_diffArray_usingcmsis, size);
	compute_error_dsp(TEST_ARRAY, c_output, c_diffArray_usingcmsis, size);
	compute_error_dsp(TEST_ARRAY, cmsis_output, cmsis_diffArray_usingcmsis, size);


	//Calculating average using CMSIS stats function
	asm_average_usingcmsis = compute_mean_dsp(asm_diffArray_usingcmsis, size);
	c_average_usingcmsis = compute_mean_dsp(c_diffArray_usingcmsis, size);
	cmsis_average_usingcmsis = compute_mean_dsp(cmsis_diffArray_usingcmsis, size);

	//Calculating std dev using CMSIS stats function
	asm_stdDev_usingcmsis = compute_stddev_dsp(asm_diffArray_usingcmsis, size, 0 );
	c_stdDev_usingcmsis = compute_stddev_dsp(c_diffArray_usingcmsis, size, 0 );
	cmsis_stdDev_usingcmsis = compute_stddev_dsp(cmsis_diffArray_usingcmsis, size, 0 );

	//Calculating correlation using CMSIS stats function
	float asm_correlation = compute_correlation_dsp(TEST_ARRAY, asm_output, size);
	float c_correlation= compute_correlation_dsp(TEST_ARRAY, c_output, size);
	float cmsis_correlation = compute_correlation_dsp(TEST_ARRAY, cmsis_output, size);


	//Calculating convolution using CMSIS stats function
	compute_convolution_dsp(TEST_ARRAY, size,asm_output, size, asm_convolution_vector_usingcmsis);
	compute_convolution_dsp(TEST_ARRAY, size,c_output, size, c_convolution_vector_usingcmsis);
	compute_convolution_dsp(TEST_ARRAY, size,cmsis_output, size, cmsis_convolution_vector_usingcmsis);




//Doing analysis using CMSIS-DSP--------------------------------(Above)



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len)
{
 (void)file;
 int DataIdx;

 for (DataIdx = 0; DataIdx < len; DataIdx++)
 {
	ITM_SendChar(*ptr++);

 }
 return len;
}



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
