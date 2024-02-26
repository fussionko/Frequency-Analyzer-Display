/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "libjpeg.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h750b_discovery_qspi.h"
#include "stm32h750b_discovery_sdram.h"

#include "my_audio.h"
#include "window_function.h"

#define ARM_MATH_CM7
#include "arm_math.h"

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FFT_FLAG			0x1

#define SIZE_2_15				32768 //2^15

#define DISPLAY_QUEUE_MAX_MESSAGES			5
#define DISPLAY_QUEUE_MESSAGE_SIZE_BYTES	sizeof(fft_data_t)

#define UPDATE_QUEUE_MAX_MESSAGES			5
#define UPDATE_QUEUE_MESSAGE_SIZE_BYTES		sizeof(unsigned char)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MAP_UINT16_FLOAT(value) ((float)(value - SIZE_2_15) / SIZE_2_15)

#define FFT_PEAK_HZ(index)				((float)index * Audio.SampleRate / ((float)FFT_BUFFER_SIZE))
#define FFT_ABS_SQUARED(index, buffer) 	((buffer[index] * buffer[index]) + (buffer[index + 1] * buffer[index + 1]))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;
DMA2D_HandleTypeDef hdma2d;
JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
LTDC_HandleTypeDef hltdc;
QSPI_HandleTypeDef hqspi;
SDRAM_HandleTypeDef hsdram2;


extern uint16_t PDMBuffer[AUDIO_PDM_BUFFER_SIZE];
extern uint16_t PCMBuffer[AUDIO_PCM_BUFFER_SIZE];
extern volatile uint16_t PCMBufferIndex;

/* Definitions for GUITask */
osThreadId_t GUITaskHandle;
const osThreadAttr_t GUITask_attributes = {
  .name = "GUITask",
  .stack_size = 8192 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t ProcessFrequencyTaskHandle;
const osThreadAttr_t ProcessFrequencyTask_attributes = {
  .name = "ProcessFrequencyTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t FFTTaskHandle;
const osThreadAttr_t FFTTask_attributes = {
  .name = "FFTTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t UpdateTaskHandle;
const osThreadAttr_t UpdateTask_attributes = {
  .name = "UpdateTask",
  .stack_size = 256,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

osMessageQueueId_t DisplayQueueHandle;
const osMessageQueueAttr_t displayQueue_attributes = {
		.name = "DisplayQueue"
};

osMessageQueueId_t UpdateQueueHandle;
const osMessageQueueAttr_t updateQueue_attributes = {
		.name = "UpdateQueue"
};

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_MDMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_JPEG_Init(void);
static void MX_CRC_Init(void);
extern void TouchGFX_Task(void *argument);

void ProcessFrequencyData(void);
void WindowPrecompile();
void RemoveMean(float* buffer, uint16_t size, double mean);

__NO_RETURN void ProcessFrequencyTask(void* args);
__NO_RETURN void FFTTask(void* args);
__NO_RETURN void UpdateTask(void* args);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Window function selection
float PrecompiledWindow[NUM_WINDOW_FUNCTIONS][FFT_BUFFER_SIZE];
static uint8_t WindowFunctionIndex;


// Scaling for diffrent window functions
#ifdef FFT_MAGNITUDE_BUFFER_SCALING
float ScalingFFTBufferSum[NUM_WINDOW_FUNCTIONS];
float ScalingSquaredFFTBufferSum[NUM_WINDOW_FUNCTIONS];
#endif


// Overlap buffer setup
#if OVERLAP_FFT_BUFFER > 1

static double OverlapFFTBufferSum[OVERLAP_FFT_BUFFER];
static float OverlapFFTBuffer[OVERLAP_FFT_BUFFER][FFT_BUFFER_SIZE];
static uint16_t OverlapFFTBufferIndex[OVERLAP_FFT_BUFFER];

#else

double fftBufferInSum;
static float fftBufferIn[FFT_BUFFER_SIZE];

#endif


static float fftBufferOut[FFT_BUFFER_SIZE];

arm_rfft_fast_instance_f32 fftHandler;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();
	/* Enable the CPU Cache */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	// FFT initialization
#if (FFT_BUFFER_SIZE == 512)
	arm_rfft_512_fast_init_f32(&fftHandler);
#elif (FFT_BUFFER_SIZE == 1024)
	arm_rfft_1024_fast_init_f32(&fftHandler);
#elif (FFT_BUFFER_SIZE == 2048)
	arm_rfft_2048_fast_init_f32(&fftHandler);
#endif

	//MagnitudeBufferPtr = MagnitudeBuffer;
	WindowFunctionIndex = WINDOW_FUNCTION_NONE;

	WindowPrecompile();

	// Set overlap indices
#if OVERLAP_FFT_BUFFER > 1
	for (uint8_t i = 0; i < OVERLAP_FFT_BUFFER; ++i)
	{
		OverlapFFTBufferIndex[i] = i * (FFT_BUFFER_SIZE / OVERLAP_FFT_BUFFER);
	}
#endif
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* Explicit enabling interrupt to support debugging in CubeIDE when using external flash loader */
	__enable_irq();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_MDMA_Init();
	MX_LTDC_Init();
	MX_DMA2D_Init();
	MX_FMC_Init();
	MX_LIBJPEG_Init();
	MX_JPEG_Init();
	MX_CRC_Init();
	MX_TouchGFX_Init();
	/* Call PreOsInit function */
	MX_TouchGFX_PreOSInit();
	/* USER CODE BEGIN 2 */
	MyAudioInit();
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	DisplayQueueHandle 	= osMessageQueueNew(DISPLAY_QUEUE_MAX_MESSAGES, DISPLAY_QUEUE_MESSAGE_SIZE_BYTES, &displayQueue_attributes);
	UpdateQueueHandle 	= osMessageQueueNew(UPDATE_QUEUE_MAX_MESSAGES, UPDATE_QUEUE_MESSAGE_SIZE_BYTES, &updateQueue_attributes);
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */

	/* creation of GUITask */
	GUITaskHandle = osThreadNew(TouchGFX_Task, NULL, &GUITask_attributes);

	ProcessFrequencyTaskHandle 	= osThreadNew(ProcessFrequencyTask, NULL, &ProcessFrequencyTask_attributes);
	FFTTaskHandle 				= osThreadNew(FFTTask, NULL, &FFTTask_attributes);
	UpdateTaskHandle 			= osThreadNew(UpdateTask, NULL, &UpdateTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

void ProcessFrequencyData(void)
{
#if OVERLAP_FFT_BUFFER == 1
	static uint16_t fftIndex = 0;
#endif

	// +2 left and right channel
	for (int i = 0; i < (PDM2PCM_SAMPLES_NUMBER - 1); i += 2)
	{
		float temp = MAP_UINT16_FLOAT(PCMBuffer[PCMBufferIndex + i]);

#if OVERLAP_FFT_BUFFER > 1

		for (uint8_t fftIdx = 0; fftIdx < OVERLAP_FFT_BUFFER; ++fftIdx)
		{
			const uint16_t index = OverlapFFTBufferIndex[fftIdx];
			OverlapFFTBuffer[fftIdx][index] = temp * PrecompiledWindow[WindowFunctionIndex][index];
			OverlapFFTBufferSum[fftIdx] += OverlapFFTBuffer[fftIdx][index];

			if (++OverlapFFTBufferIndex[fftIdx] == FFT_BUFFER_SIZE)
			{
#ifdef FFT_REMOVE_MEAN

				const double mean = OverlapFFTBufferSum[fftIdx] / (double)FFT_BUFFER_SIZE;
				RemoveMean(&OverlapFFTBuffer[fftIdx][0], FFT_BUFFER_SIZE, mean);

#endif
				arm_rfft_fast_f32(&fftHandler, (float32_t*)&OverlapFFTBuffer[fftIdx], (float32_t*)&fftBufferOut, 0);

				osThreadFlagsSet(FFTTaskHandle, FFT_FLAG);

				OverlapFFTBufferSum[fftIdx] = 0;
				OverlapFFTBufferIndex[fftIdx] = 0;
			}
		}

#else

			fftBufferIn[fftIndex] = temp * PrecompiledWindow[WindowFunctionIndex][fftIndex];
			fftBufferInSum += fftBufferIn[fftIndex];

			if (++fftIndex == FFT_BUFFER_SIZE)
			{
#ifdef FFT_REMOVE_MEAN

				const double mean = fftBufferInSum / (double)FFT_BUFFER_SIZE;
				RemoveMean(fftBufferIn, FFT_BUFFER_SIZE, mean);

#endif
				arm_rfft_fast_f32(&fftHandler, (float32_t*)&fftBufferIn, (float32_t*)&fftBufferOut, 0);

				osThreadFlagsSet(FFTTaskHandle, FFT_FLAG);

				fftBufferInSum = 0.0;
				fftIndex = 0;
			}
#endif
	}
}

__NO_RETURN void UpdateTask(void* args)
{
	while (1)
	{
		unsigned char UpdateCode = 0;
		volatile osStatus_t status = osMessageQueueGet(UpdateQueueHandle, &UpdateCode, NULL, 0U);
		if (status == osOK)
		{
			switch (UpdateCode)
			{
			case WINDOW_FUNCTION_HANNING: 	WindowFunctionIndex = WINDOW_FUNCTION_HANNING; 	break;
			case WINDOW_FUNCTION_HAMMING: 	WindowFunctionIndex = WINDOW_FUNCTION_HAMMING; 	break;
			case WINDOW_FUNCTION_FLATTOP: 	WindowFunctionIndex = WINDOW_FUNCTION_FLATTOP; 	break;
			case WINDOW_FUNCTION_BARTLETT: 	WindowFunctionIndex = WINDOW_FUNCTION_BARTLETT; break;
			default: 						WindowFunctionIndex = WINDOW_FUNCTION_NONE;
			}
		}
	}
}

__NO_RETURN void ProcessFrequencyTask(void* args)
{
	uint32_t flags = 0;
	while (1)
	{
		flags = osThreadFlagsWait(AUDIO_PDM_STATE_HALF | AUDIO_PDM_STATE_FULL, osFlagsWaitAny, osWaitForever);
		if (flags != 0)
		{
		    if(PCMBufferIndex >= AUDIO_PCM_BUFFER_SIZE)
		    {
		    	PCMBufferIndex = 0;
		    }


			if ((flags & AUDIO_PDM_STATE_HALF) != 0)
			{
				SCB_InvalidateDCache_by_Addr((uint32_t *)&PDMBuffer[0], AUDIO_PDM_BUFFER_SIZE * 2);

				PDMToPCM((uint16_t*)&PDMBuffer[0], (uint16_t *)&PCMBuffer[PCMBufferIndex]);
			}
			else
			{
				SCB_InvalidateDCache_by_Addr((uint32_t *)&PDMBuffer[AUDIO_PDM_BUFFER_SIZE / 2], AUDIO_PDM_BUFFER_SIZE * 2);

				PDMToPCM((uint16_t*)&PDMBuffer[AUDIO_PDM_BUFFER_SIZE / 2], (uint16_t *)&PCMBuffer[PCMBufferIndex]);
			}

			SCB_CleanDCache_by_Addr((uint32_t*)&PCMBuffer[PCMBufferIndex], PDM2PCM_SAMPLES_NUMBER * 2);

			ProcessFrequencyData();

			PCMBufferIndex += AUDIO_PCM_BUFFER_INDEX_MOVE;
		}
	}
}




__NO_RETURN void FFTTask(void* args)
{
	static float MagnitudeSquaredBuffer[MAGNITUDE_BUFFER_SIZE] = {0};

#if OVERLAP_FFT_BUFFER > 1

	static float sumMagnitudeSquaredBuffer[MAGNITUDE_BUFFER_SIZE] = {0};
	uint8_t overlapBufferIndex = 0;

#endif

	uint32_t flags = 0;
	while(1)
	{
		flags = osThreadFlagsWait(FFT_FLAG, osFlagsWaitAll, osWaitForever);
		if ((flags & FFT_FLAG) != 0)
		{
#if OVERLAP_FFT_BUFFER == 1
			float maxMagnitudeSquared	= 0.0f;
			float peakHz 				= 0.0f;
			double magnitudeSquaredSum 	= 0.0;
#endif

			uint16_t magnitudeSquaredBufferIndex = 0;

			for (uint16_t i = 0; i < FFT_BUFFER_SIZE; i += 2)
			{
				float magnitudeSquared = FFT_ABS_SQUARED(i, fftBufferOut);

#if OVERLAP_FFT_BUFFER > 1
				sumMagnitudeSquaredBuffer[magnitudeSquaredBufferIndex] += magnitudeSquared;
#else

				magnitudeSquaredSum += magnitudeSquared;

				if (magnitudeSquared > maxMagnitudeSquared)
				{
					maxMagnitudeSquared = magnitudeSquared;
					peakHz = FFT_PEAK_HZ(magnitudeSquaredBufferIndex);
				}

#endif

				MagnitudeSquaredBuffer[magnitudeSquaredBufferIndex] = magnitudeSquared;
				magnitudeSquaredBufferIndex++;
			}

			// Avg
#if OVERLAP_FFT_BUFFER > 1

			if (++overlapBufferIndex >= OVERLAP_FFT_BUFFER)
			{
				double magnitudeSquaredSum 	= 0.0;
				float maxMagnitudeSquared	= 0.0f;
				float peakHz 				= 0.0f;

				// Skip 0 - DC
				for (uint16_t i = 0; i < MAGNITUDE_BUFFER_SIZE; i++)
				{
					MagnitudeSquaredBuffer[i] = (sumMagnitudeSquaredBuffer[i] + MagnitudeSquaredBuffer[i]) / (float)OVERLAP_FFT_BUFFER;

					if (MagnitudeSquaredBuffer[i] > maxMagnitudeSquared)
					{
						maxMagnitudeSquared = MagnitudeSquaredBuffer[i];
						peakHz = FFT_PEAK_HZ(i);
					}


					magnitudeSquaredSum += MagnitudeSquaredBuffer[i];

					sumMagnitudeSquaredBuffer[i] = 0.0f;
				}

#ifdef FFT_MAGNITUDE_BUFFER_SCALING
				magnitudeSquaredSum *= 1.0 / ScalingFFTBufferSum[WindowFunctionIndex];
#endif
				const float linearRMS = sqrtf(magnitudeSquaredSum / MAGNITUDE_BUFFER_SIZE);

				fft_data_t fftData = { &MagnitudeSquaredBuffer[0], peakHz, linearRMS };
  				osMessageQueuePut(DisplayQueueHandle, &fftData, 0U, 0U);

  				overlapBufferIndex = 0;
			}

#else
#ifdef FFT_MAGNITUDE_BUFFER_SCALING
			magnitudeSquaredSum *= 1.0 / ScalingFFTBufferSum[WindowFunctionIndex];
#endif

			const float linearRMS = sqrtf(magnitudeSquaredSum / MAGNITUDE_BUFFER_SIZE);
			fft_data_t fftData = { &MagnitudeSquaredBuffer[0], peakHz, linearRMS };
		  	osMessageQueuePut(DisplayQueueHandle, &fftData, 0U, 0U);

#endif
		}
	}
}

void WindowPrecompile()
{
	double (*WindowFunctions[])(uint16_t, uint16_t) = {
			WindowNone, WindowHanning, WindowHamming, WindowFlatTop, WindowBartlett
	};

	for (uint8_t windowFunctionIndex = 0; windowFunctionIndex < NUM_WINDOW_FUNCTIONS; ++windowFunctionIndex)
	{
		for (uint16_t i = 0; i < FFT_BUFFER_SIZE; ++i)
		{
			PrecompiledWindow[windowFunctionIndex][i] = WindowFunctions[windowFunctionIndex](i, FFT_BUFFER_SIZE);
#ifdef FFT_MAGNITUDE_BUFFER_SCALING

			ScalingFFTBufferSum[windowFunctionIndex] += PrecompiledWindow[windowFunctionIndex][i];
			ScalingSquaredFFTBufferSum[windowFunctionIndex] += PrecompiledWindow[windowFunctionIndex][i] * PrecompiledWindow[windowFunctionIndex][i];

#endif
		}
	}
}

void RemoveMean(float* buffer, uint16_t size, double mean)
{
	for (uint16_t i = 0; i < size; ++i)
	{
		buffer[i] -= mean;
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  	/** Initializes the RCC Oscillators according to the specified parameters
  	   * in the RCC_OscInitTypeDef structure.
  	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;

	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
    	Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType 		= RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              	  	  	  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
										  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource 		= RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider 	= RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider 	= RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider 	= RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider 	= RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider 	= RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider 	= RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
    	Error_Handler();
    }
    /*
	  Note : The activation of the I/O Compensation Cell is recommended with communication  interfaces
	          (GPIO, SPI, FMC, QSPI ...)  when  operating at  high frequencies(please refer to product datasheet)
	          The I/O Compensation Cell activation  procedure requires :
	        - The activation of the CSI clock
	        - The activation of the SYSCFG clock
	        - Enabling the I/O Compensation Cell : setting bit[0] of register SYSCFG_CCCSR
	 */

	 /*activate CSI clock mondatory for I/O Compensation Cell*/
	 __HAL_RCC_CSI_ENABLE();

	 /* Enable SYSCFG clock mondatory for I/O Compensation Cell */
	 __HAL_RCC_SYSCFG_CLK_ENABLE() ;

	 /* Enables the I/O Compensation Cell */
	 HAL_EnableCompensationCell();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 39;
  hltdc.Init.VerticalSync = 8;
  hltdc.Init.AccumulatedHBP = 42;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 522;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 528;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}


/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM2 memory initialization sequence
  */
  hsdram2.Instance = FMC_SDRAM_DEVICE;
  /* hsdram2.Init */
  hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 5;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  BSP_SDRAM_DeInit(0);
  if(BSP_SDRAM_Init(0) != BSP_ERROR_NONE)
  {
    Error_Handler( );
  }
  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FRAME_RATE_Pin|RENDER_TIME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DE_GPIO_Port, LCD_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_FREQ_GPIO_Port, VSYNC_FREQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|MCU_ACTIVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FRAME_RATE_Pin RENDER_TIME_Pin */
  GPIO_InitStruct.Pin = FRAME_RATE_Pin|RENDER_TIME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DE_Pin */
  GPIO_InitStruct.Pin = LCD_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_FREQ_Pin */
  GPIO_InitStruct.Pin = VSYNC_FREQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(VSYNC_FREQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_ACTIVE_Pin */
  GPIO_InitStruct.Pin = MCU_ACTIVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(MCU_ACTIVE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


/* MPU Configuration */
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
