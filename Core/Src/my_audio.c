#include "my_audio.h"

#include "wm8994/wm8994.h"
#include "stm32h750b_discovery_bus.h"

#include "pdm2pcm_glo.h"

/* SAI PDM input definitions */
#define AUDIO_IN_SAI_PDMx                       SAI4_Block_A
#define AUDIO_IN_SAI_PDMx_CLK_ENABLE()          __HAL_RCC_SAI4_CLK_ENABLE()
#define AUDIO_IN_SAI_PDMx_CLK_DISABLE()         __HAL_RCC_SAI4_CLK_DISABLE()
#define AUDIO_IN_SAI_PDMx_CLK_IN_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_IN_SAI_PDMx_CLK_IN_PIN            GPIO_PIN_5
#define AUDIO_IN_SAI_PDMx_CLK_IN_PORT           GPIOE
#define AUDIO_IN_SAI_PDMx_DATA_IN_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_IN_SAI_PDMx_DATA_IN_PIN           GPIO_PIN_4
#define AUDIO_IN_SAI_PDMx_DATA_IN_PORT          GPIOE
#define AUDIO_IN_SAI_PDMx_DATA_CLK_AF           GPIO_AF10_SAI4
#define AUDIO_IN_SAI_PDMx_IRQHandler            SAI4_IRQHandler
#define AUDIO_IN_SAI_PDMx_IRQ                   SAI4_IRQn

/* SAI PDM DMA Stream definitions */
#define AUDIO_IN_SAI_PDMx_DMAx_CLK_ENABLE()         __HAL_RCC_BDMA_CLK_ENABLE()
#define AUDIO_IN_SAI_PDMx_DMAx_STREAM               BDMA_Channel1
#define AUDIO_IN_SAI_PDMx_DMAx_REQUEST              BDMA_REQUEST_SAI4_A
#define AUDIO_IN_SAI_PDMx_DMAx_IRQ                  BDMA_Channel1_IRQn
#define AUDIO_IN_SAI_PDMx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_SAI_PDMx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define AUDIO_IN_SAI_PDMx_DMAx_IRQHandler           BDMA_Channel1_IRQHandler


#define AUDIO_FREQUENCY_48K       48000U
#define AUDIO_FREQUENCY_44K       44100U

#define AUDIO_RESOLUTION_16B	16U
#define AUDIO_RESOLUTION_32B	16U
/*------------------------------------------------------------------------------
                          USER SAI defines parameters
 -----------------------------------------------------------------------------*/
/** CODEC_AudioFrame_SLOT_TDMMode In W8994 codec the Audio frame contains 4 slots : TDM Mode
  * TDM format :
  * +------------------|------------------|--------------------|-------------------+
  * | CODEC_SLOT0 Left | CODEC_SLOT1 Left | CODEC_SLOT0 Right  | CODEC_SLOT1 Right |
  * +------------------------------------------------------------------------------+
  */
/* To have 2 separate audio stream in Both headphone and speaker the 4 slot must be activated */
#define CODEC_AUDIOFRAME_SLOT_0123                   SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2 | SAI_SLOTACTIVE_3
/* To have an audio stream in headphone only SAI Slot 0 and Slot 2 must be activated */
#define CODEC_AUDIOFRAME_SLOT_02                     SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_2
/* To have an audio stream in speaker only SAI Slot 1 and Slot 3 must be activated */
#define CODEC_AUDIOFRAME_SLOT_13                     SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_3

/* SAI peripheral configuration defines */
#define AUDIO_OUT_SAIx                           SAI2_Block_A
#define AUDIO_OUT_SAIx_CLK_ENABLE()              __HAL_RCC_SAI2_CLK_ENABLE()
#define AUDIO_OUT_SAIx_CLK_DISABLE()             __HAL_RCC_SAI2_CLK_DISABLE()

#define AUDIO_OUT_SAIx_MCLK_ENABLE()             __HAL_RCC_GPIOI_CLK_ENABLE()
#define AUDIO_OUT_SAIx_MCLK_GPIO_PORT            GPIOI
#define AUDIO_OUT_SAIx_MCLK_PIN                  GPIO_PIN_4
#define AUDIO_OUT_SAIx_MCLK_AF                   GPIO_AF10_SAI2

#define AUDIO_OUT_SAIx_SCK_ENABLE()              __HAL_RCC_GPIOI_CLK_ENABLE()
#define AUDIO_OUT_SAIx_SCK_GPIO_PORT             GPIOI
#define AUDIO_OUT_SAIx_SCK_PIN                   GPIO_PIN_5
#define AUDIO_OUT_SAIx_SCK_AF                    GPIO_AF10_SAI2

#define AUDIO_OUT_SAIx_SD_ENABLE()               __HAL_RCC_GPIOI_CLK_ENABLE()
#define AUDIO_OUT_SAIx_SD_GPIO_PORT              GPIOI
#define AUDIO_OUT_SAIx_SD_PIN                    GPIO_PIN_6
#define AUDIO_OUT_SAIx_SD_AF                     GPIO_AF10_SAI2

#define AUDIO_OUT_SAIx_FS_ENABLE()               __HAL_RCC_GPIOI_CLK_ENABLE()
#define AUDIO_OUT_SAIx_FS_GPIO_PORT              GPIOI
#define AUDIO_OUT_SAIx_FS_PIN                    GPIO_PIN_7
#define AUDIO_OUT_SAIx_FS_AF                     GPIO_AF10_SAI2

/* SAI DMA Stream definitions */
#define AUDIO_OUT_SAIx_DMAx_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_OUT_SAIx_DMAx_STREAM               DMA2_Stream1
#define AUDIO_OUT_SAIx_DMAx_REQUEST              DMA_REQUEST_SAI2_A
#define AUDIO_OUT_SAIx_DMAx_IRQ                  DMA2_Stream1_IRQn
#define AUDIO_OUT_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define AUDIO_OUT_SAIx_DMAx_IRQHandler           DMA2_Stream1_IRQHandler

/*------------------------------------------------------------------------------
                        AUDIO IN defines parameters
------------------------------------------------------------------------------*/
/* SAI peripheral configuration defines */
#define AUDIO_IN_SAIx                           SAI2_Block_B
#define AUDIO_IN_SAIx_CLK_ENABLE()              __HAL_RCC_SAI2_CLK_ENABLE()
#define AUDIO_IN_SAIx_CLK_DISABLE()             __HAL_RCC_SAI2_CLK_DISABLE()
#define AUDIO_IN_SAIx_AF                        GPIO_AF10_SAI2
#define AUDIO_IN_SAIx_SD_ENABLE()               __HAL_RCC_GPIOG_CLK_ENABLE()
#define AUDIO_IN_SAIx_SD_GPIO_PORT              GPIOG
#define AUDIO_IN_SAIx_SD_PIN                    GPIO_PIN_10

/* SAI DMA Stream definitions */
#define AUDIO_IN_SAIx_DMAx_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_IN_SAIx_DMAx_STREAM               DMA2_Stream4
#define AUDIO_IN_SAIx_DMAx_REQUEST              DMA_REQUEST_SAI2_B
#define AUDIO_IN_SAIx_DMAx_IRQ                  DMA2_Stream4_IRQn
#define AUDIO_IN_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define AUDIO_IN_SAIx_DMAx_IRQHandler           DMA2_Stream4_IRQHandler

#define AUDIO_OUT_IT_PRIORITY           14U
#define AUDIO_IN_IT_PRIORITY            15U

/* Audio codec I2C address */
#define AUDIO_I2C_ADDRESS                0x34U

#define AUDIO_IN_DEVICE_DIGITAL_MIC1      0x10U
#define AUDIO_IN_DEVICE_DIGITAL_MIC2      0x20U
#define AUDIO_IN_DEVICE_DIGITAL_MIC_LAST  AUDIO_IN_DEVICE_DIGITAL_MIC2
#define AUDIO_IN_DEVICE_DIGITAL_MIC       (AUDIO_IN_DEVICE_DIGITAL_MIC1 | AUDIO_IN_DEVICE_DIGITAL_MIC2)


#define PDM2PCM_HIGHPASS_FILTER (PDM2PCM_HIGHPASS_FILTER_COEFFICIENT * (((uint32_t)1 << 31) - 1)) // coeff * (2^31-1)

#define VOLUME_OUT_CONVERT(Volume)    (((Volume) > 100)? 63:((uint8_t)(((Volume) * 63) / 100)))

typedef struct
{
  uint32_t AudioFrequency;
  uint32_t AudioMode;
  uint32_t DataSize;
  uint32_t MonoStereoMode;
  uint32_t ClockStrobing;
  uint32_t Synchro;
  uint32_t OutputDrive;
  uint32_t SynchroExt;
  uint32_t FrameLength;
  uint32_t ActiveFrameLength;
  uint32_t SlotActive;
}MX_SAI_Config_t;

static HAL_StatusTypeDef MyMX_SAI2_Block_A_Init(SAI_HandleTypeDef *hsai, MX_SAI_Config_t *MXConfig);
static HAL_StatusTypeDef MyMX_SAI4_Block_A_Init(SAI_HandleTypeDef *hsai, MX_SAI_Config_t *MXConfig);

static HAL_StatusTypeDef MyMX_SAI2_ClockConfig(uint32_t SampleRate);
static HAL_StatusTypeDef MyMX_SAI4_ClockConfig(uint32_t SampleRate);

static HAL_StatusTypeDef MyMX_SAI2_Out_Init(void);
static HAL_StatusTypeDef MyMX_SAI4_In_Init(void);

static void MySAI_Out_MspInit(void);
static void MySAI_InPDM_MspInit(void);

static HAL_StatusTypeDef MyWM8994_Probe(void);

static void PDMToPCM_Init();

void BDMA_Channel1_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);

void HAL_DMA_HalfCpltCallback(DMA_HandleTypeDef* hdma);
void HAL_DMA_CpltCallback(DMA_HandleTypeDef* hdma);

static PDM_Filter_Handler_t PDM_FilterHandler[2];
static PDM_Filter_Config_t PDM_FilterConfig[2];

static SAI_HandleTypeDef haudio_out_sai;
static SAI_HandleTypeDef haudio_in_sai;

Audio_t Audio;

ALIGN_32BYTES (uint16_t PDMBuffer[AUDIO_PDM_BUFFER_SIZE]) __attribute__((section(".RAM_D3")));
ALIGN_32BYTES (uint16_t PCMBuffer[AUDIO_PCM_BUFFER_SIZE]);

volatile uint16_t PCMBufferIndex;

static WM8994_Object_t	WM8994Object;

HAL_StatusTypeDef MyAudioInit(void)
{
	Audio.SampleRate 		= AUDIO_FREQUENCY;
	Audio.Volume			= AUDIO_VOLUME;
	Audio.Channels 			= 2;
	Audio.BitsPerSample 	= 16;

    if (MyWM8994_Probe() != WM8994_OK)
    {
    	return HAL_ERROR;
    }

	if (MyMX_SAI4_ClockConfig(Audio.SampleRate) != HAL_OK)
	{
		return HAL_ERROR;
	}
	MySAI_InPDM_MspInit();
	MyMX_SAI4_In_Init();
	PDMToPCM_Init();

    /* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */
	if (MyMX_SAI2_ClockConfig(Audio.SampleRate) != HAL_OK)
	{
		return HAL_ERROR;
	}

    MySAI_Out_MspInit();
    MyMX_SAI2_Out_Init();

    WM8994_Init_t codec_init;
    codec_init.Resolution  = 0;

    /* Fill codec_init structure */
    codec_init.Frequency    = Audio.SampleRate;
    codec_init.InputDevice  = AUDIO_IN_DEVICE_DIGITAL_MIC;
    codec_init.OutputDevice = WM8994_OUT_HEADPHONE;

    /* Convert volume before sending to the codec */
    codec_init.Volume       = VOLUME_OUT_CONVERT(Audio.Volume);

    /* Initialize the codec internal registers */
    if (WM8994_Init(&WM8994Object, &codec_init) != 0)
    {
        return HAL_ERROR;
    }

	if (HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)PDMBuffer, (uint16_t)(AUDIO_PDM_BUFFER_SIZE)) != HAL_OK)
    {
    	return HAL_ERROR;
    }

	if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)PCMBuffer, (uint16_t)(AUDIO_PCM_BUFFER_SIZE)) != HAL_OK)
    {
    	return HAL_ERROR;
    }

    if (WM8994_Play(&WM8994Object) < 0)
    {
    	return HAL_ERROR;
    }


	return HAL_OK;
}

static HAL_StatusTypeDef MyMX_SAI2_Out_Init(void)
{
	 MX_SAI_Config_t mx_sai_config;

	  /* Prepare haudio_out_sai handle */
	  mx_sai_config.AudioFrequency    = Audio.SampleRate;
	  mx_sai_config.AudioMode         = SAI_MODEMASTER_TX;
	  mx_sai_config.ClockStrobing     = SAI_CLOCKSTROBING_RISINGEDGE;
	  mx_sai_config.MonoStereoMode    = (Audio.Channels == 1U) ? SAI_MONOMODE : SAI_STEREOMODE;
	  mx_sai_config.DataSize          = (Audio.BitsPerSample == AUDIO_RESOLUTION_32B) ? SAI_DATASIZE_32 : SAI_DATASIZE_16;
	  mx_sai_config.FrameLength       = 128;
	  mx_sai_config.ActiveFrameLength = 64;
	  mx_sai_config.OutputDrive       = SAI_OUTPUTDRIVE_ENABLE;
	  mx_sai_config.Synchro           = SAI_ASYNCHRONOUS;
	  mx_sai_config.SynchroExt        = SAI_SYNCEXT_DISABLE;
	  mx_sai_config.SlotActive        = CODEC_AUDIOFRAME_SLOT_02;

	  haudio_out_sai.Instance 			= SAI2_Block_A;

	  /* SAI peripheral initialization: this __weak function can be redefined by the application  */
	  return MyMX_SAI2_Block_A_Init(&haudio_out_sai, &mx_sai_config);
}

static HAL_StatusTypeDef MyMX_SAI4_In_Init(void)
{
	MX_SAI_Config_t mx_sai_config;

	mx_sai_config.MonoStereoMode    = SAI_STEREOMODE;
	mx_sai_config.DataSize          = SAI_DATASIZE_16;
	mx_sai_config.FrameLength       = 32;
	mx_sai_config.ActiveFrameLength = 1;
	mx_sai_config.OutputDrive 		= SAI_OUTPUTDRIVE_DISABLE;
	mx_sai_config.SlotActive 		= SAI_SLOTACTIVE_1;

	/* Prepare haudio_out_sai handle */
	haudio_in_sai.Instance 			= SAI4_Block_A;
	mx_sai_config.AudioFrequency    = Audio.SampleRate * 8;
	mx_sai_config.AudioMode 		= SAI_MODEMASTER_RX;
	mx_sai_config.ClockStrobing     = SAI_CLOCKSTROBING_FALLINGEDGE;
	mx_sai_config.Synchro 			= SAI_ASYNCHRONOUS;
	mx_sai_config.SynchroExt 		= SAI_SYNCEXT_DISABLE;

	return MyMX_SAI4_Block_A_Init(&haudio_in_sai, &mx_sai_config);
}

static HAL_StatusTypeDef MyWM8994_Probe(void)
{
	WM8994_IO_t IOCtx;

	/* Configure the audio driver */
	IOCtx.Address     = AUDIO_I2C_ADDRESS;
	IOCtx.Init        = BSP_I2C4_Init;
	IOCtx.DeInit      = BSP_I2C4_DeInit;
	IOCtx.ReadReg     = BSP_I2C4_ReadReg16;
	IOCtx.WriteReg    = BSP_I2C4_WriteReg16;
	IOCtx.GetTick     = BSP_GetTick;

	if(WM8994_RegisterBusIO (&WM8994Object, &IOCtx) != WM8994_OK)
	{
		return HAL_ERROR;
	}
	if(WM8994_Reset(&WM8994Object) != WM8994_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef MyMX_SAI2_Block_A_Init(SAI_HandleTypeDef *hsai, MX_SAI_Config_t *MXConfig)
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* Disable SAI peripheral to allow access to SAI internal registers */
	__HAL_SAI_DISABLE(hsai);

	/* Configure SAI1_Block_A */
	hsai->Init.MonoStereoMode = MXConfig->MonoStereoMode;
	hsai->Init.AudioFrequency = MXConfig->AudioFrequency;
	hsai->Init.AudioMode = MXConfig->AudioMode;
	hsai->Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai->Init.Protocol = SAI_FREE_PROTOCOL;
	hsai->Init.DataSize = MXConfig->DataSize;
	hsai->Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai->Init.ClockStrobing = MXConfig->ClockStrobing;
	hsai->Init.Synchro = MXConfig->Synchro;
	hsai->Init.OutputDrive = MXConfig->OutputDrive;
	hsai->Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
	hsai->Init.SynchroExt = MXConfig->SynchroExt;
	hsai->Init.CompandingMode = SAI_NOCOMPANDING;
	hsai->Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai->Init.Mckdiv = 0;
	hsai->Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
	hsai->Init.MckOutput = SAI_MCK_OUTPUT_DISABLE;
	hsai->Init.PdmInit.Activation = DISABLE;
	hsai->Init.PdmInit.ClockEnable = 0;
	hsai->Init.PdmInit.MicPairsNbr = 0;

	/* Configure SAI_Block_x Frame */
	hsai->FrameInit.FrameLength = MXConfig->FrameLength;
	hsai->FrameInit.ActiveFrameLength = MXConfig->ActiveFrameLength;
	hsai->FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
	hsai->FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai->FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

	/* Configure SAI Block_x Slot */
	hsai->SlotInit.FirstBitOffset = 0;
	hsai->SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai->SlotInit.SlotNumber = 4;
	hsai->SlotInit.SlotActive = MXConfig->SlotActive;

	if (HAL_SAI_Init(hsai) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	/* Enable SAI peripheral to generate MCLK */
	__HAL_SAI_ENABLE(hsai);

	return ret;
}

static HAL_StatusTypeDef MyMX_SAI4_Block_A_Init(SAI_HandleTypeDef *hsai, MX_SAI_Config_t *MXConfig)
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* Disable SAI peripheral to allow access to SAI internal registers */
	__HAL_SAI_DISABLE(hsai);

	/* Configure SAI4_Block_A */
	hsai->Init.AudioFrequency      = MXConfig->AudioFrequency;
	hsai->Init.MonoStereoMode      = MXConfig->MonoStereoMode;
	hsai->Init.AudioMode           = MXConfig->AudioMode;
	hsai->Init.NoDivider           = SAI_MASTERDIVIDER_DISABLE;
	hsai->Init.Protocol            = SAI_FREE_PROTOCOL;
	hsai->Init.DataSize            = MXConfig->DataSize;
	hsai->Init.FirstBit            = SAI_FIRSTBIT_LSB;
	hsai->Init.ClockStrobing       = MXConfig->ClockStrobing;
	hsai->Init.Synchro             = MXConfig->Synchro;
	hsai->Init.OutputDrive         = MXConfig->OutputDrive;
	hsai->Init.FIFOThreshold       = SAI_FIFOTHRESHOLD_1QF;
	hsai->Init.SynchroExt          = MXConfig->SynchroExt;
	hsai->Init.CompandingMode      = SAI_NOCOMPANDING;
	hsai->Init.TriState            = SAI_OUTPUT_RELEASED;
	hsai->Init.Mckdiv              = 0;
	hsai->Init.PdmInit.Activation  = ENABLE;
	hsai->Init.PdmInit.MicPairsNbr = 2;
	hsai->Init.PdmInit.ClockEnable = SAI_PDM_CLOCK2_ENABLE;

	/* Configure SAI_Block_x Frame */
	hsai->FrameInit.FrameLength 		= MXConfig->FrameLength;
	hsai->FrameInit.ActiveFrameLength 	= MXConfig->ActiveFrameLength;
	hsai->FrameInit.FSDefinition 		= SAI_FS_STARTFRAME;
	hsai->FrameInit.FSPolarity 			= SAI_FS_ACTIVE_HIGH;
	hsai->FrameInit.FSOffset 			= SAI_FS_FIRSTBIT;

	/* Configure SAI Block_x Slot */
	hsai->SlotInit.FirstBitOffset 	= 0;
	hsai->SlotInit.SlotSize 		= SAI_SLOTSIZE_DATASIZE;
	hsai->SlotInit.SlotNumber 		= 2;
	hsai->SlotInit.SlotActive 		= MXConfig->SlotActive;

	if (HAL_SAI_Init(hsai) != HAL_OK)
	{
		ret = HAL_ERROR;
	}

	/* Enable SAI peripheral to generate MCLK */
	__HAL_SAI_ENABLE(hsai);

	return ret;
}


static HAL_StatusTypeDef MyMX_SAI2_ClockConfig(uint32_t SampleRate)
{
	  RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

	  HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

	  /* Set the PLL configuration according to the audio frequency */
	  if(SampleRate == AUDIO_FREQUENCY_44K)
	  {
	    /* SAI clock config:
	    PLL2_VCO Input = HSE_VALUE/PLL2M = 1 Mhz
	    PLL2_VCO Output = PLL2_VCO Input * PLL2N = 429 Mhz
	    SAI_CLK_x = PLL2_VCO Output/PLL2P = 429/38 = 11.289 Mhz */
	    rcc_ex_clk_init_struct.PLL2.PLL2P = 38;
	    rcc_ex_clk_init_struct.PLL2.PLL2N = 429;
	  }
	  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
	  {
	    /* SAI clock config:
	    PLL2_VCO Input = HSE_VALUE/PLL2M = 1 Mhz
	    PLL2_VCO Output = PLL2_VCO Input * PLL2N = 344 Mhz
	    SAI_CLK_x = PLL2_VCO Output/PLL2P = 344/7 = 49.142 Mhz */

	    rcc_ex_clk_init_struct.PLL2.PLL2P = 7;
	    rcc_ex_clk_init_struct.PLL2.PLL2N = 344;
	  }
	  rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	  rcc_ex_clk_init_struct.Sai23ClockSelection = RCC_SAI2CLKSOURCE_PLL2;
	  rcc_ex_clk_init_struct.PLL2.PLL2Q = 1;
	  rcc_ex_clk_init_struct.PLL2.PLL2R = 1;
	  rcc_ex_clk_init_struct.PLL2.PLL2M = 25;

	  return HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
}

static HAL_StatusTypeDef MyMX_SAI4_ClockConfig(uint32_t SampleRate)
{
	RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;
	HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

	/* Set the PLL configuration according to the audio frequency */
	if(SampleRate == AUDIO_FREQUENCY_44K)
	{
		rcc_ex_clk_init_struct.PLL2.PLL2P = 38;
	    rcc_ex_clk_init_struct.PLL2.PLL2N = 429;
	}
	else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_32K, AUDIO_FREQUENCY_48K, AUDIO_FREQUENCY_96K */
	{
	    rcc_ex_clk_init_struct.PLL2.PLL2P = 7;
	    rcc_ex_clk_init_struct.PLL2.PLL2N = 344;
	}
	/* SAI clock config */
	rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI4A;
	rcc_ex_clk_init_struct.Sai4AClockSelection 	= RCC_SAI4ACLKSOURCE_PLL2;
	rcc_ex_clk_init_struct.PLL2.PLL2Q 			= 1;
	rcc_ex_clk_init_struct.PLL2.PLL2R 			= 1;
	rcc_ex_clk_init_struct.PLL2.PLL2M 			= 25;

	return HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
}


static void MySAI_Out_MspInit(void)
{
	GPIO_InitTypeDef gpio_init_structure;
	static DMA_HandleTypeDef hdma_sai_tx;

	/* Enable SAI clock */
	AUDIO_OUT_SAIx_CLK_ENABLE();

	/* Enable GPIO clock */
	AUDIO_OUT_SAIx_SCK_ENABLE();
	AUDIO_OUT_SAIx_SD_ENABLE();
	AUDIO_OUT_SAIx_FS_ENABLE();

    /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
	gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_AF;
	HAL_GPIO_Init(AUDIO_OUT_SAIx_FS_GPIO_PORT, &gpio_init_structure);

	gpio_init_structure.Pin = AUDIO_OUT_SAIx_SCK_PIN;
	gpio_init_structure.Alternate = AUDIO_OUT_SAIx_SCK_AF;
	HAL_GPIO_Init(AUDIO_OUT_SAIx_SCK_GPIO_PORT, &gpio_init_structure);

	gpio_init_structure.Pin = AUDIO_OUT_SAIx_SD_PIN;
	gpio_init_structure.Alternate = AUDIO_OUT_SAIx_SD_AF;
	HAL_GPIO_Init(AUDIO_OUT_SAIx_SD_GPIO_PORT, &gpio_init_structure);

	AUDIO_OUT_SAIx_MCLK_ENABLE();
    gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
    gpio_init_structure.Alternate = AUDIO_OUT_SAIx_MCLK_AF;
    HAL_GPIO_Init(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, &gpio_init_structure);


	/* Enable the DMA clock */
	AUDIO_OUT_SAIx_DMAx_CLK_ENABLE();

	/* Configure the hdma_saiTx handle parameters */
	hdma_sai_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_sai_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;


	hdma_sai_tx.Init.Request = AUDIO_OUT_SAIx_DMAx_REQUEST;
	hdma_sai_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sai_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_sai_tx.Init.Mode = DMA_CIRCULAR;
	hdma_sai_tx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_sai_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_sai_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_sai_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_sai_tx.Instance = AUDIO_OUT_SAIx_DMAx_STREAM;
	hdma_sai_tx.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_sai_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

	/* Associate the DMA handle */
	__HAL_LINKDMA(&haudio_out_sai, hdmatx, hdma_sai_tx);

	/* Deinitialize the Stream for new transfer */
	(void)HAL_DMA_DeInit(&hdma_sai_tx);

	/* Configure the DMA Stream */
	(void)HAL_DMA_Init(&hdma_sai_tx);

	/* SAI DMA IRQ Channel configuration */
	HAL_NVIC_SetPriority(AUDIO_OUT_SAIx_DMAx_IRQ, AUDIO_OUT_IT_PRIORITY, 0);
	HAL_NVIC_EnableIRQ(AUDIO_OUT_SAIx_DMAx_IRQ);
}

static void MySAI_InPDM_MspInit(void)
{
	GPIO_InitTypeDef gpio_init_structure;
	static DMA_HandleTypeDef hdma_sai_rx;
	/* Enable SAI clock */
	AUDIO_OUT_SAIx_CLK_ENABLE();

	/* Enable GPIO clock */
	AUDIO_OUT_SAIx_SCK_ENABLE();
	AUDIO_OUT_SAIx_SD_ENABLE();
	AUDIO_OUT_SAIx_FS_ENABLE();
	/* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
	gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN;
	gpio_init_structure.Mode = GPIO_MODE_AF_PP;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio_init_structure.Alternate = AUDIO_OUT_SAIx_FS_AF;
	HAL_GPIO_Init(AUDIO_OUT_SAIx_FS_GPIO_PORT, &gpio_init_structure);

	gpio_init_structure.Pin = AUDIO_OUT_SAIx_SCK_PIN;
	gpio_init_structure.Alternate = AUDIO_OUT_SAIx_SCK_AF;
	HAL_GPIO_Init(AUDIO_OUT_SAIx_SCK_GPIO_PORT, &gpio_init_structure);

	gpio_init_structure.Pin = AUDIO_OUT_SAIx_SD_PIN;
	gpio_init_structure.Alternate = AUDIO_OUT_SAIx_SD_AF;
	HAL_GPIO_Init(AUDIO_OUT_SAIx_SD_GPIO_PORT, &gpio_init_structure);


    AUDIO_IN_SAI_PDMx_CLK_ENABLE();

    AUDIO_IN_SAI_PDMx_CLK_IN_ENABLE();
    AUDIO_IN_SAI_PDMx_DATA_IN_ENABLE();

    gpio_init_structure.Pin = AUDIO_IN_SAI_PDMx_CLK_IN_PIN;
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_structure.Alternate = AUDIO_IN_SAI_PDMx_DATA_CLK_AF;
    HAL_GPIO_Init(AUDIO_IN_SAI_PDMx_CLK_IN_PORT, &gpio_init_structure);

    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio_init_structure.Pin = AUDIO_IN_SAI_PDMx_DATA_IN_PIN;
    HAL_GPIO_Init(AUDIO_IN_SAI_PDMx_DATA_IN_PORT, &gpio_init_structure);

    /* Enable the DMA clock */
    AUDIO_IN_SAI_PDMx_DMAx_CLK_ENABLE();

    /* Configure the hdma_sai_rx handle parameters */
    hdma_sai_rx.Init.Request             = AUDIO_IN_SAI_PDMx_DMAx_REQUEST;
    hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_rx.Init.PeriphDataAlignment = AUDIO_IN_SAI_PDMx_DMAx_PERIPH_DATA_SIZE;
    hdma_sai_rx.Init.MemDataAlignment    = AUDIO_IN_SAI_PDMx_DMAx_MEM_DATA_SIZE;
    hdma_sai_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

    hdma_sai_rx.Instance = AUDIO_IN_SAI_PDMx_DMAx_STREAM;

    /* Associate the DMA handle */
    __HAL_LINKDMA(&haudio_in_sai, hdmarx, hdma_sai_rx);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_rx);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_rx);

    /* SAI DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(AUDIO_IN_SAI_PDMx_DMAx_IRQ, BSP_AUDIO_IN_IT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(AUDIO_IN_SAI_PDMx_DMAx_IRQ);
}

static void PDMToPCM_Init()
{
	__HAL_RCC_CRC_CLK_ENABLE();

    for (uint32_t index = 0; index < Audio.Channels; index++)
    {
      /* Init PDM filters */
      PDM_FilterHandler[index].bit_order 		= PDM_FILTER_BIT_ORDER_MSB;
      PDM_FilterHandler[index].endianness 		= PDM_FILTER_ENDIANNESS_LE;
      PDM_FilterHandler[index].high_pass_tap 	= PDM2PCM_HIGHPASS_FILTER;
      PDM_FilterHandler[index].out_ptr_channels = Audio.Channels;
      PDM_FilterHandler[index].in_ptr_channels 	= Audio.Channels;
      PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));

      /* PDM lib config phase */
      PDM_FilterConfig[index].output_samples_number = PDM2PCM_SAMPLES;
      PDM_FilterConfig[index].mic_gain 				= PDM2PCM_MIC_GAIN;
      PDM_FilterConfig[index].decimation_factor 	= PDM_FILTER_DEC_FACTOR_64;
      PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
    }
}

void PDMToPCM(uint16_t* PDMBuf, uint16_t* PCMBuf)
{
	PDM_Filter(&((uint8_t*)(PDMBuf))[0], &(PCMBuf[0]), &PDM_FilterHandler[0]);
	PDM_Filter(&((uint8_t*)(PDMBuf))[1], &(PCMBuf[1]), &PDM_FilterHandler[1]);
}

void BDMA_Channel1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(haudio_in_sai.hdmarx);
}


void DMA2_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(haudio_in_sai.hdmarx);
}

void DMA2_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

