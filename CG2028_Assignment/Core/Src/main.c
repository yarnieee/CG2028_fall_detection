/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../Inc/ssd1306.h"
#include "../Inc/ht16k33.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <sys/stat.h>

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT   25
#define EWMA_ALPHA_GYRO_PERCENT    25
#define NORMAL_LED_DELAY_MS       1000
#define FALL_LED_DELAY_MS          150

static void UART1_Init(void);
static void UART_Send(const char *text);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
extern int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

/*=============================== Our Addition v ===============================*/
#define OLED_ADDR    (0x3C << 1)
#define MATRIX_ADDR  (0x70 << 1)

typedef enum FallState {
    NORMAL_0,
    FREEFALL_1,
    IMPACT_2,
    FALLEN_3,
	LONG_LIE_4
} FallState;

static void External_Peripherals_Init(void);
static void I2C_TestDevices(void);
static uint8_t I2C_DevicePresent(uint16_t address);
static uint16_t SoundSensor_Read(void);
static FallState FallDetector_Update(float accel_g, float gyro_dps, uint16_t sound_value, uint32_t current_time);
static void Buzzer_Set(uint8_t enabled);
static void Matrix_ShowFace(const uint8_t face[8][8]);
static void Matrix_ShowHappyFace(void);
static void Matrix_ShowSadFace(void);
static void OLED_SetInitMessage(SSD1306_HandleTypeDef *display);
static void OLED_SetFallMessage(SSD1306_HandleTypeDef *display);
static void OLED_SetSoundMessage(SSD1306_HandleTypeDef *display, uint16_t sound_value);
void HAL_SYSTICK_Callback(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
static SSD1306_HandleTypeDef oled;
static HT16K33_HandleTypeDef led_matrix;
static uint8_t oled_ready = 0;
static uint8_t led_matrix_ready = 0;

static volatile uint8_t led_fall_mode = 0;
static volatile uint8_t led_timer_enabled = 0;
static volatile uint8_t reset_requested = 0;
static volatile uint8_t detector_reset_requested = 0;
/*=============================== Our Addition ^ ===============================*/
/**
 * main runs once every N seconds (depending on HAL_Delay at the end)
 */
int main(void) { // THIS ONEEEEEEEEEEE =======================================
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);

    /*============================= Our Addition v =============================*/
    /*=============== LED AND RESET BUTTON INITIALISATION ====================*/
    led_timer_enabled = 1;
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/*======= SOUND SENSOR, BUZZER, OLED, AND LED MATRIX INITIALISATION ======*/
    External_Peripherals_Init();
    I2C_TestDevices();

    if (SSD1306_Init(&oled, &hi2c1, OLED_ADDR) == HAL_OK) {
    	OLED_SetInitMessage(&oled);
        oled_ready = 1;
    } else {
        UART_Send("SSD1306 init failed\r\n");
    }

    if (HT16K33_Init(&led_matrix, &hi2c1, MATRIX_ADDR) == HAL_OK) {
        Matrix_ShowHappyFace();
        led_matrix_ready = 1;
    } else {
        UART_Send("HT16K33 init failed\r\n");
    }
    /*============================= Our Addition ^ =============================*/

    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int  gyro_ewma_asm[3] = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    /*
    int accel_ewma_c[3] = {0, 0, 0};
    int  gyro_ewma_c[3] = {0, 0, 0};
    */

    unsigned long sample_number = 0;

    while (1) {
        int16_t accel_raw_i16[3] = {0, 0, 0};
        float  gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int      gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The supplied BSP reports gyroscope readings as floating-point raw
         * values. Convert them to signed integers before passing them to the
         * integer assembly routine. */
        for (int axis = 0; axis < 3; axis++) {
            gyro_raw_int[axis] = (int)gyro_raw_float[axis];

            accel_ewma_asm[axis] = ewma_filter(
                (int)accel_raw_i16[axis],
                accel_ewma_asm[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_asm[axis] = ewma_filter(
                gyro_raw_int[axis],
                gyro_ewma_asm[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            /*
            accel_ewma_c[axis] = ewma_filter_C(
                (int)accel_raw_i16[axis],
                accel_ewma_c[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_c[axis] = ewma_filter_C(
                gyro_raw_int[axis],
                gyro_ewma_c[axis],
                EWMA_ALPHA_GYRO_PERCENT);
             */
        }

        /* Accelerometer filtered readings are in meters per second squared. */
        float accel_mps2[3] = {
            accel_ewma_asm[0] * (9.80665f / 1000.0f),
            accel_ewma_asm[1] * (9.80665f / 1000.0f),
            accel_ewma_asm[2] * (9.80665f / 1000.0f)
        };

        /* Gyroscope filtered readings are in degrees per second. */
        float gyro_dps[3] = {
            gyro_ewma_asm[0] / 1000.0f,
            gyro_ewma_asm[1] / 1000.0f,
            gyro_ewma_asm[2] / 1000.0f
        };


        /*
        char buffer[320];
        snprintf(buffer, sizeof(buffer),
                 "Sample %lu\r\n"
                 "Accel EWMA ASM [m/s^2]: X=%8.3f Y=%8.3f Z=%8.3f\r\n"
                 "Gyro  EWMA ASM [dps]  : X=%8.3f Y=%8.3f Z=%8.3f\r\n",
                 sample_number,
                 accel_mps2[0], accel_mps2[1], accel_mps2[2],
                 gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        UART_Send(buffer);
        */

        /* Optional debugging check. This confirms that the assembly routine
         * matches the reference C routine for the current samples. */
        /*
        if ((accel_ewma_asm[0] != accel_ewma_c[0]) ||
            (accel_ewma_asm[1] != accel_ewma_c[1]) ||
            (accel_ewma_asm[2] != accel_ewma_c[2]) ||
            (gyro_ewma_asm[0] != gyro_ewma_c[0]) ||
            (gyro_ewma_asm[1] != gyro_ewma_c[1]) ||
            (gyro_ewma_asm[2] != gyro_ewma_c[2]))
        {
            UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
        }
        */

        /**************** Elderly wearable state logic starts here**************
         * Compulsory requirements:
         * 1. Use filtered accelerometer AND gyroscope readings.
         * 2. Distinguish normal activity, near-fall movements, and a real fall.
         * 3. Use a slow LED blink for normal operation and a fast blink after
         *    a fall is detected.
         **********************************************************************/

        /* TODO: replace with your fall-detection logic */

        /*=========================== Our Addition v ===========================*/
        // IMPT SECTION ========================================================

		/*========================== Input Readings ==========================*/

        float accel_magnitude =
            sqrtf(accel_mps2[0] * accel_mps2[0] +
                  accel_mps2[1] * accel_mps2[1] +
                  accel_mps2[2] * accel_mps2[2]);

        float gyro_magnitude =
            sqrtf(gyro_dps[0] * gyro_dps[0] +
                  gyro_dps[1] * gyro_dps[1] +
                  gyro_dps[2] * gyro_dps[2]);

        uint16_t sound_value = SoundSensor_Read();

        uint32_t current_time = HAL_GetTick();

		/*===================== Process Input Readings =======================*/
        if (reset_requested) {
        	sample_number = 0;
            led_fall_mode = 0;
            Buzzer_Set(0);
            BSP_LED_Off(LED2);

            if (led_matrix_ready) {
                Matrix_ShowHappyFace();
            }

            if (oled_ready) {
                OLED_SetInitMessage(&oled);
            }

            reset_requested = 0;
            detector_reset_requested = 1;
        }

        // Fall detector is called here, every cycle of main (
        FallState fall_state = FallDetector_Update(
            accel_magnitude,
            gyro_magnitude,
            sound_value,
            current_time
        );

        int fall_detected = (fall_state == FALLEN_3);
        //led_fall_mode = (uint8_t) fall_detected;
        led_fall_mode = 1;

		/*========================== Outputs =================================*/

		/*========================== BUZZER ==================================*/
        if (fall_detected) {
            Buzzer_Set(1);
        } else {
            Buzzer_Set(0);
        }

		/*========================== LED MATRIX ==============================*/
        // Show a happy face during normal operation and a sad face after a fall.
        static int last_matrix_state = -1;
        if (led_matrix_ready && last_matrix_state != fall_detected) {
        	if (fall_detected) {
        		Matrix_ShowSadFace();
        	} else {
        		Matrix_ShowHappyFace();
        	}
	        last_matrix_state = fall_detected;
        }

		/*========================== OLED MATRIX =============================*/
        static int last_oled_state = -1;
        if (oled_ready && last_oled_state != fall_detected) {
    		if (fall_detected) {
    			OLED_SetFallMessage(&oled);
    		} else {
    			OLED_SetInitMessage(&oled);
    		}

    		last_oled_state = fall_detected;
        }

        // Optional update to OLED to display live sound value
        OLED_SetSoundMessage(&oled, sound_value);

        char message[256];
        snprintf(message, sizeof(message),
                 "Sample %lu\r\n"
                 "Accel_Magnitude = %.3f\r\n"
                 "Gyro_Magnitude  = %.3f\r\n"
        		 "Sound ADC       = %u\r\n"
        		 "Fall State      = %d\r\n"
        		 "======================\r\n",
                 sample_number,
                 accel_magnitude,
                 gyro_magnitude,
				 (unsigned int)sound_value,
				 (int)fall_state);
        UART_Send(message);

        sample_number++;

        HAL_Delay(20); // 20ms delay for 50 samples per second
        /*=========================== Our Addition ^ ===========================*/

        /*
        BSP_LED_Toggle(LED2);
        HAL_Delay(fall_detected ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS);
        */
    }
}

int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    /* Reference implementation for verification only. The assembly routine
     * must be used in the actual sensor-processing and detection pipeline. */
    int numerator = alpha_percent * new_data
                  + (100 - alpha_percent) * old_output;
    return numerator / 100;
}

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1) { }
    }
}

/*=============================== Our Addition v ===============================*/
static void External_Peripherals_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Buzzer: PB4 / Arduino D5 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

    /* Sound sensor: PC5 / Arduino A0 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* I2C1: PB8 = SCL, PB9 = SDA */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 configuration */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00702681;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        while (1) {}
    }

    /* ADC1 configuration */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        while (1) {}
    }

    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = ADC_CHANNEL_14;       /* PC5 */
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0;

    HAL_ADC_ConfigChannel(&hadc1, &channel);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}
static void I2C_TestDevices(void)
{
    if (I2C_DevicePresent(OLED_ADDR))
    {
        UART_Send("OLED detected\r\n");
    }
    else
    {
        UART_Send("OLED not detected\r\n");
    }

    if (I2C_DevicePresent(MATRIX_ADDR))
    {
        UART_Send("LED matrix detected\r\n");
    }
    else
    {
        UART_Send("LED matrix not detected\r\n");
    }
}
static uint8_t I2C_DevicePresent(uint16_t address)
{
    return HAL_I2C_IsDeviceReady(
        &hi2c1,
        address,
        2,
        100
    ) == HAL_OK;
}
static uint16_t SoundSensor_Read(void)
{
    uint16_t value = 0;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    return value;       /* 0 to 4095 */
}

/**
 * FallDetector_Update is called by main once every N seconds (depending on HAL_Delay at the end)
 *
 * takes accel_mps (array of floats)
 * gyro_dps (array of floats)
 *
 * returns FallState
 */
static FallState FallDetector_Update( // THIS ONEEEEEEEEEEE ==================
    float accel_mps2,
    float gyro_dps,
    uint16_t sound_value,
    uint32_t current_time
) {
	// Tracks fall state
    static FallState state = NORMAL_0;
    // Compare against current time to get duration of current state.
    static uint32_t state_start_time = 0;
    static uint8_t impact_detected   = 0; // Boolean
    static float accel_baseline_max[3] = {FLT_MIN, FLT_MIN, FLT_MIN};
    static float accel_baseline_min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};

    // Sound variables
    static uint8_t sound_detected = 0; // Boolean
    static float sound_baseline = 2048.0f;
    const float LOUD_SOUND_CONST = 300.0f;

    // Quiet samples describes number of records detected as "low activity"
    // Raises chance that the person has fallen and is incapacitated or unconscious
    static uint16_t quiet_samples = 0;

    // Threshold constants, used to compare against active values
    const float FREEFALL_MPS2          = 6.00f; // MPS2 is metres per second squared
    const float IMPACT_MPS2            = 12.00f;
    const float ACCEL_BASELINE         = 10.00f;
    const float GYRO_DPS_THRESHOLD_MAX = 60.0f; // DPS is degrees-per-second
    const float GYRO_DPS_THRESHOLD_MIN = 30.0f; // Threshold for
    const float ACCEL_DEVIATION        = 0.25f; 

    const uint32_t NEAR_FALL_TIMEOUT_MS = 1000U;
    const uint32_t CANDIDATE_TIMEOUT_MS = 10000U;
    const uint32_t LONG_LIE_TIMEOUT_MS = 300000U; //5 minute timeout
    const uint16_t MIN_NUM_OF_QUIET_SAMPLES = 15U;



    if (detector_reset_requested) {
        state = NORMAL_0;
        state_start_time = 0;
        impact_detected = 0;
        sound_detected = 0;
        quiet_samples = 0;
        sound_baseline = 2048.0f;

        detector_reset_requested = 0;
    }

    // Estimate the normal sound level.
    sound_baseline = (0.99f * sound_baseline) + (0.01f * (float)sound_value);

    float sound_difference = fabsf((float)sound_value - sound_baseline);

    uint8_t loud_sound = (sound_difference > LOUD_SOUND_CONST);

    switch (state) {
    case NORMAL_0:
        if (accel_mps2 < FREEFALL_MPS2 || gyro_dps > GYRO_DPS_THRESHOLD_MAX) {
            state = FREEFALL_1;
            state_start_time = current_time;

        } else {
        	if (accel_mps2 > IMPACT_MPS2) {
				impact_detected = 1;
				state = IMPACT_2;
				state_start_time = current_time;
				quiet_samples = 0;

			}

        	if (accel_mps2 > accel_baseline_max) {
        		accel_baseline_max = accel_mps2;
        	}

        	if (accel_mps2 < accel_baseline_min) {
				accel_baseline_min = accel_mps2;
			}
        }

        break;

    case FREEFALL_1:
        if (loud_sound) {
            sound_detected = 1;
        }

        if (accel_mps2 > IMPACT_MPS2) {
            state = IMPACT_2;
            state_start_time = current_time;

        } else if ((current_time - state_start_time) > NEAR_FALL_TIMEOUT_MS) {
            detector_reset_requested = 1;

        }

        break;

    case IMPACT_2:
        if (loud_sound) {
            sound_detected = 1;
        }

        // Person is relatively still after the possible impact.
        if (gyro_dps < GYRO_DPS_THRESHOLD_MIN &&
            accel_mps2 > ACCEL_BASELINE - 3.00f &&
            accel_mps2 < ACCEL_BASELINE + 3.00f) {
            quiet_samples++;
        } else {
            quiet_samples = 0;
        }

        if ((current_time - state_start_time) > CANDIDATE_TIMEOUT_MS) {
            // Confirmed fall only if...
        	// 1. An impact occurred
        	// 2. AND he person became still
            // 3. AND Rotation or sound supports the event
            if (quiet_samples >= MIN_NUM_OF_QUIET_SAMPLES) {
                state = FALLEN_3;
                state_start_time = current_time;
            } else {
                detector_reset_requested = 1;
            }
        }

        break;

    case FALLEN_3:
    	if ((current_time - state_start_time) > LONG_LIE_TIMEOUT_MS) {
    		state = LONG_LIE_4;
			state_start_time = current_time;
    	}
        
        break;
    case LONG_LIE_4:
        // Stay confirmed until the user resets the device
    	break;
    }

    return state;
}
static void Buzzer_Set(uint8_t enabled)
{
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_4,
        enabled ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}
static void Matrix_ShowFace(const uint8_t face[8][8])
{
    HT16K33_Clear(&led_matrix);

    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t column = 0; column < 8; column++) {
            if (face[row][column] != 0) {
                HT16K33_SetPixel(&led_matrix, row, column, 1);
            }
        }
    }

    HT16K33_Update(&led_matrix);
}
static void Matrix_ShowHappyFace(void) {
    static const uint8_t happy_face[8][8] = {
        {0, 0, 1, 1, 1, 1, 0, 0},
        {0, 1, 0, 0, 0, 0, 1, 0},
        {1, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 1, 1, 0, 0, 1},
        {0, 1, 0, 0, 0, 0, 1, 0},
        {0, 0, 1, 1, 1, 1, 0, 0}
    };

    Matrix_ShowFace(happy_face);
}
static void Matrix_ShowSadFace(void) {
    static const uint8_t sad_face[8][8] = {
        {0, 0, 1, 1, 1, 1, 0, 0},
        {0, 1, 0, 0, 0, 0, 1, 0},
        {1, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 1, 1, 0, 0, 1},
        {1, 0, 1, 0, 0, 1, 0, 1},
        {0, 1, 0, 0, 0, 0, 1, 0},
        {0, 0, 1, 1, 1, 1, 0, 0}
    };

    Matrix_ShowFace(sad_face);
}
static void OLED_SetInitMessage(SSD1306_HandleTypeDef *display)
{
    SSD1306_Clear(display);
    SSD1306_SetCursor(display, 16, 0);
    SSD1306_WriteString(display, "FALL DETECTOR");
    SSD1306_SetCursor(display, 16, 32);
    SSD1306_WriteString(display, "SYSTEM READY");
    SSD1306_Update(display);
}
static void OLED_SetFallMessage(SSD1306_HandleTypeDef *display)
{
	SSD1306_Clear(display);
	SSD1306_SetCursor(display, 0, 0);
	SSD1306_WriteString(display, "IVE FALLEN  CALL 995");
	SSD1306_SetCursor(display, 0, 16);
	SSD1306_WriteString(display, "FAMILY NUM: 8655 4322");
	SSD1306_SetCursor(display, 8, 32);
	SSD1306_WriteString(display, "NAME: TAN WEI SONG");
	SSD1306_SetCursor(display, 8, 48);
	SSD1306_WriteString(display, "AGE: 85");
	SSD1306_Update(display);
}
static void OLED_SetSoundMessage(SSD1306_HandleTypeDef *display, uint16_t sound_value)
{
	SSD1306_SetCursor(&oled, 56, 48);
	char oled_text[24];
	snprintf(oled_text, sizeof(oled_text), "SOUND: %u", sound_value);
	SSD1306_WriteString(&oled, oled_text);
	SSD1306_Update(&oled);
}
void HAL_SYSTICK_Callback(void)
{
	// NOTE: must add "HAL_SYSTICK_IRQHandler();" to
	// "void SysTick_Handler(void)" in stm32l4xx.it.c

    static uint32_t elapsed_ms = 0;
    static uint8_t previous_mode = 0xFF;

    if (!led_timer_enabled)
    {
        return;
    }

    // Restart the timing when the fall state changes
    if (led_fall_mode != previous_mode)
    {
        elapsed_ms = 0;
        previous_mode = led_fall_mode;
        BSP_LED_Off(LED2);
    }

    elapsed_ms++;

    uint32_t toggle_period = led_fall_mode ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS;

    if (elapsed_ms >= toggle_period)
    {
        elapsed_ms = 0;
        BSP_LED_Toggle(LED2);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == BUTTON_EXTI13_Pin)
    {
        // Ignore switch bounce for 250 ms
        if ((current_time - last_press_time) > 250U)
        {
            reset_requested = 1;
            last_press_time = current_time;
        }
    }
}
/*=============================== Our Addition ^ ===============================*/

/* Do not modify these lines. They suppress UART-related warnings. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _fstat(int file, struct stat *st) { (void)file; (void)st; return 0; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _close(int file) { (void)file; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
