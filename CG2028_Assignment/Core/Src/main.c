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
#include "../Inc/grove_multi_switch.h"
#include "wifi.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <sys/stat.h>

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT   40
#define EWMA_ALPHA_GYRO_PERCENT    30
#define NORMAL_LED_DELAY_MS        1000
#define FALL_LED_DELAY_MS          150

static void UART1_Init(void);
static void UART_Send(const char *text);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

/*============================ Additional Configurations ===============================*/
#define DEMO_MODE 1
#define MAX_MESSAGE_LENGTH 128	// adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000
#define OLED_ADDR    (0x3C << 1)
#define MATRIX_ADDR  (0x70 << 1)
#define SWITCH_ADDR  (0x03 << 1)

typedef enum FallState {
    NORMAL_0,
    FREEFALL_1,
    IMPACT_2,
    FALLEN_3,
	LONG_LIE_4
} FallState;

// Main Function Declarations
static FallState FallDetector_Update(float accel_mps2, float gyro_dps, uint32_t current_time);
static void UpdateSoundStatus(uint16_t sound_value, uint32_t current_time);
static void UpdateBuzzer(FallState fall_state, uint32_t current_time);
static void HandleFallStateChange(FallState fall_state);
static void LogStatus(FallState fall_state, float accel_magnitude, float gyro_magnitude, uint8_t loud_sound_detected, uint32_t current_time);
static void ProcessResetRequest(void);

// Initialization Function Declarations
static void External_Peripherals_Init(void);
static void I2C_TestDevices(void);
static uint8_t I2C_DevicePresent(uint16_t address);
static void Wifi_Full_Init(void);
static void I2C_Devices_Full_Init(void);

// Peripherals Helper Declarations
static void ProcessSwitchEvents(void);
static uint16_t SoundSensor_Read(void);
static void Buzzer_Set(uint8_t enabled);

// Matrix Helper Function Declarations
static void Matrix_ShowFace(const uint8_t face[8][8]);
static void Matrix_ShowHappyFace(void);
static void Matrix_ShowSadFace(void);

// OLED Function Declarations
static void OLED_SetInitMessage(SSD1306_HandleTypeDef *display);
static void OLED_SetFallMessage(SSD1306_HandleTypeDef *display);
static void OLED_SetLongLieMessage(SSD1306_HandleTypeDef *display);

// Interrupt Function Declarations
void HAL_SYSTICK_Callback(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void SPI3_IRQHandler(void);

SPI_HandleTypeDef hspi3;
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
static SSD1306_HandleTypeDef oled;
static HT16K33_HandleTypeDef led_matrix;
static GroveMultiSwitch_HandleTypeDef grove_switch;
static uint8_t oled_ready = 0;
static uint8_t led_matrix_ready = 0;
static uint8_t switch_ready = 0;

static volatile uint8_t reset_requested          = 0;
static volatile uint8_t detector_reset_requested = 0;
static volatile uint8_t fall_detected            = 0;
static volatile uint8_t loud_sound_detected      = 0; // Status flag to indicate when a potential impact sound is detected
static volatile uint8_t led_fall_mode            = 0;
static volatile uint8_t led_timer_enabled        = 0;

// Wifi related constants
// Matt's Wifi
//const char* WiFi_SSID = "DIDSBSAYYOGA";
//const char* WiFi_password = "5\\5F987i";
// Yi An's Wifi
const char* WiFi_SSID = "NOT-A-MAC";
const char* WiFi_password = "sF9@oZ6BX!";
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK;	// WiFi security your router / Hotspot
const uint16_t SOURCE_PORT = 1234;
const uint16_t DEST_PORT = 2028; // 'server' port number - this is the port Packet Sender listens to
uint8_t ipaddr[4] = {10, 249, 88, 87}; // IP address of our laptop wireless lan adapter

// Threshold constants for FallDetector_Update(), used to compare against active values
const float FREEFALL_THRESHOLD_MPS2 = 6.00f; // MPS2 is metres per second squared
const float IMPACT_THRESHOLD_MPS2   = 16.0f;
const float ACCEL_BASELINE          = 10.0f;
const float FALLEN_ACCEL_RANGE      = 4.0f;
const float LONG_LIE_ACCEL_RANGE    = 5.0f;
const float GYRO_DPS_THRESHOLD_MAX  = 90.0f; // DPS is degrees-per-second
const float GYRO_DPS_THRESHOLD_MIN  = 30.0f;

const uint32_t FREEFALL_TIMEOUT_MS = 1000U;   // 1 second timeout
const uint32_t IMPACT_TIMEOUT_MS   = 2500U;   // 2.5 second timeout
const uint32_t LONG_LIE_TIMEOUT_MS = (DEMO_MODE) ? 5000U : 600000U; // Demo timeout: 5 seconds; Deployment timeout: 10 minutes
const uint16_t MIN_NUM_OF_INACTIVITY_SAMPLES = 45U;

int main(void) {
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);

    /*====================== WIFI INITIALISATION =============================*/
	Wifi_Full_Init();

    /*=============== LED AND RESET BUTTON INITIALISATION ====================*/
    led_timer_enabled = 1;
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/*= SOUND SENSOR, BUZZER, OLED, LED MATRIX & 5 WAY SWITCH INITIALISATION =*/
    External_Peripherals_Init();
    I2C_TestDevices();
    I2C_Devices_Full_Init();

    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int  gyro_ewma_asm[3] = {0, 0, 0};

    /* Boolean check for filter initialization */
    uint8_t filter_initialized = 0;

    // while loop runs once every 20ms
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
        }

        if (!filter_initialized) {
        	/* Use the first real sensor readings as the initial filter state.*/
			for (int axis = 0; axis < 3; axis++) {
				accel_ewma_asm[axis] = (int)accel_raw_i16[axis];
				gyro_ewma_asm[axis] = gyro_raw_int[axis];
			}

			filter_initialized = 1;
        } else {
        	/* Apply EWMA from the second sample onward.*/
			for (int axis = 0; axis < 3; axis++) {
				accel_ewma_asm[axis] = ewma_filter(
					(int)accel_raw_i16[axis],
					accel_ewma_asm[axis],
					EWMA_ALPHA_ACCEL_PERCENT);

				gyro_ewma_asm[axis] = ewma_filter(
					gyro_raw_int[axis],
					gyro_ewma_asm[axis],
					EWMA_ALPHA_GYRO_PERCENT);
			}
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

        /**************** Elderly wearable state logic starts here**************
         * Compulsory requirements:
         * 1. Use filtered accelerometer AND gyroscope readings.
         * 2. Distinguish normal activity, near-fall movements, and a real fall.
         * 3. Use a slow LED blink for normal operation and a fast blink after
         *    a fall is detected.
         **********************************************************************/

		/*========================== Input Readings ============================*/

        float accel_magnitude =
            sqrtf(accel_mps2[0] * accel_mps2[0] +
                  accel_mps2[1] * accel_mps2[1] +
                  accel_mps2[2] * accel_mps2[2]);

        float gyro_magnitude =
            sqrtf(gyro_dps[0] * gyro_dps[0] +
                  gyro_dps[1] * gyro_dps[1] +
                  gyro_dps[2] * gyro_dps[2]);

        uint16_t sound_value = SoundSensor_Read();

        if (switch_ready) {
            ProcessSwitchEvents();
        }

        uint32_t current_time = HAL_GetTick();

		/*===================== Process Input Readings =======================*/
        /*===================== FALL DETECTOR MACHINE ========================*/
        /* Fall detector machine is called here, every cycle of main () */
        /* THIS FUNCTION IS VERY IMPORTANT */
        FallState fall_state = FallDetector_Update(accel_magnitude, gyro_magnitude, current_time);
        fall_detected = (fall_state == FALLEN_3 || fall_state == LONG_LIE_4);
        led_fall_mode = fall_detected;

		/*===================== SOUND SENSOR =================================*/
        UpdateSoundStatus(sound_value, current_time);

		/*========================== Outputs =================================*/
		/*========================== BUZZER ==================================*/
        UpdateBuzzer(fall_state, current_time);

		/*=============== UART, LED MATRIX AND OLED ==========================*/
        HandleFallStateChange(fall_state);

        /*================= UART/WIFI LOGGING ================================*/
        LogStatus(fall_state, accel_magnitude, gyro_magnitude, loud_sound_detected, current_time);

        if (reset_requested) {
        	ProcessResetRequest();
        }

        /* Nominal 20 ms delay; actual loop period is slightly longer. At most 50 samples per second */
        HAL_Delay(20);
    }
}

static void UART1_Init(void) {
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

    if (HAL_UART_Init(&huart1) != HAL_OK) {
        while (1) {}
    }
}
static void UART_Send(const char *text) {
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

// Main Function Definitions
static FallState FallDetector_Update(
    float accel_mps2,
    float gyro_dps,
    uint32_t current_time
) {
	// Tracks fall state
    static FallState state = NORMAL_0;

    // Compare against current time to get duration of current state.
    static uint32_t state_start_time = 0;
    static uint32_t last_motion_time = 0;

    // Inactivity samples describes number of records detected as "low activity"
    // Raises chance that the person has fallen and is incapacitated or unconscious
    static uint16_t inactivity_samples = 0;

    // Boolean checks
    static uint8_t impact_accel_detected = 0;
    static uint8_t impact_gyro_detected = 0;
    static uint8_t fallen_movement_detected = 0;
    static uint8_t long_lie_movement_detected = 0;

    if (detector_reset_requested) {
        state = NORMAL_0;
        state_start_time           = 0;
        last_motion_time           = 0;
        inactivity_samples         = 0;
        impact_accel_detected      = 0;
        impact_gyro_detected       = 0;
        fallen_movement_detected   = 0;
        long_lie_movement_detected = 0;
        detector_reset_requested   = 0;
    }

    switch (state) {
    case NORMAL_0: //possible: if within certain ms of each other it crosses the threshold
        if (accel_mps2 < FREEFALL_THRESHOLD_MPS2) {
            state = FREEFALL_1;
            state_start_time = current_time;
        }

        break;

    case FREEFALL_1:
    	if (accel_mps2 > IMPACT_THRESHOLD_MPS2) {
    		impact_accel_detected = 1;
    	}

		if (gyro_dps > GYRO_DPS_THRESHOLD_MAX) {
			impact_gyro_detected = 1;
		}

        if (impact_accel_detected && impact_gyro_detected) {
            state = IMPACT_2;
            state_start_time = current_time;
        } else if ((current_time - state_start_time) > FREEFALL_TIMEOUT_MS) {
        	state = NORMAL_0;
            detector_reset_requested = 1;
        }

        break;

    case IMPACT_2:
        // Whether elderly is moving is based on the rotational speed and acceleration at rest
        fallen_movement_detected =
        	(gyro_dps >= GYRO_DPS_THRESHOLD_MIN) ||
    		(accel_mps2 < ACCEL_BASELINE - FALLEN_ACCEL_RANGE) ||
    		(accel_mps2 > ACCEL_BASELINE + FALLEN_ACCEL_RANGE);

        // Person is relatively still after the possible impact.
        if (!fallen_movement_detected) {
            inactivity_samples++;
        } else {
            inactivity_samples = 0;
        }

        if ((current_time - state_start_time) > IMPACT_TIMEOUT_MS) {
            // Confirmed fall only if the person became still for an extended period of time
            if (inactivity_samples >= MIN_NUM_OF_INACTIVITY_SAMPLES) {
                state = FALLEN_3;
                state_start_time = current_time;
                last_motion_time = current_time;
            } else {
            	state = NORMAL_0;
                detector_reset_requested = 1;
            }
        }

        break;

    case FALLEN_3:
        // Whether elderly is moving is based on the rotational speed and acceleration at rest
        long_lie_movement_detected =
        	(gyro_dps >= GYRO_DPS_THRESHOLD_MIN) ||
    		(accel_mps2 < ACCEL_BASELINE - LONG_LIE_ACCEL_RANGE) ||
    		(accel_mps2 > ACCEL_BASELINE + LONG_LIE_ACCEL_RANGE);

        if (long_lie_movement_detected) {
            // The person moved, so restart the long-lie timer.
            last_motion_time = current_time;
        } else if ((current_time - last_motion_time) >= LONG_LIE_TIMEOUT_MS) {
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
static void UpdateSoundStatus(uint16_t sound_value, uint32_t current_time) {
	// Sound value at normal conditions
	static float sound_baseline = 1000.0f;
	// Number of consecutive loud sound samples
	static uint16_t loud_sound_samples = 0;
	// Compare against current time to get duration of current state.
	static uint32_t last_loud_sound_time = 0;

	// Sound constants
	const float MAX_SOUND_DIFF = 2000.0f;
	const uint16_t MIN_NUM_OF_LOUD_SOUND_SAMPLES = 5U;
	const uint32_t IMPACT_SOUND_TIMEOUT_MS = 5000U;   // 5 second timeout

	// Estimate the normal sound level.
	sound_baseline = (0.99f * sound_baseline) + (0.01f * (float)sound_value);

	float sound_difference = fabsf((float)sound_value - sound_baseline);

	if (sound_difference > MAX_SOUND_DIFF) {
		loud_sound_samples++;
	} else {
		loud_sound_samples = 0;
	}

	if (loud_sound_samples >= MIN_NUM_OF_LOUD_SOUND_SAMPLES) {
		loud_sound_detected = 1;
		last_loud_sound_time = current_time;
	}

	if ((current_time - last_loud_sound_time) > IMPACT_SOUND_TIMEOUT_MS) {
		loud_sound_detected = 0;
	}

}
static void UpdateBuzzer(FallState fall_state, uint32_t current_time) {
    static uint8_t buzzer_state = 0;
    static uint32_t last_buzzer_toggle = 0;
    uint32_t buzzer_period = (fall_state == LONG_LIE_4) ? 250U : 500U;

    if (!fall_detected) {
    	buzzer_state = 0;
    	Buzzer_Set(0);
    } else if (current_time - last_buzzer_toggle >= buzzer_period){
        buzzer_state = !buzzer_state;
        Buzzer_Set(buzzer_state);
        last_buzzer_toggle = current_time;
    }
}
static void HandleFallStateChange(FallState fall_state) {
	static FallState previous_state = NORMAL_0;

	if (fall_state == previous_state) {
		return;
	}

	char state_message[64];
	const char *state_text = "UNKNOWN STATE";

	switch (fall_state) {
	case NORMAL_0:
		state_text = "NORMAL";
		break;
	case FREEFALL_1:
		state_text = "FREEFALL";
		break;
	case IMPACT_2:
		state_text = "IMPACT";
		break;
	case FALLEN_3:
		state_text = "FALL CONFIRMED";
		if (led_matrix_ready) {
			Matrix_ShowSadFace();
		}
		if (oled_ready) {
			OLED_SetFallMessage(&oled);
		}
		break;
	case LONG_LIE_4:
		state_text = "LONG LIE ESCALATION: NO MOVEMENT";
		if (oled_ready) {
			OLED_SetLongLieMessage(&oled);
		}
	}

	snprintf(state_message, sizeof(state_message), "\r\n%s\r\n", state_text);

	// Sends
	UART_Send(state_message);

	uint16_t Datalen;
	WIFI_SendData(1, (uint8_t*)state_message, (uint16_t)strlen(state_message), &Datalen, WIFI_WRITE_TIMEOUT);

	previous_state = fall_state;
}
static void LogStatus(
		FallState fall_state,
		float accel_magnitude,
		float gyro_magnitude,
		uint8_t loud_sound_detected,
		uint32_t current_time) {
    static uint32_t last_log_time = 0;

    if ((current_time - last_log_time) >= 250U) {
		char log_message[MAX_MESSAGE_LENGTH];
		snprintf(log_message, sizeof(log_message),
				 "Time: %6u | State: %d | Accel: %6.2f | Gyro: %6.2f | Possible Impact Sound: %u\r\n",
				 (unsigned int) current_time,
				 (int) fall_state,
				 accel_magnitude,
				 gyro_magnitude,
				 loud_sound_detected);
		UART_Send(log_message);
		snprintf(log_message, sizeof(log_message),
				 "%d, %6u, %6.2f, %6.2f, %u\r\n",
				 (int) fall_state,
				 (unsigned int) current_time,
				 accel_magnitude,
				 gyro_magnitude,
				 loud_sound_detected);
	    uint16_t Datalen;
		WIFI_SendData(1, (uint8_t*)log_message, (uint16_t)strlen(log_message), &Datalen, WIFI_WRITE_TIMEOUT);
		last_log_time = current_time;
    }
}
static void ProcessResetRequest(void) {
	led_fall_mode = 0;
	Buzzer_Set(0);
	BSP_LED_Off(LED2);

	if (led_matrix_ready) {
		Matrix_ShowHappyFace();
	}

	if (oled_ready) {
		OLED_SetInitMessage(&oled);
	}

	UART_Send("RESET\r\n");

	reset_requested = 0;
	detector_reset_requested = 1;
}

// Initialization Function Definitions
static void External_Peripherals_Init(void) {
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

    // If configuration or calibration failed, then UART error message is shown with blinking LED
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) {
        UART_Send("ADC channel configuration failed\r\n");
        while (1) {
            BSP_LED_Toggle(LED2);
            HAL_Delay(200);
        }
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        UART_Send("ADC calibration failed\r\n");
        while (1) {
            BSP_LED_Toggle(LED2);
            HAL_Delay(200);
        }
    }
}
static void I2C_TestDevices(void) {
    if (I2C_DevicePresent(OLED_ADDR)) {
        UART_Send("OLED DETECTED\r\n");
    } else {
        UART_Send("OLED NOT DETECTED\r\n");
    }

    if (I2C_DevicePresent(MATRIX_ADDR)) {
        UART_Send("LED MATRIX DETECTED\r\n");
    } else {
        UART_Send("LED MATRIX NOT DETECTED\r\n");
    }

    if (I2C_DevicePresent(SWITCH_ADDR)) {
        UART_Send("5 WAY SWITCH DETECTED\r\n");
    } else {
        UART_Send("5 WAY SWITCH NOT DETECTED\r\n");
    }
}
static uint8_t I2C_DevicePresent(uint16_t address) {
    return HAL_I2C_IsDeviceReady(
        &hi2c1,
        address,
        2,
        100
    ) == HAL_OK;
}
static void Wifi_Full_Init(void) {
	if(WIFI_Init() == WIFI_STATUS_OK) {
		UART_Send("WIFI_INIT SUCCESS\r\n");
	} else {
		UART_Send("WIFI_INIT FAILED\r\n");
		while(1); // halt computations if a connection could not be established with the server
	}

	if(WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security) == WIFI_STATUS_OK) {
		UART_Send("WIFI_CONNECT SUCCESS\r\n");
	} else {
		UART_Send("WIFI_CONNECT FAILED\r\n");
		while(1); // halt computations if a connection could not be established with the server
	}

	if(WIFI_Ping(ipaddr, 3, 200) == WIFI_STATUS_OK) {
		UART_Send("PING SUCCESS\r\n");
	} else {
		UART_Send("PING FAILED\r\n");
	}

	// Make a TCP connection
	if(WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT) == WIFI_STATUS_OK) {
		UART_Send("TCP CONNECTION SUCCESS\r\n");
	} else {
		UART_Send("TCP CONNECTION FAILED\r\n");
		while(1); // halt computations if a connection could not be established with the server
	}
}
static void I2C_Devices_Full_Init(void) {
	if (SSD1306_Init(&oled, &hi2c1, OLED_ADDR) == HAL_OK) {
		OLED_SetInitMessage(&oled);
		oled_ready = 1;
		UART_Send("SSD1306 INIT SUCCESS\r\n");
	} else {
		UART_Send("SSD1306 INIT FAILED\r\n");
	}

	if (HT16K33_Init(&led_matrix, &hi2c1, MATRIX_ADDR) == HAL_OK) {
		Matrix_ShowHappyFace();
		led_matrix_ready = 1;
		UART_Send("HT16K33 INIT SUCCESS\r\n");
	} else {
		UART_Send("HT16K33 INIT FAILED\r\n");
	}

	if (GroveMultiSwitch_Init(&grove_switch, &hi2c1, SWITCH_ADDR) == HAL_OK) {
		switch_ready = 1;
		UART_Send("5 WAY SWITCH INIT SUCCESS\r\n");
	} else {
		UART_Send("5 WAY SWITCH INIT FAILED\r\n");
	}
}

// Peripherals Helper Function Definitions
static void ProcessSwitchEvents(void) {
    GroveMultiSwitch_EventTypeDef event;

    if (GroveMultiSwitch_ReadEvent(&grove_switch, &event) != HAL_OK ||
        !(event.event & GROVE_MULTI_SWITCH_EVENT_PRESENT)) {
        return;
    }

    if (oled_ready) {
	    if (event.button[0] & GROVE_MULTI_SWITCH_SINGLE_CLICK) {
    		SSD1306_Clear(&oled);
			SSD1306_SetCursor(&oled, 16, 16);
			SSD1306_WriteString(&oled, "NAME: OLD MAN");
			SSD1306_SetCursor(&oled, 16, 32);
			SSD1306_WriteString(&oled, "DOB: 1970-01-01");
			SSD1306_Update(&oled);
	    } else if (event.button[1] & GROVE_MULTI_SWITCH_SINGLE_CLICK) {
    		SSD1306_Clear(&oled);
			SSD1306_SetCursor(&oled, 16, 16);
			SSD1306_WriteString(&oled, "BLOOD TYPE: O");
			SSD1306_SetCursor(&oled, 16, 32);
			SSD1306_WriteString(&oled, "ALLERGIES: NA");
			SSD1306_Update(&oled);
	    } else if (event.button[2] & GROVE_MULTI_SWITCH_SINGLE_CLICK) {
    		SSD1306_Clear(&oled);
			SSD1306_SetCursor(&oled, 16, 16);
			SSD1306_WriteString(&oled, "AGE: 100");
			SSD1306_Update(&oled);
	    } else if (event.button[3] & GROVE_MULTI_SWITCH_SINGLE_CLICK) {
    		SSD1306_Clear(&oled);
			SSD1306_SetCursor(&oled, 0, 16);
			SSD1306_WriteString(&oled, "  HOME NUM: 8654 3210");
			SSD1306_SetCursor(&oled, 0, 32);
			SSD1306_WriteString(&oled, "FAMILY NUM: 8655 4322");
			SSD1306_Update(&oled);
	    } else if (event.button[4] & GROVE_MULTI_SWITCH_SINGLE_CLICK) {
    		OLED_SetInitMessage(&oled);
    	}
    }
}
static uint16_t SoundSensor_Read(void) {
    uint16_t value = 0;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);

    return value;       /* 0 to 4095 */
}
static void Buzzer_Set(uint8_t enabled) {
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_4,
        enabled ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

// Matrix Helper Function Definitions
static void Matrix_ShowFace(const uint8_t face[8][8]) {
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
        {1, 0, 0, 1, 1, 0, 0, 1},
        {1, 0, 1, 0, 0, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 1, 0, 1},
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
        {1, 0, 0, 1, 1, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 0, 0, 1, 0, 1},
        {0, 1, 0, 0, 0, 0, 1, 0},
        {0, 0, 1, 1, 1, 1, 0, 0}
    };

    Matrix_ShowFace(sad_face);
}

// OLED Helper Function Definitions
static void OLED_SetInitMessage(SSD1306_HandleTypeDef *display) {
    SSD1306_Clear(display);
    SSD1306_SetCursor(display, 16, 16);
    SSD1306_WriteString(display, "FALL DETECTOR");
    SSD1306_SetCursor(display, 16, 32);
    SSD1306_WriteString(display, "SYSTEM READY");
    SSD1306_Update(display);
}
static void OLED_SetFallMessage(SSD1306_HandleTypeDef *display) {
	SSD1306_Clear(display);
	SSD1306_SetCursor(display, 0, 0);
	SSD1306_WriteString(display, "IVE FALLEN  HELP ME");
	SSD1306_SetCursor(display, 0, 16);
	SSD1306_WriteString(display, "FAMILY NUM: 8655 4322");
	SSD1306_SetCursor(display, 8, 32);
	SSD1306_WriteString(display, "NAME: OLD MAN");
	SSD1306_SetCursor(display, 8, 48);
	SSD1306_WriteString(display, "AGE: 85");
	SSD1306_Update(display);
}
static void OLED_SetLongLieMessage(SSD1306_HandleTypeDef *display) {
    SSD1306_Clear(display);
    SSD1306_SetCursor(display, 16, 0);
    SSD1306_WriteString(display, "NO MOVEMENT");
    SSD1306_SetCursor(display, 16, 16);
    SSD1306_WriteString(display, "LONG LIE ALERT");
    SSD1306_SetCursor(display, 16, 32);
    SSD1306_WriteString(display, "CALL 995");
    SSD1306_Update(display);
}

// Interrupt Function Definitions
void HAL_SYSTICK_Callback(void) {
	// NOTE: Added "HAL_SYSTICK_IRQHandler();" to
	// "void SysTick_Handler(void)" in stm32l4xx.it.c

    static uint32_t elapsed_ms = 0;
    static uint8_t previous_mode = 0xFF;

    if (!led_timer_enabled) {
        return;
    }

    // Restart the timing when the fall state changes
    if (led_fall_mode != previous_mode) {
        elapsed_ms = 0;
        previous_mode = led_fall_mode;
        BSP_LED_Off(LED2);
    }

    elapsed_ms++;

    uint32_t toggle_period = led_fall_mode ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS;

    if (elapsed_ms >= toggle_period) {
        elapsed_ms = 0;
        BSP_LED_Toggle(LED2);
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_1) {
    	SPI_WIFI_ISR();
    }

    if (GPIO_Pin == BUTTON_EXTI13_Pin) {
        // Ignore switch bounce for 250 ms
        if ((current_time - last_press_time) > 250U) {
            reset_requested = 1;
            last_press_time = current_time;
        }
    }
}
void SPI3_IRQHandler(void) {
	HAL_SPI_IRQHandler(&hspi3);
}

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
