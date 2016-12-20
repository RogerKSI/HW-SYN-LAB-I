/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2559 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define M_PI 3.14159265359
#define audiofreq 46875.0
double freq[] = { 220.0, 246.942, 130.813, 146.832, 164.814, 174.614, 195.998 };
uint16_t graph[7][400];
int ns[100];
int width = 80, height = 23;
int px, py;
int monster = 0;
int score = 0;
int sx[1010], sy[1010];
int ex[1010], ey[1010];
int status[1000];
int ani = 0;
char map[30][90];
int lose = 0;
char initmap[][81] =
		{
				"                                                                               ",
				"                                                      Score :   0000           ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"                                                                               ",
				"###############################################################################",
				"###############################################################################" };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int rand(unsigned long int time, int value) {
	time = time * 1103515245UL + 12345;
	return (unsigned int) (time / 65536) % value;
}

void accelerameter_init() {
	//SPI transmit
	// bring low to activate slave
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	// set 0x67 to 0x20
	uint8_t address, data;
	address = 0x20;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	data = 0x67;
	HAL_SPI_Transmit(&hspi1, &data, 1, 50);
	// bring cs high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

void microphone_init() {
	return;
}

void speaker_init() {
	uint8_t send[2];
	uint8_t tmp;

	// power-up sequence

	// hold reset low
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	// delay until power supplies are stable
	HAL_Delay(100);

	// bring reset high
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

	// 3. set 0x01 to 0x02 (power ctl. 1)
	send[0] = 0x02;
	send[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);

	// desired register setting

	// set volume a,b
	send[0] = 0x20;
	send[1] = 0xF0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);
	send[0] = 0x21;
	send[1] = 0xF0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);

	// required initialization setting

	// set 0x00 to 0x99
	send[0] = 0x00;
	send[1] = 0x99;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);
	// set 0x47 to 0x80
	send[0] = 0x47;
	send[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);
	// set 1'b to bit 7 to 0x32
	send[0] = 0x32;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x94, &tmp, 1, 1000);
	send[1] = tmp | 128;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);
	// set 0'b to bit 7 to 0x32
	send[0] = 0x32;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c1, 0x94, &tmp, 1, 1000);
	send[1] = tmp & 127;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);
	// 4.5. Write 0x00 to register 0x00.
	send[0] = 0x00;
	send[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);

	// set 0x9E to 0x02 (power ctl.1)
	send[0] = 0x02;
	send[1] = 0x9E;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, send, 2, 1000);

	// 0 to 0xFFF
	for (int i = 0; i < 7; i++) {
		ns[i] = (int) (audiofreq / freq[i]);
		for (int j = 0; j < ns[i]; j++) {
			graph[i][j] = (sin(2 * M_PI * j / ns[i]) + 1) * ((0xfff + 1) / 2);
		}
	}
}

void get_accelerameter(int8_t *axis) {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	// transmit address for receive data
	uint8_t address;
	// x
	//////////////////////////////////////////////////
	address = 0x29 + 0x80;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &axis[0], 1, 50);

	// y
	//////////////////////////////////////////////////

	address = 0x2B + 0x80;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &axis[1], 1, 50);

	// z
	//////////////////////////////////////////////////
	address = 0x2C + 0x80;
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	HAL_SPI_Receive(&hspi1, &axis[2], 1, 50);
	//////////////////////////////////////////////////

	// bring cs high
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}
int absolute(int x) {
	return x < 0 ? -x : x;
}

int get_microphone() {
	int n = 100;
	uint16_t pdm_stream[n * 17];
	int pdm_signal[n * 17];
	int buffer[n * 17];
	int pcm_signal[n * 17];
	int reverse[17];

	HAL_I2S_Receive(&hi2s2, pdm_stream, n, 1000);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < 16; j++) {
			reverse[j] = pdm_stream[i] % 2;
			pdm_stream[i] >>= 1;
		}

		for (int j = 0; j < 16; j++) {
			pdm_signal[i * 16 + j] = reverse[15 - j];
		}
	}

	for (int i = 7; i + 8 < n * 16; i++) {
		int tmp = 0;
		for (int j = -7; j <= 8; j++)
			tmp += pdm_signal[i + j];
		buffer[i] = absolute(tmp - 8);
	}

	int ans = 0;
	for (int i = 14; i + 16 < n * 16; i++) {
		for (int j = -7; j <= 8; j++)
			ans += buffer[i + j];
	}

	char a[100];
	int len = sprintf(a, "%d", ans);
	return ans;
}

void playsound(char *str, int size) {
	for (int i = 0; i < size; i++) {
		if (str[i] - 'A' >= 0 && str[i] - 'A' < 7) {

			for (int j = 0; j < (int) (freq[str[i] - 'A'] / 2); j++)
				HAL_I2S_Transmit(&hi2s3, graph[str[i] - 'A'], ns[str[i] - 'A'],
						1000);
		}
	}
}

void writemario() {
	ani++;
	ani %= 2;
	map[px - 2][py] = 'O';
	map[px - 1][py] = '|';
	map[px - 1][py - 1] = '/';
	map[px - 1][py + 1] = '\\';
	map[px][py - 1 + ani] = '/';
	map[px][py + ani] = '\\';

}

void writemap() {
	for (int i = 0; i < height; i++) {
		HAL_UART_Transmit(&huart2, map[i], width, 1000);
		HAL_UART_Transmit(&huart2, "\n\r", 2, 1000);
	}
	HAL_UART_Transmit(&huart2, "\033[0;0H", strlen("\033[0;0H"), 1000);
}

void resetmap() {
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			map[i][j] = initmap[i][j];
}

void writemonster() {
	for (int i = 1; i <= monster; i++) {
		if (status[i] == 0)
			continue;
		map[ex[i] - 2][ey[i]] = '-';
		map[ex[i] - 1][ey[i]] = '|';
		map[ex[i]][ey[i]] = '-';
		int temp = i;
		int p = 0;
		while (temp) {
			p++;
			map[ex[i] - 2][ey[i] - p] = '-';
			map[ex[i] - 1][ey[i] - p] = (temp % 10) + '0';
			map[ex[i]][ey[i] - p] = '-';
			temp /= 10;
		}
		map[ex[i] - 2][ey[i] - p - 1] = '-';
		map[ex[i] - 1][ey[i] - p - 1] = '|';
		map[ex[i]][ey[i] - p - 1] = '-';

		sx[i] = ex[i] - 2;
		sy[i] = ey[i] - p - 1;

	}
}

void addmonster() {
	monster++;
	status[monster] = 1;
	ex[monster] = 20;
	ey[monster] = 79;
	sx[monster] = 20;
	sy[monster] = 79;
}

int checkcollision() {
	int c = 0;
	for (int i = 1; i <= monster; i++) {
		if (status[i] == 1) {
			if (px >= sx[i] && py + 1 >= sy[i] && ex[i] >= px - 2
					&& ey[i] >= py - 1) {
				if (px <= 18) {
					playsound("AD", 2);
					score++;
					status[i] = 0;
				} else
					return 1;
			}
		} else
			c++;
	}

	return 0;
}

void writescore() {
	sprintf(&map[1][65], "%04d", score);
}
void writeend() {
	sprintf(&map[4][37], "Your score : %4d !!", score);
	sprintf(&map[6][30], "press user button to start new game");
}
void updatemonster() {
	for (int i = 1; i <= monster; i++)
		if (status[i] == 1) {
			ey[i]--;
			sy[i]--;
			if (ey[i] < 0)
				status[i] = 0;
		}
}
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2S2_Init();
	MX_I2S3_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	accelerameter_init();
	microphone_init();
	speaker_init();
	int firsttime=0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//delay for stable sound
		if (firsttime==0 || HAL_GPIO_ReadPin(GPIOA, 1 << 0)) {
			firsttime=1;
			int8_t axis[4];
			int maxer = 10;
			int countdown = 3;
			int count = 0;
			lose = 0;
			score = 0;
			monster = 0;
			px = 20;
			py = 15;

			HAL_Delay(1000);
			int old_sound = get_microphone();

			while (1) {
				resetmap();
				int new_sound = get_microphone();
				if (px >= 5 && new_sound > old_sound + 1000 && px >= 19) {
					px -= 7;
				} else {
					if (px <= 19)
						px++;
				}
				old_sound = new_sound;

				int ran = rand((HAL_GetTick() * 3 + 5) * 7 + 9, 5);
				if (ran == 4) {
					count++;
					if (count >= maxer) {
						addmonster();
						count = 0;
						if (maxer > 0) {
							countdown--;
							if (countdown == 0) {
								maxer--;
								countdown = 3;
							}

						}
					}
				}

				get_accelerameter(axis);
				if (py >= 10 && axis[0] >= 30)
					py -= 2;
				else if (py <= 75 && axis[0] <= -30)
					py += 2;

				updatemonster();
				writemonster();
				writemario();
				writescore();
				writemap();
				int tmp = checkcollision();
				if (tmp == 1) {
					lose = 1;
					break;
				}

			}

			writeend();
			writemap();
		}
		HAL_Delay(1000);
	}

	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

}

/* I2S2 init function */
static void MX_I2S2_Init(void) {

	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}

}

/* I2S3 init function */
static void MX_I2S3_Init(void) {

	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}

}

/* SPI1 init function */
static void MX_SPI1_Init(void) {

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA9   ------> USB_OTG_FS_VBUS
 PA10   ------> USB_OTG_FS_ID
 PA11   ------> USB_OTG_FS_DM
 PA12   ------> USB_OTG_FS_DP
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_I2C_SPI_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BOOT1_Pin */
	GPIO_InitStruct.Pin = BOOT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
	 Audio_RST_Pin */
	GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : VBUS_FS_Pin */
	GPIO_InitStruct.Pin = VBUS_FS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
	GPIO_InitStruct.Pin = OTG_FS_ID_Pin | OTG_FS_DM_Pin | OTG_FS_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
