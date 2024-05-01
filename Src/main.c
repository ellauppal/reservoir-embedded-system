#include "main.h"

#define ZONE_MAX_RPM 2000
#define ZONE_MIN_RPM 1200

volatile uint8_t CURR_WALL_CLK = 0;

uint16_t clock_hours = 0;

typedef struct Zone{
	uint16_t start_time;
	uint16_t stop_time;
	uint16_t diff_time;
	uint16_t elapsed_time;
	uint16_t pwm;
} Zone;

Zone ZONE[4];
int curr_zone = -1;

uint16_t max_depth = 10;
uint16_t min_depth = 100;

uint16_t clock_secs;

uint32_t water_percent = 100;

uint16_t last_hour = 0;

/* US100 Code BEGIN */

uint8_t cmd_dist = 0x55; /* Trigger value */

volatile uint8_t us100_Rx_flag = 0;
volatile uint8_t term_Rx_flag = 0;

/* Two bytes to store distance detected from US100 */
uint8_t us100_buffer[2] = {0};

uint8_t msg_buffer[64] = {0};
volatile uint16_t distance = 0;

/* US100 Code END */

/* Option from command line */
uint8_t option;
uint8_t option_arr[2];
uint16_t byte2;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

volatile uint16_t rpm_tick_count = 0;
volatile uint16_t rpm_tick_count_old = 0;
volatile uint16_t rpm = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);

/* Private user code ---------------------------------------------------------*/
void ADC_Select_CH(int CH)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	switch(CH)
	{
	case 0:
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 1:
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 2:
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 3:
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 4:
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 5:
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 6:
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 7:
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 8:
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 9:
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 10:
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 11:
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 12:
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 13:
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 14:
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	case 15:
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	break;
	}
}

void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
	 uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
	 int Abit0 = (DIGITA_VAL ) & 1;  	// extract Abit0 of the 4-bit value
	 int Abit1 = (DIGITA_VAL >> 1) & 1;  // extract Abit1 of the 4-bit value
	 int Abit2 = (DIGITA_VAL >> 2) & 1;  // extract Abit2 of the 4-bit value
	 int Abit3 = (DIGITA_VAL >> 3) & 1;  // extract Abit3 of the 4-bit value

	 uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
	 int Bbit0 = (DIGITB_VAL ) & 1;  	// extract Bbit0 of the 4-bit value
	 int Bbit1 = (DIGITB_VAL >> 1) & 1;  // extract Bbit1 of the 4-bit value
	 int Bbit2 = (DIGITB_VAL >> 2) & 1;  // extract Bbit2 of the 4-bit value
	 int Bbit3 = (DIGITB_VAL >> 3) & 1;  // extract Bbit3 of the 4-bit value

	 if (Abit0 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);

	 }
	 if (Abit1 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);

	 }
	 if (Abit2 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);

	 }
	 if (Abit3 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);

	 }


	 if (Bbit0 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);

	 }
	 if (Bbit1 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);

	 }
	 if (Bbit2 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET);

	 }
	 if (Bbit3 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);

	 }
}


uint16_t getInput(){
	  uint8_t txd_msg_buffer[1000] = {0};

	  term_Rx_flag = 0;
	  HAL_UART_Receive_IT(&huart6, option_arr, 2);

	  while(term_Rx_flag == (00)){}; /* After transmitting, wait for flag to be set */

	  term_Rx_flag = 0;

	  byte2 = (uint16_t)(option_arr[0] - '0')*10 + (option_arr[1] - '0');

	  sprintf((char*)txd_msg_buffer, "%u", byte2);
	  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	  return byte2;
}

uint64_t scaleTime(uint16_t time_passed){
	// example lets say 24: 24 hours -> 2.4 minutes -> 2.4 x 60
	uint64_t time_in_secs = time_passed * 6;
	return time_in_secs;
}

void setZone(int zone){
	/* Servo Motor Code */
	uint16_t txd_msg_buffer[1000] = {0};
	  if (zone == 0){
		 TIM2->CCR1 = 500;
	  } else if (zone == 1){
		  TIM2->CCR1 = 1700;
	  } else if (zone == 2){
		 TIM2->CCR1 = 1325;
	  } else if (zone == 3){
		  TIM2->CCR1 = 800;
	  }
}

uint16_t getPotentiometer()
{
	uint16_t txd_msg_buffer[1000] = {0};
	ADC_Select_CH(9);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);

	uint16_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
	uint16_t percent = (uint16_t)(ADC_CH9/2.55);
	HAL_ADC_Stop(&hadc1);


	return percent;
}

uint64_t getPwm(uint16_t option_num, uint8_t zone)
{
	if (option_num == 0)
	{
		return (uint64_t)(ZONE_MIN_RPM + ((getPotentiometer()/100) * (ZONE_MAX_RPM - ZONE_MIN_RPM)));
	}
	else if(option_num == 1)
	{
		return (uint64_t)(ZONE_MIN_RPM + (0.60 * (ZONE_MAX_RPM - ZONE_MIN_RPM)));
	}
	else if (option_num == 2)
	{
		return (uint64_t)(ZONE_MIN_RPM + (0.80 * (ZONE_MAX_RPM - ZONE_MIN_RPM)));
	}
	else if (option_num == 3)
	{
		return (uint64_t)(ZONE_MIN_RPM + (0.99 * (ZONE_MAX_RPM - ZONE_MIN_RPM)));
	}
}

void display_distance()
{
	  uint8_t digit_a = 0;
	  uint8_t digit_b = 0;
	  HAL_UART_Receive_IT(&huart1,us100_buffer,2); /* The two bytes are capture sin the us100 buff */
	  HAL_UART_Transmit(&huart1, &cmd_dist, 1, 500);

	  while(us100_Rx_flag == (00)){}; /* After transmitting, wait for flag to be set */

	  uint16_t full_16_bits = (us100_buffer[0] << 8) + us100_buffer[1];

	  water_percent = (uint64_t)( (270 - (full_16_bits - 30)) / 2.7 );
	  if (water_percent > 99)
	  {
		  water_percent = 99;
	  }

	  uint8_t txd_msg_buffer[1000] = {0};

	  digit_b = water_percent%10;
	  digit_a = water_percent/10;
	  DIGITS_Display(digit_a, digit_b);
}

void run_mode_print(){
	  uint8_t txd_msg_buffer[1000] = {0};

	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	uint16_t pwm_percent;

	if(ZONE[curr_zone].pwm == 0)
	{

		pwm_percent = getPotentiometer();

	}
	else if (ZONE[curr_zone].pwm == 1)
	{
		pwm_percent = 60;
	}
	else if (ZONE[curr_zone].pwm == 2)
	{
		pwm_percent = 80;
	}
	else if (ZONE[curr_zone].pwm == 3)
	{
		pwm_percent = 99;
	}

	rpm = (rpm_tick_count/2);
	rpm_tick_count = 0;

	sprintf((char*)txd_msg_buffer, "\r\n %u | %i | %u | %u | %u | \r\n", clock_hours, curr_zone, pwm_percent, rpm, water_percent);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t txd_msg_buffer[1000] = {0};
  uint8_t count = 0;
  TIM3->ARR = 10000-1;

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();

  int TIM2_Ch1_DCVAL = 500;
  int TIM2_CH1_STEP = 20;

  /* Servo Code INIT */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM2->PSC = 16-1;
  TIM2->ARR = 20000-1;
  TIM2->CCR1 = TIM2_Ch1_DCVAL;

  /* DC Motor Code INIT */
  HAL_TIM_Base_Init(&htim3);
  int TIM3_CH1_DCVAL = 1200;
  int TIM3_CH1_STEP = 100;

  sprintf((char*)txd_msg_buffer,"\r\nSETUP MODE\r\n"
		  "\r\nUse the following list for entering the PWM option for the MOTOR SPEED for a ZONE or INLET: \r\n"
		  "0) Manual Control (the Potentiometer setting in Run Mode for the chosen Zone or Inlet); \r\n"
		  "1) 60% PWM; 2) 80% PWM; 3) 99% PWM. \r\n\r\n"
		  "INLET MOTOR SPEED PWM (option 0-3): ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

  ZONE[0].pwm = getInput();

  sprintf((char*)txd_msg_buffer, "\r\nZONE 1 MOTOR SPEED PWM (option 0-3): ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

  ZONE[1].pwm = getInput();

  sprintf((char*)txd_msg_buffer, "\r\nZONE 2 MOTOR SPEED PWM (option 0-3): ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

  ZONE[2].pwm = getInput();

   sprintf((char*)txd_msg_buffer, "\r\nZONE 3 MOTOR SPEED PWM (option 0-3): ");
   HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

   ZONE[3].pwm = getInput();

   // ---------------------------------------------------

   sprintf((char*)txd_msg_buffer,"\r\n\r\nNOTE: WALL-CLOCK TIME INPUT is expressed in 24 hour format.\r\n"
		  "The entry options are: 0 – midnight, 1 – 1:00am, 2- 2:00am, ..., 12 – noon, 13- 1:00pm, ..., 23 – 11:00pm\r\n\r\n"
		   "CURRENT WALL CLOCK TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	CURR_WALL_CLK = getInput();

	// ---------------------------------------------------

	sprintf((char*)txd_msg_buffer, "\r\nINLET WALL CLOCK START TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[0].start_time = getInput();

	sprintf((char*)txd_msg_buffer, "\r\nINLET WALL CLOCK STOP TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[0].stop_time = getInput();

	ZONE[0].diff_time = ZONE[0].stop_time - ZONE[0].start_time;

	// ---------------------------------------------------

	sprintf((char*)txd_msg_buffer, "\r\nZONE 1 WALL CLOCK START TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[1].start_time = getInput();

	sprintf((char*)txd_msg_buffer, "\r\nZONE 1 WALL CLOCK STOP TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[1].stop_time = getInput();

	ZONE[1].diff_time = ZONE[1].stop_time - ZONE[1].start_time;

	// ---------------------------------------------------

	sprintf((char*)txd_msg_buffer, "\r\nZONE 2 WALL CLOCK START TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[2].start_time = getInput();

	sprintf((char*)txd_msg_buffer, "\r\nZONE 2 WALL CLOCK STOP TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[2].stop_time = getInput();

	ZONE[2].diff_time = ZONE[2].stop_time - ZONE[2].start_time;

	// ---------------------------------------------------

	sprintf((char*)txd_msg_buffer, "\r\nZONE 3 WALL CLOCK START TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[3].start_time = getInput();

	sprintf((char*)txd_msg_buffer, "\r\nZONE 3 WALL CLOCK STOP TIME (0-23): ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	ZONE[3].stop_time = getInput();

	ZONE[3].diff_time = ZONE[3].stop_time - ZONE[3].start_time;

	// ---------------------------------------------------

	sprintf((char*)txd_msg_buffer, "\r\n");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
	}

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	sprintf((char*)txd_msg_buffer, "\r\nRUN MODE\r\n");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "\r\n Wall-Clock Time| Zone/Inlet | Motor Speed %PWM | Motor RPM | Water Reservoir Depth |\r\n");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

  	/* TIMER INIT */
	int entered = 0;
	if (CURR_WALL_CLK != 0)
	{
		clock_secs = CURR_WALL_CLK * 6;
	}

	if (ZONE[0].start_time != CURR_WALL_CLK && ZONE[1].start_time != CURR_WALL_CLK && ZONE[2].start_time != CURR_WALL_CLK && ZONE[3].start_time != CURR_WALL_CLK){
		entered = 1;
		HAL_TIM_Base_Start_IT(&htim5);
	}

  while (1)
  {
	  if (clock_secs == scaleTime(ZONE[0].start_time)){
		  curr_zone = 0;
	  } else if (clock_secs == scaleTime(ZONE[1].start_time)){
		  curr_zone = 1;
	  } else if (clock_secs == scaleTime(ZONE[2].start_time)){
		  curr_zone = 2;
	  } else if (clock_secs == scaleTime(ZONE[3].start_time)){
		  curr_zone = 3;
	  } else {
		  curr_zone = -1;
	  }

	  if (curr_zone == 0){
		  rpm_tick_count = 0;
		  rpm_tick_count_old = 0;
		  	if (entered == 0 && ZONE[0].start_time == CURR_WALL_CLK){
		  		 entered = 1;
		  		 HAL_TIM_Base_Start_IT(&htim5);
		  	}
			setZone(curr_zone);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			uint64_t wait_time = scaleTime(ZONE[0].stop_time);
			while((wait_time > clock_secs) && (water_percent > 0))
			{
				TIM3->CCR1 = 0;
				TIM3->CCR3 = getPwm(ZONE[0].pwm, 0);
				/* Purple */
				HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_SET);
			    HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
			    display_distance();
			    if (clock_hours != last_hour)
				{
					run_mode_print();
					last_hour = clock_hours;
				}
			    else if (water_percent == 99 || water_percent == 0)
			    {
				    run_mode_print();
			    }
			}
			HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
			TIM3->CCR3 = 0;
	  } else if (curr_zone > 0) {
		  rpm_tick_count = 0;
		  rpm_tick_count_old = 0;
		  	if (entered == 0 && ZONE[curr_zone].start_time == CURR_WALL_CLK){
		  		 entered = 1;
		  		 HAL_TIM_Base_Start_IT(&htim5);
		  	}
			setZone(curr_zone);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			uint64_t wait_time = scaleTime(ZONE[curr_zone].stop_time);
			while ((wait_time > clock_secs)  && (water_percent > 0))
			{
				TIM3->CCR3 = 0;
				TIM3->CCR1 = getPwm(ZONE[curr_zone].pwm, 0);
				if (curr_zone == 1)
				{
				    HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_SET);
				}
				else if (curr_zone == 2)
				{
					HAL_GPIO_WritePin(GPIOB, GREEN_Pin, GPIO_PIN_SET);
				}
				else if (curr_zone == 3)
				{
					HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_SET);
				}
			    display_distance();
			    if (clock_hours != last_hour)
			    {
				    run_mode_print();
				    last_hour = clock_hours;
			    }
			    else if (water_percent >= 99 || water_percent == 0)
			    {
				    run_mode_print();
			    }
			}
			HAL_GPIO_WritePin(GPIOB, BLUE_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, RED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GREEN_Pin, GPIO_PIN_RESET);
			TIM3->CCR1 = 0;
	  }
	  if (clock_hours == 24 || water_percent == 0)
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  break;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin|RED_Pin|DIGIT_B0_Pin|DIGIT_B1_Pin
                          |DIGIT_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLUE_Pin|DIGIT_A0_Pin|GREEN_Pin|DIGIT_A1_Pin
                          |DIGIT_A2_Pin|DIGIT_A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_B3_Pin RED_Pin DIGIT_B0_Pin DIGIT_B1_Pin
                           DIGIT_B2_Pin */
  GPIO_InitStruct.Pin = DIGIT_B3_Pin|RED_Pin|DIGIT_B0_Pin|DIGIT_B1_Pin
                          |DIGIT_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Tick_Pin */
  GPIO_InitStruct.Pin = RPM_Tick_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_Tick_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BLUE_Pin DIGIT_A0_Pin GREEN_Pin DIGIT_A1_Pin
                           DIGIT_A2_Pin DIGIT_A3_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin|DIGIT_A0_Pin|GREEN_Pin|DIGIT_A1_Pin
                          |DIGIT_A2_Pin|DIGIT_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART6){
		term_Rx_flag = 01; // This flag is set to show that a receiver interrupt has occurred
	}
	if (huart->Instance == USART1){

		us100_Rx_flag = 01; // This flag is set to show that a receiver interrupt has occurred
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == RPM_Tick_Pin)
	{
		rpm_tick_count += 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		clock_secs += 1;
		if (curr_zone==0){
			ZONE[0].elapsed_time += 1;
		} else if (curr_zone==1){
			ZONE[1].elapsed_time += 1;
		} else if(curr_zone==2){
			ZONE[2].elapsed_time += 1;
		} else if(curr_zone==3){
			ZONE[3].elapsed_time += 1;
		}

		clock_hours = clock_secs/6; /* 6 seconds = 1 hour */

	}
}

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
