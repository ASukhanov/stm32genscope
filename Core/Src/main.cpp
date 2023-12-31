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
/*
 *------|----|-----------
 * MCU  Nucleo Device Description
 *------|----|-----------
 * ADC pins v0.4+
 *------|----|-----------
 * PA0,  A0:  ADC1.1
 * PF0,  D7:  ADC1.10,
 * PB0.  D3:  ADC1.15
 * PA1,  A1:  ADC1.2
 * PA6,  A5:  ADC2.3
 * PA7,  A6:  ADC2.4
 * PA4,  A3:  ADC2.17
 * PF1,  D8:  ADC2.10
 *------|----|-----------
 * Timers
 *------|----|-----------
 * PA10, D0:  TIM2.CH4, 32-bit Gate-Delay Generator output.
 * PA15, D5:  TIM2.CH1, 32-bit Gate-Delay Generator input.
 * PA9,  D1:  TIM1.CH2, TIM_ADC trigger input, Used for triggering ADC on its falling edge.
 * PA8,  D9:  TIM1.CH1, TIM_ADC output, for testing. Does not fire when adc_trig=ADC1P
 * PA11, D10: TIM4.CH1, 16-bit pulser, output. Used for debugging as a generator of Trigger and input for ADC.
 *#PB4,  D12: SPI3.MISO,
 *#PB5,  D11: SPI3.MOSI
 * PB5,  D11: LED_RED
 *#PB3,  D13: SPI3.SCLK
 *------|----|-----------
 * DACs
 *------|----|-----------
 *#PA4,  A3:  DAC1_OUT1, Waveform generator
 *PA5,  A4:  DAC1_OUT2, Simple DAC
 *------|----|-----------
 * GPIO_PINS
 *------|----|-----------
 * PB8,  LD2: sleep toggle, UART1 input toggle
 * #PA12, D2:  LED_BLUE, WFG_START: Resets on DAC_ConvCpltCallbackCh1, sets on DAC_ConvHalfCpltCallbackCh1. Also toggles when: TIM_PULSER_PulseFinished,  TIM_ADC_Trigger, TIM_GDG_DelayElapced.
 * PA12, D2:  LED_GREEN, ADC_ConvCpltCallback
 * PB6,	 D6:  LED_BLUE, Send data, Comp3_Callback
 * #PB7, D4:  LED_RED, Flashes During command execution. Sets in case of UART_ErrorCalback.
 * PB7,  D6:  Comp3 output,
 * PG10  NRST ,not functional

 Test using miniterm:
miniterm /dev/ttyACM0 7372800

 Test using stmscope.py
python3 python/stmscope.py -g
info
dict: {'PVs': ['version', 'debug', 'fec', 'sleep', 'pulser_width', 'pulser_period', 'pulser_prescaler', 'pulser_fire', 'pulser_inverted', 'gdg_trig', 'gdg_front', 'gdg_tail', 'gdg_inverted', 'nADC', 'adc_reclen', 'adc_srate', 'adc_delay', 'adc_trig', 'adc_prescaler', 'adc_sampletime', 'dac', 'trig_level']}
  reply: {'PVs': ['version', 'fec', 'pwm_period', 'pwm_width', 'pwm_prescaler', 'gdg_front', 'gdg_tail', 'nADC', 'adc_reclen', 'adc_srate', 'adc_delay', 'wfg_shape', 'wfg_level', 'dac']}
  Testing:
  Connect signal to ADC1 (Pa0)
  Start python3 python/stm32scope.py -q -g
set adc_trig ADC1P
set adc_sampletime 2
set adc_prescaler 4
  The adc_srate will be 708333, and 100-sample waveform should cover 1.058 ms,
*/
#define VERSION "v1.0.4 2023-12-31"//Release with STM32CubeIDE v1.14.0
//ISSUE: 'set pulse fire 4' does not work after 'set pulse fire 1'
//TODO: restore ADC setting when adc_trig changes
//TODO: handling of pv_pulser could be done much simpler
//TODO: set adc_sampletime
//TODO: WRN: ADC2 Busy
//TODO: gdg_fire
//TODO: wfg_trig circular. see HAL_DMA_Init in stm32g4xx_hal_msp.c
//TODO: wfg_trig pulser does not work: start_wfg in callback
//ISSUE: if wfg_trig=pulser, the start_wfg in the callback results in jitter ~1ms
//Note: The jitter is almost zero when wfg_step = 4.
//The tight coupling could be achieved by using circular DMA mode and generate exact number of pulses on each trigger. The DAC timer could be TIM15 and for triggerin from TIM4 it need to be sourced from ITR3.

#define mega 1000000
#define SYSCLK 170000000 //system clock, Hz
#define STRINGIZE_NX(A) #A
#define STRINGIZE(A) STRINGIZE_NX(A)
#define MCU_FAMILY "STM32G"
#define MCU_STM32 'G'
#define MCU_CORE "431"

//`````````````````Additional peripherals`````````````````````````````````````
//#define WFG# Uncomment to instantiate the waveform generator
//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
// GPIOs of the STM32G431
#define LED_LD2 GPIOB,GPIO_PIN_8
//#define LED_BLUE GPIOA,GPIO_PIN_12 // PA12
#define LED_BLUE GPIOB,GPIO_PIN_6 // PB6(D6)
//#define WFG_START LED_BLUE
#define LED_GREEN GPIOA,GPIO_PIN_12 // PA10, ADC_start, was PA10=D0
#define LED_RED GPIOB,GPIO_PIN_5
// LEDs are inverted.
#define LED_ON GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET

//`````````````````UART-RX defines````````````````````````````````````````````
#define TTY_RXBUF_SIZE 80
#define COMMAND_MAX_LENGTH TTY_RXBUF_SIZE
#define TIM_ADC_DEADTIME 10000// Limit of the ADC trigger frequency to 100 Hz. This is mostly important for level triggering of noisy signals
// interrupt-based receiving

//#define TTY_BAUDRATE 115200// max for Receive_IT()
#define TTY_BAUDRATE 7372800// max for Receive_IT()
//#define TTY_BAUDRATE 8000000// too much

// DMA-based receiving. Note DMA is working at 10Mbaud in vcom_G431 test but fails here even at 9600
//#define RECEIVING_DMA
//#define UART_RECEIVE HAL_UART_Receive_DMA
//#define TTY_BAUDRATE 7372800
//#define TTY_BAUDRATE 9216000//
//#define TTY_BAUDRATE 115200//
//#define TTY_BAUDRATE 11059200// Bad

#define MCU_VERSION "{\"MCU\":\"" MCU_FAMILY MCU_CORE"\", \"soft\":\"" VERSION"\", \"clock\":" STRINGIZE(SYSCLK) ", \"baudrate\":" STRINGIZE(TTY_BAUDRATE) "}"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstring>
#include <cctype>
#include <stdio.h>
#include "stm32genscope.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADC variables
/* Number of channels in master ADC, it should be equal to hadc1.Init.NbrOfConversion.
 * the slave ADC should have the same number. */
#define NConversions 4
// First active channel, it is needed to calculate sampling rate
#define ADC1FirstChannel 1

//#define NADC2 hadc2.Init.NbrOfConversion
#define MAX_ADC_SAMPLES 100// if it is more than 1 then it causes ADC DMA BUSY if mode is not continuous
// timing for 8ch 11us for REPEAT1 and 15us for REPEAT=4, 188us for 128 MAX_ADC_SAMPLES
#define MaxSamples NConversions*MAX_ADC_SAMPLES// number of conversions
#define BytesPerSample 2
#define Dual 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp3;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;

//SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
//DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
#define huart_data huart2
#define huart_tty huart2
#define TIM_ADC TIM1
#define htim_adc htim1
#define TIM_PULSER TIM4
#define htim_pulser htim4
static uint32_t tim_pulser_count = 0;
#define WFG
#ifdef WFG
	#define DAC1_WFG DAC_CHANNEL_2
	DMA_HandleTypeDef hdma_dac1_ch2;//used in stm32g4xx_hal_msp
#endif// PA4=A3
#define TIM_GDG TIM2
#define htim_gdg htim2
static uint32_t tim_gdg_count = 0;
#define COMP_DAC DAC3

// TTY messages
struct UARTMsg_TTY ttyMsg = {0, 0, UARTID_TTY};// Messages for terminal connection

// Binary packet
struct UARTMsg_Bin {
	struct UARTMsg_Header h;
	char payload[4+4*MaxSamples];
} bbuf = {4 + BytesPerSample*MaxSamples*Dual, ((NConversions*Dual-1)<<2) + ((BytesPerSample-1)&0x3), 'A'};

// UART buffer
uint8_t buffer_rx[TTY_RXBUF_SIZE + 1];
uint8_t usart_buf[COMMAND_MAX_LENGTH];
bool command_received = false;
//bool ttyTX_suspended = false;
//uint16_t usart_buf_length=0;
bool RX_overrun = false;
//bool ADC_suspended = false;

#ifdef WFG
bool wfg_auto = false;// forces wfg_start() in main loop
//`````````````````WFG waveforms``````````````````````````````````````````````
// sine, python code: ((np.sin(x*2*np.pi/len(x))+1)*2048).astype(int)
//TODO: change to uint16
struct {
	uint32_t n;
	uint32_t d[10];
} waveform_shortsine = {10, 2048, 3251, 3995, 3995, 3251, 2048,  844,  100,  100,  844};

struct {
	uint32_t n;
	uint32_t d[100];
} waveform_sine = {100,
		   2048, 2176, 2304, 2431, 2557, 2680, 2801, 2919, 3034, 3145, 3251,
	       3353, 3449, 3540, 3626, 3704, 3777, 3842, 3901, 3952, 3995, 4031,
	       4059, 4079, 4091, 4095, 4091, 4079, 4059, 4031, 3995, 3952, 3901,
	       3842, 3777, 3704, 3626, 3540, 3449, 3353, 3251, 3145, 3034, 2919,
	       2801, 2680, 2557, 2431, 2304, 2176, 2048, 1919, 1791, 1664, 1538,
	       1415, 1294, 1176, 1061,  950,  844,  742,  646,  555,  469,  391,
	        318,  253,  194,  143,  100,   64,   36,   16,    4,    0,    4,
	         16,   36,   64,  100,  143,  194,  253,  318,  391,  469,  555,
	        646,  742,  844,  950, 1061, 1176, 1294, 1415, 1538, 1664, 1791,
	       1919};
// triangle: xx=np.linspace(-2048,2048,51).astype(int); np.array(list(4096 - abs(xx))[:-1] + list(abs(xx))[:-1])
struct {
	uint32_t n;
	uint32_t d[100];
} waveform_triangle = {100, 0, 83, 167, 250, 334, 417, 501, 585, 668, 752, 835, 919, 1002, 1086, 1170, 1253, 1337, 1420, 1504, 1587, 1671, 1755, 1838, 1922, 2005, 2089, 2172, 2256, 2340, 2423, 2507, 2590, 2674, 2757, 2841, 2925, 3008, 3092, 3175, 3259, 3342, 3426, 3510, 3593, 3677, 3760, 3844, 3927, 4011, 4095, 4095, 4011, 3927, 3844, 3760, 3677, 3593, 3510, 3426, 3342, 3259, 3175, 3092, 3008, 2925, 2841, 2757, 2674, 2590, 2507, 2423, 2340, 2256, 2172, 2089, 2005, 1922, 1838, 1755, 1671, 1587, 1504, 1420, 1337, 1253, 1170, 1086, 1002, 919, 835, 752, 668, 585, 501, 417, 334, 250, 167, 83, 0};
struct {
	uint32_t n;
	uint32_t d[100];
} waveform_saw = {100, 0, 41, 83, 125, 167, 208, 250, 292, 334, 376, 417, 459, 501, 543, 585, 626, 668, 710, 752, 793, 835, 877, 919, 961, 1002, 1044, 1086, 1128, 1170, 1211, 1253, 1295, 1337, 1378, 1420, 1462, 1504, 1546, 1587, 1629, 1671, 1713, 1755, 1796, 1838, 1880, 1922, 1963, 2005, 2047, 2089, 2131, 2172, 2214, 2256, 2298, 2340, 2381, 2423, 2465, 2507, 2548, 2590, 2632, 2674, 2716, 2757, 2799, 2841, 2883, 2925, 2966, 3008, 3050, 3092, 3133, 3175, 3217, 3259, 3301, 3342, 3384, 3426, 3468, 3510, 3551, 3593, 3635, 3677, 3718, 3760, 3802, 3844, 3886, 3927, 3969, 4011, 4053, 4095, 0};
static struct {
	uint32_t n;
	uint32_t d[1000];
} waveform;// waveform buffer
#endif
//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
// Definitions of PVs, they are initialized in the main()
PV pv_version = {"version", "MCU and software version, system clock", 	T_str, 1, F_R};
//PV pv_clock =   {"clock", "System clock", 	T_u4, 1, F_R, "Hz"};
PV pv_debug = {"debug", "Show debugging messages", 	T_b, 1, F_WE};
PV pv_fec = 	{"fec", 	"For experts. Start/Stop board peripherals except ADCs", T_str, 1, F_WED};
PV pv_sleep = 	{"sleep", 	"Sleep in the program loop", 		T_u2,   1, F_R, "ms"};
//PV pv_duart = 	{"duart",	"UART for sending data, 2:UART2 (ST-LINK),1:UART1, shared with TTY",
//		T_B,	1, F_WE};
PV pv_pulser_period = {"pulser_period", "Period of the pulser at pin PA11, also defines the period of the WFG",
		T_u2,	1, F_WE, "clk", 0, 65535};
PV pv_pulser_width = {"pulser_width", "The pulse width of the pulser.",
		T_u2,	1, F_WE, "clk", 0, 65535};
PV pv_pulser_prescaler = {"pulser_prescaler", "Clock prescaler of the pulser (pin PA11), also affects WFG",
		T_u2,	1, F_WE, "clk", 0, 65535};
PV pv_pulser_fire = {"pulser_fire", "Generate number of pulses, 0 for endless generation", T_u2, 1, F_WE};
PV pv_pulser_inverted = {"pulser_inverted", "Inverted polarity of pulser output", T_b, 1, F_WE};
static char gdg_trig_legalValues[] = "ExtP,ExtN,disable,auto";
PV pv_gdg_trig = {"gdg_trig","Trigger of the Gate-Delay-Generator, ExtP/ExtN-external positive/negative edge",
	T_str, 1, F_WED};
PV pv_gdg_front = {"gdg_front", "Gate-delay generator. Front time of the output pulse relative to the trigger",
	T_u4,	1, F_WE, "clk", 0, 2147483646};//because the liteMCUFEC cannot handle (int32_t)4294967295};
PV pv_gdg_tail = {"gdg_tail", "Gate-Delay Generator. Tail time of the output pulse relative to the trigger",
	T_u4,	1, F_WE, "clk", 0, 2147483646};//(int32_t);//4294967295};
PV pv_gdg_prescaler = {"gdg_prescaler", "Clock prescaler of the Gate_Delay Generator",
	T_u2,	1, F_WE, "clk", 0, 65535};
PV pv_gdg_inverted = {"gdg_inverted", "Inverted output of the Gate-Delay Generator (pin PA10)", T_b, 1, F_WE};
//PV pv_gdg_fire = {"gdg_fire", "Generate number of pulses  at pin PA10, 0 for endless generation", T_u2, 1, F_WE};
PV pv_nADC = 	{"nADC", 	"Number of ADC channels",T_B,   1, F_R};
PV pv_adc_reclen = 	{"adc_reclen", 	"Record length of ADCs", T_u2,  1, F_R};
PV pv_adc_srate = {"adc_srate", "Sampling rate of ADCs", T_u4,  1, F_R, "Hz"};
PV pv_adc_delay = {"adc_delay", "Delay of ADC sampling after trigger.",
	T_u2, 1, F_WE, "us", 1, 65535};
PV pv_adc_trig =  {"adc_trig", "ADC triggering mode, ADC1P: rising edge of ADC1, ExtN: falling edge of external TTL",
	T_str, 1, F_WED};
static char adc_trig_legalValues[] = "auto,oneshot,ADC1P,ExtP,ExtN,disable";
static int adc_prescaler[] =              {1,2,4,6,8,10,12,16,32,64,128,256};
static char adc_prescaler_legalValues[] =     "4,6,8,10,12,16,32,64,128,256";
static int adc_sampleTime[] =             {2,6,12,24,47,92,247,640}; //sample rate minus 0.5 clocks
static char adc_sampletime_legalValues[] =  "2,6,12,24,47,92,247,640";
PV pv_adc_prescaler =  {"adc_prescaler", "Prescaler of the ADC clock", T_str, 1, F_WED, "clk"};
PV pv_adc_sampletime =  {"adc_sampletime", "ADC sampling time in clocks", T_str, 1, F_WED, "clk"};
#ifdef WFG
	PV pv_wfg_shape = {"wfg_shape", "Shape of the generated waveform, period is defined by pulser_period", T_str, 1, F_WED};
	static char wfg_shape_legalValues[] = "off,flat,saw,triangle,sine,shortsine,noise";
	PV pv_wfg_level =  {"wfg_level", "Amplitude of the generated waveform, period is defined by pulser_period", T_u2, 1, F_WE};
	//PV pv_wfg_offs =  {"wfg_offset", "Offset of the generated waveform", T_u2, 1, F_WE};
	PV pv_wfg_step =  {"wfg_step", "Interval between points", T_u2, 1, F_WE, "us"};
	//PV pv_wfg_prescaler =  {"wfg_prescaler", "Clock prescaler of the waveform", T_u2, 1, F_WE};
	static char wfg_trig_legalValues[] = "off,auto,pulser,gdg,circular";
	PV pv_wfg_trig = {"wfg_trig", "Trigger of the waveform generator", T_str, 1, F_WED, };
#endif
PV pv_trig_level = {"trig_level", "Level of the trigger comparator COMP3, attached to ADC1",
	T_u2, 1, F_WE, "cnt", 1, 4095};
//PV pv_reset =  {"reset", "Reset board", T_u2, 1, F_WE};

#define NPV 25
PV *PVs[NPV] = {
  &pv_version,
  &pv_debug,
  &pv_fec,
  &pv_sleep,
  &pv_pulser_width,
  &pv_pulser_period,
  &pv_pulser_prescaler,
  &pv_pulser_fire,
  &pv_pulser_inverted,
  &pv_gdg_trig,
  &pv_gdg_front,
  &pv_gdg_tail,
  &pv_gdg_inverted,
  //&pv_gdg_fire,
  &pv_nADC,
  &pv_adc_reclen,
  &pv_adc_srate,
  &pv_adc_delay,
  &pv_adc_trig,
  &pv_adc_prescaler,
  &pv_adc_sampletime,
  &pv_wfg_shape,
  &pv_wfg_level,
  &pv_wfg_step,
  //&pv_wfg_prescaler,
  &pv_wfg_trig,
  &pv_trig_level,
  //&pv_reset,
};
//uint32_t adc_initial_trigConv = 0;
bool adc_errorCallback = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init();
static void MX_TIM2_Init();
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
//static void MX_SPI3_Init(void);
static void MX_COMP3_Init(void);
static void MX_DAC3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*uint32_t ns2clk(uint32_t ns){
	return (ns*(SYSCLK/mega))/1000;
}
uint32_t clk2ns(uint32_t nclk){
}*/

#define TTY_RECEIVE_INTERRUPT//
#ifdef TTY_RECEIVE_INTERRUPT
void tty_receive(void){
	__HAL_UART_CLEAR_FLAG(&huart_tty, UART_CLEAR_OREF);
	HAL_UART_Receive_IT(&huart_tty, buffer_rx, TTY_RXBUF_SIZE);
}
#else
// Blocking receiving might work only for single character transfer
int tty_receive(void){
	if (HAL_UART_Receive(&huart_tty, buffer_rx, TTY_RXBUF_SIZE, pv_sleep.value.i2)\
		== HAL_TIMEOUT)
		return 1;
	return 0;
}
#endif
static volatile uint32_t tick_endOfTransfer = 0;
void send_data(){
  if (command_received)
	  // Don't send binary data during command processing
	  return;
  uint8_t feod[] = {4,0,0,69}; //EndOfData, l=4, id='E'
  uint8_t lfeod = 4;
  uint32_t curtick = HAL_GetTick();
  uint32_t dt = curtick;
  if (dt > tick_endOfTransfer)
	  dt -= tick_endOfTransfer;
  else
	  dt = tick_endOfTransfer - dt;
  if (dt > pv_sleep.value.u2){
	  tick_endOfTransfer = curtick;
	  HAL_GPIO_WritePin(LED_BLUE, LED_ON);// not visible, overshadowed by the green
	  HAL_UART_Transmit(&huart_data, (uint8_t*)&bbuf, bbuf.h.l, HAL_MAX_DELAY);//2ms for 128 MAX_ADC_SAMPLES
	  HAL_UART_Transmit(&huart_data, feod, lfeod, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(LED_BLUE, LED_OFF);
  }
  /*status = HAL_LIN_SendBreak(huart); // Most of the clients do not handle breaks.
  if (status != HAL_OK){
	  HAL_GPIO_TogglePin(LED_LD2);
  }*/
}
//``````````````````Functions for TTY communications```````````````````````````
int index( const int a[], size_t size, int value )
{
    size_t index = 0;
    while ( index < size && a[index] != value ) ++index;
    return ( index == size ? -1 : index );
}
bool starts_with(const char *str1, const char *str2){
	return strncmp(str1, str2, strlen(str2)) == 0;
}
bool ends_with(const char *str1, const char *str2){
	int l2 = strlen(str2);
	int l1 = strlen(str1);
	return strncmp(&(str1[l1-l2]), str2, l2) == 0;
}
char* lower(char* dst, char* src){
	for (int i=0; src[i]; i++){
		dst[i] = tolower(src[i]);
	}
	return dst;
}
void send_tty(UART_HandleTypeDef *huart = &huart_tty){
  /*Send ttyMsg buffer to tty*/
  uint16_t l = strlen(ttyMsg.msg);
  if (l == 0){
	  return;}
  if (not pv_debug.value.b){
	  if (ttyMsg.msg[0] == '.')
		return;
  }
  ttyMsg.h.l = 4 + l;
  if (HAL_UART_Transmit(huart, (uint8_t*)&(ttyMsg), ttyMsg.h.l, HAL_MAX_DELAY) != HAL_OK){
	  HAL_GPIO_TogglePin(LED_LD2);
  }
}
void send_str_tty(const char* str)
{
  strcpy(ttyMsg.msg, str);
  send_tty();
}
uint32_t assign_bit(uint32_t src, uint32_t bitmask, int bitvalue){
	src = src & ~bitmask;
	if (bitvalue != 0) src = src | bitmask;
	return src;
}
PV* pvof(const char* pvname){
	PV* pv = NULL;
	if (pvname == NULL){
		sprintf(ttyMsg.msg, "ERR: PV is not provided");
		send_tty();
		return NULL;
	}
	for(int i=0; i< NPV; i++){
		//if (starts_with(PVs[i]->name, pvname)){
		if (strcmp(PVs[i]->name, pvname)==0){
			pv = PVs[i];}
	}
	if (pv == NULL){
		sprintf(ttyMsg.msg, "ERR: Wrong PV name '%s'", pvname);
		send_tty();
		return NULL;
	}
	return pv;
}
int reply_value(const char* pvname){
	char *buf = ttyMsg.msg;
	strcpy(buf,"");
	PV* pv = pvof(pvname);
	if (pv == NULL){
		return 1;
	}
	sprintf(buf, "{\"%s\":", pvname);
	int l = strlen(buf);
	pv->val2strn(buf+l, TTY_TXBUF_SIZE-l-1);
	strcat(buf, "}");
	send_tty();
	return 0;
}
int reply_info(const char* pvname){
	int len, cnt;
	char *buf = ttyMsg.msg;
	if (pvname == NULL){
		strcpy(buf, "{\"PVs\":[");
		for (int i=0; i< NPV; i++){
			len = strlen(buf);
			cnt = snprintf(buf+len, TTY_TXBUF_SIZE-len, "\"%s\",",PVs[i]->name);
			if (cnt >= TTY_TXBUF_SIZE-len){
				snprintf(buf, TTY_TXBUF_SIZE, "ERR: buf too small %i in pvnames2json\n", TTY_TXBUF_SIZE);
				send_tty();
				return 1;
			}
		}
		buf[strlen(buf)-1] = 0;//crop the last comma
		strcat(buf, "]}");
	}else{
		strcpy(buf,"");
		PV* pv = pvof(pvname);
		if (pv == NULL)
			return 2;
		pv->info(buf, TTY_TXBUF_SIZE);
	}
	send_tty();
	return 0;
}
int start_adc(){
	// Enable data taking from ADCs.
	//send_str_tty(".Starting ADC");
	// dual ADC conversions
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_ADC_Start(&hadc2); // Start slave ADC first!
	if (status != HAL_OK){
		if (status == HAL_BUSY)
			send_str_tty("WRN: ADC2 Busy");
	}
	status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)&(bbuf.payload), MaxSamples);
	if (status == HAL_OK){
		return 0;
	}
	if (status == HAL_BUSY){
		send_str_tty("WRN: ADC DMA Busy");
		return 0;
	}else{
		sprintf(ttyMsg.msg, "ERR: HAL_ADCEx status: %hu\r\n", status);
		send_tty();
	}
	return 2;
}
int stop_adc(){
	// Disable data taking from ADCs.
	send_str_tty(".stop_adc");
	send_str_tty(".ADC disabled");
	//DNU: The HAL_ADCEx_MultiModeStop_DMA causes ErrorCallbac in following init_adc
	//HAL_StatusTypeDef status = HAL_OK;
	//status = HAL_ADCEx_MultiModeStop_DMA(&hadc1);
	//if (status != HAL_OK){
	//	send_str_tty("WRN: Stop DMA");
	//	return 1;
	//}
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
	return 0;
}
int adc_config_channel(ADC_HandleTypeDef *hadc, int ch, int rank, uint32_t stime){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ch;
	sConfig.Rank = rank;
	sConfig.SamplingTime = stime;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
		return 1;
	return 0;
}
int adc_srate(ADC_HandleTypeDef *hadc){
	// return sampling rate of the ADCs
	int bits = ((hadc->Instance->SMPR1)>>(3*ADC1FirstChannel))&0x7;
	int stime = adc_sampleTime[bits]+13;// 13 clocks for 12-bit conversion
	int prescaler = adc_prescaler[((hadc->Init.ClockPrescaler)>>18)&0xf];
	return (uint32_t)(SYSCLK/prescaler/stime/NConversions);
}
void stop_tim_pulser(){
	send_str_tty(".stop_tim_pulser");
	HAL_TIM_PWM_Stop_IT(&htim_pulser, TIM_CHANNEL_1);
}
int start_tim_pulser(){
	stop_tim_pulser();
	//send_str_tty(".start_tim_pulser");
	TIM_PULSER->CCR1 = pv_pulser_width.value.u2;
	TIM_PULSER->ARR = pv_pulser_period.value.u2;
	tim_pulser_count = pv_pulser_fire.value.u2;
	if (HAL_TIM_PWM_Start_IT(&htim_pulser, TIM_CHANNEL_1) != HAL_OK)
		return 1;
	sprintf(ttyMsg.msg, ".<start_tim_pulser CCR1=%lu, ARR=%lu",TIM_PULSER->CCR1,TIM_PULSER->ARR);
	send_tty();
	return 0;
}
void stop_tim_adc(){
	send_str_tty(".stop_tim_adc");
#ifdef DBG_INT
	HAL_TIM_PWM_Stop_IT(&htim_adc, TIM_CHANNEL_2);
#else
	HAL_TIM_PWM_Stop(&htim_adc, TIM_CHANNEL_2);
#endif
}
int start_tim_adc(){
	stop_tim_adc();
	//send_str_tty(".start_tim_adc");
	HAL_StatusTypeDef status;
	TIM_ADC->CCR1 = pv_adc_delay.value.u2;
	TIM_ADC->ARR = TIM_ADC_DEADTIME;
#ifdef DBG_INT
	status = HAL_TIM_PWM_Start_IT(&htim_adc, TIM_CHANNEL_2);
#else
	status = HAL_TIM_PWM_Start(&htim_adc, TIM_CHANNEL_2);
#endif
	if (status != HAL_OK){
		send_str_tty("WRN: start_tim_adc not OK");
		return 1;
	}
	sprintf(ttyMsg.msg, ".<start_tim_adc CCR2=%lu, ARR=%lu",TIM_ADC->CCR2,TIM_ADC->ARR);
	send_tty();
	return 0;
}
void stop_tim_gdg(){
	send_str_tty(".stop_tim_gdg");
	HAL_TIM_PWM_Stop_IT(&htim_gdg, TIM_CHANNEL_4);
	//HAL_TIM_OnePulse_Stop(&htim_gdg, TIM_CHANNEL_4);
}
int start_tim_gdg(){
	stop_tim_gdg();
	send_str_tty(".start_tim_gdg");
	TIM_GDG->CCR1 = pv_gdg_front.value.u4;
	TIM_GDG->ARR = pv_gdg_tail.value.u4;
	//tim_gdg_count = pv_gdg_fire.value.u2;
	if (HAL_TIM_PWM_Start_IT(&htim_gdg, TIM_CHANNEL_4) != HAL_OK)
		return 1;
	sprintf(ttyMsg.msg, ".<start_tim_gdg CCR4=%lu, ARR=%lu",TIM_GDG->CCR4,TIM_GDG->ARR);
	send_tty();
	return 0;
}
#ifdef WFG
//bool wafeform_is_dynamic(const char* waveform_name){
//	return (strstr("saw,triangle,sine,noise", waveform_name) != NULL);
//}
void stop_wfg(){
	send_str_tty(">stop_wfg");
	DAC1->CR &= 0xFF3FFF3F;// clear WAVE bits
	HAL_DAC_Stop(&hdac1, DAC1_WFG);
	HAL_DAC_Stop_DMA(&hdac1, DAC1_WFG);
}
int start_dac_wfg(){
	send_str_tty(">start_dac_wfg");
	if (HAL_DAC_Start(&hdac1, DAC1_WFG) !=  HAL_OK)
		return 1;
	return 0;
}
int start_wfg(){
	// start WFG using buffered waveform
	//send_str_tty(">start_wfg");
	//TIM6->PSC = pv_wfg_prescaler.value.u2;
	HAL_DAC_StateTypeDef state;
	state = HAL_DAC_GetState(&hdac1);
	if (state == HAL_DAC_STATE_BUSY){
		send_str_tty("DAC1 BUSY");
		return 10;
	}
#ifdef DO_NOT_COMPLAIN_DAC// the following section is always complain when wfg_trig=auto
	if (state != HAL_DAC_STATE_READY){
		send_str_tty("DAC1 not ready");
		if (state == HAL_DAC_STATE_ERROR){
			send_str_tty("DAC1 STATE_ERROR");
		}else{
			send_str_tty("DAC1 wrong state");
		}
	}
#endif
	TIM6->ARR = pv_wfg_step.value.u2;
	if (starts_with(pv_wfg_shape.value.str,"n")){
		if (start_dac_wfg())
			return 2;
		if (HAL_DACEx_NoiseWaveGenerate(&hdac1, DAC1_WFG, DAC_LFSRUNMASK_BITS11_0) != HAL_OK)
			return 3;
	}else{
		if (HAL_DAC_Start_DMA(&hdac1, DAC1_WFG, (uint32_t*)waveform.d, waveform.n, DAC_ALIGN_12B_R) != HAL_OK)
			return 4;
	}
	return 0;
}
int set_wfg_shape(const char* val){
	stop_wfg();
	wfg_auto = false;
	if (starts_with(val,"off"))
		return 0;
	if (starts_with(val,"f")){
		pv_wfg_shape.set("flat");
		return start_dac_wfg();
	}
	// dynamic waveforms
	if (starts_with(pv_wfg_trig.value.str, "auto")){
		wfg_auto = true;
	}
	if (starts_with(val,"n")){
		pv_wfg_shape.set("noise");
		return start_wfg();
	}else if (starts_with(val,"shortsine")){
		pv_wfg_shape.set("shortsine");
		memcpy(&waveform, &waveform_shortsine, (waveform_shortsine.n+1)*sizeof(uint32_t));
	}else if (starts_with(val,"sine")){
		pv_wfg_shape.set("sine");
		memcpy(&waveform, &waveform_sine, (waveform_sine.n+1)*sizeof(uint32_t));
	}else if (starts_with(val,"t")){
		pv_wfg_shape.set("triangle");
		memcpy(&waveform, &waveform_triangle, (waveform_triangle.n+1)*sizeof(uint32_t));
	}else if (starts_with(val,"saw")){
		pv_wfg_shape.set("saw");
		memcpy(&waveform, &waveform_saw, (waveform_saw.n+1)*sizeof(uint32_t));
	}else{
		wfg_auto = false;
		return 1;
	}
	return start_wfg();
}
#endif
int start_peripherals(){
	// Start peripherals except ADCs
	send_str_tty(".Starting peripherals");
	//if (start_tim_gdg())
	//	return 4;
	//if (start_tim_adc())
	//	return 3;
	// DACs
#ifdef WFG
	if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
		return 1;
	//if (HAL_DAC_Start(&hdac1, DAC1_WFG) !=  HAL_OK)
	//	return 1;
	//send_str_tty("dac1 started");
#endif
	//if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK)
	//	return 1;
	/* Triangle mode makes sense if one wants to add a triangle on top of existing waveform
	if (HAL_DACEx_TriangleWaveGenerate(&hdac1, DAC1_WFG, DAC_TRIANGLEAMPLITUDE_2047) != HAL_OK){
		  Error_Handler();
	}
	*/
	HAL_DAC_Start(&hdac3, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2000u);

	// COMP
	if (HAL_COMP_Start(&hcomp3) != HAL_OK)
		return 2;
	return 0;
}
void stop_peripherals(){
	send_str_tty(".Stopping peripherals");
	stop_tim_gdg();
	stop_tim_pulser();
	stop_tim_adc();
#ifdef WFG
	HAL_TIM_Base_Stop(&htim6);
	stop_wfg();
	send_str_tty("dac1 stopped");
#endif
	HAL_COMP_Stop(&hcomp3);
}

int set_adc_trig(char* val)
{
  if (starts_with(val, "dis")){
	pv_adc_trig.set("disabled");
	stop_adc();
  }else if (starts_with(val, "auto")) {
	pv_adc_trig.set("auto");
	stop_adc();
  	MX_ADC1_Init();
  }else if (starts_with(val, "one")){
	pv_adc_trig.set("oneshot");
	stop_adc();
	MX_ADC1_Init();
	if (start_adc()){
		HAL_GPIO_WritePin(LED_RED, LED_ON);
		return 1;
	}
  }else if (starts_with(val, "Ext") or starts_with(val, "ADC")){
	static char valcopy[6];
	strncpy(valcopy,val,6);
	pv_adc_trig.set(valcopy);
	MX_TIM1_Init();
	if (start_tim_adc()){
		HAL_GPIO_WritePin(LED_RED, LED_ON);
	    return 2;
	}
	stop_adc();
	MX_ADC1_Init();
	if (start_adc()){
		HAL_GPIO_WritePin(LED_RED, LED_ON);
		return 1;
	}
  }else{
	  return 3;
  }
  return 0;
}

int send_tty_value_not_legal(char* val){
	sprintf(ttyMsg.msg, "ERR: The value `%s` is not legal",val);
	send_tty();
	return 1;
}
int send_tty_errmsg(int err, const char* msg){
	if (strlen(msg)>0){
		sprintf(ttyMsg.msg, msg);
		send_tty();
	}
	return err;
}
int handle_command(char* str)
// handle command from uart_buf
{
	char *cmd, *arg, *val;
	char *saveptr;
	char delimiters[] = " ,.\r";
	//char* str = (char*)usart_buf;
	int notOK = 0;
	uint32_t tmpu32 = 0;
	PV* pv;
  //send_str_tty(str);//echo
  //printf ("Splitting string \"%s\" into tokens:\n",str);
  cmd = lower(str,strtok_r(str, delimiters, &saveptr));
  arg = strtok_r(NULL, delimiters, &saveptr);
  if (cmd[0] == 3){//CtrlC, it is equivalent to 'set adc disable'
	  stop_adc();
	  pv_adc_trig.set("disabled");
	  return 0;
  }else if (starts_with(cmd, "info")) {
	if (reply_info(arg))
		return 1;
  }else if (starts_with(cmd, "get")) {
	if (reply_value(arg))
		return send_tty_errmsg(1, "ERR: in info");
  }else if (starts_with(cmd, "set")) {
	val = strtok_r(NULL, delimiters, &saveptr);
	if (val == NULL){
		return send_tty_errmsg(1, "ERR: Value missing");
	}
	if (starts_with(arg, "fec")){
		if (starts_with(val, "sta")){
			if (start_peripherals() == 0){
				pv_fec.set("Started");
				return 0;
			}else
				return send_tty_errmsg(1, "ERR: starting peripherals");
		}else if (starts_with(val, "sto")){
			stop_peripherals();
			return 0;
		}else{
			return(send_tty_value_not_legal(val));
		}
	//`````````````Pulser`````````````````````````````````````````````````````
  	}else if (starts_with(arg, "pulser_width")) {
		if (pv_pulser_width.set(val))
			return send_tty_value_not_legal(val);
		TIM_PULSER->CCR1 = pv_pulser_width.value.u4;
		//if (((TIM_PULSER->CR1) & 1) == 0)
	}else if (starts_with(arg, "pulser_period")) {
		if ((uint32_t)(atol(val)) < (TIM_PULSER->CCR1)){
			sprintf(ttyMsg.msg, "ERR: tail %s cannot be sooner than front %lu\n", val, TIM_PULSER->CCR1);
			send_tty();
			return 2;
		}
		if (pv_pulser_period.set(val))
			return send_tty_value_not_legal(val);
		TIM_PULSER->ARR = pv_pulser_period.value.u4;
	}else if (starts_with(arg, "pulser_prescaler")) {
		if (pv_pulser_prescaler.set(val))
			return send_tty_value_not_legal(val);
		TIM_PULSER->PSC = pv_pulser_prescaler.value.u2;
	}else if (starts_with(arg, "pulser_inverted")) {//TODO
		if (pv_pulser_inverted.set(val))
			return send_tty_value_not_legal(val);
		MX_TIM4_Init();
		if (start_tim_pulser())
			return send_tty_errmsg(1, "ERR: in start_tim_pulser");
	}else if (starts_with(arg, "pulser_fire")) {
		if (pv_pulser_width.value.u4 == 0)
			return send_tty_errmsg(1, "ERR: pulser front time is not set");
		if (pv_pulser_period.value.u4 == 0)
			return send_tty_errmsg(1, "ERR: pulser tail time is not set");
		if (pv_pulser_fire.set(atoi(val)))
			return send_tty_value_not_legal(val);
		if (start_tim_pulser())
			return send_tty_errmsg(1, "ERR: in start_tim_pulser");
	//`````````````GDG````````````````````````````````````````````````````````
	}else if (starts_with(arg, "gdg_trig")){
		if (starts_with(val, "dis")){
			pv_gdg_trig.set("disabled");
			stop_tim_gdg();
		}else if (starts_with(val, "Ext")){
			static char valcopy[6];
			strncpy(valcopy,val,6);
			pv_gdg_trig.set(valcopy);
			MX_TIM2_Init();
			if (start_tim_gdg())
				return send_tty_errmsg(1, "ERR: in start_tim_gdg");
		}else if (starts_with(val, "auto")){
			pv_gdg_trig.set("auto");
			HAL_TIM_OnePulse_DeInit(&htim_gdg);
			MX_TIM2_Init();
			if (start_tim_gdg())
				return send_tty_errmsg(1, "ERR: in set gdg_trig auto");
		}else{
			return send_tty_value_not_legal(val);
		}
	}else if (starts_with(arg, "gdg_pre")){
		pv_gdg_prescaler.set(val);
		TIM_GDG->PSC = pv_gdg_prescaler.value.u2;
	}else if (starts_with(arg, "gdg_front")) {
		if (pv_gdg_front.set(val))
			return send_tty_value_not_legal(val);
		TIM_GDG->CCR4 = pv_gdg_front.value.u4;
	}else if (starts_with(arg, "gdg_tail")) {
		if (pv_gdg_tail.set(val))
			return send_tty_value_not_legal(val);
		TIM_GDG->ARR = pv_gdg_tail.value.u4;
	}else if (starts_with(arg, "gdg_inverted")) {
		if (pv_gdg_inverted.set(val))
			return send_tty_value_not_legal(val);
		//stop_tim_gdg();
		MX_TIM2_Init();
		if (start_tim_gdg())
			return send_tty_errmsg(1, "ERR: in start_tim_gdg");
	//}else if (starts_with(arg, "gdg_fire")) {
	//	if (pv_gdg_fire.set(atoi(val)))
	//		return send_tty_value_not_legal(val);
	//	if (start_tim_gdg())
	//		return send_tty_errmsg(1, "ERR: in start_tim_gdg");
	//`````````````ADC````````````````````````````````````````````````````````
	}else if (starts_with(arg, "adc_delay")) {
		if (pv_adc_delay.set(val))
			return send_tty_value_not_legal(val);
		// The start_tim_adc() has no effect, set_adc_trig() works.
		//if (start_tim_adc())
		//	return send_tty_errmsg(1, "ERR: in set_adc_delay");
		if (set_adc_trig(pv_adc_trig.value.str))
			return send_tty_errmsg(1, "ERR: in set_adc_trig");
	}else if (starts_with(arg, "adc_trig")){
		if (set_adc_trig(val))
			return send_tty_errmsg(1, "ERR: in set_adc_trig");
	}else if (starts_with(arg, "adc_prescaler")){
		notOK = atoi(val);
		int idx = index(adc_prescaler, 12, notOK);
		if (idx < 2){
			return send_tty_value_not_legal(val);
		}
		stop_adc();
		tmpu32 = idx<<18;
		hadc1.Init.ClockPrescaler = tmpu32;
		hadc2.Init.ClockPrescaler = tmpu32;
		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);
		static char prescaler_str[] = "      ";
		sprintf(prescaler_str,"%i",notOK);
		pv_adc_prescaler.set(prescaler_str);
		pv_adc_srate.set(adc_srate(&hadc1));
		reply_value("adc_srate");
		//}
		if (start_adc())
			return send_tty_errmsg(1, "ERR: in start_adc");
	}else if (starts_with(arg, "adc_sampletime")){
		notOK = atoi(val);
		if (notOK == 2)
			tmpu32 = ADC_SAMPLETIME_2CYCLES_5;
		else if (notOK == 6)
			tmpu32 = ADC_SAMPLETIME_6CYCLES_5;
		else if (notOK == 12)
			tmpu32 = ADC_SAMPLETIME_12CYCLES_5;
		else if (notOK == 24)
			tmpu32 = ADC_SAMPLETIME_24CYCLES_5;
		else if (notOK == 47)
			tmpu32 = ADC_SAMPLETIME_47CYCLES_5;
		else if (notOK == 92)
			tmpu32 = ADC_SAMPLETIME_92CYCLES_5;
		else if (notOK == 247)
			tmpu32 = ADC_SAMPLETIME_247CYCLES_5;
		else if (notOK == 640)
			tmpu32 = ADC_SAMPLETIME_640CYCLES_5;
		else {
			send_tty_value_not_legal(val);
			return 16;
		}
		//if (hadc1.Init.ExternalTrigConv == ADC_SOFTWARE_START)
		stop_adc();
		adc_config_channel(&hadc1, ADC_CHANNEL_1, ADC_REGULAR_RANK_1, tmpu32);
		adc_config_channel(&hadc1, ADC_CHANNEL_10, ADC_REGULAR_RANK_2, tmpu32);
		adc_config_channel(&hadc1, ADC_CHANNEL_15, ADC_REGULAR_RANK_3, tmpu32);
		adc_config_channel(&hadc1, ADC_CHANNEL_2, ADC_REGULAR_RANK_4, tmpu32);
		adc_config_channel(&hadc2, ADC_CHANNEL_3, ADC_REGULAR_RANK_1, tmpu32);
		adc_config_channel(&hadc2, ADC_CHANNEL_4, ADC_REGULAR_RANK_2, tmpu32);
		adc_config_channel(&hadc2, ADC_CHANNEL_17, ADC_REGULAR_RANK_3, tmpu32);
		adc_config_channel(&hadc2, ADC_CHANNEL_10, ADC_REGULAR_RANK_4, tmpu32);
		//pv_adc_sampletime.set(notOK);
		static char sampletime_str[] = "      ";
		sprintf(sampletime_str,"%i",notOK);
		pv_adc_sampletime.set(sampletime_str);
		pv_adc_srate.set(adc_srate(&hadc1));
		reply_value("adc_srate");
		//}
		if (start_adc() != 0)
			return send_tty_errmsg(1, "ERR: in start_adc");
		if (start_tim_adc())
			return send_tty_errmsg(1, "ERR: in start_tim_adc");
#ifdef WFG
	}else if (starts_with(arg, "wfg_level")) {
		if (pv_wfg_level.set(val))
			return send_tty_value_not_legal(val);
		HAL_DAC_SetValue(&hdac1, DAC1_WFG, DAC_ALIGN_12B_R, pv_wfg_level.value.u2);
	}else if (starts_with(arg, "wfg_shape")) {
		if (set_wfg_shape(val) != 0)
			return send_tty_value_not_legal(val);
	}else if (starts_with(arg, "wfg_trig")) {
		if (starts_with(val,"auto")){
			pv_wfg_trig.set("auto");
			stop_wfg();
		}else if(starts_with(val,"off")){
			pv_wfg_trig.set("off");
			stop_wfg();
		}else if(starts_with(val,"p")){
			pv_wfg_trig.set("pulser");
			wfg_auto = false;
			stop_wfg();
		}else
			return send_tty_value_not_legal(val);
#endif
	}else if (starts_with(arg, "trig_level")){
		if (pv_trig_level.set(val))
			return send_tty_value_not_legal(val);
		HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pv_trig_level.value.u2);
	}else{
		// Set values for other ordinary parameters
		pv = pvof(arg);
		if (pv == NULL)
			return 1;
		notOK = pv->set(val);
		if (notOK){
			sprintf(ttyMsg.msg, "ERR: setting %s to %s", pv->name, val);
			send_tty();
			return 1;
		}
	}
	// `Set` was successful, reply with OK.
	sprintf(ttyMsg.msg, "OK: `%s`", buffer_rx);
	send_str_tty("OK");
  }else if (strlen(str) == 0){
	  send_str_tty("INF: Legal commands: info, get, set.");
  }else{
	  sprintf(ttyMsg.msg, "ERR: Illegal command: '%s'. Legal commands: info, get, set.",str);
	  send_tty();
	  return 1;
  }
  return 0;
}
int handle_commands(){
	//HAL_GPIO_WritePin(LED_RED, LED_ON);
	char cmd[80];
	char *saveptr;
	HAL_GPIO_WritePin(LED_RED, LED_OFF);
	// Short pulse the Blue to indicate new command
	HAL_GPIO_WritePin(LED_BLUE, LED_OFF);
	HAL_GPIO_WritePin(LED_BLUE, LED_ON);
	HAL_Delay(1);//To indicate on the scopeTODO: remove after debugging
	HAL_GPIO_WritePin(LED_BLUE, LED_OFF);
	if (buffer_rx[0] == '\r'){
		send_str_tty("WRN: first char is carriage return");
		buffer_rx[0] = ' ';
	}
	strchr((char*)buffer_rx,'\r')[0] = 0;// replace first \r by 0
	strcpy((char*)usart_buf, (char*)buffer_rx);
	int r = 0;

	char* pch = strtok_r((char*)usart_buf, ";\r", &saveptr);
	while (pch != NULL){
		strcpy(cmd, pch);
		r = handle_command(cmd);
		if (r != 0){
			//sprintf(ttyMsg.msg, "ERR executing `%s': %i", cmd, r);
			//send_tty();
			break;
		}
		pch = strtok_r (NULL, ";\r", &saveptr);
	}
	return r;
	//HAL_GPIO_WritePin(LED_RED, LED_OFF);
}
//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//HAL_StatusTypeDef status = HAL_OK;
	//uint32_t tmpu32;
	//HAL_UART_StateTypeDef ttyState;

  /* USER CODE END 1 */

  //```````````````MCU Configuration------------------------------------------

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //pv_clock.set(SYSCLK);
  pv_fec.legalValues = (char*)"start,stop";
  pv_sleep.opLow = 5;
  pv_sleep.opHigh = 32767;
  pv_sleep.set(100);
  pv_version.set(MCU_VERSION);
  //pv_duart.opLow = 1;
  //pv_duart.opHigh = 2;
  //pv_duart.set(2);
  //```````````````Pulser
  pv_pulser_prescaler.set(1700-1);
  //pv_pulser_width.set(TIM_PULSER->CCR1);
  //pv_pulser_period.set(TIM_PULSER->ARR);
  pv_pulser_width.set(500);
  pv_pulser_period.set(14000);
  pv_pulser_fire.set(1);
  //```````````````GDG
  pv_gdg_trig.legalValues = gdg_trig_legalValues;
  pv_gdg_trig.set("disabled");
  pv_gdg_prescaler.set(0);
  pv_gdg_front.set(10000);
  pv_gdg_tail.set(170000);
  //pv_gdg_fire.set(0);
  //```````````````ADC````````````````````````````````````````````````````````
  pv_adc_trig.set("ExtP");
  pv_adc_delay.set(1);
  pv_adc_prescaler.legalValues = adc_prescaler_legalValues;
  pv_adc_prescaler.set("8");
  pv_adc_sampletime.legalValues = adc_sampletime_legalValues;
  pv_adc_sampletime.set("640");//as in MX_ADC1_INIT
  pv_nADC.set(NConversions*Dual);
  pv_adc_reclen.set(MAX_ADC_SAMPLES);
  pv_adc_trig.legalValues = adc_trig_legalValues;
  //pv_adc_trig.set("ADC1P");
  pv_adc_trig.set("disabled");
#ifdef WFG  //`````WFG````````````````````````````````````````````````````````
  pv_wfg_shape.legalValues = wfg_shape_legalValues;
  pv_wfg_shape.set("off");
  pv_wfg_trig.legalValues = wfg_trig_legalValues;
  pv_wfg_trig.set("off");
  //pv_wfg_period.set(TIM6->ARR);
  pv_wfg_step.set(100);
  //pv_wfg_prescaler.set(TIM6->PSC);
#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_USART2_UART_Init();
  // Print message at 9600 Baud
  send_str_tty(MCU_VERSION);

  // change to working Baudrate
  huart_tty.Init.BaudRate = TTY_BAUDRATE;
  if (HAL_UART_Init(&huart_tty) != HAL_OK)
  {
    Error_Handler();
  }
  pv_debug.set(1);
  send_str_tty(MCU_VERSION);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();// Done interactively
  MX_TIM4_Init();// Done interactively
  //MX_SPI3_Init();
  MX_COMP3_Init();
  MX_DAC3_Init();
  /* USER CODE BEGIN 2 */
  pv_adc_srate.set(adc_srate(&hadc1));
  HAL_GPIO_WritePin(LED_GREEN, LED_OFF);
  //HAL_GPIO_WritePin(LED_GREEN, LED_ON);
  HAL_GPIO_WritePin(LED_BLUE, LED_OFF);
  //HAL_GPIO_WritePin(LED_BLUE, LED_ON);
  HAL_GPIO_WritePin(LED_RED, LED_OFF);
  //`````Initialization of PVs````````````````````````````````````````````````
  //tmpu32 = ns2clk(1000);
  //}

  // Start reception of input characters
  //HAL_UART_EnableReceiverTimeout(&huart_tty);
  tty_receive();

  //`````Calibration``````````````````````````````````````````````````````````
  if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED))
      Error_Handler();
  if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED))
      Error_Handler();
  snprintf(ttyMsg.msg, TTY_TXBUF_SIZE, "MCUFEC %s, use 'info' to see supported PVs", VERSION);
  send_tty();
  uint32_t prevtick = 0;
  uint32_t curtick = 0;

  if (start_peripherals() == 0)
	  pv_fec.set("Started");
  else
	  pv_fec.set("Stopped");

  //adc_initial_trigConv = hadc1.Init.ExternalTrigConv;
  send_str_tty(".Debugging of MCU is off. To enable it: `set debug 1`");
  pv_debug.set(0);
  //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Do not use HAL_Delay in the main loop, it may lock if an ISR is calling it
	//HAL_Delay(pv_period.value.u2);
	while (1){
		curtick = HAL_GetTick();
		//if ((curtick%1000) == 0)
		//	HAL_GPIO_TogglePin(LED_LD2);
		if (command_received)
			break;
		if (curtick - prevtick > pv_sleep.value.u4){
			HAL_GPIO_TogglePin(LED_LD2);
			break;
		}
	}
	prevtick = curtick;

	if (adc_errorCallback){
		MX_ADC1_Init();
		MX_ADC2_Init();
		start_adc();
		adc_errorCallback = false;
	}
	/*
	if ((not RX_overrun) and ADC_suspended){
		//ADC_suspended = false;
		start_ADC();
		send_str_tty(".ADC_resumed");
		continue;
	}*/
#ifdef TTY_RECEIVE_INTERRUPT
	if (command_received){
		//ttyTX_suspended = true;
		int r;
		r = handle_commands();
		command_received = false;
	}
	#ifdef USELESS
		ttyState = HAL_UART_GetState(&huart_tty);
		if (ttyState != HAL_UART_STATE_BUSY_RX){
			send_str_tty("TTY BUSY_RX");
			tty_receive();
		}
	#endif
#else
 	if (tty_receive())
 		continue;
	command_received = true;
 	handle_commands();
 	command_received = false;
	sprintf(ttyMsg.msg, "Commands processed: %s", buffer_rx);
	send_tty();
#endif

	// if adc_trig==auto then restart ADC
	if (starts_with(pv_adc_trig.value.str, "auto")) {
		if (start_adc() != 0){
			send_str_tty("ERR: in start_adc");
			return 1;
		}
	}
	if (wfg_auto){
		int notOK = start_wfg();
		if (notOK){
			send_str_tty("ERR: in start_wfg()");
		}
	}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  send_str_tty(".ADC1_Init");
  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  if (starts_with(pv_adc_trig.value.str, "Ext") or starts_with(pv_adc_trig.value.str, "ADC"))
  	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  else
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = DISABLE;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_NONE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  //sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */
  send_str_tty(".ADC2_Init");
  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */
  send_str_tty(".COMP3_Init");
  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
	  hcomp3.Instance = COMP3;
	  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
	  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_70MV;
	  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_EVENT_RISING;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
  send_str_tty(".DAC1_Init");
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  //sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC3_Init(void)
{

  /* USER CODE BEGIN DAC3_Init 0 */
  send_str_tty(".DAC3_Init");
  /* USER CODE END DAC3_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC3_Init 1 */

  /* USER CODE END DAC3_Init 1 */

  /** DAC Initialization
  */
  hdac3.Instance = DAC3;
  if (HAL_DAC_Init(&hdac3) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC3_Init 2 */

  /* USER CODE END DAC3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init()
{

  /* USER CODE BEGIN TIM1_Init 0 */
  send_str_tty(".TIM1_Init");
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 170-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  // Limit the update frequency of the tim_adc by fixing its period.
  htim1.Init.Period = TIM_ADC_DEADTIME;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  if (starts_with(pv_adc_trig.value.str,"ADC1")){// ADC is level triggered at CH1
	sSlaveConfig.InputTrigger = TIM_TS_ETRF;
	if (ends_with(pv_adc_trig.value.str,"P"))
		sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
	else
		sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
  }else{
	sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	if (starts_with(pv_adc_trig.value.str, "ExtN"))
		sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
	else
		sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  }
  //?sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (starts_with(pv_adc_trig.value.str,"ADC1")){
	  HAL_TIMEx_RemapConfig(&htim1, TIM_TIM1_ETR_COMP3);// effective for trig='COMP3'
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pv_adc_delay.value.u2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init()
{
  /* USER CODE BEGIN TIM2_Init 0 */
  send_str_tty(".TIM2_Init");
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  //htim2.Init.Prescaler = 0;
  htim4.Init.Prescaler = pv_gdg_prescaler.value.u2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim2.Init.Period = 170000;
  htim2.Init.Period = pv_gdg_tail.value.u4;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (starts_with(pv_gdg_trig.value.str, "Ext")){
	  if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
	  {
		Error_Handler();
	  }
	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
	  sSlaveConfig.TriggerFilter = 0;
  }
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
	  {
		Error_Handler();
	  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  //sConfigOC.Pulse = 17000;
  sConfigOC.Pulse = pv_gdg_front.value.u4;
  if (pv_gdg_inverted.value.B){
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  }else{
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  }
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}
/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init()
{
  /* USER CODE BEGIN TIM4_Init 0 */
  send_str_tty(".TIM4_Init");
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = pv_pulser_prescaler.value.u2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = pv_pulser_period.value.u2;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;//TIM_MASTERSLAVEMODE_ENABLE?
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pv_pulser_width.value.u2;
  if (pv_pulser_inverted.value.B){
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  }else{
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  }
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
  send_str_tty(".TIM6_Init");
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 170-1;
  //htim6.Init.Prescaler = pv_wfg_prescaler.value.u2;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  //htim6.Init.Period = 10;
  htim6.Init.Period = pv_wfg_step.value.u2;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  //DontDoThis!//if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin|LD2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : LED_GREEN_Pin */
	  GPIO_InitStruct.Pin = LED_GREEN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin LD2_Pin */
	  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin|LD2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//`````````````````Callbacks``````````````````````````````````````````````````
#ifdef RECEIVING_DMA
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_WritePin(LED_GREEN, LED_ON);
}
#endif
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_StatusTypeDef status = HAL_OK;
	if (huart != &huart_tty)
		return;
    HAL_GPIO_TogglePin(LED_LD2);
    if (command_received){
    	send_str_tty("WRN: Busy with command processing");
    }
#if TTY_RXBUF_SIZE == 1
	//HAL_UART_Transmit(&huart, (uint8_t*)&(buffer_rx), 1, HAL_MAX_DELAY);
    if (buffer_rx[0] == TTY_CR) {
        usart_buf[usart_buf_length] = 0;
        //usart_send_newline();
        command_received = true;
        //handle_command();
        usart_buf_length = 0;
        return;// and do not enable reception
    } else if (buffer_rx[0] == TTY_LF) {
        // ignore
    } else {
        if (usart_buf_length == COMMAND_MAX_LENGTH) {
            //usart_send_newline();
        	send_str_tty("command too long");
            usart_buf_length = 0;
            return;
        }
        usart_buf[usart_buf_length++] = buffer_rx[0];
        // echo
        //usart_send(buffer_rx[0]);
    }
#else
    command_received = true;
#endif
    if (RX_overrun){
    	send_str_tty(".RX_overrun recovered.");
    }
    RX_overrun = false;
    // enable reception of next input buffer
	tty_receive();
	HAL_GPIO_WritePin(LED_RED, LED_OFF);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(LED_RED, LED_ON);
	uint32_t ttyError = HAL_UART_GetError(&huart_tty);
	if (ttyError != HAL_OK){
	}
	if (ttyError == HAL_UART_ERROR_ORE){
		RX_overrun = true;
		send_str_tty("ERR: RX_overrun.");
	}else if (ttyError == HAL_UART_ERROR_RTO){
		send_str_tty("ERR: UART Receive Timeout.");
	}
	tty_receive();
}
void HAL_UART_RxFifoFullCallback(UART_HandleTypeDef *huart)
{//Never gets here
	if (huart == &huart_tty)
		;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim_adc){
		// not getting here
		;//HAL_GPIO_TogglePin(LED_BLUE);
	}else if (htim == &htim_pulser){
		send_str_tty("Pulser PeriodElapsed");
	}else if (htim == &htim_gdg){
		;//send_str_tty("GDG PeriodElapsed");
	}
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim_adc){
		// fired
		;//HAL_GPIO_TogglePin(LED_BLUE);
	}else if (htim == &htim_pulser){
		if (starts_with(pv_wfg_trig.value.str, "pulser")){
			//send_str_tty("about to start wfg");
			start_wfg();//!!! Interrupt level of TIM_PULSER should be higher, than for DMA, otherwise it will hang.
		}
	}else if (htim == &htim_gdg){
		;//send_str_tty("GDG DElayElapsed");
	}
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim_adc){
		//fired
		HAL_GPIO_TogglePin(LED_BLUE);
	}else if (htim == &htim_pulser){
		if (tim_pulser_count > 0){
			tim_pulser_count--;
			if (tim_pulser_count == 0){
				HAL_TIM_PWM_Stop_IT(&htim_pulser, TIM_CHANNEL_1);
				//HAL_GPIO_WritePin(LED_BLUE, LED_ON);//
			}
		}
	}else if (htim == &htim_gdg){
		//send_str_tty("GDG PWM_PulseFinished");
		if (tim_gdg_count > 0){
			tim_gdg_count--;
			if (tim_gdg_count == 0){
				//HAL_TIM_PWM_Stop_IT(&htim_gdg, TIM_CHANNEL_4);
				stop_tim_gdg();
				//HAL_GPIO_WritePin(LED_BLUE, LED_ON);//
				send_str_tty(".GDG exhausted");//DNW
			}
		}
	}
}
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim){
	// not getting here
	send_str_tty("TIM_Trigger");
	if (htim == &htim_adc){
		;//HAL_GPIO_TogglePin(LED_BLUE);
	}else if (htim == &htim_gdg){
		send_str_tty("GDG Trigger");
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim_pulser)
		;
}
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin){
    if (gpio_pin == GPIO_PIN_4){
    	// no idea why we are getting here
    	;//HAL_GPIO_TogglePin(LED_BLUE);
    	sprintf(ttyMsg.msg, "INF: EXTI callback for pin %i",gpio_pin);
    	send_tty();
    }
}

//``````````````````Functions for ADC readout``````````````````````````````````
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  HAL_GPIO_WritePin(LED_GREEN, LED_ON);
  send_data();
  if (starts_with(pv_adc_trig.value.str, "Ext")
		  or starts_with(pv_adc_trig.value.str, "ADC")){
	  start_adc();
  }
  HAL_GPIO_WritePin(LED_GREEN, LED_OFF);
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  //HAL_GPIO_WritePin(LED_GREEN, LED_OFF);
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  HAL_GPIO_WritePin(LED_RED, LED_ON);
  send_str_tty("ERR: ADC ErrorCallback");
  adc_errorCallback = true;
}
//``````````````````DAC
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  //HAL_GPIO_WritePin(WFG_START, LED_OFF); //because TIM_ADC starts on falling edge.
  send_str_tty("DAC_ConvCplt");
}
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  //HAL_GPIO_WritePin(WFG_START, LED_ON);
	send_str_tty("DAC_ConvHalfCplt");
}
void HAL_DAC_DMA_UnderrunCallbackCH1(DAC_HandleTypeDef* hdac)
{
  HAL_GPIO_WritePin(LED_RED, LED_ON);
  send_str_tty("ERR: DAC_DMA CH1 underrun");
}
void HAL_DAC_DMA_ErrorCallbackCH1(DAC_HandleTypeDef* hdac)
{
  HAL_GPIO_WritePin(LED_RED, LED_ON);
  send_str_tty("ERR: DAC_DMA CH1 error");
}
void HAL_DAC_DMA_UnderrunCallbackCH2(DAC_HandleTypeDef* hdac)
{
  HAL_GPIO_WritePin(LED_RED, LED_ON);
  send_str_tty("ERR: DAC_DMA CH2 underrun");
}
void HAL_DAC_DMA_ErrorCallbackCH2(DAC_HandleTypeDef* hdac)
{
  HAL_GPIO_WritePin(LED_RED, LED_ON);
  send_str_tty("ERR: DAC_DMA CH2 error");
}
#ifdef COMP3_INTERRUPT// Could be too intense.
void HAL_COMP_TriggerCallback (COMP_HandleTypeDef * hcomp)
{
  int sleepcount = 1300;// sleeps for ~100 us
  //send_str_tty("INF: Comp trigger");
  if (starts_with(pv_adc_trig.value.str,"ADC1")){
	  HAL_GPIO_WritePin(LED_BLUE,LED_ON);
	  while(sleepcount--)
		  ;
	  HAL_GPIO_WritePin(LED_BLUE,LED_OFF);
  }
}
#endif

//,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  send_str_tty("ERR: Handler. Board suspended! ##############################");
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(LED_RED, LED_ON);
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
