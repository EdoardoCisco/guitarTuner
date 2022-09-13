#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define TEST_LENGTH_SAMPLES 512
#define SAMPLE_LENGTH 512
#define E 82
#define A 110
#define D 147
#define G 196
#define B 247
#define e 330
#define ON 5000U
#define OFF 0U
#define BPM_COUNTER(bpm)  ((32769*60)/bpm)

/* ------------------------------------------------------------------
 * Global variables for FFT Bin Example
 * ------------------------------------------------------------------- */
uint32_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;
//uint32_t maxIndex = 0;

#define SMCLK_FREQUENCY     24000000 
#define SAMPLE_FREQUENCY    700 

/* Graphic library context */
Graphics_Context g_sContext;

/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif
static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[32];

/* FFT data/processing buffers*/
float hann[SAMPLE_LENGTH];
int16_t data_array1[SAMPLE_LENGTH];
int16_t data_array2[SAMPLE_LENGTH];
int16_t data_input[SAMPLE_LENGTH * 2];
int16_t data_output[SAMPLE_LENGTH];

volatile int switch_data = 0;
/*---------------------------------------------------
----------------------------------------------------*/

/* flag moving*/
uint8_t selectedOptionFlag=0;
uint8_t OldFlag=0;

/*screen*/
Graphics_Context g_sContext;

/* ADC results buffer I/O joystick*/
static uint16_t resultBuffer[2];

/*metronome control*/
static uint_fast16_t printedBPM=100;
uint_fast16_t BPM_val = BPM_COUNTER(100);

/* Timer_A PWM Configuration Parameter */
const Timer_A_PWMConfig pwmConfig =
{
    TIMER_A_CLOCKSOURCE_SMCLK,
    TIMER_A_CLOCKSOURCE_DIVIDER_1,
    (SMCLK_FREQUENCY / SAMPLE_FREQUENCY),
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_OUTPUTMODE_SET_RESET,
    (SMCLK_FREQUENCY / SAMPLE_FREQUENCY) / 2
};

/* Timer for metronome controller each interrupt take a buz*/
Timer_A_UpModeConfig upMetronome =
{
    TIMER_A_CLOCKSOURCE_ACLK,//set
    TIMER_A_CLOCKSOURCE_DIVIDER_1,//set
    0,
    TIMER_A_TAIE_INTERRUPT_ENABLE,
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
    TIMER_A_DO_CLEAR
};

/*Timer setup for buzzer*/
Timer_A_CompareModeConfig compareConfigBuzzer = {
    TIMER_A_CAPTURECOMPARE_REGISTER_4,         
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE, 
    TIMER_A_OUTPUTMODE_TOGGLE_SET, 
    OFF
    };

const Timer_A_UpModeConfig upBuzzer = {
    TIMER_A_CLOCKSOURCE_SMCLK,                      
    TIMER_A_CLOCKSOURCE_DIVIDER_6,        
    10000,                              
    TIMER_A_TAIE_INTERRUPT_DISABLE,         
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    
    TIMER_A_DO_CLEAR                       
    };

/* rectangle for screen*/
const Graphics_Rectangle cleanMet = {38, 88, 84, 96};
const Graphics_Rectangle unitDigit = {81, 88, 83, 96};
const Graphics_Rectangle tensDigit = {59, 88, 61, 96};
const Graphics_Rectangle hundredsDigit = {38, 88, 40, 96};

const Graphics_Rectangle barRedLeft = { 5, 82, 15, 94};
const Graphics_Rectangle barRedRight = {113, 82, 123, 94};
const Graphics_Rectangle barYellowLeft = {20, 72, 32, 104};
const Graphics_Rectangle barYellowRight = {96, 72, 108, 104};
const Graphics_Rectangle barGreenHalf = {56, 57, 72, 117};

#endif
