#include "headers/screenFunction.h"
#include <ti/devices/msp432p4xx/inc/msp.h>
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "Include/arm_const_structs.h"
#include "Include/arm_math.h"
/* #include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h" */
#include <stdio.h> //not needed
#include <time.h>  //timer

#define TEST_LENGTH_SAMPLES 512
#define SAMPLE_LENGTH 512

/* ------------------------------------------------------------------
 * Global variables for FFT Bin Example
 * ------------------------------------------------------------------- */
uint32_t fftSize = SAMPLE_LENGTH;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
volatile arm_status status;

#define SMCLK_FREQUENCY 48000000
#define SAMPLE_FREQUENCY 1024

/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 1024
#elif defined(__GNUC__)
__attribute__((aligned(1024)))
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

uint32_t color = 0;

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig = {TIMER_A_CLOCKSOURCE_SMCLK,
                               TIMER_A_CLOCKSOURCE_DIVIDER_1,
                               (SMCLK_FREQUENCY / SAMPLE_FREQUENCY),
                               TIMER_A_CAPTURECOMPARE_REGISTER_1,
                               TIMER_A_OUTPUTMODE_SET_RESET,
                               (SMCLK_FREQUENCY / SAMPLE_FREQUENCY) / 2};
/*  handler variables   */
uint8_t selectedOptionFlag;
int8_t Number = 100;
uint8_t timerEnable;
/* ADC results buffer */
static uint16_t resultBuffer[2];

void _adcInit() {
  /* Configures Pin 6.0 and 4.4 as ADC input */
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0,
                                             GPIO_TERTIARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4,
                                             GPIO_TERTIARY_MODULE_FUNCTION);

  /* Initializing ADC (ADCOSC/64/8) */
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

  /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
   * with internal 2.5v reference */
  ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
  ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                  ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                  ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

  /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
   *  is complete and enabling conversions */
  ADC14_enableInterrupt(ADC_INT1);

  /* Enabling Interrupts */
  Interrupt_enableInterrupt(INT_ADC14);
  Interrupt_enableMaster();

  /* Setting up the sample timer to automatically step through the sequence
   * convert.
   */
  ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
}

void _hwInit() {
  /* Halting WDT and disabling master interrupts */
  WDT_A_holdTimer();
  Interrupt_disableMaster();

  /* Set the core voltage level to VCORE1 */
  PCM_setCoreVoltageLevel(PCM_VCORE1);

  /* Set 2 flash wait states for Flash bank 0 and 1*/
  FlashCtl_setWaitState(FLASH_BANK0, 2);
  FlashCtl_setWaitState(FLASH_BANK1, 2);

  /* Initializes Clock System */
  CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
  CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

  _adcInit();
}

void setHandlersGPIO() {
  // port 3.5 button down
  // port 4.1 joystik selection
  // port 5.1 button back

  /*set port 3 pin 5 */
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);
  GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);

  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN5);
  GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN5);

  Interrupt_enableInterrupt(INT_PORT3);

  /*set port 4 pin 1*/
  GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
  GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);

  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN1);
  GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);

  Interrupt_enableInterrupt(INT_PORT4);

  /*set port 5 pin 1*/
  GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
  GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);

  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN1);
  GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN1);

  Interrupt_enableInterrupt(INT_PORT5);

  /*active interrupt*/
  Interrupt_enableMaster();
}

/* main.c */
void main(void) {
  _hwInit();
  setHandlersGPIO();
  initScreen();
  selectedOptionFlag = 0;

  /*	int i;
          int n;
  */
  printChoice();
  while (1) {
    switch (selectedOptionFlag) {
    case 0:
      selectOption(selectedOptionFlag);
      break;
    case 1:
      selectOption(selectedOptionFlag);
      break;
    case 64:
      drawBar(&selectedOptionFlag);
      printChoice();
      break;
    case 65:
      metronomeBPMValue(&selectedOptionFlag);
      printChoice();
      break;
    }
    MAP_ADC14_toggleConversionTrigger();
    printf("%d\n", selectedOptionFlag);
  }
}

void ADC14_IRQHandler(void) {
  uint64_t status;
   int i;
  status = ADC14_getEnabledInterruptStatus();
  ADC14_clearInterruptFlag(status);

  /* ADC_MEM1 conversion completed */
  if (status & ADC_INT1) {
    /* Store ADC14 conversion results */
    resultBuffer[0] = ADC14_getResult(ADC_MEM0); // left/right
    resultBuffer[1] = ADC14_getResult(ADC_MEM1); // up/down
    selectedOptionFlag &= ~BIT2;
    selectedOptionFlag &= ~BIT3;
    if (resultBuffer[1] > 14000) {
      if (selectedOptionFlag < 2) {
        selectedOptionFlag &= ~BIT0;
      } else if (selectedOptionFlag & BIT6 | BIT5 | BIT4) { // problem
        selectedOptionFlag |= BIT2;
      }
    } else if (resultBuffer[1] < 2000) {
      if (selectedOptionFlag < 2) {
        selectedOptionFlag |= BIT0;
      } else if (selectedOptionFlag & BIT6 | BIT5 | BIT4) { // problem
        selectedOptionFlag |= BIT3;
      }
      /*left right ok*/
    } else if (resultBuffer[0] > 14000) {
      if (selectedOptionFlag > 3) {

        if (selectedOptionFlag & BIT6) {
          selectedOptionFlag |= BIT5;
          selectedOptionFlag ^= BIT6;

        } else if (selectedOptionFlag & BIT5) {
          selectedOptionFlag |= BIT4;
          selectedOptionFlag ^= BIT5;
        }
      }
    } else if (resultBuffer[0] < 2000) {
      if (selectedOptionFlag > 3) {

        if ((selectedOptionFlag & BIT4)) {
          selectedOptionFlag |= BIT5;
          selectedOptionFlag ^= BIT4;

        } else if ((selectedOptionFlag & BIT5)) {
          selectedOptionFlag |= BIT6;
          selectedOptionFlag ^= BIT5;
        }
      }
    }
     for(i=0;i<300000;++i); //use  timer module delay
  }
}

  /*handler for port 3 pin 5 [paly pause buton ]*/
  void PORT3_IRQHandler(void) {
    uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
    if (status & GPIO_PIN5 && selectedOptionFlag > 6) {
      selectedOptionFlag ^= BIT7;
    }
  }

  /*handler for port 4 pin 1 [joistick selection]*/
  void PORT4_IRQHandler(void) {
    uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
    if (status & GPIO_PIN1 && selectedOptionFlag < 3) {
      selectedOptionFlag |= BIT6;
    }
  }

  /*handler for port 5 pin 1 button [back option];*/
  void PORT5_IRQHandler(void) {
    uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
    if (status & GPIO_PIN1 && selectedOptionFlag > 3) {
      selectedOptionFlag = 0;
    }
  }
