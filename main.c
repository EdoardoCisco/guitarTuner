#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <third_party/CMSIS/Include/arm_const_structs.h>
#include <third_party/CMSIS/Include/arm_math.h>

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


/*screen colors*/
void cleanScreen();
void printChoice();
void selectOption();

/*metronome control*/
void incrementBPMvalue();
void decrementBPMvalue();
char* printChar();
void metronomeBPMValue();
void selectedDigit();
void buzz();

/* Tuner control*/
void noteDetection();
int8_t noteCorrection(char, float32_t);
void drawBar(int8_t);
void hannConfig();

/* Handlers */
void DMA_INT1_IRQHandler(void);
void ADC14_IRQHandler(void);
void PORT3_IRQHandler(void);
void PORT4_IRQHandler(void);
void PORT5_IRQHandler(void);
void TA1_N_IRQHanlder(void);



void _adcInit() {

   // ADC14_disableConversion();
//    ADC14_disableModule();
  /* Configures Pin 6.0 and 4.4 as ADC input */
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0,
                                              GPIO_TERTIARY_MODULE_FUNCTION);
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4,
                                              GPIO_TERTIARY_MODULE_FUNCTION);
// 
  /* Initializing ADC (ADCOSC/64/8) */
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_HSMCLK, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

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
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
}

void setHandlersGPIO() {
  // port 3.5 button down
  // port 4.1 joystik selection
  // port 5.1 button back

  /* Configures Pin 6.0 and 4.4 as ADC input */
//  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0,
//                                             GPIO_TERTIARY_MODULE_FUNCTION);
//  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4,
//                                             GPIO_TERTIARY_MODULE_FUNCTION);

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

void _buzzerInit(){
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,
    GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Timer_A0 for Up Mode and starting */
    Timer_A_configureUpMode(TIMER_A0_BASE, &upBuzzer);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    /* Initialize compare registers to generate PWM */
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfigBuzzer); // For P2.7

    /*setup timer for metronome*/
    Timer_A_configureUpMode(TIMER_A1_BASE, &upMetronome);
    Interrupt_enableInterrupt(INT_TA1_N);
    Interrupt_enableMaster();
}

void _graphicsInit(){
    /* Initializes display */                                            
        Crystalfontz128x128_Init();                                    
                                                                          
        /* Set default screen orientation */                              
        Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

        /* Initializes graphics context */
      Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,
                           &g_sCrystalfontz128x128_funcs);
      Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
      Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
      GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
      Graphics_clearDisplay(&g_sContext);

}

void _adcDmaInit(){
   // ADC14_disableInterrupt(ADC_INT1);
   // ADC14_disableConversion();
   // ADC14_disableModule();
  /* Configuring GPIOs (4.3 A10) */
 GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION);
    /* Initializing ADC (MCLK/1/1) */
    Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);
 ADC14_enableModule();
 ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);
 ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE5, false);
  /* Configuring ADC Memory */
 ADC14_configureSingleSampleMode(ADC_MEM2, true);
 ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A10, false);
  /* Set ADC result format to signed binary */
 ADC14_setResultFormat(ADC_SIGNED_BINARY);
 ADC14_enableConversion();

}

void _dmaInit()
{
   //Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig);
 /* Configuring DMA module */
 DMA_enableModule();
 DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);
 DMA_disableChannelAttribute( DMA_CH7_ADC14, UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK );
 /* Setting Control Indexes. In this case we will set the source of the
  * DMA transfer to ADC14 Memory 0
  *  and the destination to the
  * destination data array. */
 DMA_setChannelControl( UDMA_PRI_SELECT | DMA_CH7_ADC14, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
 DMA_setChannelTransfer( UDMA_PRI_SELECT | DMA_CH7_ADC14, UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[2], data_array1, SAMPLE_LENGTH);

 DMA_setChannelControl( UDMA_ALT_SELECT | DMA_CH7_ADC14, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
 DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14, UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[2], data_array2, SAMPLE_LENGTH);

 /* Assigning/Enabling Interrupts */
 DMA_assignInterrupt(DMA_INT1, 7);
 Interrupt_enableInterrupt(INT_DMA_INT1);
 DMA_assignChannel(DMA_CH7_ADC14);
 DMA_clearInterruptFlag(7);
 Interrupt_enableMaster();
 /* Now that the DMA is primed and setup, enabling the channels. The ADC14
  * hardware should take over and transfer/receive all bytes */
 DMA_enableChannel(7);

 hannConfig();

}

void _hwInit(){
    /* Halting WDT and disabling master interrupts */
    WDT_A_holdTimer();
    Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_24);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // dma
    CS_initClockSignal(CS_HSMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_128); // joystick
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); // pwm sampler, microphone 
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    _dmaInit();
    _graphicsInit();
    _adcInit();
    setHandlersGPIO();
    _buzzerInit();

}


int main(void)
{
    _hwInit();

    selectedOptionFlag = 0;

    printChoice();
    selectOption();

    while(1)
    { 
    if(OldFlag==selectedOptionFlag)
    {
    PCM_gotoLPM3();
    }else
    {
        switch (selectedOptionFlag)
        {
            case 0:
               selectOption();
                    break;
            case 1:
                selectOption();
                    break;
            case 72:
                noteDetection();
                printChoice();
                selectOption();
                    break;
            case 73:
                metronomeBPMValue();
                printChoice();
                selectOption();
                    break;
        }
        OldFlag=selectedOptionFlag;
    }
 }



}

void hannConfig(){
 
    // Initialize Hann Window
  int n;
 for(n = 0; n < SAMPLE_LENGTH; ++n)
 {
     hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
 }

}

void cleanScreen(){
    Graphics_clearDisplay(&g_sContext);
}

void printChoice(){
    cleanScreen();
    Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)"GUITAR TUNER",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    48,
                                    OPAQUE_TEXT);

    Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)"METRONOME",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    80,
                                    OPAQUE_TEXT);
}

void selectOption(){
    if(selectedOptionFlag==0){
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)">",
                                    AUTO_STRING_LENGTH,
                                    21,
                                    80,
                                    OPAQUE_TEXT);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)">",
                                    AUTO_STRING_LENGTH,
                                    21,
                                    48,
                                    OPAQUE_TEXT);
    }else if(selectedOptionFlag==1){
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_drawStringCentered(&g_sContext,
                                   (int8_t *)">",
                                   AUTO_STRING_LENGTH,
                                   21,
                                   48,
                                   OPAQUE_TEXT);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawStringCentered(&g_sContext,
                                   (int8_t *)">",
                                   AUTO_STRING_LENGTH,
                                   21,
                                   80,
                                   OPAQUE_TEXT);

        }
}

void incrementBPMvalue(){
    if(selectedOptionFlag & BIT3 && printedBPM < 500 ){
            printedBPM += 100;
    }else if(selectedOptionFlag & BIT4 && printedBPM < 550 ){
            printedBPM += 10;
    }else if(selectedOptionFlag & BIT5 && printedBPM < 555 ){
            printedBPM += 1;
   }
}

void decrementBPMvalue(){
    if(selectedOptionFlag & BIT3 && printedBPM > 140 ){
            printedBPM -= 100;
    }else if(selectedOptionFlag & BIT4 && printedBPM > 40 ){
            printedBPM -= 10;
    }else if(selectedOptionFlag & BIT5 && printedBPM > 31 ){
            printedBPM -= 1;
    }
}

char* printChar(){
  char BPM[5];
  sprintf(BPM, "%d", printedBPM);
  char tmp[5];
  if(printedBPM<100){
      tmp[0]='0';
      strcat(tmp, BPM);
      strcpy(BPM, tmp);
  }
  return BPM;
}

void metronomeBPMValue(){ 

  cleanScreen();
  char BPM[5];
  uint_fast8_t oldBPM = 0;

  while(selectedOptionFlag !=0){

    if(selectedOptionFlag == OldFlag && oldBPM == printedBPM ){
      PCM_gotoLPM3();

    }else{
        if(selectedOptionFlag & BIT7){
            selectedDigit();
            upMetronome.timerPeriod = BPM_COUNTER(printedBPM);  /*  Timer*/
            Timer_A_configureUpMode(TIMER_A1_BASE, &upMetronome);/* Setup*/

            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);//metronome timer start
           while(selectedOptionFlag & BIT7){
                PCM_gotoLPM0();
            }
            Timer_A_stopTimer(TIMER_A1_BASE);//metronome timer stop
            Timer_A_clearTimer(TIMER_A1_BASE);
            selectedDigit();
            OldFlag=selectedOptionFlag;
            oldBPM=printedBPM;
        }else{
            strcpy(BPM, printChar());
            cleanScreen();
            GrContextFontSet(&g_sContext, &g_sFontCm48);
            Graphics_drawStringCentered(&g_sContext,(int8_t *)BPM,AUTO_STRING_LENGTH,64,64,OPAQUE_TEXT);
            GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
            selectedDigit();
            OldFlag=selectedOptionFlag;
            oldBPM=printedBPM;
      }
    }
  }
}

void selectedDigit(){

     if(selectedOptionFlag & BIT7){
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_fillRectangle(&g_sContext, &cleanMet);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

     }else if(selectedOptionFlag & BIT3){// centinaia
        Graphics_fillRectangle(&g_sContext, &hundredsDigit);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_fillRectangle(&g_sContext, &tensDigit);
        Graphics_fillRectangle(&g_sContext, &unitDigit);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    }else if(selectedOptionFlag & BIT4){// decine

        Graphics_fillRectangle(&g_sContext, &tensDigit);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_fillRectangle(&g_sContext, &hundredsDigit);
        Graphics_fillRectangle(&g_sContext, &unitDigit);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    }else if(selectedOptionFlag & BIT5){// unita
        Graphics_fillRectangle(&g_sContext, &unitDigit);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_fillRectangle(&g_sContext, &hundredsDigit);
        Graphics_fillRectangle(&g_sContext, &tensDigit);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    }
}


void drawBar(int8_t bar){
    cleanScreen();
    switch(bar){                                    //draw bars
        case 0:
             //half bar
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_GREEN);
                Graphics_fillRectangle(&g_sContext, &barGreenHalf);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        case 1:
            //left yellow
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                Graphics_fillRectangle(&g_sContext, &barYellowLeft);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        case 2:
             //left red
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                Graphics_fillRectangle(&g_sContext, &barRedLeft);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        case -1:
             //right yellow
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                Graphics_fillRectangle(&g_sContext, &barYellowRight);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        case -2:
            //right red
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
                Graphics_fillRectangle(&g_sContext, &barRedLeft);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    }

}


void noteDetection(){

  _adcDmaInit();

  int i = 0;

while (selectedOptionFlag !=0)
{
        PCM_gotoLPM0();
  /* Computer real FFT using the completed data buffer */
  if(switch_data & 1)
  {
      for(i = 0; i < 512; i++)
      {
          data_array1[i] = (int16_t)(hann[i] * data_array1[i]);
      }
      arm_rfft_instance_q15 instance;
      status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,doBitReverse);

              arm_rfft_q15(&instance, data_array1, data_input);
  }
  else
  {
      for(i = 0; i < 512; i++)
      {
          data_array2[i] = (int16_t)(hann[i] * data_array2[i]);
      }
      arm_rfft_instance_q15 instance;
      status = arm_rfft_init_q15(&instance, fftSize, ifftFlag,
                                 doBitReverse);
              arm_rfft_q15(&instance, data_array2, data_input);
  }
  /* Calculate magnitude of FFT complex output */
  for(i = 0; i < 1024; i += 2){
      data_output[i / 2] = (int32_t)(sqrtf((data_input[i] *
                           data_input[i]) + (data_input[i + 1] * data_input[i + 1])));
  }
  q15_t maxValue;
  uint32_t maxIndex = 0;
  char *note;
  arm_max_q15(data_output, fftSize, &maxValue, &maxIndex);

  printf("pre Max=%d\n",maxIndex);

  float32_t indexToFreq=maxIndex*700/512;
  printf("post Max=%f\n",indexToFreq);
/*
  if(73 <= indexToFreq <= 92){
      strcpy(&note, "E");
  } else if(98 <= indexToFreq <= 123){
      strcpy(&note, "A");
  } else if(131 <= indexToFreq <= 165){
      strcpy(&note, "D");
  } else if(175 <= indexToFreq <= 220){
      strcpy(&note, "G");
  } else if(220 < indexToFreq <= 277){
      strcpy(&note, "B");
  } else if(294 <= indexToFreq <= 370){
      strcpy(&note, "e");
  }
  drawBar(noteCorrection(note, indexToFreq));
  GrContextFontSet(&g_sContext, &g_sFontCm48);    //draw letter
  Graphics_drawStringCentered(&g_sContext,(int8_t *)note,AUTO_STRING_LENGTH,64,20,OPAQUE_TEXT);           //change center
  GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
*/
}
 _adcInit();
}

int8_t noteCorrection(char note, float32_t maxIndex){
  int16_t dif;
  float32_t dist;
  switch(note){
      case 'E':     dif = E - maxIndex; dist = abs(dif/E);
        break;
      case 'A':     dif = A - maxIndex; dist = abs(dif/A);
        break;
      case 'D':     dif = D - maxIndex; dist = abs(dif/D); 
        break;
      case 'G':     dif = G - maxIndex; dist = abs(dif/G); 
        break;
      case 'B':     dif = B - maxIndex; dist = abs(dif/B); 
        break;
      case 'e':     dif = e - maxIndex; dist = abs(dif/e); 
        break;
        default: break;
  }
 if(dif >= 0){
     if(dist <= 0.024){
         return 0;
     }else if(0.024 < dist && dist  < 0.06){
         return 1;
     }else if(dist >= 0.06){
         return 2;
     }
 }else if(dif < 0){
     if(dist <= 0.024){
         return 0;
     }else if(0.024 < dist && dist < 0.06){
         return -1;
     }else if(dist >= 0.06){
         return -2;
     }
 }else{ return 2;}
}

void buzz(){
        compareConfigBuzzer.compareValue = ON;
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfigBuzzer);
        uint_fast8_t i;
        for(i=0; i < 22500; ++i);
        compareConfigBuzzer.compareValue = OFF;
        Timer_A_initCompare(TIMER_A0_BASE, &compareConfigBuzzer);
}

/* Completion interrupt for ADC14 MEM0 */
void DMA_INT1_IRQHandler(void)
{
    // /* Switch between primary and alternate bufferes with DMA's PingPong mode */
 if(DMA_getChannelAttribute(7) & UDMA_ATTR_ALTSELECT)
 {
     DMA_setChannelControl(
         UDMA_PRI_SELECT | DMA_CH7_ADC14,
         UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
         UDMA_DST_INC_16 | UDMA_ARB_1);
     DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                            UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[2],
                            data_array1, SAMPLE_LENGTH);
     switch_data = 1;
 }
 else
 {
     DMA_setChannelControl(
         UDMA_ALT_SELECT | DMA_CH7_ADC14,
         UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
         UDMA_DST_INC_16 | UDMA_ARB_1);
     DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
                            UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[2],
                            data_array2, SAMPLE_LENGTH);
     switch_data = 0;
 }
}

void ADC14_IRQHandler(void) {
  uint64_t status;
  status = ADC14_getEnabledInterruptStatus();
  ADC14_clearInterruptFlag(status);

  if (status & ADC_INT1) {
    /* Store ADC14 conversion results */
    resultBuffer[0] = ADC14_getResult(ADC_MEM0); // left/right
    resultBuffer[1] = ADC14_getResult(ADC_MEM1); // up/down

      /*up down */
    if (resultBuffer[1] > 15000) {
      if (selectedOptionFlag < 2) {
        selectedOptionFlag ^= BIT0;
      } else if (selectedOptionFlag & BIT3 | BIT5 | BIT4) {
          incrementBPMvalue();
          }
    } else if (resultBuffer[1] < 1000) {
      if (selectedOptionFlag < 2) {
        selectedOptionFlag ^= BIT0;
      } else if (selectedOptionFlag & BIT3 | BIT5 | BIT4) {
          decrementBPMvalue();

      }
      /*left right */
    } else if (resultBuffer[0] > 15000 && selectedOptionFlag > 3) {
      if (selectedOptionFlag & BIT3) {
        selectedOptionFlag ^= BIT3;
        selectedOptionFlag |= BIT4;

      } else if (selectedOptionFlag & BIT4) {
        selectedOptionFlag ^= BIT4;
        selectedOptionFlag |= BIT5;

      } else if (selectedOptionFlag & BIT5) {
        selectedOptionFlag ^= BIT5;
        selectedOptionFlag |= BIT3;
      }
    } else if (resultBuffer[0] < 1000 && selectedOptionFlag > 3) {

      if ((selectedOptionFlag & BIT3)) {
        selectedOptionFlag ^= BIT3;
        selectedOptionFlag |= BIT5;

      } else if ((selectedOptionFlag & BIT4)) {
        selectedOptionFlag ^= BIT4;
        selectedOptionFlag |= BIT3;

      } else if (selectedOptionFlag & BIT5) {
        selectedOptionFlag ^= BIT5;
        selectedOptionFlag |= BIT4;
      }
    }
  }
}

/*handler for port 3 pin 5 [paly pause buton ]*/
void PORT3_IRQHandler(void) {
  uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
  GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
  if (status & GPIO_PIN5 && selectedOptionFlag > 6) {
    selectedOptionFlag ^= BIT7; //if 0 to 1 if 1 to 0
  }
}

/*handler for port 4 pin 1 [joistick selection]*/
void PORT4_IRQHandler(void) {
  uint_fast16_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
  GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
  if (status & GPIO_PIN1 && selectedOptionFlag < 3) {
    selectedOptionFlag ^= BIT6; //if 0 to 1 if 1 to 0
    selectedOptionFlag ^= BIT3; 
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

void TA1_N_IRQHandler(void){ //metronome control

    Timer_A_clearInterruptFlag(TIMER_A1_BASE);
    buzz();
}

/* selectedOption Flag
--> 0 main
    BIT0    main selection
    BIT1    left/Up
    BIT2    rigt/Down
    BIT3    digit selection
    BIT4    digit selection
    BIT5    digit selection
    BIT6    back menu   
    BIT7    play/pause methronome


*/
