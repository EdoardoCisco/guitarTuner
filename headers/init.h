#ifndef __INIT_H__
#define __INIT_H__

#include "global.h"

void hannConfig(){

  // Initialize Hann Window
  int n;
  for(n = 0; n < SAMPLE_LENGTH; ++n)
  {
      hann[n] = 0.5f - 0.5f * cosf((2 * PI * n) / (SAMPLE_LENGTH - 1));
  }

}

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

#endif
