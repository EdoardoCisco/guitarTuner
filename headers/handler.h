#ifndef __HANDLER_H__
#define __HANDLER_H__

#include "global.h"

/* Completion interrupt for ADC14 MEM0 */
void DMA_INT1_IRQHandler(void){
    /* Switch between primary and alternate bufferes with DMA's PingPong mode */
    if(DMA_getChannelAttribute(7) & UDMA_ATTR_ALTSELECT){
        DMA_setChannelControl(
            UDMA_PRI_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE |
            UDMA_DST_INC_16 | UDMA_ARB_1);
        DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
                                UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[2],
                                data_array1, SAMPLE_LENGTH);
        switch_data = 1;
    }
    else{
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

#endif
