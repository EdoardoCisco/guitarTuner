#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <third_party/CMSIS/Include/arm_const_structs.h>
#include <third_party/CMSIS/Include/arm_math.h>

#include "headers/global.h"
#include "headers/init.h"
#include "headers/handler.h"
#include "headers/screenFunction.h"


/*metronome control*/
void incrementBPMvalue();
void decrementBPMvalue();
void metronomeBPMValue();
void buzz();

/* Tuner control*/
void noteDetection();
int8_t noteCorrection(char, float32_t);

int main(void)
  {
    _hwInit();

    selectedOptionFlag = 0;

    printChoice();
    selectOption();

    while(1){ 
      if(OldFlag==selectedOptionFlag){
        PCM_gotoLPM3();
      }else{
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




void noteDetection(){

  _adcDmaInit();

  int i = 0;

  while (selectedOptionFlag !=0){
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

/* selectedOptionFlag
--> 0 main
    BIT0    main selection
    BIT1    
    BIT2    
    BIT3    digit selection
    BIT4    digit selection
    BIT5    digit selection
    BIT6    back menu   
    BIT7    play/pause methronome
*/
