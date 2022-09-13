#ifndef __SCREENFUNCTION_H__
#define __SCREENFUNCTION_H__


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
                Graphics_fillRectangle(&g_sContext, &barRedRight);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    }

}



#endif
