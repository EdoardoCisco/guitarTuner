#include "headers/screenFunction.h"
#include "grlib/grlib.h"
#include "LcdDriver/HAL_I2C.h"
#include "LcdDriver/HAL_OPT3001.h"
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

/* Graphic library context */
Graphics_Context g_sContext;

void initScreen(){
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

void selectOption(uint8_t Flag){
    if(Flag==0){
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
        }else if(Flag==1){
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

void metronomeBPMValue(uint8_t *Flag){
    cleanScreen();
    while(*Flag !=0){
        MAP_ADC14_toggleConversionTrigger();
        printf("\t\t%d\n",*Flag);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_drawStringCentered(&g_sContext,(int8_t *)incrementBPMValue(Flag),AUTO_STRING_LENGTH,64,64,OPAQUE_TEXT);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    selectedDigit(*Flag);
    }
}

void selectedDigit(uint8_t Flag){

    if(Flag & BIT7){
     Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
     Graphics_drawLineH(&g_sContext,57,69,69);
     Graphics_drawLineH(&g_sContext,57,69,70);
     Graphics_drawLineH(&g_sContext,57,69,71);
     Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    }else if(Flag & BIT6){//check bit 6
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_drawLineV(&g_sContext, 63, 69, 71);
        Graphics_drawLineV(&g_sContext, 69, 69, 71);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawLineV(&g_sContext, 57, 69, 71);

    }else if(Flag & BIT5){//check bit 5
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_drawLineV(&g_sContext, 57, 69, 71);
        Graphics_drawLineV(&g_sContext, 69, 69, 71);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawLineV(&g_sContext, 63, 69, 71);

    }else if(Flag & BIT4){//check bit 4
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_drawLineV(&g_sContext, 63, 69, 71);
        Graphics_drawLineV(&g_sContext, 57, 69, 71);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        Graphics_drawLineV(&g_sContext, 69, 69, 71);
    }
    }

int incrementBPMvalue(uint8_t Flag){
    int8_t printedBPM;
    if(Flag & BIT4){
       printedBPM += selectedOptionFLag - BIT4;
    }if(Flag & BIT5){
       printedBPM += 10*(selectedOptionFLag - BIT5);
    }if(Flag & BIT6){
       printedBPM += 100*(selectedOptionFlag - BIT6);
    }
    return printedBPM;
}

/*per disegnare barre grosse [precisone del tuner]*/

void drawBar(uint8_t * Flag){
    cleanScreen();
    int i;

    Graphics_drawLineV(&g_sContext, 51, 11, 46);
    Graphics_drawLineV(&g_sContext, 76, 11, 46);
    Graphics_drawLineH(&g_sContext, 51, 76, 11);
    Graphics_drawLineH(&g_sContext, 51, 76, 46);


    /*
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_drawLineH(&g_sContext,0,55,100);
    Graphics_drawLineH(&g_sContext,71,127,100);*/

    for(i=5;i<15;++i){      //left red
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
        Graphics_drawLineV(&g_sContext, i, 82, 94);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    }

    for(i=20;i<32;++i){     //left yellow
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
                Graphics_drawLineV(&g_sContext, i, 72, 104);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        }

    for(i=38;i<52;++i){     //left green
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_GREEN);
        Graphics_drawLineV(&g_sContext, i, 64, 110);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        }

    for(i=56;i<72;++i){     //half bar
       // Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
        Graphics_drawLineV(&g_sContext, i, 57, 117);
       // Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    }

    for(i=77;i<91;++i){     //right green
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_GREEN);
        Graphics_drawLineV(&g_sContext, i, 64, 110);
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        }

    for(i=96;i<108;++i){        //right yellow
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
            Graphics_drawLineV(&g_sContext, i, 72, 104);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            }

    for(i=113;i<123;++i){       //right red
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
            Graphics_drawLineV(&g_sContext, i, 82, 94);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            }

    for(i=0;i<100000;++i);

            /******DELETE EACH BAR****/
    for(i=5;i<15;++i){          //left red
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawLineV(&g_sContext, i, 82, 94);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    }
    for(i=113;i<123;++i){       //right red
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawLineV(&g_sContext, i, 82, 94);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            }

    for(i=0;i<100000;++i);

        for(i=20;i<32;++i){         //left yellow
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                    Graphics_drawLineV(&g_sContext, i, 72, 104);
                    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            }
        for(i=96;i<108;++i){        //right yellow
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_drawLineV(&g_sContext, i, 72, 104);
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
                }

        for(i=0;i<100000;++i);

        for(i=38;i<52;++i){         //left green
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawLineV(&g_sContext, i, 64, 110);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            }

        for(i=77;i<91;++i){         //right green
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawLineV(&g_sContext, i, 64, 110);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
            }

        for(i=0;i<100000;++i);

        for(i=56;i<72;++i){         //half bar
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
            Graphics_drawLineV(&g_sContext, i, 57, 117);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        }

/*******CORRECT TUNE*******/
    for(i=56;i<71;++i){    //half bar
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_GREEN);
            Graphics_drawLineV(&g_sContext, i, 57, 117);
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
        }
    while(*Flag!=0){}
}
