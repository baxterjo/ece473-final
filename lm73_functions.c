// lm73_functions.c       
// Jordan Baxter

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>


uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

//********************************************************************************
//                              lm73_temp_convert
//
//given a temperature reading from an LM73, the address of a buffer
//array, and a format (deg F or C) it formats the temperature into ascii in 
//the buffer pointed to by the arguement.
//******************************************************************************
void lm73_temp_convert(char* temp_digits, uint16_t lm73_temp, uint8_t f_not_c){
    int16_t raw_temp = lm73_temp & ~(1<<15);    //Take off sign bit from lm73 and place in raw_temp data.
    raw_temp >>= 5;                               //Shave off unused bits in LM73 resolution configuration.
    if(lm73_temp & (1<<15)){                    //If sign bit is present, make raw data negative.
        raw_temp *= -1;
    }
    float temp = raw_temp * 0.25;
    if(f_not_c){
        dtostrf(temp, 4, 1, temp_digits);
    } else {
        temp = temp * 1.8 + 32;
        dtostrf(temp, 4, 1, temp_digits);
    }


}//lm73_temp_convert
//******************************************************************************
