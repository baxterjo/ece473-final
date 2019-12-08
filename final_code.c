// final_code.c 
// Jordan Baxter
// 12.03.19

/****************************
TODO Section:
+Hardware
-[X] Install radio board with level shifting circuits and antenna.
+Hardware Dependednt Firmware
-[X] LED display shows station frequency when tuning radio, else time.
-[X] One of the encoders will adjust volume, the other will adjust station
-[X] LCD Display will be formatted as shown in lab requirements
-[X] Radio will only display odd frequencies at 200kHz intervals (e.g. 88.1, 88.3, 88.5.....)
-[] Can choose between radio and buzzer on alarm
-[] ExC: Station Presets
-[] Exc: Siganl strength indication
-[] ExC: AM radio functionality
***********************************/


#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define U8TRUE 0xFF

/****************************
Seven Seg BCD display definitions

These definitions are redundant considering the array below. 
But can be nice to manipulate individual digits if needed.

Structure for digits:
dp, g, f, e, d, c, b, a

Structure for colon:
nc, nc, nc, nc, nc, l3, l2, l1
****************************/
#define sega 0
#define segb 1
#define segc 2
#define segd 3
#define sege 4
#define segf 5
#define segg 6
#define segdp 7

#define dt7_0           0b01000000 //Decimal to 7-segment (dt7)
#define dt7_1           0b01111001
#define dt7_2           0b00100100
#define dt7_3           0b00110000
#define dt7_4           0b00011001
#define dt7_5           0b00010010
#define dt7_6           0b00000010
#define dt7_7           0b01111000
#define dt7_8           0b00000000
#define dt7_9           0b00010000
#define dt7_colon       0b01111100
#define dt7_dot         0b01111011
#define dt7_blank       0b01111111

/****************************
Encoder state definitions
****************************/
#define CW              1
#define CCW             -1
#define enc_rstrip      0b00001100    //This will strip the two right encoder digits from SPDR data.
#define enc_lstrip      0b00000011    //This will strip the two left encoder digits from SPDR data.


/****************************
Multiplexer select and SPI definitions

Structure for PortB:
_________________________________________________________________________________
|____7____|____6____|____5____|____4____|____3____|____2____|____1____|____0____|
|___pwm___|__sel2___|__sel1___|__sel0___|_SPI_MISO|_SPI_MOSI|_SPI_CLK_|___SS_0__|
****************************/

#define digit4      0b00000000
#define digit3      0b00010000
#define colon_seg   0b00100000
#define digit2      0b00110000
#define digit1      0b01000000
#define dec5        0b01010000
#define dec6        0b01100000
#define com_enable  0b01110000
#define spi_SS1     0b00000001

/****************************
Structure for PortE:
****************************/
#define sft_ld_n    0b01000000

/****************************
Update flags
****************************/
#define RADIO_FLAG  0
#define TEMP_FLAG   1



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <stdlib.h>
#include <string.h>
#include "lm73_functions.h"
#include "hd44780.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"



//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12] = 
{
  dt7_0,
  dt7_1,
  dt7_2,
  dt7_3,
  dt7_4,
  dt7_5,
  dt7_6,
  dt7_7,
  dt7_8,
  dt7_9,
  dt7_colon,
  dt7_blank
};

uint8_t digit_display[5] = 
{
  digit1,
  digit2,
  colon_seg,
  digit3,
  digit4
};
/****************************
Typedef Enum Definitions
****************************/
typedef enum{
  TIME_NOT_SET,
  SET_TIME,
  CLOCK,
  SET_ALARM,
  TUNE
} display_mode_t;


//******************************************************************************
//                             Global Variable Declarations
//******************************************************************************
//Time keeping Variables
  int8_t colon = FALSE;                     //Colon display variable, this will turn on and off every second, global because it is read in TCNT0 ISR
  int8_t pm = FALSE;                        //PM tracker, this will indicate AM or PM functionality, global because it is read in TCNT0 ISR
  volatile int16_t mSec = 0;                //mSec tracker, not really milliseconds, increments every 1/512 seconds, global because it is incremented in TCNT0 ISR
  volatile int8_t sec = 0;                  //Sec tracker, will increment every 512 mSec, global because it is incremented in TCNT0 ISR
  int8_t min = 0;                           //Min tracker, increments once every 60 min, global because it is read in TCNT0 ISR 
  int8_t hr = 12;                           //Hour tracker, increments once every 60 min. Initialized at 12 to indicate time has not beed set, global because it is read in TCNT0 ISR

//Delayed update flag
//Structure:
//|unused|unused|unused|unused|unused|unused|temp update|radio update|
  uint8_t delayed_update_flag = FALSE;

//Alarm Variables
  uint8_t alarm_active = FALSE;
  uint8_t alarm_set = FALSE;                    //Alarm set flag, global because it is read in TCNT0 ISR       
  uint8_t alarm_hr = 12;                    //Alarm hour variable, global because it is read in TCNT0 ISR
  int8_t alarm_min = 0;                     //Alarm min variable, global because it is read in TCNT0 ISR
  int8_t alarm_pm = FALSE;                  //Alarm AM/PM variable, global because it is read in TCNT0 ISR
  volatile int8_t tta_sec = 0;              //Time to alarm in seconds.
  volatile int16_t tta_min = 0;             //Time to alarm in minutes.

  enum alarm_mode{
    RADIO,
    BUZZER
  };

  enum alarm_mode a_mode = RADIO;


//Mode Variables
  display_mode_t d_mode = TIME_NOT_SET;     //Clock display state, global because it is read in TCNT0 ISR

//Display Variables
  volatile uint8_t digit_count = 0;         //Start a digit counter, global because it is incremented in TCNT0 ISR
  volatile int16_t disp_value = 0;          //Initialize display value variable, global because it is modified in TCNT0 ISR
  volatile uint8_t segment_data[5];         //holds data to be sent to the segments. logic zero turns segment on, global because it is modified in TCNT0 ISR
  volatile uint8_t brightness = 100;        //Starts brightness at ~40% duty cycle, due to gamma correction, this equates to ~ full brightness on the LED display

//Audio Variables
uint8_t volume = 6;

//ADC Variables
volatile uint16_t adc_result;

//LCD Variables
char lcd_str_h[16];
char lcd_str_l[16];


//Temp Sensor Variables
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;

//Radio Variables
enum radio_band{
  FM, 
  AM, 
  SW};
volatile enum radio_band current_radio_band = FM;

uint8_t rad_onoff = 1;

extern uint8_t si4734_wr_buf[9];          //buffer for holding data to send to the si4734 
extern uint8_t si4734_rd_buf[15];         //buffer for holding data recieved from the si4734
extern uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  
extern uint8_t si4734_revision_buf[16];   //buffer for holding revision  data

volatile uint8_t STC_interrupt;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

uint16_t current_fm_freq = 9990;
uint16_t past_fm_freq = 9990;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;
uint8_t rssi;
uint8_t sig_str;
uint8_t tune_tim;

char uart1_tx_buf[40];      //holds string to send to crt
char uart1_rx_buf[40];      //holds string that recieves data from uart


//External Comm Variables
volatile uint8_t  rcv_rdy;
char              rx_char;
char              rx_buf[16];

//Used in debug mode for UART1
char uart1_tx_buf[40];      //holds string to send to crt
char uart1_rx_buf[40];      //holds string that recieves data from uart

//Button Variables
  volatile uint8_t button_flags = 0;        //Flag that will be triggered by button being pressed, global because it is modified in TCNT0 ISR
  volatile uint16_t state[8] = {0,          //Button Debounce state, global because it is modified in TCNT0 ISR
  0, 
  0, 
  0, 
  0, 
  0, 
  0, 
  0};

//Global variable declarations
   
//***************************************************************************
//                            tcnt0_init  
//***************************************************************************                                    
//Initializes timer counter 0 in normal mode with a 128 prescaler and enables
//timer interrupts.
//
void tcnt0_init(){
  ASSR  |=  (1 << AS0);                 //ext 32kHz osc TOSC
  TIMSK |= (1 << OCIE0);                //Output compare match mode
  TCCR0 |= (1 << WGM01) | (1 << CS00);  //Set CTC mode, No Prescaler
  OCR0 = 63;                            //Set output compare register to 63. This sets the timer interrrupt frequency to 512.

}//tcnt0_init

//***************************************************************************
//                            tcnt1_init  
//***************************************************************************                                    
//Initializes timer counter 1 to generate tone that will be output at PC6.
//Starts prescaler at 1024, this will change dynamically as tone changes.
//Initial tone will be c to check amperage output at different volumes

void tcnt1_init(){
  TIMSK |= (0 << OCIE1A);                              //Change this to change the initial status of audio output.
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10); //Set TCNT1 to CTC mode, 1024 prescaler
  TCCR1C = 0;                                         //No force output compare.
  OCR1A = 477;                                        //Set OCR1A to 477 (tuned to middle c, stepped down by 3 octaves)
}//tcnt1_init

//***************************************************************************
//                            tcnt2_init  
//***************************************************************************                                    
//Initializes timer counter 2 to generate a PWM signal at PE6 using fast PWM.
//PWM must have frequency >> tcnt0 to affect the brightness of the LED display
//which already has an inherent PWM.

void tcnt2_init(){
  TCCR2 |= (1 << WGM20) | (1 << WGM21) | (1 << COM20) | (1 << COM21) | (1 << CS20); //Set TCNT2 to fast PWM mode, inverting mode (PNP transistor), no prescaler.
  OCR2 = brightness;                                                                 //Set duty cycle to 75%
}//tcnt2_init

//***************************************************************************
//                            tcnt3_init  
//***************************************************************************                                    
//Initializes timer counter 3 as volume control.
//This will be a pwm output with a minimum resolution of 45 steps. (This was determined by the gain to voltage chart on the audio amplifier)
//Hardware is configured for volume at OC3B
//
void tcnt3_init(){
  TCCR3A |= (1 << COM3B1) | (1 << WGM30); //Set PE4 to output non-inverting PWM and start FAST PWM 8 bit setting.
  TCCR3B |= (1 << WGM32) | (1 << CS30);   //Finish setting fast pwm 8 bit and set prescaler to 1.
  OCR3B = volume;                              //Set Initial volume to ~0.
}
//***********************************************************************
//                            spi_init                               
//***********************************************************************
void spi_init(){
  DDRF |= 0x08;                       //Set up LCD slave select.
  PORTF &= 0xF7;                      //

  DDRB   |= 0x07;                     //output mode for SS, MOSI, SCLK
  PORTB = 0;                          //Write port B to 0 to handle glitching

  SPCR   = (1 << MSTR) | (1 << SPE);  //master mode, clk low on idle, leading edge sample

  SPSR   = (1 << SPI2X);              //choose double speed operation
 }//spi_init

 //***********************************************************************
//                            adc_init                               
//***********************************************************************

void adc_init(){
  DDRF &= ~(_BV(DDF7));                                     //Make port F bit 7 ADC input.
  PORTF &= ~(_BV(PF7));                                     //Turn off pullup resistor.

  ADMUX |= (1<<REFS0) | 0x07;                               //single-ended, input PORTF bit 7, right adjusted, 10 bits

  ADCSRA |= (1<<ADEN) | 0x07 | (1 << ADIE);                 //ADC enabled, don't start yet, single shot mode, enable ADC interrupt
}
//************************************************************************
//                                   segment_sum   
//************************************************************************                                 
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum, int8_t colon, int8_t pm, display_mode_t d_mode) {
  
  uint8_t length = 0;
  if(sum > 999){
    length = 3;
  } else if(sum > 99){
    length = 2;
  } else if(sum > 9){
    length = 1;
  }
  /**************
   * Update values directly in a loop.
   **************/
  //This method is modified from previous labs due to the erroneous used of temprarary data. This case statement
  //makes a loop that directly updates segment data from the given value in the function call. 
  //Rather than storing the data in a temporary digit array, shifting those digits to a temparary 7seg array
  //then finally shifting that temp array into the segment data array. This is more direct, and only uses modulus
  //and division math when required.
  
  for(int i = 0; i < 5; i++){
    switch (i){
      case 0:
        if(3 - i <= length){
          segment_data[i] = dec_to_7seg[sum / 1000]; 
        } else {
          segment_data[i] = dt7_blank;
        }
        if(d_mode != SET_ALARM && d_mode != SET_TIME && !(alarm_set && colon)){ // Turns dots on if setting the time or alarm, flashes dots if the alarm is set.
          segment_data[i] |= 1 << segdp;
        }
        break;
      case 1:
        if(3 - i <= length){
          segment_data[i] = dec_to_7seg[(sum % 1000) / 100];
        } else {
          segment_data[i] = dt7_blank;
        }
        if(d_mode != SET_ALARM && d_mode != SET_TIME && !(alarm_set && colon)){
          segment_data[i] |= 1 << segdp;
        }
        break;
      case 2:
        if(colon){                                      //Check to see if colon is being displayed
          segment_data[i] = dt7_colon;                  //If it is, display it
        }else{
          segment_data[i] = dt7_blank;                  //If not, make segment blank
        }
        if(pm){                                         //Check to see if AM or PM
            segment_data[i] ^= ~dt7_dot;                //If pm, turn on L3               
          }
        break;
      case 3:
        if(3 - i <= length){
          segment_data[i] = dec_to_7seg[(sum % 100) / 10];
        } else {
          segment_data[i] = dt7_blank;
        }
        if(d_mode != SET_ALARM && d_mode != SET_TIME && !(alarm_set && colon) && !(tune_tim > 0)){
          segment_data[i] |= 1 << segdp;
        }
        break;
      case 4:
        if(3 - i <= length){
          segment_data[i] = dec_to_7seg[sum % 10];
        } else {
          segment_data[i] = dt7_blank;
        }
        if(d_mode != SET_ALARM && d_mode != SET_TIME && !(alarm_set && colon)){
          segment_data[i] |= 1 << segdp;
        }
        break;
        default:
        break;

    }

  }
}//segment_sum
//***********************************************************************************

//***********************************************************************************
//                                   Button Handler                                    
//This function handles all button presses. It takes in the button flag and a pointer to the display mode and
//performs an operation based on which button is pressed.

void button_pressed(uint8_t buttons, display_mode_t* d_mode, int8_t* sec_p, uint8_t* alarm_set_p){
  static display_mode_t last_mode;                              //Initialize static variable to keep track of last mode on display, this will allow toggleing of modes.
  uint8_t button = 0;
  for(int i = 0; i < 8; ++i){                                   //Check to see which button was pressed
    if(buttons & (1 << i)){
      break;
    }
    button++;
  }
    switch (button){
      case 0:
        if(tta_min == 0 && tta_sec == 0){
          tta_sec += 10;
        }
        break;
      case 1:
      if(*d_mode == CLOCK){
        if(!*alarm_set_p){
          *alarm_set_p = U8TRUE;       //Toggle the alarm
          tta_min = (alarm_hr - hr) * 60 + (alarm_min - min - 1); //Set time to alarm in minutes. by taking the difference between alarm time and clock time.
          if(pm != alarm_pm){
            tta_min += 12*60;                                  //If alarm_pm and clock pm are not equal, assume user sets alarm for future, add 12 hours.
          } else if(tta_min < 0 && alarm_pm == pm){
            tta_min += 24 * 60;
          }
          tta_sec = 60 - sec;                                  //Set the time to alarm in seconds by subtracting the clock seconds by 60
          cursor_home();
          strncpy(&lcd_str_h[0], "ALARM", 5);
          string2lcd(lcd_str_h);
        } else  {
          *alarm_set_p = FALSE;
          cursor_home();
          strncpy(&lcd_str_h[0], "     ", 5);
          string2lcd(lcd_str_h);
        }
      } else if(*d_mode == SET_ALARM){
        if(a_mode == RADIO){
          a_mode = BUZZER;
          lcd_str_l[15] = 'B';
          home_line2();
          string2lcd(lcd_str_l);
        } else if(a_mode == BUZZER) {
          a_mode = RADIO;
          lcd_str_l[15] = 'R';
          home_line2();
          string2lcd(lcd_str_l);
        }
      }
        
        break;
      case 2:
        if(rad_onoff == 0){
          fm_pwr_up();
          fm_tune_freq();
          current_radio_band = FM;
          rad_onoff = 1;
        } else {
          radio_pwr_dwn();
          rad_onoff = 0;
        }
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        break;
      case 6:
        if(*d_mode == CLOCK){
          *d_mode = SET_ALARM;
        } else {
          *alarm_set_p = U8TRUE;
          tta_min = (alarm_hr - hr) * 60 + (alarm_min - min - 1); //Set time to alarm in minutes. by taking the difference between alarm time and clock time.
          if(pm != alarm_pm){
            tta_min += 12*60;                                  //If alarm_pm and clock pm are not equal, assume user sets alarm for future, add 12 hours.
          } else if(tta_min < 0 && alarm_pm == pm){
            tta_min += 24 * 60;
          }
          tta_sec = 60 - sec;                                  //Set the time to alarm in seconds by subtracting the clock seconds by 60
          cursor_home();
          strncpy(&lcd_str_h[0], "ALARM", 5);
          string2lcd(lcd_str_h);
          *d_mode = CLOCK;
        }
        break;
      case 7:
        if(*d_mode != SET_TIME){     //Button 7 toggles time setting mode
          last_mode = CLOCK;         //Save the last mode to toggle back to when done.
          *d_mode = SET_TIME;
        } else {
          *sec_p = 0;
          *d_mode = last_mode;
        }
        break;
      default:
        break;

    }
}

//***********************************************************************************
//                                   Encoder Turn Handlers                                    
//Functions that are called at the end of an encoder FSM, all are passed the count mode and a pointer to the display value.
//Both of these do the same thing right now but can be modified during later labs for different use.
//Inputs: Turn direction, count mod, and a pointer to the display value.
//Function: Checks which mode the display is in, and behaves according to which way the dial was turned.


void left_turned(int8_t direction, display_mode_t display_mode, int8_t* hr_p, uint8_t* alarm_hrp){
  switch(display_mode){
    case TIME_NOT_SET:
      break;
    case SET_TIME:
      *hr_p += (1 * direction);
      break;
    case CLOCK:
      if(current_radio_band == FM){
        current_fm_freq += 20 * direction;
      }
      break;
    case SET_ALARM:
      *alarm_hrp += (1 * direction);
      break;
    default:
      break;
    
  }
}


void right_turned(int8_t direction, display_mode_t display_mode, uint8_t* min_p, uint8_t* alarm_minp){
  switch(display_mode){
    case TIME_NOT_SET:
      break;
    case SET_TIME:
      *min_p += (1 * direction);
      break;
    case CLOCK:
      volume += 5 * direction;
      if(volume > (250)){
        volume = 250;
      }
      if(OCR3B < 5){
        volume = 6;
      }
      break;
    case SET_ALARM:
      *alarm_minp += (1 * direction);
      break;
    default:
      break;
    
  }
}//Encoder handlers
//***********************************************************************************
//***********************************************************************************


//***********************************************************************************
//                                Main
uint8_t main(){
//*******************************************************
//                 Local Variable Declarations
//*******************************************************
//Alarm Variables
  

//Encoder Variables
  int8_t  l_direction = 0;          //Initialize left direction tracker.
  int8_t  r_direction = 0;          //Initialize right direction tracker.
  uint8_t l_past;                   //Initialize left encoder past state tracker
  uint8_t r_past;                   //Initialize right encoder past state tracker

//*******************************************************
//                          Setup
//*******************************************************
  DDRC |= 0x40;                     //Set port c bit 6 as output (buzzer out)
  DDRB |= 0xF0;                     //set port b bits 4-7 B as outputs
  DDRE |= 0x56;                     //Set port e bits 1,3,4 and 6 as output.
  spi_init();                       //Call SPI init
  tcnt0_init();
  tcnt1_init();
  tcnt2_init();
  tcnt3_init();
  sei();
  lcd_init();
  adc_init();
  uart_init();
  init_twi();
  clear_display();
  cursor_home();
  strncpy(&lcd_str_h[0], "                ", 16);               //Initialize all characters in the LCD strings to blanks.
  strncpy(&lcd_str_l[0], "                ", 16);
  if(a_mode = RADIO){
    lcd_str_l[15] = 'R';
  }else if(a_mode = BUZZER){
    lcd_str_l[15] = 'B';
  }

  lm73_wr_buf[0] = LM73_PTR_TEMP;                               //load lm73_wr_buf[0] with temperature pointer address
  twi_start_wr(LM73_ADDRESS, &lm73_wr_buf[0], TWI_BUFFER_SIZE); //start the TWI write process
  while(twi_busy()){}                                           //wait for the xfer to finish

  //hardware reset of Si4734 written by Roger Traylor

  PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
  DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
  PORTE |=  (1<<PE2); //hardware reset Si4734 
  _delay_us(200);     //hold for 200us, 100us by spec         
  PORTE &= ~(1<<PE2); //release reset 
  _delay_us(30);      //5us required because of my slow I2C translators I suspect
                      //Si code in "low" has 30us delay...no explaination given
  DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

  EICRB |= 0xC0;      //Turn on the external interrupt
  EIMSK |= (1<<7);

  fm_pwr_up();
  while(twi_busy()){}
  current_fm_freq = 9990;
  past_fm_freq = current_fm_freq;
  dtostrf((float)current_fm_freq / 100, 5, 1, &lcd_str_h[8]);
  strncpy(&lcd_str_h[6], "S=", 2);
  //13
  switch(current_radio_band){
    case FM:
      strncpy(&lcd_str_h[13], "FM",2);
      break;
    case AM:
      strncpy(&lcd_str_h[13], "AM",2);
      break;
    case SW:
      strncpy(&lcd_str_h[13], "SW",2);
      break;
  }
  fm_tune_freq();

  cursor_home();
  string2lcd(lcd_str_h);

  



  

  /********************
   * Enter loop
   *******************/
  while(1){
  /********************
   * Radio Update
   *******************/
    switch(current_radio_band){
      case FM:
        if(past_fm_freq != current_fm_freq){
          past_fm_freq = current_fm_freq;
          while(twi_busy()){}
          fm_tune_freq(current_fm_freq);
          dtostrf((float)current_fm_freq / 100, 5, 1, &lcd_str_h[8]);
          cursor_home();
          strncpy(&lcd_str_h[13], "FM",2);
          string2lcd(lcd_str_h);
          tune_tim = 3;
        }
        break;
      case AM:
        break;
      case SW:
        break;
      default:
      break;
    }
      delayed_update_flag &= ~(1<<RADIO_FLAG);
      fm_rsq_status();
      rssi = si4734_tune_status_buf[4];
      //redefine rssi to be a thermometer code
      if(rssi<= 8) {sig_str = 0x00;} else
      if(rssi<=16) {sig_str = 0x01;} else
      if(rssi<=24) {sig_str = 0x03;} else
      if(rssi<=32) {sig_str = 0x07;} else
      if(rssi<=40) {sig_str = 0x0F;} else
      if(rssi<=48) {sig_str = 0x1F;} else
      if(rssi<=56) {sig_str = 0x3F;} else
      if(rssi<=64) {sig_str = 0x7F;} else
      if(rssi>=64) {sig_str = 0xFF;} 
    
    

    

  /********************
   * SPI Read/Write
   *******************/
      PORTE |= sft_ld_n;                  //Send rising reg clock edge to encoders, this will allow the encoder to shift in new data.
      PORTB &= ~spi_SS1;                  //Select SPI slave 1
      SPDR = sig_str;                   //send relative signal strength to the bargraph. 
      while (bit_is_clear(SPSR,SPIF)){}   //spin till SPI data has been sent
      PORTB |= spi_SS1;                   //Deselect SPI slave 1
      PORTE &= ~sft_ld_n;                 //Send falling reg clock edge to encoders, this will keep them in LD mode until next read.
      uint8_t encoder_raw = SPDR;         //Update encoder tracking value to new value.

  /********************
   * Encoder FSM
   *******************/
  //The following segment of code is a finite state machine for the rotary encoders, it ensures a full swing from one resting position to another occurs
  //before executing functions related to the rotation of the encoders.


  uint8_t enc_l = (encoder_raw & enc_lstrip);                     //Strip the left encoder data from raw data.
  uint8_t enc_r = (encoder_raw & enc_rstrip) >> 2;                //Strip the right encoder data from the raw data and shift it to the LSB of the int.

  switch (enc_l){                                                 //Enter switch 
    case 1:                                                       //If current encoder state is 1, execute this segment
    if(l_past == 3){                                              //Check if past state was 3
        l_direction = CCW;                                        //If above statement is true, update direction to CCW and past state to current
      }
      break;                                                      //Exit switch
    case 2:                                                       //If current encoder state is 2 execute this segment
      if (l_past == 3){                                           //Check if past state was 3
        l_direction = CW;                                         //If above statement is true, update direction to CW and past state to current
      }
      break;                                                      //Exit switch
    case 3:                                                       //If current encoder state is 3, execute this statement
      if(l_past == 2 && l_direction == CCW){                      //If past state was 2 and curent direction is CCW, a full CCW click has occured
        left_turned(l_direction, d_mode, &hr, &alarm_hr);                    //Execute left turned function
      } else if (l_past == 1 && l_direction == CW){               //If past state was 1 and current direction is CW, a full CW click has occured
        left_turned(l_direction, d_mode, &hr, &alarm_hr);                    //Execute left turned function
      }
      l_direction = FALSE;                                        //Change direction to FALSE (No Direction)
      break;                                                      //Exit switch
      default:                                                    //Default code, all encoder values are covered in switch, so this is not needed
        break;                                                    //Exit switch
  }
  l_past = enc_l;

  switch (enc_r){                                                 //Execute same code as above, for right encoder
    case 1:
      if(r_past == 3){
        r_direction = CCW;
      }
      break;
    case 2:
      if (r_past == 3){
        r_direction = CW;
      }
      break;
    case 3:
      if(r_past == 2 && r_direction == CCW){
        right_turned(r_direction, d_mode, &min, &alarm_min);
      } else if (r_past == 1 && r_direction == CW){
        right_turned(r_direction, d_mode, &min, &alarm_min);
      }
      r_direction = FALSE;
      break;
      default:
        break;
  }
  r_past = enc_r;


  /********************
   * Button Handler
   *******************/
  if(button_flags){                     //If button flag is set.
    button_pressed(button_flags, &d_mode, &sec, &alarm_set);
    button_flags = 0;
  }


  /********************
   * Clock Updates
   *******************/
  //Clock
  if(sec >= 60){
    sec = 0;
    if(d_mode != TIME_NOT_SET && d_mode != SET_TIME){
      min++;
    }
  }
  if(min >= 60){
    min = 0;
    hr++;
  } else if (min < 0){
    min = 59;
    hr--;
  }
  if(hr > 12){
    hr = 1;
    pm = !pm;
  } else if(hr < 1){
    hr = 12;
    pm = !pm;
  }

  //Alarm
  if(alarm_min >= 60){
    alarm_min = 0;
    alarm_hr++;
  } else if (alarm_min < 0){
    alarm_min = 59;
    alarm_hr--;
  }
  if(alarm_hr > 12){
    alarm_hr = 1;
    alarm_pm = !alarm_pm;
  } else if(alarm_hr < 1){
    alarm_hr = 12;
    alarm_pm = !alarm_pm;
  }

  if(tta_sec < 0){
    tta_sec = 59;
    tta_min--;
  }

  if(d_mode == CLOCK && tta_min <= 0 && tta_sec <= 0 && alarm_set && sec % 2 && a_mode == BUZZER){
    TIMSK |= (1 << OCIE1A);
  } else {
    TIMSK &= ~(1 << OCIE1A);
    PORTC = 0;
  }
    
  /********************
   * Audio Updates
   *******************/
  OCR3B = volume;
  
/********************
   * Display Updates
   *******************/
  colon = sec % 2;
  if(OCR2 > brightness){
    OCR2--;
  } else if (OCR2 < brightness){
    OCR2++;
  }
  ADCSRA |= (1 << ADSC);              //Poke ADSC to begin ADC.

/********************
   * Temp Sensor Updates
   *******************/
  if(delayed_update_flag & (1<<TEMP_FLAG)){
    delayed_update_flag  &= ~(1<<TEMP_FLAG);
    uart_putc('T');                                               //Initiate remote sensor read
    uart_putc('\0');
    twi_start_rd(LM73_ADDRESS, &lm73_rd_buf[0], 2); //Initiate read temperature data from LM73 (2 bytes)
    while(twi_busy()){}                         //wait for it to finish
    lm73_temp = lm73_rd_buf[0];                 //save high temperature byte into lm73_temp
    lm73_temp <<= 8;                            //shift it into upper byte 
    lm73_temp |= lm73_rd_buf[1];                //"OR" in the low temp byte to lm73_temp 
    lm73_temp_convert(&lcd_str_l[0], lm73_temp, 0); // Convert read temp to string and place in lcd string low.
    lcd_str_l[4] = 'F';                         //Write the units to the lcd.
    lcd_str_l[5] = ' ';
    home_line2();
    string2lcd(lcd_str_l);
  }

  if(rcv_rdy){
    rcv_rdy = 0;
    for(int i = 0; i<5; i++){
      lcd_str_l[i+5] = rx_buf[i];
    }
    lcd_str_l[10] = 'F';
    home_line2();
    string2lcd(lcd_str_l);
  }
    
    }//while
}//main

//******************************************************************************
//                            Interrupt service routines
//

ISR(TIMER0_COMP_vect){
/********************
 * Check buttons, increment counter, and increment digit to display.
 *******************/
  mSec++;
  if(mSec >= 512){
    mSec = 0;
    sec++;
    delayed_update_flag = U8TRUE;
    if(tune_tim > 0){
      tune_tim--;
    }
    if(tta_sec > 0 || tta_min > 0){
      tta_sec--;
      if(d_mode == CLOCK && tta_min <= 0 && tta_sec <= 0 && alarm_set){
        volume = 255 * 0.5;
        // if(a_mode == RADIO && rad_onoff == 0){
        //   //fm_pwr_up();
        //   //while(twi_busy()){}
        //   //fm_tune_freq();
        //   rad_onoff = 1;
        // } else {
        //   radio_pwr_dwn();
        //   rad_onoff = 0;
        // }
      }
    }
  }
  DDRA = 0x00;                      //make PORTA an input port with pullups 
  PORTA = 0xFF;
  PORTB |= com_enable;              //enable tristate buffer for pushbutton switches
  for(uint8_t i = 0; i < 8; ++i){   //initiate for loop
    state[i] = (state[i] << 1) | (! bit_is_clear(PINA, i)) | 0xE000; 
    if(state[i] == 0xF000){
      button_flags = 1 << i;
    }
  }
  PORTB &= ~com_enable;             //disable tristate buffer for pushbutton switches
  DDRA = 0xFF;                      //make PORTA an output
  digit_count++;                    //Increment digit counter
  /********************
   * Display value
   *******************/ 
    
    if(digit_count > 4){
      digit_count = 0;
    }
    switch (d_mode){                            //This switch decides what to display based on mode.
      case SET_ALARM:
        disp_value = (alarm_hr * 100) + alarm_min;
        segsum(disp_value, colon, alarm_pm, d_mode);
        break;
      default:                                  //Default is display clock.
        if(tune_tim > 0){
          segsum(current_fm_freq / 10, 0, 0, d_mode);
        } else{
          disp_value = (hr * 100) + min;
          segsum(disp_value, colon, pm, d_mode);  //break up the disp_value to 4, BCD digits in the array: call (segsum)
        }
        
        break;
    }
    
    PORTA = segment_data[digit_count];          //send 7 segment code to LED segments
    switch (d_mode){                            //Enter display mode switch
      case TIME_NOT_SET:                        //Check to see if the time is not set
        if(sec % 2){                            //If the time is not set, flash the display every second.
          PORTB = digit_display[digit_count];   //Send display digit to MUX
        } else {
          PORTB = dec6;
        }
        break;
      default:
        PORTB = digit_display[digit_count];     //Send display digit to MUX
    }
}//TIMER0_OVF_vect

ISR(TIMER1_COMPA_vect){
  PORTC ^= (1 << 6);
}

ISR(ADC_vect){
  brightness = ADC / 8;    //Map a 10 bit value to a 7 bit value.
}

ISR(USART0_RX_vect){
static  uint8_t  i;
  rx_char = UDR0;               //get character
  rx_buf[i++]=rx_char;          //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    i=0;  
  }
}

//******************************************************************************
//                          External Interrupt 7 ISR  
// Written by Roger Traylor                   
// Handles the interrupts from the radio that tells us when a command is done.
// The interrupt can come from either a "clear to send" (CTS) following most
// commands or a "seek tune complete" interrupt (STC) when a scan or tune command
// like fm_tune_freq is issued. The GPIO2/INT pin on the Si4734 emits a low
// pulse to indicate the interrupt. I have measured but the datasheet does not
// confirm a width of 3uS for CTS and 1.5uS for STC interrupts.
//
// I am presently using the Si4734 so that its only interrupting when the 
// scan_tune_complete is pulsing. Seems to work fine. (12.2014)
//
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){STC_interrupt = TRUE;}
//******************************************************************************

