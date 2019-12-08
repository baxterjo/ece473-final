


//Port E inital values and setup.  This may be different from yours for bits 0,1,6.

//                                      DDRE:  0 1 0 1 0 1 1 0
//              (^ edge int from radio) bit 7--| | | | | | | |--bit 0 USART0 RX
//           (shift/load_n for 74HC165) bit 6----| | | | | |----bit 1 USART0 TX
//                                      bit 5------| | | |------bit 2 (radio reset, active high)
//(TCNT3 PWM output for volume control) bit 4--------| |--------bit 3 (Unused)

DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
DDRE  |= 0x40; //Port E bit 6 is shift/load_n for encoder 74HC165
DDRE  |= 0x08; //Port E bit 3 is TCNT3 PWM output for volume
PORTE |= 0x04; //radio reset is on at powerup (active high)
PORTE |= 0x40; //pulse low to load switch values, else its in shift mode


//Given the hardware setup reflected above, here is the radio reset sequence.
//hardware reset of Si4734
PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
PORTE |=  (1<<PE2); //hardware reset Si4734 
_delay_us(200);     //hold for 200us, 100us by spec         
PORTE &= ~(1<<PE2); //release reset 
_delay_us(30);      //5us required because of my slow I2C translators I suspect
                    //Si code in "low" has 30us delay...no explaination given
DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt


//Once its setup, you can set the station and get the received signal strength.

fm_pwr_up();        //power up radio
while(twi_busy()){} //spin while TWI is busy 
fm_tune_freq();     //tune to frequency      

//retrive the receive strength and display on the bargraph display
while(twi_busy()){}                //spin while TWI is busy 
fm_rsq_status();                   //get status of radio tuning operation
static uint8_t rssi =  si4734_tune_status_buf[4]; //get tune status 
//redefine rssi to be a thermometer code
  if(rssi<= 8) {rssi = 0x00;} else
  if(rssi<=16) {rssi = 0x01;} else
  if(rssi<=24) {rssi = 0x03;} else
  if(rssi<=32) {rssi = 0x07;} else
  if(rssi<=40) {rssi = 0x0F;} else
  if(rssi<=48) {rssi = 0x1F;} else
  if(rssi<=56) {rssi = 0x3F;} else
  if(rssi<=64) {rssi = 0x7F;} else
  if(rssi>=64) {rssi = 0xFF;}