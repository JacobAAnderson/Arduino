/* The 5 different modes are:
 *     SLEEP_MODE_IDLE         -the least power savings 
 *     SLEEP_MODE_ADC
 *     SLEEP_MODE_PWR_SAVE
 *     SLEEP_MODE_STANDBY
 *     SLEEP_MODE_PWR_DOWN     -the most power savings
 */

#include <avr/sleep.h>
#include <avr/power.h> 

void setup() {
  
set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

sleep_enable();          // enables the sleep bit in the mcucr register so sleep is possible. just a safety pin 

// Power down the various components of the board
power_adc_disable();
power_spi_disable();
power_timer0_disable();
power_timer1_disable();
power_timer2_disable();
power_twi_disable();

sleep_mode();            // here the device is actually put to sleep!!

// sleep_disable();         // first thing after waking from sleep: disable sleep...

// power_all_enable();      // power up the board


}

void loop() {
  

}
