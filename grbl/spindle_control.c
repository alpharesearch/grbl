/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


void spindle_init()
{    
  // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
  // combined unless configured otherwise.
  #ifdef VARIABLE_SPINDLE
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #endif     
  // Configure no variable spindle and only enable pin.
  #else  
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  #endif
  
  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
  #endif
  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
    #else
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
    #endif
  #endif  
}

#ifdef VARIABLE_SPINDLE
  uint8_t calculate_pwm_from_rpm(float rpm)
  {
     // TODO: Install the optional capability for frequency-based output for servos.
     #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
     rpm -= SPINDLE_MIN_RPM;
     if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent uint8 overflow
     return (uint8_t) floor( rpm*(255.0/SPINDLE_RPM_RANGE) + 0.5);
  }
#endif


void spindle_set_state(uint8_t state, float rpm)
{
  // Halt or set spindle direction and rpm. 
  if (state == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      if (state == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif

    #ifdef VARIABLE_SPINDLE
      // TODO: Install the optional capability for frequency-based output for servos.
      #ifdef CPU_MAP_ATMEGA2560
      	TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02 | (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER); // set to 1/8 Prescaler
        OCR4A = 0xFFFF; // set the top 16bit value
        uint16_t current_pwm;
      #else
        TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        
        /* Comments by Michael Franzl for grbl's laser modification.
         * 
         * Calculating the highest prescaler value possible that still is
         * suitable for 256-bit grayscale laser engraving. Having the lowest
         * possible PWM base frequency may help to use cheaper laser driver
         * circuits, and might be more 'easy' on the laser diode itself.
         * 
         * Laser engraving info is sent as tiny line fragments.
         * The maximum throughput speed of those line fragments is
         * limited at least by 2 factors:
         * 
         *   1. grbl's algorothms
         *   2. the serial connection
         * 
         * Assuming that grbl's algorithms are faster than the serial connection,
         * the bottleneck limitation now becomes the serial port bandwidth.
         * One "pixel" is defined by the minimum G-Code exchange with a
         * string length of bytes_per_pixel = 15:
         * 
         *     X0000.0S000\nok\n
         * 
         * Having a serial bit_rate=115200Hz, one can find the absolute maximum
         * pixels that can be drawn per second (8 symbol bits + 1 stop bit):
         * 
         *     max_px_per_sec = floor(baud_rate / 9 / bytes_per_pixel) = 853
         * 
         * Or the inverse:
         * 
         *     min_time_per_px = 1.172ms
         * 
         * Assuming a rather fine resolution of 10 pixels per millimeter (254 dpi),
         * 
         *     mm_per_px = 0.1
         * 
         * we get a maximum possible feed rate of
         * 
         *     max_mm_per_sec = mm_per_px * max_px_per_sec = 85.3
         * 
         * Or in mm per minute:
         * 
         *     max_mm_per_min = 5118
         * 
         * One pixel should receive at least one full PWM cycle, so
         * 
         *     pwm_time_per_px = min_time_per_px = 1.172ms
         * 
         * Next, we calculate the period of the available PWM signals,
         * using User Manual ATmega p. 158. Calculations for Arduino Uno
         * which has a 16MHz clock.
         * 
         * PWM_t is the period of the PWM signal
         * PWM_f is the base frequency of the PWM signal
         * 
         * CS = 0x00 No clock source
         * CS = 0x01 No prescaling, PWM_f = 16MHz/256/1 = 62500 Hz; PWM_t = 16µs
         * CS = 0x02 1/8 prescaler, PWM_f = 16MHz/256/8 = 7812.5 Hz; PWM_T = 128µs
         * CS = 0x03 1/64 prescaler, PWM_f = 16MHz/256/64 = 976.5625 Hz; PWM_t = 1.024ms
         * CS = 0x04 1/256 prescaler, PWM_f = 16MHz/256/256 = 244.1406 Hz; PWM_t = 4.096ms
         * 
         * We find that a presaler of 1/64 will give us PWM_t = 1024µs. So we get
         * 
         *     pwm_cycles_per_pixel = min_time_per_px / PWM_t = 1.14
         * 
         * Having 1.14 PWM cycles per pixel should guarantee that one pixel
         * gets enough black/white subsampling. Assuming a 50% duty cycle
         * of the PWM, the "line" that one PWM cycle draws on the workpiece is
         * 
         *     pwm_subsampling_line = mm_per_px / pwm_cycles_per_pixel / 2 = 43.8µm
         * 
         * Which is about 1/2 of the thickness of a human hair and of the
         * focal point of the laser of about 100µm.
         * 
         * Therefore, it is more than enough to set 1/64 prescaler below.
         */
        
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // 1/8 th prescaler
        uint8_t current_pwm;
      #endif

     
      #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
      else { 
        rpm -= SPINDLE_MIN_RPM; 
        if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
      }
      current_pwm = floor( rpm*(PWM_MAX_VALUE/SPINDLE_RPM_RANGE) + 0.5);
      #ifdef MINIMUM_SPINDLE_PWM
        if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
      #endif
      OCR_REGISTER = current_pwm; // Set PWM pin output
  
      // On the Uno, spindle enable and PWM are shared, unless otherwise specified.
      #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) 
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      #endif
      
    #else
      // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
      // if the spindle speed value is zero, as its ignored anyhow.      
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
    #endif

  }
}


void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}
