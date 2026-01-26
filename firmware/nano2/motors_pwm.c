#include "motors.h"
#include "diag.h"
#include "state.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>

#define F_CPU 10000000 /* 10MHz */
#include <util/delay.h>

// Use the full cycle to reduce the time spent in interrupts
#undef DUTY_MAX
#define DUTY_MAX 255

#define NUM_CHANNELS 6

// Desired PWM values for each output
uint16_t pwm_values[NUM_CHANNELS] = { 0, 0, 0, 0, 0, 0 };
uint16_t pwm_values_buffered[NUM_CHANNELS] = { 0, 0, 0, 0, 0, 0 };

// Count TCA0 periods
uint16_t period_count = 0;

static bool pin_check(char portname, PORT_t *port, uint8_t pin)
{
    uint8_t bm = 1<<pin;
    // Set the pin as an input
    port->DIRCLR = bm;
    // Wait a very tiny time
    _delay_ms(1);
    bool state = (port->IN & bm);
    if (state) {
        // High!
        diag_println("motor pin_check FAIL: port %c %d stuck high",
            portname, pin);
        return false;
    }
    return true;
}

static void motors_check()
{
    // Check that all of our outputs are working
    // Onboard pull down resistors should hold the signal lines all low.
    // If any are high, it is an error.
    // pwm outputs = PB0,1,2 and PA3,4,5
    bool ok = true;
    ok = ok && pin_check('A', &PORTA,3);
    ok = ok && pin_check('A', &PORTA,4);
    ok = ok && pin_check('A', &PORTA,5);
    ok = ok && pin_check('B', &PORTB,0);
    ok = ok && pin_check('B', &PORTB,1);
    ok = ok && pin_check('B', &PORTB,2);
    if (! ok) {
        diag_println("Motor pin stuck high"); // never returns
        diag_println("Continuing anyway");
    }
}

void motors_init()
{
    motors_check();
    // Set all the ports which need to be outputs, to be outputs.
    // should all be low initially.
    // pwm outputs = PB0,1,2 and PA3,4,5
    // PB0, PB1, PB2
    // clear initial output
    PORTB.OUTCLR = 1 << 0;
    PORTB.OUTCLR = 1 << 1;
    PORTB.OUTCLR = 1 << 2;
    PORTA.OUTCLR = 1 << 3;
    PORTA.OUTCLR = 1 << 4;
    PORTA.OUTCLR = 1 << 5;
    // set directions to out
    PORTB.DIRSET = 1 << 0;
    PORTB.DIRSET = 1 << 1;
    PORTB.DIRSET = 1 << 2;
    PORTA.DIRSET = 1 << 3;
    PORTA.DIRSET = 1 << 4;
    PORTA.DIRSET = 1 << 5;
    //
    // Timer A should be in split mode
    //
    // As CLK_PER is 3.33 mhz, 
    // HOWEVER, in split mode we have 2x 8-bit timers,
    // so we cannot count to 3000.
    //
    // We can count to 256 and tweak the compare values in interrupts
    uint8_t period_val = DUTY_MAX;
    // Enable split mode
    TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
    // Enable all comparators in split mode
    TCA0.SPLIT.CTRLB = 
        TCA_SPLIT_LCMP0EN_bm |
        TCA_SPLIT_LCMP1EN_bm |
        TCA_SPLIT_LCMP2EN_bm |
        TCA_SPLIT_HCMP0EN_bm |
        TCA_SPLIT_HCMP1EN_bm |
        TCA_SPLIT_HCMP2EN_bm;
    // Set the period - use the same period for both halves.
    TCA0.SPLIT.HPER = period_val;
    TCA0.SPLIT.LPER = period_val;
    // Keep the halves in sync
    TCA0.SPLIT.HCNT = 0;
    TCA0.SPLIT.LCNT = 0;
    // Finally, turn the timer on.
    // Previously:
    // Divide by 64, 3.333mhz / 64 =~ 52khz
    // 52khz / period_val =~ 260hz = PWM frequency.
    //
    // Now:
    // Divide by 2, 3.333mhz / 2 =~ 1.666mhz
    // 1.666mhz / 256 =~ 6.5khz periods =~ 153.6 microsecond periods
    // 8 periods is 1.228 milliseconds
    // 
    TCA0.SPLIT.CTRLA = 
        //TCA_SPLIT_CLKSEL_DIV64_gc | // divide sys. clock by 64 
        TCA_SPLIT_CLKSEL_DIV2_gc | // divide sys. clock by 2
        TCA_SPLIT_ENABLE_bm;
    // To set the PWM duty, we can now write to
    // TCA0.HCMP0, .HCMP1, .HCMP2, .LCMP0, .LCMP1, .LCMP2
    // these would have values of 0.. period_val
    // here are sample values
    // TCA0.SPLIT.LCMP0 = 80; // WO0 / PB0
    // TCA0.SPLIT.LCMP1 = 100; // WO1 / PB1
    // TCA0.SPLIT.LCMP2 = 120; // WO2 / PB2
    // TCA0.SPLIT.HCMP0 = 20; // WO3 / pin PA3
    // TCA0.SPLIT.HCMP1 = 40; // WO4 / PA4
    // TCA0.SPLIT.HCMP2 = 60; // W05 / PA5
    TCA0.SPLIT.HCMP0 = 0;
    TCA0.SPLIT.LCMP0 = 0;
    TCA0.SPLIT.LCMP2 = 0;
    TCA0.SPLIT.LCMP1 = 0;
    TCA0.SPLIT.HCMP2 = 0;
    TCA0.SPLIT.HCMP1 = 0;
}

// RX board has slightly wonky pin ordering to simplify layout
//
//   PA3 - Ch 6
//   PA4 - Ch 1
//   PA5 - Ch 2
//
//   PB0 - Ch 5
//   PB1 - Ch 4
//   PB0 - Ch 5

// Pin IDs
// Motor 1 = weapon
#define MOTOR_1F 0
#define MOTOR_1R 1
// Left drive
#define MOTOR_2F 2
#define MOTOR_2R 3
// Right drive
#define MOTOR_3F 4
#define MOTOR_3R 5

// Table mapping the motor ID to the compare register
// which controls its duty.
// TODO: Update to reorder the channels
static uint8_t volatile * motor_map[] = {
    & (TCA0.SPLIT.HCMP0), // 1F, PA3
    & (TCA0.SPLIT.LCMP0), // 1R, PB0
    & (TCA0.SPLIT.LCMP2), // 2F, PB2
    & (TCA0.SPLIT.LCMP1), // 2R, PB1
    & (TCA0.SPLIT.HCMP2), // 3F, PA5
    & (TCA0.SPLIT.HCMP1), // 3R, PA4
};
/*
static uint8_t enable_map[] = {
    TCA_SPLIT_HCMP0EN_bm, // 1F, PA3
    TCA_SPLIT_LCMP0EN_bm, // 1R, PB0
    TCA_SPLIT_LCMP2EN_bm, // 2F, PB2
    TCA_SPLIT_LCMP1EN_bm, // 2R, PB1
    TCA_SPLIT_HCMP2EN_bm, // 3F, PA5
    TCA_SPLIT_HCMP1EN_bm, // 3R, PA4
};
*/
static uint8_t volatile * invert_map[] = {
    & (PORTA.PIN3CTRL), // 1F, PA3
    & (PORTB.PIN0CTRL), // 1R, PB0
    & (PORTB.PIN2CTRL), // 2F, PB2
    & (PORTB.PIN1CTRL), // 2R, PB1
    & (PORTA.PIN5CTRL), // 3F, PA5
    & (PORTA.PIN4CTRL), // 3R, PA4
};

static void set_pin_duty(uint8_t signal_id, uint8_t duty)
{
    volatile uint8_t * compare_register=motor_map[signal_id]; 
    volatile uint8_t * invert_register=invert_map[signal_id]; 
    //uint8_t enable_mask=enable_map[signal_id];

    if (duty == 255)
    {
      *invert_register &= ~PORT_INVEN_bm;
      *compare_register = 0;
    }
    else
    {
      *invert_register |= PORT_INVEN_bm;
      *compare_register = duty;
    }
}

void set_motor_direction_duty(uint8_t motor_id, int16_t direction_and_duty)
{
    // Not a thing on this board
}

void enable_motor_brake(uint8_t motor_id)
{
    // Not a thing on this board
}

void motors_loop()
{

}

void motors_all_off()
{
    // Set all outputs to failsafe values
    for (int channel_index = 0; channel_index < NUM_CHANNELS; ++channel_index)
    {
      pwm_values[channel_index] = 1500;
    }

    set_pin_duty(0, 0);
    set_pin_duty(1, 63);
    set_pin_duty(2, 127);
    set_pin_duty(3, 128);
    set_pin_duty(4, 192);
    set_pin_duty(5, 255);

    // Enable the underflow interrupt for one channel (they should remain in sync)
    //TCA0.SPLIT.INTCTRL =
    //  TCA_SPLIT_LUNF_bm;
}

void set_pwm_outputs(uint16_t* sticks)
{
    /*
    static uint8_t counter = 0;

    if (counter != 0)
    {
      return;
    }

    ++counter;

    // Buffer the new values for each channel
    for (int channel_index = 0; channel_index < NUM_CHANNELS; ++channel_index)
    {
      pwm_values[channel_index] = sticks[channel_index];
    }

    // Enable the underflow interrupt for one channel (they should remain in sync)
    TCA0.SPLIT.INTCTRL =
      TCA_SPLIT_LUNF_bm;
    */
    set_pin_duty(0, 0);
    set_pin_duty(1, 63);
    set_pin_duty(2, 127);
    set_pin_duty(3, 128);
    set_pin_duty(4, 192);
    set_pin_duty(5, 255);
}

ISR(TCA0_LUNF_vect)
{
  ++period_count;

  // Start the next cycle
  if (period_count >= 130)
  {
    // Disable the interrupt
    //TCA0.SPLIT.INTCTRL =
    //  TCA_SPLIT_LUNF_bm;

    for (int channel_index = 0; channel_index < NUM_CHANNELS; ++channel_index)
    {
      pwm_values[channel_index] = 1500;//pwm_values_buffered[channel_index];
    }
    period_count = 0;
  }
  else
  {
    for (int channel_index = 0; channel_index < NUM_CHANNELS; ++channel_index)
    {
      uint16_t value = pwm_values[channel_index];

      if (value == 0) // Expired
      {
        set_pin_duty(channel_index, 255);
      }
      else if (value < 256) // Expiring this cycle
      {
        set_pin_duty(channel_index, value);
      }
      else  // Expiring on a future cycle
      {
        set_pin_duty(channel_index, 128);
        value -= 256;
      }
    }
  }
}
