#include "motors.h"
#include "diag.h"
#include "state.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdbool.h>

#define F_CPU 10000000 /* 10MHz */
#include <util/delay.h>

#define NUM_CHANNELS 7  // 6 channels as outputs and one for the downtime

#define INT_MASK (TCA_SINGLE_CMP0_bm)

// Desired PWM values for each output
volatile uint16_t pwm_values[NUM_CHANNELS] = { 0, 0, 0, 0, 0, 0, 20000 };
volatile uint16_t pwm_values_buffered[NUM_CHANNELS] = { 0, 0, 0, 0, 0, 0, 20000 };

// Track which channel is being output
volatile uint16_t active_channel = 0;

// RX board has slightly wonky pin ordering to simplify layout
//
//   PA3 - Ch 6
//   PA4 - Ch 1
//   PA5 - Ch 2
//
//   PB0 - Ch 5
//   PB1 - Ch 4
//   PB2 - Ch 3

static uint8_t dummy = 0; // Dummy register for downtime

static volatile uint8_t* portclr_map[NUM_CHANNELS] =
{
    &(PORTA.OUTCLR),
    &(PORTA.OUTCLR),
    &(PORTB.OUTCLR),
    &(PORTB.OUTCLR),
    &(PORTB.OUTCLR),
    &(PORTA.OUTCLR),
    &(dummy),
};

static volatile uint8_t* portset_map[NUM_CHANNELS] =
{
    &(PORTA.OUTSET),
    &(PORTA.OUTSET),
    &(PORTB.OUTSET),
    &(PORTB.OUTSET),
    &(PORTB.OUTSET),
    &(PORTA.OUTSET),
    &(dummy),
};

static volatile uint8_t portmask_map[NUM_CHANNELS] =
{
    (1 << 4),
    (1 << 5),
    (1 << 2),
    (1 << 1),
    (1 << 0),
    (1 << 3),
    0,
};
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

    // Timer A in 16 bit mode
    // Setting pins in motors_loop and clearing in CMP0 interrupt
    uint16_t period_val = 20000;
    uint16_t cmp_val = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.INTCTRL = INT_MASK;
    TCA0.SINGLE.PERL = period_val & 0xff;
    TCA0.SINGLE.PERH = (period_val & 0xff00) >> 8;
    TCA0.SINGLE.CMP0L = (cmp_val & 0xff);
    TCA0.SINGLE.CMP0H = (cmp_val & 0xff00) >> 8;
    TCA0.SINGLE.INTFLAGS = 0;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;
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
  // Once the interrupt is disabled, we can start the next pulse
  if ((TCA0.SINGLE.INTCTRL & (INT_MASK)) != INT_MASK)
  {
    // Increment the channel counter
    if (++active_channel >= 6)//NUM_CHANNELS)
    {
      active_channel = 0;

      // If we're about to output channel 0, reload the latest set of values
      for (int channel_index = 0; channel_index < NUM_CHANNELS; ++channel_index)
      {
        pwm_values[channel_index] = pwm_values_buffered[channel_index];
      }
    }

    // Skip if the channel is set to 0 (failsafe)
    if (pwm_values[active_channel] == 0)
    {
      return;
    }

    cli();
    // Update the timer for the next pulse
    uint16_t cmp_val = pwm_values[active_channel] * 5;
    TCA0.SINGLE.CMP0L = (cmp_val & 0xff);
    TCA0.SINGLE.CMP0H = (cmp_val & 0xff00) >> 8;

    // Reset the timer and set the selected pin high with minimum latency
    TCA0.SINGLE.CNTL = 0;
    *portset_map[active_channel] = portmask_map[active_channel];
    TCA0.SINGLE.CNTH = 0;

    // Re-enable timer interrupt
    TCA0.SINGLE.INTCTRL = INT_MASK;
    sei();
  }
}

void motors_all_off()
{
  // Set all outputs to failsafe values - just flatline
  for (int channel_index = 0; channel_index < NUM_CHANNELS; ++channel_index)
  {
    pwm_values_buffered[channel_index] = 0;
  }
}

void set_pwm_outputs(uint16_t* sticks)
{
  // Buffer the new values for each channel
  uint16_t sum = 0;
  for (int channel_index = 0; channel_index < NUM_CHANNELS - 1; ++channel_index)
  {
    sum += sticks[channel_index];
    pwm_values_buffered[channel_index] = sticks[channel_index];
  }

  // Final channel is the downtime i.e. 20ms minus all others
  pwm_values_buffered[NUM_CHANNELS - 1] = 20000 - sum;
}

ISR(TCA0_CMP0_vect)
{
  // Clear interrupt flag
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;

  // Clear the current channel
  *portclr_map[active_channel] = portmask_map[active_channel];

  // Clear interrupt flag to signal motors_loop that we're ready for the next channel
  TCA0.SINGLE.INTCTRL = 0x00;
}

