/*

  This will have a much better experience if compiled with -O3 rather than -Os as appears to be the arduino IDE default.
  Change this in ~/.arduino15/packages/arduino/hardware/sam/x.y.z/platform.txt

  Arduino Due NMR based on Due lock-in amplifier
  April, 2020 Carl Michal

   Timers:
    Timer 2: Runs at MCLK/2, resets at 100 or 84 provided 500kHz pulses. no output pulses :-( maybe if TIOB2 worked? should per PA6 periph A (ard A4/D58)
    Timer 0: count at MCLK/2 up to 2^32-1. waveform output for pulses at phase = 0. TIOA0 PB25-B (ard D2)
    0,  2 start from SYNC, (TIOA and TIOB for timers 3,4,5 not available, also TIOA2).

   Interrupts:
    - ADC interrupt will service ADC and DAC (done-ish).
    - GPIO outputs on interrupt
    - watchdog interrupt
    - documentation.
  Pins:
     - start with AD7 (ard A0/D54 = A.16) as analog input.
       AD6 is A1. differential with either channel = 6 or 7 gives AD7-AD6

     - PB15 DAC0 (ard DAC0/D66) is sine wave output
     - latch pulse on TIOA0 PB25-B  (ard D2)
     - would have 500,000 sample pulses on TIOB2, PA6-A (ard A4) but doesn't work?
     - extra PWM output on PWMH2 = B.14 (ard D53)
     - pin 7 (C.23) as flag for interrupt active
     - pin 13 B.27 LED and general flag.

  XX TODO:

    - check  that gain is applied correctly to all channels.

  /////////////////////////////////
   To change main system clock
      - set clock.
      - TICKSPERSAMPLE and TICKSPERSAMPLE_ULL
      - PRESCAL clock for ADC - must be no more than 22 MHz.
      - optional: could use /8 prescaler for timer 2, and set RC at 21 rather than 84.


  /////////////////
  A few variables:
  rphase and tphase are the phases of the numerical oscillator for the receiver and transmitter.
  tphase is generally two DMA's ahead of rphase.

  rftw and tftw are the frequency tuning words for receiver and transmitter.


  port C pins for GPIO outputs
     C.0 =
      C.1 = D33
      C.2 = D34
      C.3 = D35
      C.4 = D36
      C.5 = D37
      C.6 = D38
      C.7 = D39
      C.8 = D40
      C.9 = D41
      C.10 =   OPCODE0
      C.11 =   OPCODE1
      C.12 = D51
      C.13 = D50
      C.14 = D49
      C.15 = D48
      C.16 = D47
      C.17 = D46
      C.18 = D45
      C.19 = D44
      C.20 = OPCODE2
      C.21 = D9
      C.22 = D8
      C.23 = D7 // USED as interrupt duration flag.
      C.24 = D6
      C.25 = D5
      C.26 - D4/D87
      C.27 =
      C.28 = D3
      C.29 = D10/D77
      C.30 = D72 (not easy)
      C.31 =

*/

// shouldn't be necessary if compiled with -O3.
//#define USE_100MHZ


// worth checking the duration of the ADC interrupt on pin 7. Should be 65us with 84MHz or 55us at 100MHz
// if not, check alignment and fudges at start of ADC_Handler

// define this to get output bits at start of event time, after interrupt, latch trigger will be too late,
// but stuff should be usable without latch chip, but with interrupt jitter. And first event will start
// early.
#define NOLATCH

#define LOOP_LEVELS 4
#define MAX_EVENT_WORDS 14000

// these are for pin 7
#define FLAG_ON   PIOC->PIO_SODR = (1 << 23);
#define FLAG_OFF   PIOC->PIO_CODR = (1 << 23);

// limits for ADC to report over/underflow
#define ADC_LOW 20
#define ADC_HIGH 4075
// For receive, table is called sin_table, it has 32768 elements.
#define NUM_TABLE 32768
// For transmit, table is tsin_table, has 16384 elements
#define TNUM_TABLE 16384
#include "sine_table.h"

#define DAC_OFFSET 50 // 25 ticks from dac trigger till result? Somehow 50 looks better.
// DAC_OFFSET is a delay added to ref_ticks - so it delays the ADC and DAC relative to input capture and output pulses

#define PULSE_DURATION 50 // for pulse output. Measured in 42 MHz or 50 MHz ticks depending on clock speed.


#define RF_ON 0x80000000

#define DMALEN 50

#ifdef USE_100MHZ
#define TICKSPERSAMPLE 100
#define TICKSPERSAMPLE_ULL 100ULL
#define CLOCK 100000000
#else
#define TICKSPERSAMPLE 84
#define TICKSPERSAMPLE_ULL 84ULL
#define CLOCK 84000000
#endif



int16_t outbuff[256]; // buffer for output.
uint16_t dac_buf[2][DMALEN]; // dac DMA buffer
int16_t adc_buf[2][DMALEN]; // adc DMA buffer
volatile uint16_t  adc_buf_num = 1; // which buffer is current

volatile uint16_t load_pos = 0; // load_pos and sent_pos point where we're reading and writing in the outbuff
uint16_t sent_pos = 0, adc_channel = 7; // the initial adc_channel. AD7 is A0.

uint16_t *dac_table; // pointer to the current table used for dac output
uint16_t dac_down[TNUM_TABLE]; // downloaded values.

// bits of status and flag bytes:

#define STATUS_RECEIVERON (1<<0)
#define STATUS_R_RELOADED (1<<1)
#define STATUS_T_RELOADED (1<<2)
#define STATUS_LAST_EVENT (1<<3) // time to reload
#define STATUS_SHUTDOWN (1<<4) // if we let it end, or reload not done in time.
#define STATUS_RUNNING (1<<5)
#define STATUS_USE_DOWNLOADED (1<<6) // may disappear ?
#define STATUS_MARKER (1<<7)

#define FLAG_GAIN ((1<<0)|(1<<1)) // uses two bits.
#define FLAG_DIFFERENTIAL (1<<2) // differential input
#define FLAG_OFFSET (1<<3)
#define FLAG_INPUT_OVERFLOW (1<<4)
#define FLAG_OUT_BUFF_OVERRUN (1<<5)
#define FLAG_DMA_XRUN (1<<6)
// these are reset after reporting:
#define FLAG_MASK (FLAG_INPUT_OVERFLOW|FLAG_OUT_BUFF_OVERRUN|FLAG_DMA_XRUN)

#define TELL_FLAG_LAST 1
#define TELL_FLAG_SHUTDOWN 2
#define TELL_FLAG_DONEDATA 4

#define LOOP_START (1 << 28)
#define LOOP_END (1 << 29)
volatile uint16_t status =  STATUS_MARKER, flags = 0, tell_flags = 0;

// phases and frequency tuning words.
// initial value is ~ 500 Hz.
volatile uint32_t tphase = 0, rphase = 0;
volatile uint32_t tftw = 82949670, rftw = 82949670;
//uint32_t ticks_per_period = (TICKSPERSAMPLE_ULL << 32) / tftw;


volatile unsigned int tsample = 0, rsample = 0; // sample numbers for transmit/receive
volatile unsigned int tnextstart, rnextstart;
unsigned long trans_phase = 0; // a phase shift applied to the transmitter by the current event, so we can subtract it off later.


// phase reference values from last dac buffer fill:


// for pwm on D53 = PWMH2 = B.14
int pwm_chan = 2;

volatile uint16_t watchdog_trigger = 0;

// timers start at 0. program starts a bit later:
#define GOFFSET 2*DMALEN*TICKSPERSAMPLE
#define TOFFSET 2*DMALEN

unsigned long program[MAX_EVENT_WORDS];
uint16_t gindex = 0, tindex, tindex0, recindex, recindex0;
// contains GPIO program, then transmit program, then receive program

void setup() {
  // stuff in here only ever called once.
  int i;
#ifdef USE_100MHZ
  //// set clock to go at 100 MHz:
#define SYS_BOARD_PLLAR (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(49UL) | CKGR_PLLAR_PLLACOUNT(0x3fUL) | CKGR_PLLAR_DIVA(3UL))
#define SYS_BOARD_MCKR ( PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK)
  //Set FWS according to SYS_BOARD_MCKR configuration
  EFC0->EEFC_FMR = EEFC_FMR_FWS(4); //4 waitstate flash access
  EFC1->EEFC_FMR = EEFC_FMR_FWS(4);
  // Initialize PLLA
  PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
  while (!(PMC->PMC_SR & PMC_SR_LOCKA));
  //PMC->PMC_MCKR = SYS_BOARD_MCKR;
  PMC->PMC_MCKR = (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK);
  while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
  SystemCoreClockUpdate();
#endif

  // turn off systick. Upload over native USB won't work.
  //SysTick->CTRL &= ~2;

  SerialUSB.begin(0);
  SerialUSB.write((char *)outbuff, 64); // send an empty packet to make sure stuff is initialized.
  SerialUSB.write((char *)outbuff, 64); // send an empty packet to make sure stuff is initialized.

  // lazy
  pinMode(13, OUTPUT); //LED
  pinMode (7, OUTPUT); //used as a flag to time stuff on a scope. - this is a port C pin...
  pinMode(33, OUTPUT);

  pmc_enable_periph_clk(ID_PIOA);
  pmc_enable_periph_clk(ID_PIOB);
  pmc_enable_periph_clk(ID_PIOC);
  pmc_enable_periph_clk(ID_PIOD);

  ///////////////// Configure our main timer - will drive DAC and ADC.
  // set up TC0, channel 2 as DACC/ADC trigger
  /* turn on the timer clock in the power management controller */
  pmc_enable_periph_clk(ID_TC2);     // enable peripheral clock TC0

  // wavesel mode, match at RC and reset
  // use TIMER_CLOCK2 = MCLK/8 or TIMER_CLOCK1 = MCLK/2
  TC_Configure(/* clock */TC0,/* channel */2, TC_CMR_WAVE
                          | TC_CMR_WAVSEL_UP_RC
                          | TC_CMR_TCCLKS_TIMER_CLOCK1
                          | TC_CMR_ACPA_CLEAR
                          | TC_CMR_ACPC_SET
                          // | TC_CMR_BCPB_CLEAR
                          // | TC_CMR_BCPC_SET // no point, doesn't work.
              );

  TC_SetRC(TC0, 2, TICKSPERSAMPLE );// duration of event will be RC.  is 500,000/s (100 MHz/ 2 / 100)
  TC_SetRA(TC0, 2, 1); // output is turned on at RC match, set off at RA match

  // To start the timer (do later):
  //  TC0->TC_CHANNEL[2].TC_CCR = 5; // enable and start. (just 1 enables but does not start - needs external trig to start in that case.)
  // TIOB2 should be on PA6-A (ard A4): But this doesn't work.
  //PIO_Configure(PIOA, PIO_PERIPH_A, 1 << 6, INPUT_PULLUP); // is pullup allowed here?
  // no output pin?
  // ok, so now have a clock running on TIOA2.

  /////////// Auxiliary PWM for capture testing or whatever.
  //set a pwm on pin 53 = PWMH2 = B.14
  // D53 = B.14 = PWMH2 need to do more work ourselves
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  // on B.14, timer output is periph_B
  PIO_Configure(PIOB, PIO_PERIPH_B, 1 << 14, 0);

  // ~500 Hz with 1MHz clock:
  PWMC_SetPeriod(PWM_INTERFACE, pwm_chan, 2001);
  PWMC_SetDutyCycle(PWM_INTERFACE, pwm_chan, 1000);

  PWMC_ConfigureChannel(PWM_INTERFACE, pwm_chan, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_EnableChannel(PWM_INTERFACE, pwm_chan);
  PWMC_ConfigureClocks(1000000, 0, CLOCK); // ask for 1 MHz clock.

  my_setup();

  //////// set interrupt priorities - USB can't interrupt DACC and ADC:
  NVIC_SetPriority(ADC_IRQn, 4); // our ADC interrupt.
  NVIC_SetPriority(UOTGHS_IRQn, 5); // USB. lower than DAC and ADC
  NVIC_SetPriority(TC0_IRQn, 2); // latch pulses
  NVIC_SetPriority(TC3_IRQn, 0); // Watchdog timer on TC3. Highest priority.

  // build a testing pulse sequence
  gindex = 0;
  program[gindex++] = 120 * TICKSPERSAMPLE; // duration in 42 MHz ticks
  program[gindex++] = 0;

  program[gindex++] = 8400;
  program[gindex++] = (1 << 25); // D5

  program[gindex++] = 8400;
  program[gindex++] = (1 << 24); // D6

  program[gindex++] = 8400;
  program[gindex++] = (1 << 24) | (1 << 25); // D6

  program[gindex++] = 4200;
  program[gindex++] = (1 << 25) | (1 << 10); // D5 + loop start (1<<10)
  program[gindex++] = 5;


  program[gindex++] = 4200;
  program[gindex++] = (1 << 11); // loop end (1<<11)

  program[gindex++] = 8400;
  program[gindex++] = (1 << 24) | (1 << 25);

  program[gindex++] = 8400;
  program[gindex++] = 0;

  program[gindex++] = (50 * 128 + 500 ) * TICKSPERSAMPLE; // the 500 is extra
  program[gindex++] = 0;

  program[gindex++] = TICKSPERSAMPLE * 300;
  program[gindex++] = (1 << 10) | (1 << 11); // has end event flagged.

  // for gpio, delay is 32 bits of 83 MHz (or 100 MHz) clock ticks.
  // for transmit and receive delays are 500 kHz samples.
  // now transmit: do all 32 bit words.
  // first word is delay (29 bits), top bit is transmit on/off
  // next two bits are normal/loop start/loop end
  // if transmit is on, then there's a 32 bit phase.

  tindex0 = gindex; // start of transmit

  // TRANSMIT
  program[gindex++] = 120; // 100 samples

  program[gindex++] = 100 | RF_ON; // transmit on
  program[gindex++] = 0; // phase

  program[gindex++] = 100 | RF_ON; // transmit on
  program[gindex++] = 1 << 31; // phase

  program[gindex++] = 100 ; // transmit off
  //program[gindex++] = RF_ON; // phase

  program[gindex++] = 50 | RF_ON | (LOOP_START); // transmit on loop start
  program[gindex++] = 0; // phase
  program[gindex++] = 5;

  program[gindex++] = 50 | (LOOP_END); // transmit off , loop end

  program[gindex++] = 100 | RF_ON; // transmit on
  program[gindex++] = 0; // phase

  program[gindex++] = 100; // transmit off

  program[gindex++] = (50 * 128);

  program[gindex++] =  300 | (3 << 28) ;

  // RECEIVE
  // finally receive: all 32 bit words - delays as for transmit
  recindex0 = gindex;
  program[gindex++] = 420;
  program[gindex++] = 50 | (LOOP_START);
  program[gindex++] = 5;

  program[gindex++] = 50 | (LOOP_END);
  program[gindex++] = 200;
  program[gindex++] = 15 * 128 | RF_ON;
  program[gindex++] = 300 | (3 << 28);

  // run the show for a little while, then kill
  start_pulses();
  while (rsample < 100);
  safe_state();
  watchdog_trigger = 0;
  load_pos = sent_pos;

}

void my_setup() {
  int i;
  //uint32_t tval;

  //////////////////DAC setup:

  // dac pins are B.15 and B.16. only use B.15 ?
  pmc_enable_periph_clk(DACC_INTERFACE_ID); // enable the DACC clock - MCLK/2
  //dacc_reset(DACC_INTERFACE); // reset the dac
  DACC->DACC_CR = DACC_CR_SWRST;
  /*
    dacc_set_transfer_mode(DACC_INTERFACE, 0); // 1 is full word, 0 is half word

    dacc_set_power_save(DACC_INTERFACE, 0, 0); // sleep mode, fast wakeup both off
    dacc_set_timing(DACC_INTERFACE, 0, 1, 47); // refresh time, max speed mode (on) and startup time.
    // want 50 us of startup time. 0x10 is 1024 DACClock periods. 50kHz.  47 is 3008 periods.
    dacc_enable_flexible_selection(DACC_INTERFACE);
  */
  //DACC->DACC_MR = DACC_MR_STARTUP_1984 | DACC_MR_TAG; // this might not be long enough startup time?
  //DACC->DACC_MR = DACC_MR_TAG | DACC_MR_REFRESH(2); // this might not be long enough startup time?
  //  DACC->DACC_MR = DACC_MR_TAG; // this might not be long enough startup time?

  // set channel:
  DACC->DACC_MR = DACC_MR_USER_SEL_CHANNEL1;
  dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  // enable selected channel:
  dacc_enable_channel(DACC_INTERFACE, 1);
  //dacc_enable_channel(DACC_INTERFACE, 1);

  /*
    // do one conversion to get startup time out of the way?
    i = DACC->DACC_ISR;
    DACC->DACC_CDR = 2048;
    // wait for conversion to complete
    while (DACC->DACC_ISR & DACC_ISR_EOC == 0);
  */
  DACC->DACC_PTCR =  0x0202;  //TXTEN, RXTEN  disable transmit, disable receive
  dacc_set_trigger(DACC, 3); // to TIO output of TC channel 2

  // fill the buffers before starting - similar to what is done in the interrupt
  for (i = 0 ; i < DMALEN; i++) {
    dac_buf[0][i] = 2048;
    dac_buf[1][i] = 2048;
  }
  // start off with tphase pointing at 3rd buffer
  // set up DMA for dac:
  DACC->DACC_TPR  =  (uint32_t)  dac_buf[0];      // DMA buffer
  DACC->DACC_TCR  =  DMALEN;
  DACC->DACC_TNPR =  (uint32_t)  dac_buf[1];      // next DMA buffer
  DACC->DACC_TNCR =  DMALEN;
  DACC->DACC_PTCR =  0x00000100;  //TXTEN, RXTEN  enable transmit,


  /////////////////// ADCC configuration:
  // power on:
  pmc_enable_periph_clk(ID_ADC); // enable the DACC clock
  ADC->ADC_CR = ADC_CR_SWRST; // reset the adc
  ADC->ADC_MR = ADC_MR_TRGEN_EN // enable external trigger
                | ADC_MR_TRGSEL_ADC_TRIG3 // set trigger source to TIOA of channel 2
#ifdef USE_100MHZ
                | ADC_MR_PRESCAL(2)   // ADCClock = MCK/((PRESCAL+1)*2) 8 bits of prescale available. min is mck/2
#else
                | ADC_MR_PRESCAL(1)
#endif
                // set this to 1 to get 84MHz /4 = 21 MHz.
                // PRESCAL is 2 for 100MHz sysclock, and 1 for 84MHz.
                | ADC_MR_TRACKTIM(0 ) // tracking time = (TRACKTIM+1)* clock period. Want ~ 250ns or more. 4 bits
                // woah.
                // https://forum.arduino.cc/index.php?topic=310557.0
                // says TRACTIM is borked. you always get 15 cycles.
                | ADC_MR_TRANSFER(1); // transfer priod is TRANSFER*2+3) * clock period. 2 bits a 1 here is 5 periods.

  // no startup time - seems phase is best if there's a small starup on the DAC, but none here.
  // would add something like ADC_MR_STARTUP_SUT24;
  // to work ok without it.
  // startup needs 40us.
  // need less than 42 ADC cycles per conversion. Transfer is 5, conversion is 25.

  // 84 MHz/4 = 21 MHz, 100 MHz/6 = 16.667 MHz for PRESCAL = 1 or 2 at 84 and 100 MHz.

  // configure ADC input channel:
  // we'll use user sequence mode, where each 4 bits in SEQR1 and SEQR2 specifies a channel.
  // we set all the unused channels to F so no physical pins get set up as ADC inputs.
  // Strangely, sometimes this seemed to be a problem - and the input capture pin would get taken over by the ADC
  // even more strangely, with this configuration, you can use the input capture pin (A7) for both simultaneously!
  ADC->ADC_SEQR2 = 0xFFFFFFFF;
  ADC->ADC_SEQR1 = adc_channel << 28 | 0x0FFFFFFF;

  ADC->ADC_MR |= ADC_MR_USEQ;
  ADC->ADC_CHER = 0x80; // only one channel in the sequence is enabled. Not the first two channels,
  // they clobber the input capture pin!
  // For reasons I can't fathom - if we put the enabled channel number into the lowest 4 bits
  // of the ADC_SEQR1, pin A7 (AD0 = TIOA1) gets disconnected from the timer.
  // its very weird though. Don't see why. Here we use element 7 - which would correspond to AD7
  // (A0) which is the one we're defaulting to anyway.
  // We use the user sequence mode so that we can change channels without ever having to
  // have 0 or more than 1 channel enabled at once.

  i = ADC->ADC_LCDR;   // read data out of last converted data register:
  ADC->ADC_CR = ADC_CR_START; // start the adc
  // make sure the ADC is complete?
  while (ADC->ADC_ISR & ADC_ISR_DRDY == 0);
  i = ADC->ADC_LCDR; // read the data again.

  ADC->ADC_IER = ADC_IER_ENDRX; // interrupt when receiver buffer is done.
  ADC->ADC_PTCR =  0x0202;  //TXTEN, RXTEN  disable transmit, disable receive

  // set up DMA:
  ADC->ADC_RPR = (uint32_t) adc_buf[0];
  ADC->ADC_RCR = DMALEN;
  ADC->ADC_RNPR = (uint32_t) adc_buf[1];
  ADC->ADC_RNCR = DMALEN;
  ADC->ADC_PTCR = 0x0001; // enable receive, triggers.

  // set up comparison to trigger on high or low values

  ADC->ADC_EMR = ADC_EMR_CMPMODE_OUT  // turn on comparisons - event when value is outside of window.
                 | ADC_EMR_CMPALL; // compare on all channels
  // thresholds in ADC_CWR:
  ADC-> ADC_CWR = ADC_LOW | (ADC_HIGH << 16);

  NVIC_DisableIRQ(ADC_IRQn);
  NVIC_ClearPendingIRQ(ADC_IRQn);
  NVIC_EnableIRQ(ADC_IRQn);

  // setup  ADC state
  if ( flags & FLAG_DIFFERENTIAL) // offset mode is on, so center is always 1/2 of power supply.
    ADC->ADC_COR |= 0xFFFF0000;
  if (flags & FLAG_OFFSET)
    ADC->ADC_COR |= 0x0000FFFF;

  ADC->ADC_CGR = flags & FLAG_GAIN; // first two bits are the CH0 gain, which are applied everywhere.


  // PORT C GPIO outputs
#define DEFAULT_OUTPUT_C (1<<19)
  // pio control:
  PIOC->PIO_PER = 0x37eff3fe;
  // allow direct writes:
  PIOC->PIO_OWER = 0x37eff3fe & ~(1 << 23); // use 23 as FLAG, remove for now

  // set default values
  PIOC->PIO_ODSR = DEFAULT_OUTPUT_C;
  // enable outputs
  PIOC->PIO_OER = 0x37eff3fe; //25 bits

  // force an output latch pulse:
  //digitalWrite(2, 1);
  //digitalWrite(2, 0);
  /* strangely, the ADC interrupt is much longer if I get rid of the two previoud digitalWrite's...
    sometimes, but not others...*  */
  PIO_Configure(PIOB, PIO_OUTPUT_0, 1 << 25, 0);
  PIOB->PIO_SODR = 1 << 25;
  asm volatile("nop\n\tnop\n\t");
  PIOB->PIO_CODR = 1 << 25;

  ///////////////// Configure Timer 2 for phase 0 output pulses
  pmc_enable_periph_clk(ID_TC0);     // enable peripheral clock
  TC_Configure(/* clock */TC0,/* channel */0,
                          TC_CMR_WAVE |
                          TC_CMR_WAVSEL_UP | // count up
                          TC_CMR_TCCLKS_TIMER_CLOCK1 | // at MCK/2
                          //TC_CMR_ACPA_SET | // set A // don't do it till turned on. Otherwise, we'll output a pulse every ~85s.
                          TC_CMR_ACPC_CLEAR);
  // output pin is B.25, arduino pin D2
  PIO_Configure(PIOB, PIO_PERIPH_B, 1 << 25, 0);

  //TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // interrupt on RC match
  //schedule the first output pulse - start at RA, end at RC ( 5 ticks later).
  // how many ticks till we get to phase 0 again?
  // freq = 500,000 * ftw/2^32, so period is 2^32/ftw/500000. Measure period in MCK/2 ticks = 50 MHz.
  // so period is 2^32/ftw/500000 * (MCK/2) so period, (in timer ticks) is:
  // 2^32*TICKSPERSAMPLE/ftw

  NVIC_ClearPendingIRQ(TC0_IRQn);
  NVIC_EnableIRQ(TC0_IRQn);
  //////////////// Configure Timer 3 as watchdog timer:
  pmc_enable_periph_clk(ID_TC3);     // enable peripheral clock TC0

  // wavesel mode, match at RC and reset
  // use TIMER_CLOCK2 = MCLK/8 or TIMER_CLOCK1 = MCLK/2, 3 is /32, 4 is / 128
  TC_Configure(/* clock */TC1,/* channel */0, TC_CMR_WAVE
                          | TC_CMR_WAVSEL_UP_RC
                          | TC_CMR_TCCLKS_TIMER_CLOCK4
              );
  // counts at 328.125 kHz or 390.625 kHz.
  TC_SetRC(TC1, 0, TICKSPERSAMPLE * 390 ); // this is around 0.1s
  TC1->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // interrupt on RC match

  NVIC_ClearPendingIRQ(TC3_IRQn);
  NVIC_EnableIRQ(TC3_IRQn);



  //////////// Enable the timers:
  //  TC0->TC_CHANNEL[0].TC_CCR = 1; // (just 1 enables but does not start - needs external trig to start in that case.)
  //  TC0->TC_CHANNEL[2].TC_CCR = 1;

  //////// start all timers 0, 1, 2 simultaneously.
  //  TC0->TC_BCR = 1;



  sent_pos = 0;
  load_pos = 0;



}

void loop() {
  int i;
  uint32_t fval;
  uint16_t * ptr_dest, sval;
  char mess[64];
  char command;
  uint8_t diff;

  //FLAG_ON //1.18us, 2.48 with IC on. then 1.9us with IC by adding switch


  ////////// Look for watchdog trigger:
  if (watchdog_trigger != 0) {
    sprintf(mess, "WATCHDOG TRIGGER %i\n", watchdog_trigger);
    my_send_message(mess);
    // if watchdog_trigger & 2 is set, we need to restart!

    safe_state();
    watchdog_trigger = 0;
    sent_pos = load_pos;
  }
  ////////// Transmit waiting data to host:
  diff = load_pos - sent_pos;
  if ( diff >= 30 || ((diff > 0) && (tell_flags & TELL_FLAG_DONEDATA))) {
    //FLAG_ON
    // wait till we can load:
    while ( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[CDC_TX] & UOTGHS_DEVEPTISR_TXINI ));
    // copy data in:
    ptr_dest =  (uint16_t *) &udd_get_endpoint_fifo_access8(CDC_TX);

    __disable_irq(); // don't let the flags update while we load and reset:
    *ptr_dest++ = status | (((uint16_t)flags) << 8);
    // turn off error flag bits
    flags &= ~FLAG_MASK;
    __enable_irq();

    if (diff > 30) diff = 30;
    *ptr_dest++ = diff; // how many 16-bit data words there are.
    for (i = 0; i < diff; i++)
      *ptr_dest++ = outbuff[(sent_pos++) & 0xff];
    for (i = diff; i < 30; i++) // fill up the buffer if it wasn't
      *ptr_dest++ = 0;
    UOTGHS->UOTGHS_DEVEPTICR[CDC_TX] = UOTGHS_DEVEPTICR_TXINIC;
    UOTGHS->UOTGHS_DEVEPTIDR[CDC_TX] = UOTGHS_DEVEPTIDR_FIFOCONC;
    //FLAG_OFF
    if ((tell_flags & TELL_FLAG_DONEDATA) && (sent_pos == load_pos)) {
      __disable_irq();
      tell_flags &= ~TELL_FLAG_DONEDATA;
      __enable_irq();
    }

  }

  if (tell_flags & TELL_FLAG_LAST) {
    my_send_message("LAST EVENT");
    __disable_irq();
    tell_flags &= ~TELL_FLAG_LAST;
    __enable_irq();
  }
  if (tell_flags & TELL_FLAG_SHUTDOWN) {
    my_send_message("SHUTDOWN");
    __disable_irq();
    tell_flags &= ~TELL_FLAG_SHUTDOWN;
    __enable_irq();

  }
  if (SerialUSB.available() != 0) {
    static char led = 0;
    command = SerialUSB.read();
    led = led ^ 1;
    digitalWrite(13, led);
    switch (command) {
      case 'Q':
        my_send_message("Due NMR v1.0.0");
        break;
      case 'F': // set frequency of transmitter and receiver together.
        SerialUSB.readBytes((char *) &fval, 4);

        //digitalWrite(13, HIGH);


        __disable_irq();
        // fixing the phase is actually a little tricky. Reading is always
        // two buffers behind writing. So to fix the read phase, we subtract off two buffers worth of the new
        // ftw.

        // ok. To set new ftw, can either set tftw, rftw,
        rphase = tphase - (2 * DMALEN ) * fval;
        tftw = fval;
        rftw = fval;
        //ticks_per_period = (TICKSPERSAMPLE_ULL << 32) / tftw;
        __enable_irq();
        sprintf(mess, "FTW: %i", fval);
        my_send_message(mess);
        //digitalWrite(13, LOW);
        break;
      case 'f': // return rftw
        {
          sprintf(mess, "F: %i", rftw);
          my_send_message(mess);
        }
        break;
      case 'T': // set transmit frequency only. Just update tftw
        SerialUSB.readBytes((char *) &fval, 4);
        __disable_irq();
        tftw = fval;

        //ticks_per_period = (TICKSPERSAMPLE_ULL << 32) / tftw;
        __enable_irq();
        // if pulses are active, restart: XX TODO


        break;
      case 't': // return tftw
        {
          sprintf(mess, "T: %i", tftw);
          my_send_message(mess);
        }
        break;
      case 'U': // use downloaded waveform
        __disable_irq();
        status |= STATUS_USE_DOWNLOADED;
        __enable_irq();
        dac_table = dac_down;
        break;
      case 'u': // don't use downloaded waveform
        __disable_irq();
        status &= ~STATUS_USE_DOWNLOADED;
        __enable_irq();
        dac_table = (uint16_t *) tzero_table; // XXX FIXME?
        break;

      case 'C': // set ADC input channel. Just give a single byte as ADx where x is the channel num.
        // want x = 0 to 14. Probably don't want 0 (input capture pin) - though it does work.
        //  15 points at a temperature sensor - which we don't turn on.
        //See 43.7.5 in Atmel manual.
        {
          uint8_t chan;
          SerialUSB.readBytes((char *) &chan, 1);
          chan = chan & 0xF;
          ADC->ADC_SEQR1 = chan << 28 | 0x0FFFFFFF;
          adc_channel = chan;
        }
        break;
      case 'c':// return the channel number register.
        {
          sprintf(mess, "C: %i", adc_channel);
          my_send_message(mess);
        }
        break;
      case 'D':
        // download program. we start with 3 16 bit numbers that are the number of
        // 32 bit words in each segment - gpio first, then transmit, then receive.
        // so we should either be stopped or LAST_EVENT flag should be on.
        // when transmitting, send those three 16bit words, flush, then send in bulk.
        uint16_t words;
        SerialUSB.readBytes((char *) &tindex0, 2); // length of gpio is start is transmit
        SerialUSB.readBytes((char *) &recindex0, 2); // length of transmit
        SerialUSB.readBytes((char *) &words, 2); // length of receive
        words += tindex0 + recindex0;
        recindex0 += tindex0;
        sprintf(mess, "%i", words);
        my_send_message(mess);

        for (i = 0; i + 16 < words ; i += 16) { // get 64 bytes at a time, 32 samples.
          SerialUSB.readBytes((char *) (program + i ), 64);
          //sent_pos = load_pos; // do this periodically so the watchdog doesn't get triggered.
          sprintf(mess, "%i of %i\n", (i + 16), words);
          my_send_message(mess);

        }
        SerialUSB.readBytes((char *) (program + i), (words - i)  * 4);
        sprintf(mess, "DONE %i", words);
        my_send_message(mess);
        /*for (i = 0; i < words; i++)
          SerialUSB.readBytes((char *) (program + i), 4);
        */

        if (status & STATUS_RUNNING) {
          __disable_irq();
#ifndef NOLATCH

          PIOC->PIO_ODSR = program[0]; // set the bits for the next event
#endif
          status &= ~STATUS_LAST_EVENT;
          status |= STATUS_T_RELOADED | STATUS_R_RELOADED;
          __enable_irq();
        }

        // if we were stopped, no problem. if we were running
        // its possible that we were too late and program was killed by an interrupt that occured during download
        // or while we were resetting for the first event. The GPIO interrupt might have stopped the timer.


        break;
      case 'S': // kill (safe state)
        safe_state();
        __disable_irq();
        status &= ~STATUS_RUNNING;
        __enable_irq();
        watchdog_trigger = 0;
        sent_pos = load_pos;
        break;
      case 'Y':  // produce pulses on D2 at start of each period
        //SysTick->CTRL &= ~2;
        program[recindex0] += 2 * DMALEN;
        start_pulses();
        __disable_irq();
        rphase -= 2 * DMALEN * rftw;
        __enable_irq();
        program[recindex0] -= 2 * DMALEN;
        break;
      /* case 'y': // don't produce those pulses:
        //SysTick->CTRL |= 2;
        TC0->TC_CHANNEL[0].TC_IDR = TC_IDR_CPCS; // don't interrupt on RC match
        TC0->TC_CHANNEL[0].TC_CMR &= ~TC_CMR_ACPA_SET; // don't turn on the pulse on RA match
        // TODO fix me. STATUS too?
        break; */
      case 'P': // set the pwm period. 16 bits. freq = 1MHz/period.
        SerialUSB.readBytes((char *) &sval, 2);
        PWMC_SetPeriod(PWM_INTERFACE, pwm_chan, sval);
        sval /= 2;
        PWM_INTERFACE->PWM_CH_NUM[pwm_chan].PWM_CDTYUPD = sval;
        //PWMC_SetDutyCycle(PWM_INTERFACE, pwm_chan, sval); // don't use this. It will assert and hang...
        break;
      case 'p': // return period of PWM. freq is 1000000/period
        sprintf(mess, "P: %i", PWM_INTERFACE->PWM_CH_NUM[pwm_chan].PWM_CPRD);
        my_send_message(mess);
        break;
      case 'K': // set clock for PWM
        SerialUSB.readBytes((char *) &fval, 4);
        PWMC_ConfigureClocks(fval, 0, CLOCK); // ask for 1 MHz clock.
        break;
      case 'W': //download a waveform. Expect 16384 16 bit words. Which contain 12 bit samples.
        for (i = 0; i < TNUM_TABLE / 32; i++) { // get 64 bytes at a time, 32 samples.
          SerialUSB.readBytes((char *) (dac_down + i * 32), 64);
          sent_pos = load_pos; // do this periodically so the watchdog doesn't get triggered.
        }
        break;
      case 'G': // write a digital pin, get two bytes: pin number, and 0/1
        SerialUSB.readBytes((char *) &sval, 2);
        {
          char pin, pval;

          pin = sval & 0xff;
          pval = sval >> 8;
          if (pin != 53 && pin != 7 && pin != 56 && pin != 57  && pin != 66 && pin != 61 && pin != 2) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, pval);
          }
        }
        break;
      case 'g': // read from a digital pin, get one byte, pin number
        {
          char pin, pval;
          SerialUSB.readBytes(&pin, 1);
          if (pin != 53 && pin != 7 && pin != 56 && pin != 57 && pin != 66 && pin != 61 && pin != 2) {
            pinMode(pin, INPUT);
            pval = digitalRead(pin);
            sprintf(mess, "pin: %i val: %i", pin, pval);
            my_send_message(mess);
          }
        }
        break;
      case 'A':// set gain, give it one byte as a binary 0, 1, 2, 3
        {
          char gain;
          SerialUSB.readBytes(&gain, 1);
          flags &= ~FLAG_GAIN;
          flags |= (gain & 3);
          // then actually set the gain.
          ADC->ADC_CGR = gain & 3; // first two bits are the CH0 gain, which are applied everywhere.
        }
        break;
      case 'L':// set differential mode
        flags |= FLAG_DIFFERENTIAL; // offset mode is on, so center is always 1/2 of power supply.
        ADC->ADC_COR |= 0xFFFF0000;
        break;
      case 'l':// unset differential mode
        flags &= ~FLAG_DIFFERENTIAL;
        ADC->ADC_COR &= ~0xFFFF0000;
        break;
      case 'O':// set offset mode
        flags |= FLAG_OFFSET;
        ADC->ADC_COR = 0xFFFF; // turn on the offset bit, so if gain is turned up, center is still at 1/2 the power supply.
        break;
      case 'o':// unset offset mode
        flags &= ~FLAG_OFFSET;
        ADC->ADC_COR &= ~0xFFFF; // turn off the offset bit
        break;
      case 'x': // ask for status
        {
          uint8_t lflags;
          __disable_irq();
          lflags = flags;
          flags &= ~FLAG_MASK;
          __enable_irq();
          sprintf(mess, "status: %i, flags: %i\n", status, lflags);
          my_send_message(mess);
        }
        break;
    }
  }
  //FLAG_OFF
}


#pragma GCC optimize ("align-functions=4")
volatile unsigned short tlooplevel = 0, rlooplevel = 0;

// CIC filter variables:
int32_t id20[2], id21[2], qd20[2], qd21[2];
int32_t i5[5], q5[5], id5[5][2], qd5[5][2];
int32_t i20, i21, q20, q21, m1, m2;

uint32_t tprog_cache, rprog_cache;

void ADC_Handler () {

  uint32_t lrftw, lrphase; // local copies
  int32_t i2out, q2out, i5out, q5out;

  uint32_t i, ic;
  int32_t di, dq;
  // filtering done here:

  int32_t sig;
  uint32_t ltftw, ltphase; // local  copies

  int16_t *adc_buffer;
  uint16_t *dac_buffer;

  int16_t lbuf_num;
  static uint16_t lload_pos = 0;
  uint16_t opcode;

  uint32_t ltsample, lrsample;
  static unsigned long tloopcount[LOOP_LEVELS], rloopcount[LOOP_LEVELS];
  static unsigned short tloopindex[LOOP_LEVELS], rloopindex[LOOP_LEVELS];


#define LOOP_RETURN 0x8000
#define  MM  5 // stages in 2nd CIC filter
  // flag so user can see on scope when the interrupt is running.
  // turning flag on and off now costs a tiny bit in terms of max lock frequency.
  // interrupt time is 83us.
  FLAG_ON

  lbuf_num = adc_buf_num ^ 1;
  adc_buf_num = lbuf_num;

  ///// Do the DAC:
  //if ( DACC->DACC_ISR & DACC_ISR_ENDTX) {               // Useless because the only one

  ltftw = tftw; // use local copy so we don't fetch it every time.

  ltphase = tphase;
  ltsample = tsample;

  dac_buffer = dac_buf[lbuf_num];
  DACC->DACC_TNPR = (uint32_t) dac_buffer;
  DACC->DACC_TNCR = DMALEN;

  // ok, parse transmit program.
  // transmit program is 32 bit words of duration (with opcode and transmit on/off built in
  // in transmit in on, next word is the phase
  // if its a loop start, next word is the number of iterations.

  if (tnextstart >= ltsample + DMALEN) { // we don't have to worry about it!
    for (i = 0; i < DMALEN; i++) {
      *dac_buffer++ = dac_table[ltphase >> 18];
      ltphase += ltftw;
    }
    ltsample += DMALEN;
  }
  else {
    for (i = 0 ; i < DMALEN ; i++) {
      uint16_t ltindex;
      ltindex = tindex & ~LOOP_RETURN;
      if (ltsample == tnextstart) { // advance tindex - we're at the end of an event.
        //if (program[ltindex] & RF_ON) //if just finished event was transmit, remove its phase. need CACHED program?
        if (tprog_cache & RF_ON)
          ltphase -= trans_phase;
        //opcode = (program[ltindex] >> 28) & 7; // CACHED program!
        opcode = (tprog_cache >> 28) & 7; // CACHED program!
        switch (opcode) {
          case (0):
            tindex += 1 + ((tprog_cache & RF_ON) == RF_ON);
            break;
          case (1): //loop start
            tindex = tindex + 2 + ((tprog_cache & RF_ON) == RF_ON);
            if ((tindex & LOOP_RETURN) == 0) {
              tloopindex[tlooplevel] = ltindex;
              // point at the next event:
              // get loop count from program
              tloopcount[tlooplevel] = program[tindex - 1] - 1;
              tlooplevel += 1;
            }
            else
              tindex &= ~LOOP_RETURN;
            break;
          case (2): // loop end
            if (tloopcount[tlooplevel - 1] > 0) { //go back
              tloopcount[tlooplevel - 1] -= 1;
              tindex = tloopindex[tlooplevel - 1] | LOOP_RETURN;
            }
            else { // go forward
              tlooplevel -= 1;
              tindex += 1 + ((tprog_cache & RF_ON) == RF_ON);
            }
            break;
          case (3): //last event
            if (status & STATUS_T_RELOADED) {
              tindex = tindex0;
              __disable_irq();
              status &= ~STATUS_T_RELOADED;
              __enable_irq();
            } // if we're not reloaded, we replay the last event.
            break;
          case 4: // subroutine: has an argument of the subroutine address
            tloopindex[tlooplevel] = tindex + 2;
            tlooplevel += 1;
            tindex = program[tindex + 1];
            break;
          case 5: // return
            tlooplevel -= 1;
            tindex = tloopindex[tlooplevel];
            break;
          case 6: // ordinary event, but with phase reset. Phase reset happens below, at start of event
            tindex += 1 + ((tprog_cache & RF_ON) == RF_ON);
            break;
          case 7:
            tindex = tindex + 2 + ((tprog_cache & RF_ON) == RF_ON);
            break;
            /*   case 7: // XXX Gaaa, I hate this. case 7 and default are bogus, but somehow make the interrupt faster.
                tindex = tindex0;

                break;
              default:
                tindex = tindex0 & ~LOOP_RETURN; */

        }


        ltindex = tindex & ~LOOP_RETURN;
        tprog_cache = program[ltindex];
        tnextstart += (program[ltindex] & 0x0fffffff);
        if (program[ltindex] & RF_ON) {
          trans_phase = program[ltindex + 1];

          dac_table = (uint16_t *) tsin_table;
          // actually shift the transmitter phase
          ltphase += trans_phase;
        }
        else
          dac_table = (uint16_t *) tzero_table;
        if (((program[ltindex] >> 28) & 7) == 7) { // do freq switch
          ltftw = program[ltindex + 1 + ((program[ltindex] & RF_ON) == RF_ON)];
          tftw = ltftw;
        }
        if (((program[ltindex] >> 28) & 7) == 6) // do phase reset
          ltphase = 0;
      }
      // generate samples:
      *dac_buffer++ = dac_table[ltphase >> 18];
      ltphase += ltftw;
      ltsample += 1;
    }
  }
  tphase = ltphase;
  tsample = ltsample;

  //asm volatile(".align 4");


  // now to the ADC:
  lrftw = rftw;
  lrsample = rsample;

  lload_pos = load_pos;
  lrphase = rphase;
  // set up DMA for next buffer
  adc_buffer =  adc_buf[lbuf_num];
  ADC->ADC_RNPR = (uint32_t) adc_buffer;
  ADC->ADC_RNCR = DMALEN;


  if ((rnextstart < lrsample + DMALEN) || (status & STATUS_RECEIVERON)) {
    uint16_t lrecindex;
    lrecindex = recindex & ~LOOP_RETURN;

    for (i = 0; i < DMALEN; i++) {
      //////////////////////
      if (lrsample == rnextstart) { // advance recindex - we're at the end of an event.
        //opcode = (program[lrecindex] >> 28) & 7; // USE CACHED program  could be just cached opcode?
        opcode = (rprog_cache >> 28) & 7;
        switch (opcode) {
          case (0):
            recindex += 1;
            break;
          case (1): //loop start
            recindex = recindex + 2;
            if ((recindex & LOOP_RETURN) == 0) {
              rloopindex[rlooplevel] = lrecindex;
              // point at the next event:
              // get loop count from program
              rloopcount[rlooplevel] = program[recindex - 1] - 1; // don't need to use cached
              rlooplevel += 1;
            }
            else
              recindex &= ~LOOP_RETURN;
            break;
          case (2):
            if (rloopcount[rlooplevel - 1] > 0) { //go back
              rloopcount[rlooplevel - 1] -= 1;
              recindex = rloopindex[rlooplevel - 1] | LOOP_RETURN;
            }
            else { // go forward
              rlooplevel -= 1;
              recindex += 1;
            }
            break;
          case (3):
            if (status & STATUS_R_RELOADED) {
              __disable_irq();
              status &= ~STATUS_R_RELOADED;
              __enable_irq();
              recindex = recindex0;
            } // if we're not reloaded we replay the last event.
            break;
          case 4: // subroutine: has an argument of the subroutine address
            rloopindex[rlooplevel] = recindex + 2;
            rlooplevel += 1;
            recindex = program[recindex + 1];
            break;
          case 5: // return
            rlooplevel -= 1;
            recindex = rloopindex[rlooplevel];
            break;
          case 6: // phase reset, handled below.
            recindex += 1;
            break;
          case 7:
            recindex = recindex + 2;
            break;
        }
        lrecindex = recindex & ~LOOP_RETURN;
        rnextstart += (program[lrecindex] & 0x0fffffff);
        rprog_cache = program[lrecindex]; // there's weird alignment stuff
        //if (rprog_cache & RF_ON) {
        if (program[lrecindex] & RF_ON) {
          __disable_irq();
          status |= STATUS_RECEIVERON;
          __enable_irq();
        }
        else {
          __disable_irq();
          status &= ~STATUS_RECEIVERON;
          __enable_irq();
          // we should really zero out the filters here, but it seems to
          // be super time expensive, so do it at the end of the interrupt
        }
        if (((program[lrecindex] >> 28) & 7) == 7) { //change frequency
          lrftw = program[lrecindex + 1];
          rftw = lrftw;
        }
        if (((program[lrecindex] >> 28) & 7) == 6) // reset phase
          lrphase = 0;
        if (((program[lrecindex] >> 28) & 7) == 3)
          tell_flags |= TELL_FLAG_DONEDATA;

      }
      //////////////////////
      if ( status & STATUS_RECEIVERON ) {
        sig = *adc_buffer++ - 2048; // zero shift - is this actually necessary?
        // the >> 3 here is a bit of a compromise. It could introduce a rounding error that would show up as a signal.
        // It should be _tiny_ though. We go from 28 to 25 bits here, only 12 of which are significant at this point.
        // so the rounding bias is down 13 bits below the data. You'd have to average pretty hard to see it.
        di = (sig * sin_table[lrphase >> 17 ]) >> 3 ; // started with >> 3
        dq = (sig * sin_table[(lrphase + (1 << 30)) >> 17]) >> 3; // shift phase by 90 degrees.
        //dq = (sig * sin_table[((lrphase>>17) + (1 << 13))&0x7fff]) >> 3; // shift phase by 90 degrees.

        lrphase += lrftw;
        lrsample += 1;

        // CIC2 filtering:
        i20 += di;
        q20 += dq;

        i21 += i20;
        q21 += q20;
        m1 += 1;
        if (m1 == 5) {
          m1 = 0;
          // CIC2 differentiate:
          id21[1] = id21[0];
          id20[1] = id20[0];
          id20[0] = i21;
          id21[0] = id20[1] - id20[0];

          qd21[1] = qd21[0];
          qd20[1] = qd20[0];
          qd20[0] = q21;
          qd21[0] = qd20[1] - qd20[0];

          // outputs as big as 2.1e8, so divide
          // making these bit shifts save .5 us off the interrupt...

          i2out = (id21[1] - id21[0]) / 1024; // / 1024; // started with /1024
          q2out = (qd21[1] - qd21[0]) / 1024; // / 1024; // 512 is maybe ok, 256 is not enough


          // CIC5 filtering. Integrator:
          i5[0] += i2out;
          q5[0] += q2out;
          for (ic = 1; ic < MM; ic++) {
            i5[ic] += i5[ic - 1];
            q5[ic] += q5[ic - 1];
          }
          m2 += 1;
          if (m2 == 5) {
            m2 = 0;
            id5[0][1] = id5[0][0];
            id5[0][0] = i5[MM - 1];
            qd5[0][1] = qd5[0][0];
            qd5[0][0] = q5[MM - 1];
            for (ic = 1; ic < MM; ic++) {
              id5[ic][1] = id5[ic][0];
              id5[ic][0] = id5[ic - 1][1] - id5[ic - 1][0];
              qd5[ic][1] = qd5[ic][0];
              qd5[ic][0] = qd5[ic - 1][1] - qd5[ic - 1][0];
            }
            // at this point, max value, when subtracting the last stage is about 13100 * 32768.
            //i5out = (id5[MM - 1][1] - id5[MM - 1][0]) / 32768; // started with /32768
            //q5out = (qd5[MM - 1][1] - qd5[MM - 1][0]) / 32768;
            // rescale to make better use of the full 16 bit output range:
            i5out = (id5[MM - 1][1] - id5[MM - 1][0]) * 3 / 65536; // started with /32768
            q5out = (qd5[MM - 1][1] - qd5[MM - 1][0]) * 3 / 65536;
            outbuff[(lload_pos++) & 0xff] = i5out;
            outbuff[(lload_pos++) & 0xff] = q5out;
          }
        }
      }
      else {
        lrsample += 1;
        lrphase += lrftw;
      }
    }
    rsample = lrsample;
    rphase = lrphase;
    load_pos = lload_pos;
  }
  else { // we didn't have to do anything!
    rsample += DMALEN;
    rphase = lrphase + DMALEN * lrftw;

    i20 = 0;  // abandon any partial points and reset filter
    i21 = 0;
    q20 = 0;
    q21 = 0;
    memset(id20, 0, 8);
    memset(id21, 0, 8);
    memset(qd20, 0, 8);
    memset(qd21, 0, 8);
    memset(i5, 0, 20);
    memset(q5, 0, 20);
    memset(id5, 0, 40);
    memset(qd5, 0, 40);
    m1 = 0;
    m2 = 0;

  }
  FLAG_OFF
}
void TC3_Handler() { // watchdog

  uint32_t isr;

  isr = TC1->TC_CHANNEL[0].TC_SR; // clear my own status register.
  //return;//  disable watchdog for now.
  // the expects here seem to make a big difference - 1.08us vs .8
  //FLAG_ON

  // read DACC and ADC status registers and look for limits:
  isr = ADC->ADC_ISR;
  if (__builtin_expect(isr & ADC_ISR_COMPE, 0)) { // ADC value out of limits
    flags |= FLAG_INPUT_OVERFLOW;
  }

  // see if there was a DMA over/under run:
  if (__builtin_expect(isr & ADC_ISR_RXBUFF || DACC->DACC_ISR & DACC_ISR_TXBUFE, 0)) {
    flags |= FLAG_DMA_XRUN;
    watchdog_trigger |= 2;
  }

  // check for send buffer overflow.
  if (__builtin_expect(load_pos - sent_pos > 256, 0)) {
    flags |= FLAG_OUT_BUFF_OVERRUN;
    sent_pos = load_pos;
    watchdog_trigger |= 1;
  }

  // stop the timers, then main loop will notice and reset everything.
  if (__builtin_expect(watchdog_trigger, 0)) {
    TC0->TC_CHANNEL[0].TC_CCR = 2;
    TC0->TC_CHANNEL[2].TC_CCR = 2;
    TC1->TC_CHANNEL[0].TC_CCR = 2; // stop watchdog too.
    status &= ~STATUS_LAST_EVENT;
    status &= ~STATUS_RUNNING;

  }
  //FLAG_OFF
}
volatile int16_t glooplevel = 0;
void TC0_Handler() { // GPIO events
  static unsigned short gloopindex[LOOP_LEVELS];
  static unsigned long gloopcount[LOOP_LEVELS];
  uint32_t i;
  uint32_t newRA, oldRA;
  uint16_t next, opcode;
  //FLAG_ON
  // this is about 0.75us.
  // the high bit of gindex is abused to indicate a loop return.

#ifdef NOLATCH
  PIOC->PIO_ODSR = program[(gindex & ~LOOP_RETURN) + 1 ]; // set the bits for the event
#endif

  i = TC0->TC_CHANNEL[0].TC_SR; // is this necessary? I think so!

  oldRA = TC0->TC_CHANNEL[0].TC_RA;
  newRA = oldRA + program[(gindex & ~LOOP_RETURN) ]; // duration of "current" event

  opcode = (program[(gindex & ~LOOP_RETURN) + 1] >> 10) & 3; // this is the previous instruction
  opcode |= (program[(gindex & ~LOOP_RETURN) + 1] >> 18) & 4; // bit 20 is the third opcode bit.

  if ((status & STATUS_LAST_EVENT)) { // we're done, shut 'er down
    TC0->TC_CHANNEL[0].TC_CCR = 2; // 2 stops the clock
    TC0->TC_CHANNEL[2].TC_CCR = 2;
    TC1->TC_CHANNEL[0].TC_CCR = 2; // stop the watchdog
    __disable_irq();
    tell_flags |= TELL_FLAG_SHUTDOWN;
    tell_flags &= ~TELL_FLAG_LAST;
    status |= STATUS_SHUTDOWN;
    status &= ~STATUS_LAST_EVENT;
    status &= ~STATUS_RUNNING;
    __enable_irq();
    safe_state();
    return;
  }

  // ok, so there's at least one more interrupt to come
  // the high bit of gindex gets abused to indicate that we're going back to a loop start event.
  // but don't restart the loop, just iterate through it.

  TC0->TC_CHANNEL[0].TC_RA = newRA;
  TC0->TC_CHANNEL[0].TC_RC = newRA + PULSE_DURATION;


  switch (opcode) {
    case 0:
      gindex += 2;
      break;
    case 1: // loop start
      // make sure we're not already in this loop
      if (gindex & LOOP_RETURN) // we're already in this loop.
        gindex &= ~LOOP_RETURN; // remove the going back marker.
      else { // start the loop
        gloopindex[glooplevel] = gindex & ~LOOP_RETURN; // mask shouldn't be needed here.
        gloopcount[glooplevel] = program[(gindex & ~LOOP_RETURN) + 2] - 1; // don't give a count < 1!
        glooplevel += 1;
      }
      gindex += 3;
      break;
    case 2:// its 2, loop end
      if (gloopcount[glooplevel - 1] > 0) { // go back if we're not done with the loops
        gloopcount[glooplevel - 1] -= 1;
        gindex = gloopindex[glooplevel - 1] | LOOP_RETURN;
      }
      else { // press onward, loop is done.
        glooplevel -= 1;
        gindex += 2;
      }
      break;
    case 3: //  // we don't have the output word for the next event yet.
      __disable_irq();
      status |= STATUS_LAST_EVENT;
      tell_flags |= TELL_FLAG_LAST;
      __enable_irq();
      gindex = 0; // return to event 0 which should be reloaded.
      //  FLAG_OFF;
      return;
      break;
    case 4:
      // subroutine
      gloopindex[glooplevel] = gindex + 2;
      glooplevel += 1;
      break;
    case 5:// return
      glooplevel -= 1;
      gindex = gloopindex[glooplevel];
      break;
  }

#ifndef NOLATCH
  PIOC->PIO_ODSR = program[(gindex & ~LOOP_RETURN ) + 1]; // set the bits for the next event
#endif


  //FLAG_OFF
}


void my_send_message(char *mess) {
  // don't use this from an irq!!

  uint8_t i;
  uint8_t *ptr_dest;
  uint8_t len;
  // every message is a full 64 byte packet. That way we know where every header is.

  len = strlen(mess);
  if (len > 64) len = 64;
  while ( UOTGHS_DEVEPTISR_TXINI != (UOTGHS->UOTGHS_DEVEPTISR[CDC_TX] & UOTGHS_DEVEPTISR_TXINI ));
  ptr_dest =  (uint8_t *) &udd_get_endpoint_fifo_access8(CDC_TX);

  for (i = 0; i < 64; i++) {
    if (i < len)
      *ptr_dest++ = mess[i];
    else
      *ptr_dest++ = 0;
  }

  //This doesn't work right if compiled with O3!! Seems it is inlined, but wrong somehow?
  /*
    for (i = 0; i < len; i++) {
    ptr_dest++ = mess[i];
    }
    for (i = len ; i < 64; i++) {
    ptr_dest++ = 0;
    } */
  UOTGHS->UOTGHS_DEVEPTICR[CDC_TX] = UOTGHS_DEVEPTICR_TXINIC;
  UOTGHS->UOTGHS_DEVEPTIDR[CDC_TX] = UOTGHS_DEVEPTIDR_FIFOCONC;
}

static inline void start_pulses()
{
  unsigned int newRA, i;
  //newRA = ;

  // redo entire DAC/ADC setup to make timing relative to GPIO pulses reliable.
  my_setup();

  watchdog_trigger = 0;
  TC1->TC_CHANNEL[0].TC_CCR = 5; // start the watchdog

  // GPIO stuff:
  sent_pos = 0;
  load_pos = 0;
  tsample = 0;

  gindex = 0; // fix up on download.

  // set up first event:
  PIOC->PIO_ODSR = program[gindex + 1];
  newRA = GOFFSET;

  TC0->TC_CHANNEL[0].TC_RA = newRA;
  TC0->TC_CHANNEL[0].TC_RC = newRA + PULSE_DURATION;

  // set up interrupts for latch pulses;
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // interrupt on RC match
  TC0->TC_CHANNEL[0].TC_CMR |= TC_CMR_ACPA_SET; // turn on the pulse on RA match

  // transmit stuff:
  rphase = 0;
  tphase = 0;
  tsample = 0;
  rsample = 0;
  // phases are 0 at sample number 2*DMALEN (start of program).
  tindex = tindex0;
  recindex = recindex0;
  tprog_cache = program[tindex];
  rprog_cache = program[recindex];
  tnextstart = program[tindex] & 0x0fffffff;
  rnextstart = program[recindex] & 0x0fffffff; // caller of start_pulses should add 2*DMALEN to the first event before calling.
  // so its active for the first time through the program, then subtract it off again immediately after call is done.
  // caller should also subtract off 2* DMALEN*rftw from fphase as soon as this function is complete.
  // why not just do these things in here? For reasons I can't fathom, it makes the ADC/DAC interrupt longer.

  glooplevel = 0;
  tlooplevel = 0;
  rlooplevel = 0;

  // is rf on during first event?
  if (program[tindex & ~LOOP_RETURN] & RF_ON) {
    trans_phase = program[(tindex & ~LOOP_RETURN) + 1];
    dac_table = (uint16_t *) tsin_table;
    // actually shift the transmitter phase
    tphase += trans_phase;
  }
  else {
    trans_phase = 0;
    dac_table = (uint16_t *) tzero_table;
  }
  __disable_irq();
  status &= ~(STATUS_R_RELOADED | STATUS_T_RELOADED | STATUS_RECEIVERON);
  __enable_irq();
  tell_flags = 0;

  i20 = 0;
  i21 = 0;
  q20 = 0;
  q21 = 0;
  memset(id20, 0, 8);
  memset(id21, 0, 8);
  memset(qd20, 0, 8);
  memset(qd21, 0, 8);
  memset(i5, 0, 20);
  memset(q5, 0, 20);
  memset(id5, 0, 40);
  memset(qd5, 0, 40);
  m1 = 0;
  m2 = 0;
  __disable_irq();
  status &= ~STATUS_SHUTDOWN;
  status &= ~STATUS_LAST_EVENT;
  status |= STATUS_RUNNING;
  __enable_irq();

  //////// start all timers 0, 2 simultaneously.

  TC0->TC_CHANNEL[0].TC_CCR = 1; // 1 enables, but doesn't start.
  TC0->TC_CHANNEL[2].TC_CCR = 1;

  //start the clocks.

  // go!
  TC0->TC_BCR = 1;

}

// on reload, set treloaded and rreloaded = 1, and set recindex0 and tindex0 to new values (if needed)
// and turn off LAST_EVENT flag in status, also write out the output word (unless NOLATCH is set).

/// PROBLEM. for transmit and receive - the event gets updated while we are still using it.
// what it should do, is when we move to a new event, cache the event, and use the cached event.

void safe_state() {
  // want to set output word to safe value, hit latch button,
  // and set DAC output to safe value.
  int i;

  TC0->TC_CHANNEL[0].TC_CCR = 2;
  TC0->TC_CHANNEL[2].TC_CCR = 2;
  TC1->TC_CHANNEL[0].TC_CCR = 2; // watchdog

  ADC->ADC_PTCR =  0x0202;  //TXTEN, RXTEN  disable transmit, disable receive
  DACC->DACC_PTCR =  0x0202;  //TXTEN, RXTEN  disable transmit, disable receive


  PIOC->PIO_ODSR = 0; // set bits to safe state.
  // hit latch button
#ifndef NOLATCH
  PIO_Configure(PIOB, PIO_OUTPUT_0, 1 << 25, 0);
  PIOB->PIO_SODR = 1 << 25;
  asm volatile("nop\n\tnop\n\t");
  PIOB->PIO_CODR = 1 << 25;

#endif

  DACC->DACC_CR = DACC_CR_SWRST; // hit reset button on DAC
  DACC->DACC_MR = DACC_MR_USER_SEL_CHANNEL0;
  dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_enable_channel(DACC_INTERFACE, 0);
  i = DACC->DACC_ISR;
  DACC->DACC_CDR = 2048;

  ADC->ADC_PTCR =  0x0202;  //TXTEN, RXTEN  disable transmit, disable receive
  DACC->DACC_PTCR =  0x0202;  //TXTEN, RXTEN  disable transmit, disable receive


}
