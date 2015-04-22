/**
* Author: Victor Santos
* Engineering Technologist on Electronic Systems
* email: victor.inboxfx@gmail.com
* phone: +55 48 84119029
*
*
* PMU measurement system - Prototype - on ARM-Cortex M3
*
* TODO: Write the project's infos
*/

#include <asf.h>
#include "conf_board.h"
/* ADC constants */
#define ADC_TRACKTIM 0
#define ADC_SETTLING_TIME ADC_SETTLING_TIME_3
#define ADC_TRANSFER 1

/* Board clocks */
#define BOARD_CLK 84000000UL
#define BOARD_SLOWCLK 32768UL

/* defines for Timer Counter channels */
#define TIOA0 0 /* TIOA0:ADC fixed trigger */
#define TIOA6 0 /* TIOA6:DAC sampling rate */
#define TIOA7 1 /* TIOA7:PPS signal "emulator" */
#define TIOA8 2 /* TIOA8:Timestamp to tag the samples */
#define TIOA0_PRESCALER 2   /* MCLK/2 : 42MHz*/
#define TIOA6_PRESCALER 32  /* MLCK/32 : 2.62 MHz*/
#define TIOA8_PRESCALER 2

#define ADC_SAMPLING_FREQ 6000UL  /* pay attention to Nyquist theorem for TIAO0! need to be at lest 120Hz (2*60Hz) : 6kHz */
#define DAC_SAMPLING_FREQ 15360UL  /* Fdac = Fsine*Nsamples_LUT = 60*256 = 15360 */
#define TIOA8_TIMESTAMP_FREQ  1000000UL  /* 1us timestamp */
#define TIOA7_DELAY_FREQ  1UL      /* 1s pps */

#define TIOA0_TC_COUNTER BOARD_CLK/(TIOA0_PRESCALER*ADC_SAMPLING_FREQ)
#define TIOA6_TC_COUNTER BOARD_CLK/(TIOA6_PRESCALER*DAC_SAMPLING_FREQ)
#define TIOA7_TC_COUNTER BOARD_SLOWCLK/TIOA7_DELAY_FREQ
#define TIOA8_TC_COUNTER BOARD_CLK/(TIOA8_PRESCALER*TIOA8_TIMESTAMP_FREQ)

struct time {
    uint16_t day;
    uint16_t month;
    uint16_t year;
    uint16_t week;
    uint16_t hour;
    uint16_t minute;
    uint16_t second;
};

struct taggedSamples {
    struct time sample_time; //time per second
    uint32_t sample_index;   //index 0..255
    uint32_t sample_data;    //ADC sampled value of pg sinewave
    uint32_t sample_timestamp; //1us timestamp reference from PPS signal
};

struct time * rtcTime;
struct taggedSamples * powergridSamples[256];

/* Circular buffer, power of two */
#define BUFSIZE 0x400 //1024 bytes
#define BUFMASK 0x3FF
volatile int samples [BUFSIZE] ;
volatile int sptr = 0 ;

volatile uint32_t us_timestamp = 0;

volatile int sinelut_index = 0;
/* Sinewave LUT: 256 points 0-4095 */
static int sineLUT[] =
{
    2048,2098,2148,2198,2248,2298,2348,2398, 2447,2496,2545,2594,2642,2690,2737,2784,
    2831,2877,2923,2968,3013,3057,3100,3143,3185,3226,3267,3307,3346,3385,3423,3459,
    3495,3530,3565,3598,3630,3662,3692,3722,3750,3777,3804,3829,3853,3876,3898,3919,
    3939,3958,3975,3992,4007,4021,4034,4045,4056,4065,4073,4080,4085,4089,4093,4094,
    4095,4094,4093,4089,4085,4080,4073,4065,4056,4045,4034,4021,4007,3992,3975,3958,
    3939,3919,3898,3876,3853,3829,3804,3777,3750,3722,3692,3662,3630,3598,3565,3530,
    3495,3459,3423,3385,3346,3307,3267,3226,3185,3143,3100,3057,3013,2968,2923,2877,
    2831,2784,2737,2690,2642,2594,2545,2496,2447,2398,2348,2298,2248,2198,2148,2098,
    2048,1997,1947,1897,1847,1797,1747,1697,1648,1599,1550,1501,1453,1405,1358,1311,
    1264,1218,1172,1127,1082,1038,995,952,910,869,828,788,749,710,672,636,
    600,565,530,497,465,433,403,373,345,318,291,266,242,219,197,176,
    156,137,120,103,88,74,61,50,39,30,22,15,10,6,2,1,
    0,1,2,6,10,15,22,30,39,50,61,74,88,103,120,137,
    156,176,197,219,242,266,291,318,345,373,403,433,465,497,530,565,
    600,636,672,710,749,788,828,869,910,952,995,1038,1082,1127,1172,1218,
    1264,1311,1358,1405,1453,1501,1550,1599,1648,1697,1747,1797,1847,1897,1947,1997
};

/*Function prototypes*/

void usart3_setup(void);
void tioa0_setup(void);
void tioa7_setup(void);
void tioa8_setup(void);
void tioa6_setup(void);
void rtc_setup(void);
void adc_setup(void);
void dac_setup (void);
void dac_write(int val, int channel);
void debug(void);


const usart_serial_options_t usart_console_settings = {
    USART_SERIAL_BAUDRATE,
    USART_SERIAL_CHAR_LENGTH,
    USART_SERIAL_PARITY,
    USART_SERIAL_STOP_BIT
};

/* Interrupt Handlers */

/* TC6 interrupt used to update LUT sinewave table index*/
void TC6_Handler(void)
{
    volatile uint32_t ul_status;
    TcChannel * tioa6 = &(TC2->TC_CHANNEL)[TIOA6];
    /* read status from TC0 status register to clear it and allow the interrupt to fire again */
    ul_status = tioa6->TC_SR;
    (void) ul_status;
    
    sinelut_index++;
    sinelut_index = sinelut_index & 0xff; /* limit index 0..255 (0xff) */
    if( sinelut_index==0x100) /* 256 */
    {
        sinelut_index=0;
    }
    dac_write (0xFFF & sineLUT[sinelut_index], 0) ;
}

/* TC8 interrupt to update 1us counter timestamp*/
void TC8_Handler(void)
{
    volatile uint32_t ul_status;
    TcChannel * tioa8 = &(TC2->TC_CHANNEL)[TIOA8];
    if ( (tioa8->TC_SR & TC_IER_ETRGS) == TC_IER_ETRGS) {
        /* read status from TC8 status register to clear it and allow the interrupt to fire again */
        ul_status = tioa8->TC_SR;
        (void) ul_status;
        us_timestamp = 0;
        } else {
        /* read status from TC8 status register to clear it and allow the interrupt to fire again */
        ul_status = tioa8->TC_SR;
        (void) ul_status;
        us_timestamp++;
    }
}

void ADC_Handler(void) {
    int val = 0;
    if (ADC->ADC_ISR & ADC_ISR_EOC7)
    {   
        PIOB->PIO_SODR = PIO_PB17;
        val = ADC->ADC_CDR[7] ;   /* get conversion result */
        //samples [sptr] = val ;        /* stick in circular buffer */
        //sptr = (sptr+1) & BUFMASK ;   /* move pointer */
        //dac_write (0xFFF & val, 1);  /* just to debug purposes, a 60Hz sinewave has to be seen here */
        PIOB->PIO_CODR = PIO_PB17;
    }
}

/* Peripheral functions */

void usart3_setup(void) {
    /* configure board GPIO to USART3 behaviour */
    gpio_configure_pin(PIN_USART3_TXD_IDX, PIN_USART3_TXD_FLAGS);
    gpio_configure_pin(PIN_USART3_RXD_IDX, PIN_USART3_RXD_FLAGS);
    /* initalize USART3 as console to use printf and scanf */
    stdio_serial_init(USART_SERIAL, &usart_console_settings);
    printf("USART3 ok\n\r");
}

/* adc trigger timer: 6kHz */
void tioa0_setup(void)
{
    /* IO Configure to output waveform from TC0 channel 0 PB25 */
    //Disable Timer Counter 0 from PIO control
    PIOB->PIO_PDR |= PIO_PB25;
    //Setup PB25 B peripheral: Timer Counter 0: channel 0
    PIOB->PIO_ABSR |= PIO_PB25;
    //Disable interrupts on pin
    PIOB->PIO_IDR |= PIO_PB25 ;
    // Set to output: Output Enable Register
    PIOB->PIO_OER |= PIO_PB25;
    // Disable pull-up : Pull-up Disable Register
    PIOB->PIO_PUDR |= PIO_PB25;
    
    // Enable TIOA0 clock
    PMC->PMC_PCER0 = 1 << ID_TC0;

    // pointer to TC0 registers for its channel 0
    TcChannel * tioa0 = &(TC0->TC_CHANNEL)[TIOA0];
    
    //Disable de TC Write Protection to Register C
    TC0->TC_WPMR = 0x54494D;
    //Disable module clock while configuring it
    tioa0->TC_CCR = TC_CCR_CLKDIS;
    //disable all interrupts
    tioa0->TC_IDR = 0xFF;
    //clear pending interrupts
    tioa0->TC_SR;
    
    // Set Mode
    tioa0->TC_CMR = TC_CMR_WAVE |    //Wave mode
    TC_CMR_TCCLKS_TIMER_CLOCK1 |    //PRESCALER = 2 (MLCK/128)
    TC_CMR_WAVSEL_UP |              //UP mode without automatic trigger on RC Compare
    TC_CMR_ACPA_CLEAR |             //RA Compare Effect on TIOA (Clear)
    TC_CMR_ACPC_SET |               //RC Compare Effect on TIOA (Set)
    TC_CMR_CPCTRG;                  //Compare RC Trigger: trigger when counter value matches the RC value
    
    // RC controls the timer counting RA controls duty cycle
    tioa0->TC_RA = TIOA0_TC_COUNTER/2; //50% duty cycle
    tioa0->TC_RC = TIOA0_TC_COUNTER;
    
    //Reset counter (SWTRG) and enable counter clock (CLKEN)
    tioa0->TC_CCR = (TC_CCR_CLKEN | TC_CCR_SWTRG);

    #if 0
    // Enable interrupt on TC0
    tioa0->TC_IER = TC_IER_CPCS;
    // Configure and enable interrupt on RC compare
    NVIC_EnableIRQ((IRQn_Type) TC0_IRQn);
    #endif

    printf("TIOA0 ok\n\r");
}

/* dac sinewave trigger frequency: 15.360kHz (externally connected to DATRG) */
void tioa6_setup(void)
{
    /* IO Configure to output waveform from TC2 channel 6 Pc25 */
    //Disable Timer Counter 2 from PIO control
    PIOC->PIO_PDR |= PIO_PC25;
    //Setup PC28 B peripheral: Timer Counter 2: channel 6
    PIOC->PIO_ABSR |= PIO_PC25;
    //Disable interrupts on pin
    PIOC->PIO_IDR |= PIO_PC25 ;
    // Set to output: Output Enable Register
    PIOC->PIO_OER |= PIO_PC25;
    // Disable pull-up: Pull-up Disable Register
    PIOC->PIO_PUDR |= PIO_PC25;
    
    // Enable TIOA6 clock
    PMC->PMC_PCER1 = 1 << (ID_TC6 - 32);

    // pointer to TC2 registers for its channel 0 TIOA6
    TcChannel * tioa6 = &(TC2->TC_CHANNEL)[TIOA6];
    
    //Disable de TC Write Protection to Register C
    TC2->TC_WPMR = 0x54494D;
    //Disable module clock while configuring it
    tioa6->TC_CCR = TC_CCR_CLKDIS;
    //disable all interrupts
    tioa6->TC_IDR = 0xFF;
    //clear pending interrupts
    tioa6->TC_SR;
    
    // Set Mode
    tioa6->TC_CMR = TC_CMR_WAVE |        //Wave mode
    TC_CMR_TCCLKS_TIMER_CLOCK3 |    //PRESCALER = 3 (MLCK/32)
    TC_CMR_WAVSEL_UP |              //UP mode without automatic trigger on RC Compare
    TC_CMR_ACPA_CLEAR |            //RA Compare Effect on TIOA (Clear)
    TC_CMR_ACPC_SET |            //RC Compare Effect on TIOA (Set)
    TC_CMR_CPCTRG;                  //Compare RC Trigger: trigger when counter value matches the RC value
    
    // RC controls the timer counting RA controls duty cycle
    tioa6->TC_RA = TIOA6_TC_COUNTER/2; //50% duty cycle
    tioa6->TC_RC = TIOA6_TC_COUNTER;
    
    //Reset counter (SWTRG) and enable counter clock (CLKEN)
    tioa6->TC_CCR = (TC_CCR_CLKEN | TC_CCR_SWTRG);

    #if 1
    // Enable interrupt on TC2
    tioa6->TC_IER = TC_IER_CPCS;
    // Configure and enable interrupt on RC compare
    NVIC_EnableIRQ((IRQn_Type) TC6_IRQn);
    #endif

    printf("TIOA6 ok\n\r");
}

/* 100ms pps simulating timer */
void tioa7_setup(void)
{
    /* IO Configure to output waveform from TC2 channel 7 Pc28 */
    //Disable Timer Counter 2 from PIO control
    PIOC->PIO_PDR |= PIO_PC28;
    //Setup PC28 B peripheral: Timer Counter 2: channel 7
    PIOC->PIO_ABSR |= PIO_PC28;
    //Disable interrupts on pin
    PIOC->PIO_IDR |= PIO_PC28 ;
    // Set to output: Output Enable Register
    PIOC->PIO_OER |= PIO_PC28;
    // Disable pull-up: Pull-up Disable Register
    PIOC->PIO_PUDR |= PIO_PC28;
    
    // Enable TIOA7 clock
    PMC->PMC_PCER1 = 1 << (ID_TC7 - 32);

    // pointer to TC2 registers for its channel 1 TIOA7
    TcChannel * tioa7 = &(TC2->TC_CHANNEL)[TIOA7];
    
    //Disable de TC Write Protection to Register C
    TC2->TC_WPMR = 0x54494D;
    //Disable module clock while configuring it
    tioa7->TC_CCR = TC_CCR_CLKDIS;
    //disable all interrupts
    tioa7->TC_IDR = 0xFF;
    //clear pending interrupts
    tioa7->TC_SR;
    
    // Set Mode
    tioa7->TC_CMR = TC_CMR_WAVE |   //Wave mode
    TC_CMR_TCCLKS_TIMER_CLOCK5 |    //PRESCALER = 2
    TC_CMR_WAVSEL_UP |              //UP mode without automatic trigger on RC Compare
    TC_CMR_ACPA_CLEAR |             //RA Compare Effect on TIOA (Clear)
    TC_CMR_ACPC_SET |               //RC Compare Effect on TIOA (Set)
    TC_CMR_CPCTRG;                  //Compare RC Trigger: trigger when counter value matches the RC value
    
    // RC controls the timer counting RA controls duty cycle
    tioa7->TC_RA = TIOA7_TC_COUNTER/2; //50% duty cycle
    tioa7->TC_RC = TIOA7_TC_COUNTER;
    
    //Reset counter (SWTRG) and enable counter clock (CLKEN)
    tioa7->TC_CCR = (TC_CCR_CLKEN | TC_CCR_SWTRG);

    #if 0
    // Enable interrupt on TC2
    tioa7->TC_IER = TC_IER_CPCS;
    // Configure and enable interrupt on RC compare
    NVIC_EnableIRQ((IRQn_Type) TC7_IRQn);
    #endif

    printf("TIOA7 ok\n\r");
}

/*TIOA8: timestamp base 1us TIOB8: Trigged by pps to start the timestamp */
void tioa8_setup(void)
{
    /* IO Configure to output waveform from TC2 channel 2 TIOA8 */
    //Disable Timer Counter 2 from PIO control
    PIOD->PIO_PDR |= PIO_PD7;
    //Setup PD7 B peripheral: TIOA8
    PIOD->PIO_ABSR |= PIO_PD7;
    //Disable interrupts on pin
    PIOD->PIO_IDR |= PIO_PD7 ;
    // Set to output: Output Enable Register
    PIOD->PIO_OER |= PIO_PD7;
    // Disable pull-up : Pull-up Disable Register
    PIOD->PIO_PUDR |= PIO_PD7;
    
    /* Configure TIOB8 as input */
    //Disable Timer Counter 2 from PIO control
    PIOD->PIO_PDR |= PIO_PD8;
    //Setup PD8 B peripheral: TIOB8
    PIOD->PIO_ABSR |= PIO_PD8;
    // Set to input: Output Disable Register
    PIOD->PIO_ODR |= PIO_PD8;
    // Disable pull-up : Pull-up Disable Register
    PIOD->PIO_PUDR |= PIO_PD8;
    
    // Enable TIOA8 clock
    PMC->PMC_PCER1 = 1 << (ID_TC8 - 32);

    // pointer to TC2 registers for its channel 1 TIOA7
    TcChannel * tioa8 = &(TC2->TC_CHANNEL)[TIOA8];
    
    //Disable de TC Write Protection to Register C
    TC2->TC_WPMR = 0x54494D;
    //Disable module clock while configuring it
    tioa8->TC_CCR = TC_CCR_CLKDIS;
    //disable all interrupts
    tioa8->TC_IDR = 0xFF;
    //clear pending interrupts
    tioa8->TC_SR;
    
    // Set Mode
    tioa8->TC_CMR = TC_CMR_WAVE | //Wave mode
    TC_CMR_TCCLKS_TIMER_CLOCK1 |  //PRESCALER = 2
    TC_CMR_WAVSEL_UP |            //UP mode without automatic trigger on RC Compare
    TC_CMR_ACPA_CLEAR |           //RA Compare Effect on TIOA (Clear)
    TC_CMR_ACPC_SET |             //RC Compare Effect on TIOA (Set)
    TC_CMR_EEVT_TIOB |            //TIOB8 selected as input for external trigger
    TC_CMR_EEVTEDG_RISING |       //Trigger happens on rising edge of input signal TIOB8
    TC_CMR_ENETRG |               //External event resets counter and start counter clock
    TC_CMR_CPCTRG;                //Compare RC Trigger: trigger when counter value matches the RC value
    
    // RC controls the timer counting RA controls duty cycle
    tioa8->TC_RA = TIOA8_TC_COUNTER/2; //50% duty cycle
    tioa8->TC_RC = TIOA8_TC_COUNTER;
    
    //Reset counter (SWTRG) and enable counter clock (CLKEN)
    tioa8->TC_CCR = (TC_CCR_CLKEN | TC_CCR_SWTRG);

    #if 1
    // Enable interrupt on TC2
    tioa8->TC_IER = TC_IER_CPCS | TC_IER_ETRGS;
    // Configure and enable interrupt on RC compare
    NVIC_EnableIRQ((IRQn_Type) TC8_IRQn);
    #endif

    printf("TIOA8 ok\n\r");
}

void rtc_setup(void)
{
    pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);
    while (!pmc_osc_is_ready_32kxtal());
    rtc_set_hour_mode(RTC, 0);
    printf("RTC ok\n\r");
}

void dac_setup (void)
{
    /* IO Configure to set DATRG as input */
    //Disable Timer Counter 2 from PIO control
    PIOA->PIO_PDR |= PIO_PA10;
    //Setup PD8 B peripheral: DATRG
    PIOA->PIO_ABSR |= PIO_PA10;
    // Set to input: Output Disable Register
    PIOA->PIO_ODR |= PIO_PA10;
    // Disable pull-up : Pull-up Disable Register
    PIOA->PIO_PUDR |= PIO_PA10;
    
    //DAC clock is 42MHz
    /* Note: DAC takes 25 clock cycles to sample: 25 * 23.81ns = 595.2ns */
    pmc_enable_periph_clk(ID_DACC); // start clocking DAC
    DACC->DACC_WPMR = 0x444143;
    DACC->DACC_CR = DACC_CR_SWRST;  // reset DAC
    
    DACC->DACC_MR = DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (0) |  // trigger 1 = TIO output of TC0-channel 0  //changed to zero to use an external clock as trigger, in order to avoid using an entire Timer Counter just for this
    DACC_MR_USER_SEL_CHANNEL0 |  // select channel 0
    DACC_MR_REFRESH (0x01) |     //datasheet: refresh in every 24.38us (bit of a guess... I'm assuming refresh not needed at 48kHz) //example value: 0x0F
    DACC_MR_STARTUP_1536;        //24 = 1536 cycles which I think is in range 23..45us since DAC clock = 42MHz (typical value regarding datasheet)

    DACC->DACC_IDR = 0x0F ;          // no interrupts
    DACC->DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1; // enable channel 0   
    printf("DAC ok\n\r");
}

/*write value to DAC Data register*/
void dac_write (int val, int channel)
{
    if (channel == 0) {
        DACC->DACC_MR &= DACC_MR_USER_SEL_CHANNEL0;
        } else {
        DACC->DACC_MR |= DACC_MR_USER_SEL_CHANNEL1;
    }
    DACC->DACC_CDR = val & 0xFFF;
}

void adc_setup(void) {
    
    /* Configure ADC input to A0, Channel A7 : A16 */
    // Enable IO
    PIOA->PIO_PER |= PIO_PA16;
    // Set to input : Output Disable Register
    PIOA->PIO_ODR |= PIO_PA16;
    // Disable pull-up
    PIOA->PIO_PUDR |= PIO_PA16;

    #if 0
    //Configure ADTRG PA.11  to externally trigger ADC
    //Disable PA11 from PIO control
    PIOA->PIO_PDR = PIO_PA11;
    //Setup PA11 B peripheral: ADTRG
    PIOA->PIO_ABSR = PIO_PA11;
    // Set to input: Output Disable Register
    PIOA->PIO_ODR = PIO_PA11;
    // Disable pull-up : Pull-up Disable Register
    PIOA->PIO_PUDR = PIO_PA11;
    #endif

    // Enable ADC clock
    PMC->PMC_PCER1 = 1 << (ID_ADC - 32); //register offset, necessary to PCER1 peripherals
    
    //Disable ADC Write Protect Mode Register
    ADC->ADC_WPMR = 0x414443;
    
    //Disable interrupts
    ADC->ADC_IDR = 0xFFFFFFFF ;
    
    /* Initialize ADC. */
    /*
    * 12 bits resolution, Hardware trigger, External trigger on PA.11, Normal mode, Freerun
    *
    * Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
    * MCK = 84MHZ, PRESCAL = 1, then:
    * ADCClock = 84MHz / ((1+1) * 2) = 21MHz;
    */
    /* Formula:
    * Startup Time = startup value / ADCClock
    * Startup time = 16 / 21MHz = 762ns
    */
    /* Formula:
    * Transfer Time = (TRANSFER * 2 + 3) / ADCClock
    * Tracking Time = (TRACKTIM + 1) / ADCClock
    * Settling Time = settling value / ADCClock
    * Settling Time = 3 / 21MHz = 142.9ns

    * Transfer Time = (1 * 2 + 3) / 21MHz = 238.1ns
    * Tracking Time = (0 + 1) / 21MHz = 47.62ns
    */
    
    /* Note: ADC takes 25 clock cycles to sample: 25 * 47.62ns = 1.19us*/
    /* Total sampling time counting TRANSFER AND TRACKING: 1.48us */
    /* ADC Sampling Rate: ~670ksps*/
    ADC->ADC_IER = 0x80;        //enable interrupt End-Of-Conversion on channel 7: Arduino Due PIN A0
    ADC->ADC_CHDR = 0xFFFF ;    // disable all channels
    //Enable ADC channel 7, Arduino pin A0
    ADC->ADC_CHER = 0x80;
    ADC->ADC_COR = 0x00000000 ;   // All offsets off
    //Reset ADC Mode Register
    ADC->ADC_MR = 0;
    //Configure ADC Mode Register: 21MHz ADC Normal mode
    ADC->ADC_MR = 0x10380103; //trigged by timer counter 0
    ADC->ADC_ACR = 0x100;     //Temperature sensor disable, IBCTL = 01

    // Configure and enable ADC interrupt
    NVIC_EnableIRQ((IRQn_Type) ADC_IRQn);
    printf("ADC ok\n\r");
}

/* set io pins for debug timing*/
void debug(void) {
     /* IO Configure to output ADC interrupt timing */
     PIOB->PIO_PER |= PIO_PB17;
     //Disable interrupts on pin
     PIOB->PIO_IDR |= PIO_PB17 ;
     // Set to output: Output Enable Register
     PIOB->PIO_OER |= PIO_PB17;
     // Disable pull-up : Pull-up Disable Register
     PIOB->PIO_PUDR |= PIO_PB17;   
}

int main (void)
{
    sysclk_init();
    board_init();
    
    #if 1
    delay_init(sysclk_get_cpu_hz());
    #endif

    usart3_setup();
    dac_setup();
    tioa0_setup();
    tioa7_setup();
    tioa8_setup();
    tioa6_setup();
    adc_setup();
    debug();

    #if 0
    rtc_setup();
    rtc_set_date(RTC, 2015, 04, 15, 3);
    rtc_set_time(RTC, 17, 30, 55);
    #endif
    
    printf("perifericos ok\n\r");
    char c = 0;
    (void) c;
    TcChannel * tioa8 = &(TC2->TC_CHANNEL)[TIOA8];
    (void) tioa8;
    while (1) {
        //loop teste usart3
        //scanf("%c", &c);
        //printf("char recebido: %c\n", c);
        #if 0
        rtc_get_time(RTC, rtcTime->hour, rtcTime->minute, rtcTime->second);
        rtc_get_date(RTC, rtcTime->year, rtcTime->month, rtcTime->day, rtcTime->week);
        printf("Horario atual: %ld/%ld/%ld - %ld:%ld:%ld\n\r", rtcTime->day, rtcTime->month, rtcTime->year, rtcTime->hour, rtcTime->minute, rtcTime->second);
        delay_ms(500);
        #endif
        //printf("TIOA8: CV: %ld\n\r", tioa8->TC_CV);
        printf("us Timestamp: %ld\n\r", us_timestamp);
    }
}
