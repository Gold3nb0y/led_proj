#include "common.h"
#include "libopencm3/stm32/common/timer_common_all.h"
#include "libopencm3/stm32/f0/nvic.h"
#include "libopencm3/stm32/f4/gpio.h"
#include "libopencm3/stm32/f4/rcc.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include <stdint.h>

void _close_r(void) {}
void _lseek_r(void) {}
void _read_r(void) {}
void _write_r(void) {}

//one bit takes a total of 1.25us(microseconds)
//24 bits per pkt is 30us or 0.03ms / the 30 us, is equal to 
//1MHZ tick, but I want this in 16mhz, so I can multiply by 16
//the result is 480 clock ticks

#define NUM_LEDS 3
#define WRITE_DELAY 480
#define CLK_MHZ 16
#define CLK_HZ 16000000
#define RR_HZ 30
#define UPDATE_INTERVAL 533333
#define LED_WAIT (UPDATE_INTERVAL - (NUM_LEDS * WRITE_DELAY))
//use a 32bit timer
#define LED_TIM TIM5

const struct rcc_clock_scale my_clk = { /* 84MHz */
		.pllm = 16,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_2WS,
		.ahb_frequency  = 84000000,
		.apb1_frequency = 42000000,
		.apb2_frequency = 84000000,
};

volatile uint32_t ticks;

//this overwrites the system ticker
void sys_tick_handler(void){
    ticks++;
}

static void msleep(uint32_t delay){
    uint32_t wake = ticks + delay;
    while(wake > ticks);
}

static void systick_setup(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(16000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

//I should try to get access to the full 84Mhz clock at some point
static void clock_setup(void)
{
    //I have to make this custom
	//rcc_clock_setup_pll(&my_clk);
	//just using the 16mhz clk
    rcc_periph_clock_enable(RCC_GPIOD);
    //rcc_osc_on(RCC_HSE);
    //while(!rcc_is_osc_ready(RCC_HSE));

    //rcc_set_main_pll_hse(25, 336, 2, 7, 0);
    //rcc_osc_on(RCC_PLL);

    //while(!rcc_is_osc_ready(RCC_PLL));
    //rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);
    //rcc_set_ppre1(RCC_CFGR_PPRE_DIV4);
    //rcc_set_ppre2(RCC_CFGR_PPRE_DIV2);

    //rcc_periph_clock_enable(RCC_GPIOD);
}

void tim2_isr(void){
    //disable other interrupts, giving me full access to the clock
    __asm__("cpsid i");
    TIM2_CR1 &= ~(1); //stop counter while writing
                      
    write_chain();             

    TIM2_CR1 &= ~(1); //stop counter while writing
    __asm__("cpsie i");
    GPIOC_ODR &= ~(GPIO13); //clear put the divice in reset mode
}


static void setup_led_write_clk(void){
    TIM2_CR1 = 0x000; //generate interrupt, enable update
    TIM2_ARR = LED_WAIT; //generate interrupt, enable update
    TIM2_CR1 = 0x003; //generate interrupt, enable update
}

static void init_led(void){
    //config C13 as output
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
}

int main(void){
    int i;
    //__asm__("bkpt");
    clock_setup();
    systick_setup();
    init_led();
    for(;;){
        msleep(1000);
        gpio_toggle(GPIOC, GPIO13);
        msleep(2000);
        gpio_toggle(GPIOC, GPIO13);
    }

    return 0;
}
