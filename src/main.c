#include "common.h"
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

#define NUM_LEDS 3
#define CLK_MHZ 16
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
