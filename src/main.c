#include "common.h"
#include "libopencm3/stm32/common/timer_common_all.h"
#include "libopencm3/stm32/f4/gpio.h"
#include "libopencm3/stm32/f4/nvic.h"
#include "libopencm3/stm32/f4/rcc.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include "led.h"
//#include "i2c.h"
#include "libopencm3/stm32/i2c.h"
//
#include <stdint.h>

//void _close_r(void) {}
//void _lseek_r(void) {}
//void _read_r(void) {}
//void _write_r(void) {}

extern void write_chain(uint32_t num_leds);


//one bit takes a total of 1.25us(microseconds)
//24 bits per pkt is 30us or 0.03ms / the 30 us, is equal to 
//1MHZ tick, but I want this in 16mhz, so I can multiply by 16
//the result is 480 clock ticks

#define NUM_LEDS 3
#define WRITE_DELAY 105
#define CLK_MHZ 16
#define CLK_HZ 84000000
#define RR_HZ 30
#define UPDATE_INTERVAL 2800000
#define SYSCFG 0x40013C00
#define SYSCFG_EXTCR3 0x10
#define LED_WAIT (UPDATE_INTERVAL - (NUM_LEDS * WRITE_DELAY))
//use a 32bit timer
#define LED_TIM TIM5

#define GYRO_ADDR 0x68
#define ACC_X 0x3B
#define ACC_XH 0x3B
#define ACC_XL 0x3C

#define ACC_Y 0x3D
#define ACC_YH 0x3D
#define ACC_YL 0x3E

#define ACC_Z 0x3F
#define ACC_ZH 0x3F
#define ACC_ZL 0x40

uint16_t x = 0, y = 0, z = 0;

led_t led_map[NUM_LEDS] = { 0 };

const struct rcc_clock_scale my_clk = { /* 84MHz */
		.pllm = 25,
		.plln = 336,
		.pllp = 4,
		.pllq = 7,
		.pllr = 0,
		.pll_source = RCC_CFGR_PLLSRC_HSE_CLK,
		.hpre = RCC_CFGR_HPRE_NODIV,
		.ppre1 = RCC_CFGR_PPRE_DIV2,
		.ppre2 = RCC_CFGR_PPRE_NODIV,
		.voltage_scale = PWR_SCALE1,
		.flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN,
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
	systick_set_reload(84000); //clock up to 84mHz
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

//I should try to get access to the full 84Mhz clock at some point
static void clock_setup(void)
{
    //I have to make this custom
	rcc_clock_setup_pll(&rcc_hse_16mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    systick_setup();
}


void tim2_isr(void){
    //disable other interrupts, giving me full access to the clock
    __asm__("cpsid i");
                      
    write_chain(3);             
    //gpio_toggle(GPIOC, GPIO14);

    TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
    __asm__("cpsie i");
}


static void setup_led_write_clk(void){
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);
    rcc_periph_clock_enable(RCC_TIM2);

    TIM_CNT(TIM2) = 1;
    TIM_PSC(TIM2) = 2800; //30000 ticks per second
    TIM_ARR(TIM2) = 1000; //stop every time I reach 1000 to service the request
    TIM_DIER(TIM2) |= TIM_DIER_UIE;
    TIM_CR1(TIM2) |= TIM_CR1_CEN;
}

static void init_leds(void){
    //config C13 as output
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

static void init_led(led_t *store, unsigned char r, unsigned char g, unsigned char b){
    store->r = r;
    store->g = g;
    store->b = b;
}

static void enable_b10_irq(void){
    uint16_t reg;
    
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO10);

    reg = *(uint16_t *)(SYSCFG + SYSCFG_EXTCR3) & 0xF0FF;
    *(uint16_t *)(SYSCFG + SYSCFG_EXTCR3) = reg | 0x100;

    nvic_enable_irq(NVIC_EXTI15_10_IRQ);
    nvic_set_priority(NVIC_EXTI15_10_IRQ, 2);
}

static void init_i2c(void){
    //setup the NVIC to accept interrupts on b10
    //enable_b10_irq();
    rcc_periph_clock_enable(RCC_GPIOB);
    //i2c_reset(I2C1);
    //not availible
    //rcc_set_i2c_clock_hsi(I2C1);
    
    //enable i2c 1
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    //gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);

    rcc_periph_clock_enable(RCC_I2C1);

    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, rcc_apb1_frequency / 1e6);
    i2c_peripheral_enable(I2C1);
}

static uint8_t read_i2c_reg(uint8_t reg){
    uint8_t ret;
    i2c_transfer7(I2C1, GYRO_ADDR, &reg, 1, &ret, 1);
    return ret;
}

static uint16_t read_u16(uint8_t reg){
    uint16_t ret;
    ret = read_i2c_reg(reg) << 8; 
    ret |= read_i2c_reg(reg + 1); 
    return ret;
}

static void poll_i2c(void){
    x = read_u16(ACC_X);
    y = read_u16(ACC_Y);
    z = read_u16(ACC_Z);
}

void exti15_10_isr(void){
    //disable other interrupts, giving me full access to the clock
    __asm__("cpsid i");
                      
    //write_chain(3);             
    poll_i2c();
    //gpio_toggle(GPIOC, GPIO14);

    TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
    __asm__("cpsie i");
}


static void init_gyro(void){
    uint8_t data[2];
    data[0] = 117; //interupt enable register
    data[1] = 1; //data ready
    i2c_transfer7(I2C1, GYRO_ADDR, data, 2, NULL, 0);
}

static void init_led_map(void){
    led_t tmp;
    init_led(&tmp, 0xff, 0, 0);
    memcpy(&led_map[0], &tmp, 3);
    init_led(&tmp, 0, 0xff, 0);
    memcpy(&led_map[1], &tmp, 3);
    init_led(&tmp, 0, 0, 0xff);
    memcpy(&led_map[2], &tmp, 3);
}

int main(void){
    //int i;
    //__asm__("bkpt");
    led_t tmp;
    clock_setup();
    systick_setup();
    init_leds();
    gpio_toggle(GPIOC, GPIO14);
    clock_setup();
    setup_led_write_clk();
    init_led_map();
    init_i2c();
    init_gyro();
    for(;;){
        gpio_toggle(GPIOC, GPIO13);
        memcpy(&tmp, &led_map[0], 3);
        memcpy(&led_map[0], &led_map[1], 3);
        memcpy(&led_map[1], &led_map[2], 3);
        memcpy(&led_map[2], &tmp, 3);
        msleep(1000); // approximate for 30hz refresh rate
    }

    return 0;
}
