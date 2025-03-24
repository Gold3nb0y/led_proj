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
#include "lcd.h"
#include "libopencm3/stm32/i2c.h"
#include <stdint.h>
#include "gyro.h"

//void _close_r(void) {}
//void _lseek_r(void) {}
//void _read_r(void) {}
//void _write_r(void) {}

extern void write_chain(uint32_t num_leds);
extern volatile uint32_t ticks;
extern gyro_t gyro;


//one bit takes a total of 1.25us(microseconds)
//24 bits per pkt is 30us or 0.03ms / the 30 us, is equal to 
//1MHZ tick, but I want this in 16mhz, so I can multiply by 16
//the result is 480 clock ticks
led_t led_map[NUM_LEDS] = { 0 };
led_t tmp;
uint32_t iterations = 0;

//this overwrites the system ticker
void sys_tick_handler(void){
    ticks++;
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
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_TIM2);
    systick_setup();
}


void tim2_isr(void){
    //disable other interrupts, giving me full access to the clock
    bool update_info = false;
    __asm__("cpsid i");
                      
    write_chain(NUM_LEDS);             

    TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
    __asm__("cpsie i");
}


static void setup_led_write_clk(void){
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);

    TIM_CNT(TIM2) = 1;
    TIM_PSC(TIM2) = 2800; //30000 ticks per second
    TIM_ARR(TIM2) = 1000; //stop every time I reach 1000 to service the request
    TIM_DIER(TIM2) |= TIM_DIER_UIE;
    TIM_CR1(TIM2) |= TIM_CR1_CEN;
}

static void init_leds(void){
    //config C13 as output
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
}

static void init_led(led_t *store, unsigned char r, unsigned char g, unsigned char b){
    store->r = r;
    store->g = g;
    store->b = b;
}

static void init_i2c(void){
    //enable i2c 1
    gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO6 | GPIO7);

    rcc_periph_clock_enable(RCC_I2C1);

    i2c_peripheral_disable(I2C1);
    i2c_set_clock_frequency(I2C1, 42); //apb1 frequency is at 42 mhz
    i2c_set_fast_mode(I2C1); 
    i2c_set_ccr(I2C1, 35); 
    i2c_set_trise(I2C1, 14); 
                             
    i2c_peripheral_enable(I2C1);
}

static void init_led_map(void){
    init_led(&tmp, 0, 0, 0);
    memcpy(&led_map[0], &tmp, 3);
    init_led(&tmp, 0, 0, 0);
    memcpy(&led_map[1], &tmp, 3);
    init_led(&tmp, 0, 0, 0);
    memcpy(&led_map[2], &tmp, 3);
}

static void start_lcd(void){
    char buf[0x10];
    memset(buf, ' ', 0x10);
    sprintf(buf, "VER: %d", 1);
    lcd_write_line("Ch3f 0S :3", 0);
    lcd_write_line(buf, 1);
    lcd_update_display();
}

int main(void){
    bool update_info = false;
    uint8_t count;
    char buf[0x10];
    clock_setup();
    systick_setup();
    msleep(20);
    lcd_init();
    init_leds();
    gpio_toggle(GPIOC, GPIO14);
    clock_setup();
    init_led_map();
    init_i2c();
    gyro_init();

    lcd_clear();
    start_lcd();
    msleep(10000);
    setup_led_write_clk();
    gyro_print_data();
    for(;;){
        gpio_toggle(GPIOC, GPIO13);
        update_info = gyro_update();
        if(gyro.update_display){
            gyro_calc_led_color(&tmp);
            if(count > 5){
                //gyro_print_data();
                count = 0;
                sprintf(buf, "R:%x G:%x B:%x", led_map[0].r, led_map[0].g, led_map[0].b);
                lcd_write_line(buf, 0);
                lcd_update_display();
            }
            memcpy(&led_map[0], &tmp, 3);
            memcpy(&led_map[1], &tmp, 3);
            memcpy(&led_map[2], &tmp, 3);
            count++;
        }
        msleep(200); //sleep for 1 second
    }

    return 0;
}
