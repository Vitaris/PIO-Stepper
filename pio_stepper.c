#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h"
#include "blink.pio.h"

#include "lcd.h"
#include "button.h"

// Timers
uint64_t old_cycle_time = 0;
struct repeating_timer servo_timer;
struct repeating_timer LCD_refresh_timer;

// LCD
lcd_t lcd;

// Buttons
button_t F1;
button_t F2;
button_t F3;
button_t F4;

bool F1_pressed;
bool F2_pressed;
bool F3_pressed;
bool F4_pressed;
bool some_released;

bool blink_500ms;
bool lcd_refresh;

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);
void update_blink(PIO pio, uint sm, uint freq);
uint offset;
uint offset_encoder;

bool stepper_enable = false;
uint speed;

void core1_entry() {

    // LCD
    lcd = lcd_create(2, 3, 4, 5, 6, 7, 8, 16, 2);
    int enc_old = 0;
    while (1)
    {
        if (lcd_refresh == true)
        {
            int32_t enc = quadrature_encoder_get_count(pio1, 0);
            string2LCD(lcd, 0, 0, "Enc:");
            string2LCD(lcd, 13, 0, "rev");
            string2LCD(lcd, 13, 1, "r/s");
            // int2LCD(lcd, 0, 1, 6, a++); 
            float2LCD(lcd, 4, 0, 8, (float)enc / 8000.0);
            float2LCD(lcd, 4, 1, 8, (float)(enc - enc_old) * 10 / 8000.0);

            if (F1_pressed){
                string2LCD(lcd, 0, 1, "F1");
            } 
            else if (F2_pressed){
                string2LCD(lcd, 0, 1, "F2");
            }
            else if (F3_pressed){
                string2LCD(lcd, 0, 1, "F3");
            }
            else if (F4_pressed){
                string2LCD(lcd, 0, 1, "F4");
            } 
            else if (some_released) {
                string2LCD(lcd, 0, 1, "  ");
            }

            F1_pressed = false;
            F2_pressed = false;
            F3_pressed = false;
            F4_pressed = false;
            some_released = false;

            enc_old = enc;
            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {
    button_compute(F1);
    button_compute(F2);
    button_compute(F3); 
    button_compute(F4);

    if (F1->state_dropped){
        F1_pressed = true;
    } 
    else if (F2->state_dropped){
        pio_sm_set_enabled(pio0, 1, false);
        pio_sm_restart(pio0, 1);
        F2_pressed = true;
    }
    else if (F3->state_dropped){
        pio_sm_set_enabled(pio0, 1, true);
        blink_pin_forever(pio0, 1, offset, 0, speed);
        speed += 10000;
        F3_pressed = true;
    }
    else if (F4->state_dropped){
        if (stepper_enable){
            stepper_enable = false;
        } 
        else {
            stepper_enable = true;
        }
        F4_pressed = true;
    } 
    else if (F1->state_raised || F2->state_raised || F3->state_raised || F4->state_raised) {
        some_released = true;
    }

    if (stepper_enable) {
        gpio_put(14, 1);
    } else {
        gpio_put(14, 0);
    }

    return true;
}

bool LCD_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    return true;
}

int main() {
    // cutter = create_machine();

    // Initialize
    // stdio_init_all();

    PIO pio = pio0;
    offset = pio_add_program(pio, &blink_program);
    blink_pin_forever(pio, 0, offset, 25, 2);
    speed = 10000;

    F1 = create_button(10);
    F2 = create_button(11);
    F3 = create_button(12);
    F4 = create_button(13);

    offset_encoder = pio_add_program(pio1, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio1, 0, offset_encoder, 16, 0);

    stepper_enable = false;
    gpio_init(14);
    gpio_set_dir(14, GPIO_OUT);

    // Timer for servo control
    add_repeating_timer_ms(-1, servo_timer_callback, NULL, &servo_timer);

    // 100ms LCD refresh timer
    add_repeating_timer_ms(-100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
    // Launch core1
    multicore_launch_core1(core1_entry);

    // Initial wait 
    // busy_wait_ms(500);

    while (1)
    {
        tight_loop_contents();
    }
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    // printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}

void update_blink(PIO pio, uint sm, uint freq) {
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}