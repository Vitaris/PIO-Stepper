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

bool blink_500ms;
bool lcd_refresh;

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

void core1_entry() {

    // LCD
    lcd = lcd_create(2, 3, 4, 5, 6, 7, 8, 16, 2);
    int a = 0;
    while (1)
    {
        if (lcd_refresh == true)
        { 
            string2LCD(lcd, 0, 0, "Abc");
            int2LCD(lcd, 0, 1, 6, a++); 
            // float2LCD(lcd, 0, 2, 8, cutter->servo_0->servo_position);

            if (a > 100000) {
                a = 0;
            }

            lcd_refresh = false;
        }
    }
}

bool servo_timer_callback(struct repeating_timer *t) {
    // machine_compute(cutter);
    return true;
}

bool LCD_refresh_timer_callback(struct repeating_timer *t) {
    lcd_refresh = true;
    return true;
}

int main() {
    // cutter = create_machine();

    // Initialize
    stdio_init_all();


    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    blink_pin_forever(pio, 0, offset, 1, 1);

    // Timer for servo control
    add_repeating_timer_ms(-1, servo_timer_callback, NULL, &servo_timer);

    // 100ms LCD refresh timer
    add_repeating_timer_ms(100, LCD_refresh_timer_callback, NULL, &LCD_refresh_timer);
    
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

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}