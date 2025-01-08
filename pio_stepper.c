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
#include "pulser.pio.h"

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
void update_blink(PIO pio, uint sm, uint freq);

PIO pio = pio0;

void core1_entry() {

    // LCD
    int a = 25;
    lcd = lcd_create(2, 3, 4, 5, 6, 7, 8, 16, 2);
    int enc_old = 0;
    while (1)
    {
        if (lcd_refresh == true)
        {
            int enc = quadrature_encoder_get_count(pio1, 0);
            string2LCD(lcd, 0, 0, "Enc:");
            string2LCD(lcd, 13, 0, "rev");
            string2LCD(lcd, 13, 1, "r/s");
            // int2LCD(lcd, 0, 1, 6, a++); 
            float2LCD(lcd, 4, 0, 8, (float)enc / 8000.0);
            float2LCD(lcd, 4, 1, 8, (float)(enc - enc_old) * 10 / 8000.0);

            pio->txf[0] = a--;
            if (a <= 0) {
                a = 25;
                }

            enc_old = enc;
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
    // stdio_init_all();


    
    uint offset = pio_add_program(pio, &pulser_program);
    pulser_program_init(pio0, 0, offset, 0);
    // blink_pin_forever(pio, 0, offset, 25, 2);
    // blink_pin_forever(pio, 1, offset, 0, 125000);

    pio_add_program(pio1, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio1, 0, 0, 16, 0);

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