cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pio_stepper)

pico_sdk_init()

add_executable(pio_stepper
    pio_stepper.c
    lcd.c
    button.c
    # pid/PID.c
    # servo_motor/servo_pwm.c
    # # servo_motor/pos_controller.c
    # servo_motor/servo_motor.c
    # machine/machine_controller.c
    # machine/mark_detector.c
)

pico_generate_pio_header(pio_stepper ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
pico_generate_pio_header(pio_stepper ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
pico_generate_pio_header(pio_stepper ${CMAKE_CURRENT_LIST_DIR}/stepper.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(pio_stepper 1)
pico_enable_stdio_uart(pio_stepper 0)

pico_add_extra_outputs(pio_stepper)

target_link_libraries(pio_stepper pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        hardware_adc
        )
        