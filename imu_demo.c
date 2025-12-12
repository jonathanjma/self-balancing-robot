/**
 * Self-Balancing Robot!
 */

// Include standard libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
// Include PICO libraries
#include "pico/multicore.h"
#include "pico/stdlib.h"
// Include hardware libraries
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
// Include custom libraries
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"
// #include "vga16_graphics_v2.h"

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// GPIO for PWM
#define PWM_L_FW 4
#define PWM_L_BW 5
#define PWM_R_FW 6
#define PWM_R_BW 7

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV 25.0
uint slice_num_l; /// todo later swithc based on orientation of robot
uint slice_num_r;

// Min and max motor control values
#define MIN_DUTY_CYCLE 1900 // minimmum duty cycle to get the motors moving on ground
#define MAX_DUTY_CYCLE 4000
#define MAX_CTRL 2100 // 4000-1900
#define MIN_CTRL -2100

// Bias for the z-axis acceleration
#define az_bias float2fix15(0.22)

// Starting time to keep track of when the angle sequence playback started
static int begin_time;

typedef enum {
    FORWARDS,
    BACKWARDS,
    STABLE
} direction_state_t;
volatile direction_state_t current_direction = STABLE; // assume default not pressed

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 complementary_angle;
fix15 accel_angle, gyro_angle;

volatile int target_angle = 0; // always want target angle to be 0 because that is it facing upwards

// Character array for VGA graph text
char screentext[40];
char str_buffer[256] = {0}; // string buffer for the PID text to display on the screen

// Draw speed for VGA graph
int threshold = 250;

// Semaphores
static struct pt_sem vga_semaphore;
static struct pt_sem serial_semaphore;

// PWM duty cycle
volatile int control_l = 0;
volatile int control_r = 0;
volatile int filtered_control = 0;
volatile int filtered_old_control = 0;

// PID parameters
volatile float kp = 890;
volatile float kd = 420;
volatile float ki = 10; // next time tune look at georige suggestions

volatile int PID_output;

float error_sum = 0;
float error_sum_max = 2000;

volatile int counter = 0;
volatile int global_counter = 0;

#define IMU_POWER 26
#define LED_PIN 25
#define UART_ID uart0
#define UART_RX_PIN 13

const int positive_bias_correction = -11;
const int negative_bias_correction = 3;
const int true_zero_correction = -4;

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(slice_num_l);

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    mpu6050_read_raw(acceleration, gyro);

    // Complementary filter
    // Don't need ax because we are just rotating about the x axis
    fix15 ax = acceleration[0];
    fix15 az = acceleration[2] + az_bias;

    // tan = opposite/adjacent = a_z/-a_x (atan2 takes args in reverse order)
    accel_angle = multfix15(divfix(ax, az), oneeightyoverpi);

    // We are rotating about the y axis
    fix15 gy = -gyro[1];
    fix15 gyro_angle_delta = multfix15(gy, zeropt001);
    gyro_angle = complementary_angle - gyro_angle_delta;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(gyro_angle, zeropt999) + multfix15(accel_angle, zeropt001);
    if (global_counter > 19000) {
        current_direction = STABLE; // after 17 total seconds (4 seconds after backwards) stay stable forever
        gpio_put(LED_PIN, 1);
    } else if (global_counter > 15000) {
        current_direction = BACKWARDS; // after 13 total seconds (5 seconds after stable) go backwards
        gpio_put(LED_PIN, 0);
    } else if (global_counter > 8000) {
        current_direction = STABLE; // after 8 total seconds (3 seconds after stable) be stable for 5 seconds
        gpio_put(LED_PIN, 1);
    } else if (global_counter > 5000) {
        current_direction = FORWARDS; // after 5 seconds go forwards
        gpio_put(LED_PIN, 0);
    } else {
        current_direction = STABLE; // first 5 seconds be stable
        gpio_put(LED_PIN, 1);
    }

    if (current_direction == STABLE) {
        if (fix2float15(complementary_angle) >= true_zero_correction) { // prevent positive bias/lean
            target_angle = positive_bias_correction;
        } else { // address negative bias
            target_angle = negative_bias_correction;
        }
        counter = 0;
    } else {
        if (counter > 0) { // have target angle positive so go backwards
            counter++;     // do it for 200 ms (frequency of ISR is 1000times a second)
            if (counter >= 150) {
                counter = 0; // reset counter
            }
            if (current_direction == BACKWARDS) {
                target_angle = -14;
            } else if (current_direction == FORWARDS) {
                target_angle = 5;
            }
        }
        // if equal 0 then stable from the reset above and if negative it is counting to stay negative until 2 seconds has passed
        else if (counter <= 0) { // keep target angle stable for 2 seconds
            counter--;
            if (counter <= -2750) {
                counter = 1; // go back to greater than 0 so go to top conditional of going forwards a bit
            }
            // stabalization code
            if (fix2float15(complementary_angle) >= true_zero_correction) { // prevent positive bias/lean
                target_angle = positive_bias_correction;
            } else { // address negative bias
                target_angle = negative_bias_correction;
            }
        }
    }

    float error = target_angle - fix2float15(complementary_angle);

    // Accumulate and clamp error sum for integration term
    error_sum += error;
    error_sum = min(error_sum, error_sum_max);
    error_sum = max(error_sum, -error_sum_max);

    // negate PID output so robot stays upright
    PID_output = (error * kp) + fix2int15(gy) * kd + error_sum * ki;
    if (PID_output > MAX_CTRL) {
        PID_output = MAX_CTRL;
    } else if (PID_output < MIN_CTRL) {
        PID_output = MIN_CTRL;
    }
    if (PID_output >= 0) {
        PID_output += MIN_DUTY_CYCLE;
    } else {
        PID_output -= MIN_DUTY_CYCLE;
    }

    // pwm_set_chan_level(slice_num_l, PWM_CHAN_A, kp);
    // pwm_set_chan_level(slice_num_l, PWM_CHAN_B, 0);
    // pwm_set_chan_level(slice_num_r, PWM_CHAN_A, kp);
    // pwm_set_chan_level(slice_num_r, PWM_CHAN_B, 0);

    if (PID_output <= 0) {
        pwm_set_chan_level(slice_num_l, PWM_CHAN_A, PID_output);
        pwm_set_chan_level(slice_num_l, PWM_CHAN_B, 0);
        pwm_set_chan_level(slice_num_r, PWM_CHAN_A, PID_output);
        pwm_set_chan_level(slice_num_r, PWM_CHAN_B, 0);
    } else {
        pwm_set_chan_level(slice_num_l, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num_l, PWM_CHAN_B, -PID_output);
        pwm_set_chan_level(slice_num_r, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice_num_r, PWM_CHAN_B, -PID_output);
    }
    global_counter++; // update global counter each interrupt
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
    PT_SEM_SIGNAL(pt, &serial_semaphore);
}

// // Thread that draws to VGA display
// static PT_THREAD(protothread_vga(struct pt *pt)) {
//     // Indicate start of thread
//     PT_BEGIN(pt);
//
//     // We will start drawing at column 81
//     static int xcoord = 81;
//
//     // Rescale the measurements for display
//     static float OldRange = 500.; // (+/- 250)
//     static float NewRange = 150.; // (looks nice on VGA)
//     static float OldMin = -250.;
//     static float OldMax = 250.;
//
//     // Control rate of drawing
//     static int throttle;
//
//     // Draw the static aspects of the display
//     setTextSize(1);
//     setTextColor(WHITE);
//
//     // Draw bottom plot
//     drawHLine(75, 430, 5, CYAN);
//     drawHLine(75, 355, 5, CYAN);
//     drawHLine(75, 280, 5, CYAN);
//     drawVLine(80, 280, 150, CYAN);
//     sprintf(screentext, "0");
//     setCursor(50, 350);
//     writeString(screentext);
//     sprintf(screentext, "3500");
//     setCursor(50, 280);
//     writeString(screentext);
//
//     // Draw top plot
//     drawHLine(75, 230, 5, CYAN);
//     drawHLine(75, 155, 5, CYAN);
//     drawHLine(75, 80, 5, CYAN);
//     drawVLine(80, 80, 150, CYAN);
//     sprintf(screentext, "0");
//     setCursor(50, 150);
//     writeString(screentext);
//     sprintf(screentext, "90");
//     setCursor(45, 75);
//     writeString(screentext);
//     sprintf(screentext, "-90");
//     setCursor(45, 225);
//     writeString(screentext);
//
//     while (true) {
//         // Wait on semaphore
//         PT_SEM_WAIT(pt, &vga_semaphore);
//         // Increment drawspeed controller
//         throttle += 1;
//         // If the controller has exceeded a threshold, draw
//         if (throttle >= threshold) {
//             // Zero drawspeed controller
//             throttle = 0;
//
//             // Erase a column
//             drawVLine(xcoord, 0, 480, BLACK);
//
//             // Draw bottom  plot (multiply by 0.089 to scale from +/-3500 to +/-250)
//             // Low pass motor control signal so the graph is less noisy
//             filtered_old_control = filtered_control;
//             filtered_control = filtered_old_control + ((control_l - filtered_old_control) >> 5);
//             drawPixel(xcoord, 430 - (int)(NewRange * ((float)((filtered_control * 0.0714) - OldMin) / OldRange)), ORANGE);
//
//             // Draw top plot (multiply by 2.8  to scale from +/-90 to +/-250)
//             // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro_angle) * 2.8) - OldMin) / OldRange)), RED);
//             // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(accel_angle) * 2.8) - OldMin) / OldRange)), GREEN);
//             drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(complementary_angle) * 2.8) - OldMin) / OldRange)), WHITE);
//             drawPixel(xcoord, 230 - (int)(NewRange * ((float)(((target_angle) * 2.8) - OldMin) / OldRange)), BLUE);
//
//             // Update horizontal cursor
//             if (xcoord < 609) {
//                 xcoord += 1;
//             } else {
//                 xcoord = 81;
//             }
//
//             // Draw PID info text on screen
//             setTextColorBig(WHITE, BLACK);
//             setCursor(10, 20);
//             sprintf(str_buffer, "Kp:  %.2f   Ki: %.2f   Kd: %.2f   Targ_ang: %d     Curr_ang: %d         ", (kp), ki, kd, target_angle, fix2int15(complementary_angle));
//             writeStringBig(str_buffer);
//         }
//     }
//     // Indicate end of thread
//     PT_END(pt);
// }

// Thread for serial monitor to debug instead of the vga
static PT_THREAD(protothread_serial_core1(struct pt *pt)) {
    // Indicate start of thread
    PT_BEGIN(pt);

    // Control rate of drawing
    static int throttle;

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &serial_semaphore);
        // Increment drawspeed controller
        throttle += 1;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) {
            // Zero drawspeed controller
            throttle = 0;
            // sprintf(pt_serial_out_buffer, "current PID output: %d\n", PID_output);
            sprintf(pt_serial_out_buffer, "current angle: %f\n", fix2float15(complementary_angle));
            serial_write;
            sprintf(pt_serial_out_buffer, "target angle: %d\n", target_angle);
            serial_write;
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread for changing draw speed, PID parameters, or target angle
static PT_THREAD(protothread_serial(struct pt *pt)) {
    PT_BEGIN(pt);
    static char classifier;
    static int test_in;
    static float float_in;
    while (1) {
        sprintf(pt_serial_out_buffer, "input a command (t for timestep, p for kp, i for ki, d for kd, and a for target angle): ");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%c", &classifier);

        if (classifier == 'p') {
            sprintf(pt_serial_out_buffer, "kp: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%f", &float_in);
            if (float_in >= 0) {
                kp = float_in;
            }
        } else if (classifier == 'd') {
            sprintf(pt_serial_out_buffer, "kd: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%f", &float_in);
            if (float_in >= 0) {
                kd = float_in;
            }
        } else if (classifier == 'i') {
            sprintf(pt_serial_out_buffer, "ki: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%f", &float_in);
            if (float_in >= 0) {
                ki = float_in;
            }
        } else if (classifier == 'a') {
            sprintf(pt_serial_out_buffer, "target angle (-90 to 90 degrees): ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%d", &test_in);
            if (test_in >= -90 && test_in <= 90) {
                target_angle = test_in;
            }
        } else if (classifier == 'm') {
            sprintf(pt_serial_out_buffer, "positive number to go forwards, negative number to go backwards, 0 to stop: ");
            serial_write;
            serial_read;
            sscanf(pt_serial_in_buffer, "%d", &test_in);
            if (test_in > 0) {
                current_direction = FORWARDS;
            } else if (test_in < 0) {
                current_direction = BACKWARDS;
            } else {
                current_direction = STABLE;
            }
        } else {
            sprintf(pt_serial_out_buffer, "invalid command");
            serial_write;
        }
    }
    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    // pt_add_thread(protothread_vga);
    pt_add_thread(protothread_serial_core1);
    pt_schedule_start;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true);

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    // initVGA();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    // code to reset the I2C when reboot
    gpio_init(IMU_POWER);
    gpio_set_dir(IMU_POWER, GPIO_OUT);
    gpio_set_drive_strength(IMU_POWER, GPIO_DRIVE_STRENGTH_12MA);
    gpio_put(IMU_POWER, 0);
    sleep_ms(1000);
    gpio_put(IMU_POWER, 1);
    sleep_ms(1000);

    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    gpio_set_function(PWM_L_FW, GPIO_FUNC_PWM);
    gpio_set_function(PWM_L_BW, GPIO_FUNC_PWM);
    gpio_set_function(PWM_R_FW, GPIO_FUNC_PWM);
    gpio_set_function(PWM_R_BW, GPIO_FUNC_PWM);

    // Find out which PWM slice
    slice_num_l = pwm_gpio_to_slice_num(PWM_L_FW);
    slice_num_r = pwm_gpio_to_slice_num(PWM_R_FW);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num_l);
    pwm_set_irq_enabled(slice_num_l, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num_l, WRAPVAL);
    pwm_set_clkdiv(slice_num_l, CLKDIV);
    pwm_set_wrap(slice_num_r, WRAPVAL);
    pwm_set_clkdiv(slice_num_r, CLKDIV);

    // This sets duty cycle
    pwm_set_chan_level(slice_num_l, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num_l, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num_r, PWM_CHAN_A, 0);
    pwm_set_chan_level(slice_num_r, PWM_CHAN_B, 0);

    // Start the channel
    pwm_set_mask_enabled(11u << slice_num_l);

    // Initialize the LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the UART channel and RX pin
    // uart_init(UART_ID, 4800);
    // uart_set_format(UART_ID, 8, 1, UART_PARITY_ODD);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    // gpio_pull_up(UART_RX_PIN);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial);

    pt_schedule_start;
}
