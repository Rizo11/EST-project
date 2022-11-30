/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include <math.h>
#include "nrf_gpio.h"
#include "pca10059.h"

#define ON (1)
#define OFF (0)
#define SW1         NRF_GPIO_PIN_MAP(1,6)
#define RED         LED2_R
#define GREEN       LED2_G
#define BLUE        LED2_B

void cfg_leds();
void cfg_sws();
void turn_on_led(int led_id);
void turn_off_led(int led_id);

int main(void)
{
    /* Configure board. */
    bsp_board_init(BSP_INIT_LEDS);
    
    cfg_leds();
    cfg_sws();

    // RRGGGB
    int leds[] = {1,1,2,2,2,3};
    // int leds1[] = {RED, RED, GREEN, GREEN, GREEN, BLUE};

    int current_led = 0;
    while (true)
    {   
        while (!nrf_gpio_pin_read(SW1))
        {

            // nrf_gpio_pin_write(leds[current_led%6], ON);
            bsp_board_led_on(leds[current_led%6]);
            nrf_delay_ms(1000);

            // nrf_gpio_pin_write(leds[current_led%6], OFF);
            bsp_board_led_off(leds[current_led%6]);
            current_led++;

            // avoiding overflow of ints
            if (current_led >= 6666)
            {
                current_led = 0;
            }
            
        }

        // additional task led stays on even when button is released
        // nrf_gpio_pin_write(leds[current_led%6], ON);
        bsp_board_led_on(leds[current_led%6]);
    }
    
}

/**
 * configuring pins P0.08(RED), P1.09(GREEN), P0.12(BLUE) as output 
*/
void cfg_leds()
{
    nrf_gpio_cfg_output(RED);
    nrf_gpio_cfg_output(GREEN);
    nrf_gpio_cfg_output(BLUE);
}


void cfg_sws()
{
    // the correct version is pull down, but it is not working
    nrf_gpio_cfg_input(SW1, NRF_GPIO_PIN_PULLUP);
}

