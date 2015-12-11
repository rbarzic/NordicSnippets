#include <nrf.h>
#include <stdint.h>
#include <stdbool.h>

#define PIN_LED  (19UL)
#define PIN_LED2  (20UL)

void led_config(void) {

    // Configure GPIO pin as output with standard drive strength.
    NRF_GPIO->PIN_CNF[PIN_LED] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    NRF_GPIO->PIN_CNF[PIN_LED2] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void led_startup() {
    for(int i=0;i<2;i++)
    {
        uint32_t volatile tmo;

        tmo = 1000000;
        while (tmo--);
        NRF_GPIO->OUTSET = (1UL << PIN_LED);
        NRF_GPIO->OUTCLR = (1UL << PIN_LED2);

        tmo = 1000000;
        while (tmo--);
        NRF_GPIO->OUTSET = (1UL << PIN_LED2);
        NRF_GPIO->OUTCLR = (1UL << PIN_LED);
    }
    // Turn-off led
    NRF_GPIO->OUTSET = (1UL << PIN_LED);
    NRF_GPIO->OUTSET = (1UL << PIN_LED2);

}
void led_on(void) {
    NRF_GPIO->OUTCLR = (1UL << PIN_LED);
}

void led_off(void) {
    NRF_GPIO->OUTSET = (1UL << PIN_LED);
}


void led2_on(void) {
    NRF_GPIO->OUTCLR = (1UL << PIN_LED2);
}

void led2_off(void) {
    NRF_GPIO->OUTSET = (1UL << PIN_LED2);
}
