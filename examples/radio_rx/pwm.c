#include <nrf.h>
#include "pwm.h"

// 4 16-bit words - must be 32-bits aligned
uint32_t m_buf32[2];
uint16_t* m_buf;


void pwm_init(void) {
    NRF_GPIO->DIRSET = (1 << PWM_PIN_0) | (1 << PWM_PIN_1) | (1 << PWM_PIN_2);
    NRF_GPIO->OUTCLR = (1 << PWM_PIN_0) | (1 << PWM_PIN_1) | (1 << PWM_PIN_2);

    NRF_PWM0->PRESCALER   = PWM_PRESCALER_PRESCALER_DIV_16; // 1 us
    NRF_PWM0->PSEL.OUT[0] = PWM_PIN_0;
    NRF_PWM0->PSEL.OUT[1] = PWM_PIN_1;
    NRF_PWM0->PSEL.OUT[2] = PWM_PIN_2;
    NRF_PWM0->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM0->DECODER     = (PWM_DECODER_LOAD_Individual   << PWM_DECODER_LOAD_Pos) |
        (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM0->LOOP      = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);

    NRF_PWM0->COUNTERTOP = 20000; // 20ms period


    NRF_PWM0->SEQ[0].CNT = 4; //((sizeof(buf) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    NRF_PWM0->SEQ[0].PTR = (uint32_t)&m_buf[0];
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;

    NRF_PWM0->ENABLE = 1;

    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    while (NRF_PWM0->EVENTS_SEQEND[0] == 0);
    NRF_PWM0->EVENTS_SEQEND[0] = 0;

}
