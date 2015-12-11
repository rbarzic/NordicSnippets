#include <nrf.h>
#include <stdbool.h>
#include "led.h"
#include "buttons.h"
#include "pwm.h"

#include "../radio_protocol.h"

volatile rc_packet_t c_packet;

static int32_t throttle = 0;    // used
static int32_t yaw      = 500;  // unused
static int32_t pitch    = 500;  // used
static int32_t roll     = 500;  // unused


#define SERVO_MAX ((1UL << 15) | 1900)
#define SERVO_MIN ((1UL << 15) | 1100)
#define SERVO_MID ((1UL << 15) | 1500)

extern uint16_t* m_buf;
void medium_delay(void) {
    uint32_t volatile tmo;

    tmo = 100000;
    while (tmo--);

}




volatile bool packet_received;
bool error_in_packet;


void pwm_update_servos(void) {
    m_buf[0] = (uint16_t) ((1<<15) | (1000 + throttle));
    m_buf[1] = (uint16_t) ((1<<15) | (1000 + pitch));
    m_buf[2] = (uint16_t) ((1<<15) | (1000 + yaw));
    m_buf[3] = (uint16_t) ((1<<15) | (1000 + roll));
}
void RADIO_IRQHandler(void) {


    if (NRF_RADIO->EVENTS_CRCOK) {

        NRF_RADIO->EVENTS_CRCOK = 0;

    }

    if (NRF_RADIO->EVENTS_END) {

        NRF_RADIO->EVENTS_END = 0;

    }
    if (NRF_RADIO->EVENTS_ADDRESS) {
        NRF_RADIO->EVENTS_ADDRESS = 0;
    }
    packet_received = true;
    //medium_delay();
    //
}


int main(void)
{
  // Packet receive buffer
  volatile uint8_t packet[16];

  // Start HFCLK from crystal oscillator. The radio needs crystal to function correctly.
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

  led_config();
  led_startup();


  // Configure radio with 2Mbit Nordic proprietary mode
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;

  // Configure packet with no S0,S1 or Length fields and 8-bit preamble.
  NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) |
                     (0 << RADIO_PCNF0_S0LEN_Pos) |
                     (0 << RADIO_PCNF0_S1LEN_Pos) |
                     (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos) |
                     (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);

  // Configure static payload length of 16 bytes. 3 bytes address, little endian with whitening enabled.
  NRF_RADIO->PCNF1 =  (16 << RADIO_PCNF1_MAXLEN_Pos) |
                      (16 << RADIO_PCNF1_STATLEN_Pos) |
                      (2  << RADIO_PCNF1_BALEN_Pos) |
                      (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
                      (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

  // initialize whitening value
  NRF_RADIO->DATAWHITEIV = 0x55;

  // Configure address Prefix0 + Base0
  NRF_RADIO->BASE0   = 0x0000BABE;
  NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;

  // Use logical address 0 (BASE0 + PREFIX0 byte 0)
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;

  // Initialize CRC (two bytes)
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
  NRF_RADIO->CRCPOLY = 0x0000AAAA;
  NRF_RADIO->CRCINIT = 0x12345678;

  // Enable fast rampup, new in nRF52
  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) |
                        (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);

  // receiving packets at 2400MHz
  NRF_RADIO->FREQUENCY = 0 << RADIO_FREQUENCY_FREQUENCY_Pos;

  // Configure address of the packet and logic address to use
  NRF_RADIO->PACKETPTR = (uint32_t)&c_packet;


  for(int i=0;i<4;i++)
      m_buf[i] = SERVO_MID;

  pwm_init();
  // Continually receive packet


#if 1
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

  NRF_RADIO->INTENSET = (RADIO_INTENSET_END_Enabled << RADIO_INTENSET_END_Pos); // |
  // (RADIO_INTENSET_ADDRESS_Enabled << RADIO_INTENSET_ADDRESS_Pos);

  NVIC_SetPriority(RADIO_IRQn, 3);
  NVIC_EnableIRQ(RADIO_IRQn);


  NRF_RADIO->TASKS_RXEN = 1;
  packet_received = false;


  while(1) {
      error_in_packet = false;
      __WFI();
      if(packet_received) {
          packet_received = false;
          if((c_packet. magic1 == MAGIC1) && (c_packet. magic2 == MAGIC2)) {
              led_on();
              if(c_packet.throttle > 1000) {
                  throttle = 0;
                  error_in_packet = true;
              } else {
                  throttle = c_packet.throttle;
              }

              if(c_packet.yaw > 1000) {
                  yaw = 500;
                  error_in_packet = true;
              } else {
                  yaw = c_packet.yaw;
              }
              if(c_packet.pitch > 1000) {
                  pitch = 500;
                  error_in_packet = true;
              } else {
                  pitch = c_packet.pitch;
              }
              if(c_packet.roll > 1000) {
                  roll = 500;
                  error_in_packet = true;
              } else {
                  roll = c_packet.roll;
              }

              if(! error_in_packet) {
                  pwm_update_servos();

                  NRF_PWM0->TASKS_SEQSTART[0] = 1;
                  while (NRF_PWM0->EVENTS_SEQEND[0] == 0);
                  NRF_PWM0->EVENTS_SEQEND[0] = 0;

              }
              medium_delay();
              led_off();
          }
          NRF_RADIO->TASKS_RXEN = 1;


          //
      }
  }
#else

  // Configure shortcuts to start as soon as READY event is received, and disable radio as soon as packet is received.
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);


  while (1)
  {

      NRF_RADIO->TASKS_RXEN = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    NRF_RADIO->EVENTS_DISABLED = 0;






    if (NRF_RADIO->EVENTS_CRCOK)
    {

        if(c_packet.magic1 == MAGIC1) {
            led_on();
            medium_delay();
            led_off();
        }
        NRF_RADIO->EVENTS_CRCOK = 0;
        c_packet.magic1 = 0;
        c_packet.magic2 = 0;
    }
  }
  #endif
}
