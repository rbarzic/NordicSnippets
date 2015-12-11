#include <nrf.h>
#include <stdbool.h>
#include "led.h"
#include "buttons.h"
#include "../radio_protocol.h"

#define THROTTLE_STEP 50
#define PITCH_STEP    20

void medium_delay(void) {
    uint32_t volatile tmo;

    tmo = 1000000;
    while (tmo--);

}

void nrf_radio_init(uint32_t base,uint32_t prefix,uint32_t* packet) {

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
  NRF_RADIO->BASE0   = base;
  NRF_RADIO->PREFIX0 = prefix << RADIO_PREFIX0_AP0_Pos;

  // Use logical address 0 (BASE0 + PREFIX0 byte 0)
  NRF_RADIO->TXADDRESS = 0 << RADIO_TXADDRESS_TXADDRESS_Pos;

  // Initialize CRC (two bytes)
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
  NRF_RADIO->CRCPOLY = 0x0000AAAA;
  NRF_RADIO->CRCINIT = 0x12345678;

  // Enable fast rampup, new in nRF52
  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) |
                        (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);

  // 0dBm output power, sending packets at 2400MHz
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos;
  NRF_RADIO->FREQUENCY = 0 << RADIO_FREQUENCY_FREQUENCY_Pos;

  // Configure address of the packet and logic address to use
  NRF_RADIO->PACKETPTR = (uint32_t)&packet[0];

  // Configure shortcuts to start as soon as READY event is received, and disable radio as soon as packet is sent.
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);


}


// min : 0
// max : 1000
static int32_t throttle = 0;    // used
static int32_t yaw      = 500;  // unused
static int32_t pitch    = 500;  // used
static int32_t roll     = 500;  // unused

// Button 1 : throttle ++
// Button 3 : throttle --

// Button 2 : forward
// Button 4 : backward

void update_packet(rc_packet_t * pkt) {
    pkt->magic1 = MAGIC1;
    pkt->magic2 = MAGIC2;
    pkt->yaw       = (uint16_t)yaw;
    pkt->pitch     = (uint16_t)pitch;
    pkt->roll      = (uint16_t)roll;
    pkt->throttle  = (uint16_t)throttle;
}

rc_packet_t c_packet;

int main(void)
{
  // Packet to send
  uint8_t packet[16] = "demopacket";


  bool a_button_was_pressed = false;
  led_config();
  led_startup();


  // Start HFCLK from crystal oscillator. The radio needs crystal to function correctly.
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

//  // Configure radio with 2Mbit Nordic proprietary mode
//  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;
//
//  // Configure packet with no S0,S1 or Length fields and 8-bit preamble.
//  NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) |
//                     (0 << RADIO_PCNF0_S0LEN_Pos) |
//                     (0 << RADIO_PCNF0_S1LEN_Pos) |
//                     (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos) |
//                     (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
//
//  // Configure static payload length of 16 bytes. 3 bytes address, little endian with whitening enabled.
//  NRF_RADIO->PCNF1 =  (16 << RADIO_PCNF1_MAXLEN_Pos) |
//                      (16 << RADIO_PCNF1_STATLEN_Pos) |
//                      (2  << RADIO_PCNF1_BALEN_Pos) |
//                      (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |
//                      (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);
//
//  // initialize whitening value
//  NRF_RADIO->DATAWHITEIV = 0x55;
//
//  // Configure address Prefix0 + Base0
//  NRF_RADIO->BASE0   = 0x0000BABE;
//  NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;
//
//  // Use logical address 0 (BASE0 + PREFIX0 byte 0)
//  NRF_RADIO->TXADDRESS = 0 << RADIO_TXADDRESS_TXADDRESS_Pos;
//
//  // Initialize CRC (two bytes)
//  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
//                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
//  NRF_RADIO->CRCPOLY = 0x0000AAAA;
//  NRF_RADIO->CRCINIT = 0x12345678;
//
//  // Enable fast rampup, new in nRF52
//  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) |
//                        (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
//
//  // 0dBm output power, sending packets at 2400MHz
//  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos;
//  NRF_RADIO->FREQUENCY = 0 << RADIO_FREQUENCY_FREQUENCY_Pos;
//
//  // Configure address of the packet and logic address to use
//  NRF_RADIO->PACKETPTR = (uint32_t)&packet[0];
//
//  // Configure shortcuts to start as soon as READY event is received, and disable radio as soon as packet is sent.
//  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
//                      (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);

  // nrf_radio_init(0x0000BABE,0x41,(uint32_t*)packet);
  nrf_radio_init(0x0000BABE,0x41,(uint32_t*)&c_packet);
  buttons_config();
  // Continuously send the same packet
  while (1)
  {

      if(button_pressed(BUTTON1_PIN)) {
          a_button_was_pressed = true;
          throttle += THROTTLE_STEP;
          if(throttle>1000) throttle=1000;
      }
      if(button_pressed(BUTTON3_PIN)) {
          a_button_was_pressed = true;
          throttle -= THROTTLE_STEP;
          if(throttle<0) throttle=0;
      }
      if(button_pressed(BUTTON2_PIN)) {
          a_button_was_pressed = true;
          pitch += PITCH_STEP;
          if(pitch>1000) pitch=1000;
      }
      if(button_pressed(BUTTON4_PIN)) {
          a_button_was_pressed = true;
          pitch -= PITCH_STEP;
          if(pitch<0) pitch=0;
      }

      update_packet(&c_packet);

      if(a_button_was_pressed) {
          led_on();
          NRF_RADIO->TASKS_TXEN = 1;
          while (NRF_RADIO->EVENTS_DISABLED == 0);
          NRF_RADIO->EVENTS_DISABLED = 0;
          medium_delay();
          led_off();
          medium_delay();
          a_button_was_pressed = false;
      }


  }
}
