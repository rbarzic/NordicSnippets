#pragma once

#define BUTTON1_PIN (13UL)
#define BUTTON2_PIN (14UL)
#define BUTTON3_PIN (15UL)
#define BUTTON4_PIN (16UL)

int button_pressed(uint32_t button);
void buttons_config(void);
