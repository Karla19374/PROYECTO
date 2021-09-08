#ifndef __S7_H__
#define __S7_H__

#include <Arduino.h>

extern uint8_t pinA, pinB, pinC, pinD, pinE, pinF, pinG, pindp;

void configurarDisplay(uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E, uint8_t F, uint8_t G, uint8_t dp);
void desplegarSeg(uint8_t digito);
void desplegarPun(boolean punto);
#endif // __S7_H__