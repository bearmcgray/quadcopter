#ifndef __LED_H
#define __LED_H

#define LED0 0x01
#define LED1 0x02
#define LED2 0x04
#define LED3 0x08
#define LED_ALL 0x0f

typedef struct{
	int state;
}tled;

void LED_Init(tled *led);
void LED_On(tled *led, int num);
void LED_Off(tled *led, int num);
void LED_Toggle(tled *led, int num);

#endif //__LED_H
