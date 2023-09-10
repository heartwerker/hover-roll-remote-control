
#ifndef MY_UI_UX_H
#define MY_UI_UX_H
#include <Arduino.h>

void ui_blink(int how_often, int delay_time)
{
	for (int i = 0; i < (how_often*2); i++)
	{
		digitalWrite(LED_BUILTIN, i % 2);
		delay(delay_time);
	}
}

void ui_indicate_wifi_connected()
{
	pinMode(LED_BUILTIN, OUTPUT);
	ui_blink(2, 200);
}

void ui_indicate_homekit_started()
{
	ui_blink(4, 100);
}

void ui_indicate_setter_event()
{
	ui_blink(1, 100);
}



#endif // MY_UI_UX_H#