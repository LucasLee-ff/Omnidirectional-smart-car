/*
 * key.c
 *
 *  Created on: 2021Äê7ÔÂ25ÈÕ
 *      Author: windows
 */
#include "headfile.h"
#include "key.h"

void Key_Init()
{
    gpio_init(Key1, GPI, 1, IN_PULLDOWN);
    gpio_init(Key2, GPI, 1, IN_PULLDOWN);
    gpio_init(Key3, GPI, 1, IN_PULLDOWN);
}

uint8 Key_Read()
{
    if(gpio_get(Key1)==0 || gpio_get(Key2)==0 || gpio_get(Key3)==0)
    {
        systick_delay_ms(200);
        if(gpio_get(Key1)==0)
            return 1;
        else if(gpio_get(Key2)==0)
            return 2;
        else if(gpio_get(Key3)==0)
            return 3;
    }
    return 0;
}
