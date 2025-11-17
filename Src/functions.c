/*
 * functions.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "functions.h"
#include "targets.h"

// long map(long x, long in_min, long in_max, long out_min, long out_max)
//{
//     if (x < in_min) {
//         x = in_min;
//     }
//     if (x > in_max) {
//         x = in_max;
//     }
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    if (x >= in_max)
        return out_max;
		if (x <= in_min)
        return out_min;
    if (in_min > in_max)
        return map(x, in_max, in_min, out_max, out_min);
    if (out_min == out_max)
        return out_min;
    const long in_mid = (in_min + in_max) >> 1;
    const long out_mid = (out_min + out_max) >> 1;
    if (in_min == in_mid)
        return out_mid;
    if (x <= in_mid)
        return map(x, in_min, in_mid, out_min, out_mid);
    else
        return map(x, in_mid + 1, in_max, out_mid, out_max);
}

uint32_t getAbsDif(int number1, int number2)
{
    int result = number1 - number2;
    if (result < 0) {
        result = -result;
    }
    return (uint32_t)result;
}

/*
  get current value of UTILITY_TIMER timer as 16bit microseconds
 */
static inline uint16_t get_timer_us16(void) {
#if defined(STMICRO)
    return UTILITY_TIMER->CNT;
#elif defined(GIGADEVICES)
    return TIMER_CNT(UTILITY_TIMER);
#elif defined(ARTERY)
    return UTILITY_TIMER->cval;
#elif defined(NXP)
    //Return systick timer count value
    return SysTick->VAL;
#elif defined(WCH)
    return UTILITY_TIMER->CNT>>1;
#else
#error unsupported MCU
#endif
}

/*
  delay by microseconds, max 65535
 */
//void delayMicros(uint32_t micros)
//{
//#ifdef NXP
//    const uint32_t cval_start = SysTick->VAL;
//
//    //Systick timer counts down so mirror subtraction
//    while (micros > 0) {
//    	uint32_t current = SysTick->VAL;
//    	uint32_t elapsed;
//
//    	//Handle wrap-around
//        if (cval_start >= current) {
//            elapsed = cval_start - current;
//        	GPIO3->PTOR = (1 << 27);	//ENC_A
//        } else {
//            // Handle wrap-around
//            elapsed = cval_start + ((uint32_t)SysTick->LOAD - current);
////            GPIO3->PTOR = (1 << 28); 	//ENC_I
//        }
//
//        if (elapsed >= micros) {
//            break;
//        }
//	}
//
//#else
//    const uint16_t cval_start = get_timer_us16();
//    while ((uint16_t)(get_timer_us16() - cval_start) < (uint16_t)micros) {
//    }
//#endif
//}

//void delayMicros(uint32_t micros) {
//    uint32_t start = SysTick->VAL;           // Current counter value
//    uint32_t ticks = micros;                     // Since 1 tick = 1 µs at 1 MHz
//    uint32_t reload = SysTick->LOAD + 1;     // SysTick reload value
//
//    while (ticks > 0) {
//        uint32_t current = SysTick->VAL;
//        uint32_t elapsed;
//
//        if (current <= start) {
//            elapsed = start - current;
//            GPIO3->PTOR = (1 << 27);	//ENC_A
//        } else {
//            // Counter wrapped around
//            elapsed = start + (reload - current);
//        }
//
//        if (elapsed >= ticks) {
////            break;
//        	//do nothing
//        }
//    }
//}

void delayMicros(uint32_t micros) {
    uint32_t start = SysTick->VAL;
    uint32_t reload = SysTick->LOAD + 1; // Full range including zero
    uint32_t elapsed = 0;

    while (elapsed < micros) {
        uint32_t current = SysTick->VAL;

        if (start >= current) {
            // Normal case: no wrap
            elapsed += (start - current);
            GPIO3->PTOR = (1 << 27);	//ENC_A
        } else {
            // Wrap-around occurred
            elapsed += (start + (reload - current));
        }

        start = current; // Update start for next iteration
    }
}


/*
  delay in millis, convenience wrapper around delayMicros
 */
void delayMillis(uint32_t millis)
{
    while (millis-- > 0) {
        delayMicros(1000UL);
    }
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++) {
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    }
    return (crc_u);
}

uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen)
{
    uint8_t crc = 0, i;
    for (i = 0; i < BufLen; i++) {
        crc = update_crc8(Buf[i], crc);
    }
    return (crc);
}

#ifdef MCU_AT421
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode,
    uint32_t pull_up_down, uint32_t pin)
{
    gpio_periph->cfgr = (((((gpio_periph->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
}
void gpio_mode_set(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down,
    uint32_t pin)
{
    gpio_periph->cfgr = (((((gpio_periph->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
    gpio_periph->pull = ((((((gpio_periph->pull))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * pull_up_down))));
}
#endif

#ifdef MCU_AT415
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode,
    uint32_t pull_up_down, uint32_t pin)
{
    __disable_irq();
    gpio_init_type gpio_init_struct;
    gpio_default_para_init(&gpio_init_struct);

    if (GPIO_MODE_MUX) {
    }

    gpio_init_struct.gpio_mode = mode;
    gpio_init_struct.gpio_pins = pin;
    gpio_init_struct.gpio_pull = pull_up_down;

    gpio_init(gpio_periph, &gpio_init_struct);

    __enable_irq();
}
#endif
