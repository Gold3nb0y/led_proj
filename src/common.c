#include "common.h"
#include <stdint.h>

volatile uint32_t ticks;

//now make it enable and disable interrupts
void msleep(uint32_t delay){
    uint32_t wake = ticks + (delay); //scale delay down to match clock
    while(wake > ticks);
}

void memcpy(void* dest, void* src, uint32_t size){
    uint32_t i;
    char* d = dest;
    char* s = src;
    for(i = 0; i < size; i++)
        d[i] = s[i];
}

uint32_t strlen(char* str){
    uint32_t ret = 0;
    char* s = str;
    for(; *s != 0; s++)
        ret++;

    return ret;
}

void memset(void* dest, char val, uint32_t size){
    uint32_t i;
    char* d = dest;
    for(i = 0; i < size; i++)
        d[i] = val;
}

static char *utoa(unsigned int value, char *str, int base, int upper) {
    char *rc = str;
    char *ptr;
    char *low;
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    ptr = rc;
    do {
        *ptr++ = digits[value % base];
        value /= base;
    } while (value);
    *ptr-- = '\0';

    // Reverse
    for (low = rc; low < ptr; low++, ptr--) {
        char tmp = *low;
        *low = *ptr;
        *ptr = tmp;
    }

    return rc;
}

int sprintf(char *buf, const char *fmt, ...) {
    va_list args;
    char *str = buf;
    const char *p;

    va_start(args, fmt);

    for (p = fmt; *p != '\0'; p++) {
        if (*p != '%') {
            *str++ = *p;
            continue;
        }

        p++; // Skip '%'

        switch (*p) {
            case 's': {
                const char *s = va_arg(args, const char *);
                while (*s) *str++ = *s++;
                break;
            }
            case 'd': {
                int val = va_arg(args, int);
                if (val < 0) {
                    *str++ = '-';
                    val = -val;
                }
                char temp[12];
                utoa((unsigned int)val, temp, 10, 0);
                for (char *t = temp; *t; t++) *str++ = *t;
                break;
            }
            case 'u': {
                unsigned int val = va_arg(args, unsigned int);
                char temp[12];
                utoa(val, temp, 10, 0);
                for (char *t = temp; *t; t++) *str++ = *t;
                break;
            }
            case 'x':
            case 'X': {
                unsigned int val = va_arg(args, unsigned int);
                char temp[12];
                utoa(val, temp, 16, *p == 'X');
                for (char *t = temp; *t; t++) *str++ = *t;
                break;
            }
            case 'c': {
                char c = (char)va_arg(args, int);
                *str++ = c;
                break;
            }
            case '%': {
                *str++ = '%';
                break;
            }
            default:
                *str++ = '?'; // unknown format
                break;
        }
    }

    *str = '\0';
    va_end(args);
    return str - buf;
}

int snprintf(char *out, uint32_t max_len, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    uint32_t len = 0;

    while (*fmt && len < max_len - 1) {
        if (*fmt != '%') {
            out[len++] = *fmt++;
            continue;
        }

        fmt++; // Skip '%'
        bool long_flag = false;

        // Format specifiers
        switch (*fmt) {
            case 'l':
                long_flag = true;
                fmt++;
                break;
        }

        switch (*fmt) {
            case 'd': {
                int val = long_flag ? va_arg(args, long) : va_arg(args, int);
                if (val < 0) {
                    if (len < max_len - 1)
                        out[len++] = '-';
                    val = -val;
                }
                char num[16];
                int n = utoa((unsigned int)val, num, 10, false);
                for (int i = 0; i < n && len < max_len - 1; i++)
                    out[len++] = num[i];
                break;
            }
            case 'u': {
                unsigned int val = va_arg(args, unsigned int);
                char num[16];
                int n = utoa(val, num, 10, false);
                for (int i = 0; i < n && len < max_len - 1; i++)
                    out[len++] = num[i];
                break;
            }
            case 'x':
            case 'X': {
                unsigned int val = va_arg(args, unsigned int);
                char num[16];
                int n = utoa(val, num, 16, *fmt == 'X');
                for (int i = 0; i < n && len < max_len - 1; i++)
                    out[len++] = num[i];
                break;
            }
            case 'c': {
                char c = (char)va_arg(args, int);
                if (len < max_len - 1)
                    out[len++] = c;
                break;
            }
            case 's': {
                const char *s = va_arg(args, const char *);
                while (*s && len < max_len - 1)
                    out[len++] = *s++;
                break;
            }
            case '%': {
                if (len < max_len - 1)
                    out[len++] = '%';
                break;
            }
            default:
                // Unknown specifier — print '?'
                if (len < max_len - 1)
                    out[len++] = '?';
                break;
        }
        fmt++;
    }

    out[len] = '\0';
    va_end(args);
    return len;
}
