//

#include "com.h"

char COM_value[512];

char cgetc(void *p)
{
    char ch = -1;
    HAL_UART_Receive(p, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

void cgets(void *p, char *str, char end)
{
    char ch = cgetc(p);
    while (isspace(ch))
        ch = cgetc(p);
    while (ch != end)
    {
        *(str++) = ch;
        ch = cgetc(p);
    }
    *str = '\0';
}

void cgets_s(void *p, char *str, int s)
{
    if (s-- > 0)
        *(str++) = cgetc(p);
    *str = '\0';
}

int cprintf(void *p, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);

    int res = vsprintf(COM_value, format, ap);
    HAL_UART_Transmit(p, (uint8_t *)COM_value, res, 0xffff);

    return res;
}
