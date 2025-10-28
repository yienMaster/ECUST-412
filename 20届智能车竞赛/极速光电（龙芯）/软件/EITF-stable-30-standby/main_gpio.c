#include "gpio.h"
#include "tools.h"


int main()
{
    printf("%d\n",gpio_register(83));
    gpio_direction(83,true);
    for(int i=0;i<10;i++)
    {
        sleep(1);a
        gpio_toggle(83);
    }
    gpio_set(83,0);
    sleep(1);
    gpio_set(83,1);
    sleep(3);
    printf("%d\n",gpio_unregister(83));
    return 0;
}