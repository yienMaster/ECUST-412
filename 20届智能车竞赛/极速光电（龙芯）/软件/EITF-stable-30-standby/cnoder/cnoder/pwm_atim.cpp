#include "pwm_atim.h"

PWM_ATIM::PWM_ATIM(int gpio, int mux, int chNum_, int period_10ns_, int duty_cycle_10ns_, int NEG_)
    : period_10ns(period_10ns_), duty_cycle_10ns(duty_cycle_10ns_), chNum(chNum_ - 1), NEG(NEG_)
{
    {
        void *gpio_mux_buffer = map_register(GPIO_MUX_BASE_ADDR + (gpio / 16) * 0x04, PAGE_SIZE);
        REG_WRITE(gpio_mux_buffer, REG_READ(gpio_mux_buffer) | (mux << (gpio % 16 * 2)));
    }

    REG_WRITE(map_register(ATIM_BASE_ADDR + ATIM_EGR_OFFSET, PAGE_SIZE), 0x01);
    REG_WRITE(map_register(ATIM_BASE_ADDR + ATIM_CR1_OFFSET, PAGE_SIZE), 0x01);

    period_buffer = map_register(ATIM_BASE_ADDR + ATIM_ARR_OFFSET, PAGE_SIZE);
    duty_cycle_buffer = map_register(ATIM_BASE_ADDR + ATIM_CCR1_OFFSET + chNum * 0x04, PAGE_SIZE);
    ccmr_buffer[0] = map_register(ATIM_BASE_ADDR + ATIM_CCMR1_OFFSET, PAGE_SIZE);
    ccmr_buffer[1] = map_register(ATIM_BASE_ADDR + ATIM_CCMR2_OFFSET, PAGE_SIZE);
    ccer_buffer = map_register(ATIM_BASE_ADDR + ATIM_CCER_OFFSET, PAGE_SIZE);
    cnt_buffer = map_register(ATIM_BASE_ADDR + ATIM_CNT_OFFSET, PAGE_SIZE);
    bdtr_buffer = map_register(ATIM_BASE_ADDR + ATIM_BDTR_OFFSET, PAGE_SIZE);

    REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) & ~(0x7 << (chNum % 2 * 8 + 4)));
    REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) | (0x7 << (chNum % 2 * 8 + 4)));

    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 1 + NEG * 2)));
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 1 + NEG * 2)));

    REG_WRITE(period_buffer, period_10ns);
    REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

    REG_WRITE(bdtr_buffer, 0x1 << 15);
    REG_WRITE(cnt_buffer, 0);
    printf("Registers mapped successfully\n");
}

PWM_ATIM::~PWM_ATIM(void)
{
    munmap(ccmr_buffer[0], PAGE_SIZE);
    munmap(ccmr_buffer[1], PAGE_SIZE);
    munmap(period_buffer, PAGE_SIZE);
    munmap(duty_cycle_buffer, PAGE_SIZE);
    munmap(ccer_buffer, PAGE_SIZE);
    munmap(cnt_buffer, PAGE_SIZE);
    munmap(bdtr_buffer, PAGE_SIZE);
}

void PWM_ATIM::enable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 0 + NEG * 2)));
}

void PWM_ATIM::disable(void)
{
    REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 0 + NEG * 2)));
}

void PWM_ATIM::setPeriod(unsigned int period_10ns_)
{
    period_10ns = period_10ns_;
    REG_WRITE(period_buffer, period_10ns);
    REG_WRITE(cnt_buffer, 0);
}

void PWM_ATIM::setDutyCycle(unsigned int duty_cycle_10ns_)
{
    duty_cycle_10ns = duty_cycle_10ns_;
    REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);
    REG_WRITE(cnt_buffer, 0);
}