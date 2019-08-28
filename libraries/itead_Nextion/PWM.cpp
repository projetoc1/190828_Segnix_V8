#include "PWM.h"

#pragma once

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])
#define NAVIO2 3
#define NAVIO 1

int write_file(const char *path, const char *fmt, ...);
int read_file(const char *path, const char *fmt, ...);
bool check_apm();
int get_navio_version();


PWM::PWM()
{
}

bool PWM::init(unsigned int channel)
{
    int err;
    err = write_file("/sys/class/pwm/pwmchip0/export", "%u", channel);
    if (err >= 0 || err == -EBUSY)
    {
        return true;
    }
    else 
    {
        printf("Can't init channel %u\n", channel);
        return false;
    }
    return true;
}

bool PWM::enable(unsigned int channel)
{
    char path[60] = "/sys/class/pwm/pwmchip0";
    char path_ch[20];
    sprintf(path_ch, "/pwm%u/enable", channel);
    strcat(path, path_ch);
    
    if (write_file(path, "1") < 0)
    {
        printf("Can't enable channel %u\n", channel);
        return false;
    }
    return true;
}

bool PWM::set_period(unsigned int channel, unsigned int freq)
{
    int period_ns;
    char path[60] = "/sys/class/pwm/pwmchip0";
    char path_ch[20];
    sprintf(path_ch, "/pwm%u/period", channel);
    strcat(path, path_ch);

    period_ns = 1e9 / freq;
    if (write_file(path, "%u", period_ns) < 0)
    {
        printf("Can't set period to channel %u\n", channel);
        return false;
    }
    return true;
}

bool PWM::set_duty_cycle(unsigned int channel, float period)
{
    int period_ns;
    char path[60] = "/sys/class/pwm/pwmchip0";
    char path_ch[20];
    sprintf(path_ch, "/pwm%u/duty_cycle", channel);
    strcat(path, path_ch);

    period_ns = period * 1e6;
    if (write_file(path, "%u", period_ns) < 0)
    {
        printf("Can't set duty cycle to channel %u\n", channel);
        return false;
    }
    return true;
}
