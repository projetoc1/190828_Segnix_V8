#ifndef RCOUTPUT_NAVIO2_H
#define RCOUTPUT_NAVIO2_H

#include "PWM.h"

#ifndef RCOUTPUT
#define RCOUTPUT

class RCOutput
{
public:
    virtual bool initialize(int channel) = 0;
    virtual bool enable(int channel) = 0;
    virtual bool set_frequency(int channel, float frequency) = 0;
    virtual bool set_duty_cycle(int channel, float period) = 0;
};

#endif // RCOUTPUT




class RCOutput_Navio2 : public RCOutput
{
public:
    RCOutput_Navio2();
    bool initialize(int channel) override;
    bool enable(int channel) override;
    bool set_frequency(int channel, float frequency) override;
    bool set_duty_cycle(int channel, float period) override;

private:
    PWM pwm;
};

#endif // RCOUTPUT_NAVIO2_H
