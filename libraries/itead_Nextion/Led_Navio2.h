#ifndef LED_NAVIO2_H
#define LED_NAVIO2_H

#ifndef LED
#define LED

enum class Colors {
    Black,
    Red,
    Green,
    Blue,
    Cyan,
    Magenta,
    Yellow,
    White};


class Led {

public:
    virtual bool initialize() = 0;
    virtual void setColor(Colors c) = 0;

};

#endif // LED

#ifndef _RGBLED_H_
#define _RGBLED_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdint>
#include "gpio.h"
#include "Led.h"

class RGBled {
public:
    RGBled();

    bool initialize();
    void setColor(Colors color);

private:
    Navio::Pin *pinR;
    Navio::Pin *pinG;
    Navio::Pin *pinB;
};

#endif //_RGBLED_H_


class Led_Navio2 : public Led
{
public:
    Led_Navio2();
    bool initialize() override;
    void setColor(Colors color) override;

private:
    RGBled led;
};

#endif // LED_NAVIO2_H
