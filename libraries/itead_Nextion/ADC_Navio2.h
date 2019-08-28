#pragma once

#include <cstddef>

#ifndef ADC_H
#define ADC_H

class ADC
{
public:
    virtual void initialize() = 0;
    virtual int get_channel_count(void) = 0;
    virtual int read(int ch) = 0;
};

#endif // ADC_H

class ADC_Navio2 : public ADC
{
public:
    void initialize() override;
    int get_channel_count(void) override;
    int read(int ch) override;
    ADC_Navio2();
    ~ADC_Navio2();

private:
    int open_channel(int ch);

    static const size_t CHANNEL_COUNT = 6;
    int channels[CHANNEL_COUNT];
};
