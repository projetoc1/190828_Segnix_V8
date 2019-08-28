
OBJS := NexButton.o
OBJS += NexCrop.o
OBJS += NexGauge.o
OBJS += NexHardware.o
OBJS += NexHotspot.o
OBJS += NexObject.o
OBJS += NexPage.o
OBJS += NexPicture.o
OBJS += NexProgressBar.o
OBJS += NexSlider.o
OBJS += NexText.o
OBJS += NexTouch.o
OBJS += NexWaveform.o
OBJS += NexDualStateButton.o
OBJS += NexNumber.o
OBJS += NexVariable.o
OBJS += RGBled.o
OBJS += gpio.o
OBJS += I2Cdev.o
OBJS += MPU9250.o
OBJS += MS5611.o
OBJS += Ublox.o
OBJS += Util.o
OBJS += ADC_Navio2.o
OBJS += Led_Navio2.o
OBJS += LSM9DS1.o
OBJS += PWM.o
OBJS += RCInput_Navio2.o
OBJS += RCOutput_Navio2.o
OBJS += filtro.o
OBJS += MS4525DO.o
OBJS += wiringSerial.o

itead_Nextion.o: $(OBJS)
	$(SDK_LINKER) -r -o $@ $^
	rm -rf $^

%.o:%.cpp
	$(SDK_CPP_COMPILE) $(SDK_LIB_OPTIMIZE_OPTION) -fpic -c -o $@ $< -I../../include





