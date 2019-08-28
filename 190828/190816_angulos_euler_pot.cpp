//BIBLIOTECAS
#include <cmath>
#include <math.h>
#include <MS5611.h>
#include <Util.h>
#include <unistd.h>
#include <stdio.h>
#include "Nextion.h"
#include "RCOutput_Navio2.h"
#include "PWM.h"
#include <memory>
#include <string>
#include <sys/time.h>
#include "MPU9250.h"
#include "LSM9DS1.h"
#include <pthread.h>
#include <ADC_Navio2.h>
#include <MS4525DO.h>
#include <filtro.h>
#include <Ublox.h>

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

//LEITURA ADC
ADC_Navio2 ADC;

float ch0 = 0; //TENSÃO DA PLACA
float ch1 = 0; //TENSÃO DA LINHA DE SERVOS
float ch2 = 0; //TENSÃO   DO POWER MODULE
float ch3 = 0; //CORRENTE DO POWER MODULE
float ch4 = 0; //TENSÃO DA PORTA ADC2
float ch5 = 0; //TENSÃO DA PORTA ADC3 

float pot1 = 0;
float pot2 = 0;

//AHRS
AHRS euler;
float roll, pitch, yaw;

int main()
{
	//HABILITA LEITURA DE PORTAS ADC's
    ADC.initialize();

    //AHRS
    euler.sensorinit();
    euler.setGyroOffset();

    while(1)
    {
    	gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        //IGNORA LEITURA INICIAL ALTA.
        if (dt>100)
        {
            dt = 0;
        }
        segundos = segundos + dt;

        //AHRS rolagem, arfagem e guinada : roll, pitch e yaw
        //de preferência o mais próximo possível da contagem de tempo
        euler.updateIMU(dt);
        euler.getEuler(&roll, &pitch, &yaw);

        //Leitura do canal ADC
        //LEITURA INICIAL DE DOS POTENCIOMETROS
    	ch5  = ADC.read(4);
    	ch4  = ADC.read(5);
    	pot1 = ch5/1000;
    	pot2 = ch4/1000;
    	//EIXO Y
    	float teta = (93.066*pot1) - 28.598 - 80;


    	//EIXO Z
    	float tetaz = (93.066*pot2) - 28.598 - 80;

    	float eixoy = -pitch;
    	float eixoz = roll - 87.15;


    	//printf("y_pot % .2f\t z_pot % .2f\t r_ahrs % .2f\t p_ahrs % .2f\t y_ahrs % .2f\n ",teta,tetaz,roll,pitch,yaw);
    	printf("y_pot % .2f\t z_pot % .2f\t eixo_y % .2f\t eixo_z % .2f\t tempo % .2f\n",teta,tetaz,eixoy,eixoz,segundos);

    }

	return 0;
}