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
#include <filtro.h>

//DEFINIÇÕES PARA PORTAS DE PWM
#define SERVO_MIN 1000 /*mS*/
#define SERVO_MAX 2000 /*mS*/
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1
#define PWM_OUTPUT_3 2
#define PWM_OUTPUT_4 3

//AHRS
AHRS euler;
float roll, pitch, yaw;

//MUTEX
pthread_mutex_t trava;

//LEITURA ADC
ADC_Navio2 ADC;

float ch4  = 0; //CANAIS DA PORTA ADC NA NAVIO2 
float ch5  = 0; //CANAIS DA PORTA ADC NA NAVIO2
float pot1 = 0;
float pot2 = 0;

//PWM 
RCOutput_Navio2 servo1;

float comando_1 = 1000;
float comando_2 = 1000;
float comando_3 = 1000;
float comando_4 = 1000;

//TEMPO
struct timeval tv;
float dt,segundos=0,contador=0;
static unsigned long previoustime, currenttime;

//TELA NEXTION 
NexVariable setp_lat = NexVariable(0,37,"setp_lat");
NexVariable setp_lon = NexVariable(0,38,"setp_lon");

NexVariable flag_ganho = NexVariable(0,59,"flag_ganho");

NexVariable lat_p = NexVariable(0,52,"lat_p");
NexVariable lat_i = NexVariable(0,53,"lat_i");
NexVariable lat_d = NexVariable(0,54,"lat_d");

NexVariable lon_p = NexVariable(0,55,"lon_p");
NexVariable lon_i = NexVariable(0,56,"lon_i");
NexVariable lon_d = NexVariable(0,57,"lon_d");

NexVariable error_lat = NexVariable(0,58,"error_lat");
NexVariable error_lon = NexVariable(0,59,"error_lon");

NexVariable flag = NexVariable(0,60,"flag");
NexVariable flag_piloto = NexVariable(0,63,"flag_piloto");

NexVariable lat_navio2 = NexVariable(0,67,"lat_navio2");
NexVariable lon_navio2 = NexVariable(0,68,"lon_navio2");

NexVariable adc1 = NexVariable(0,69,"adc1");
NexVariable adc2 = NexVariable(0,70,"adc2");

uint32_t setpoint_lat = 0;
uint32_t setpoint_lon = 0;
uint32_t        latp  = 0;
uint32_t        lati  = 0;
uint32_t        latd  = 0;
uint32_t        lonp  = 0;
uint32_t        loni  = 0;
uint32_t        lond  = 0;
float        errorlat = 3;
float        errorlon = 4;
uint32_t        flag0 = 0;
uint32_t   flagpiloto = 0;
uint32_t    flagganho = 0;
float       latnavio2 = 1;
float       lonnavio2 = 1;
float     leituraadc1 = 6;
float     leituraadc2 = 7;

//EIXO Y
float teta_p = 0;
float teta   = 0; // EIXO Y
float xd     = 0;
float c      = 0.2;
float wd     = 0;
float kp     = -0.05; //para massa maior --> -0.013
float ki     = 0.05;   //para massa maior --> 0.01
float kd     = 0.0;
float xtp    = 0;
float xtpp   = 0;
float ctp    = 0.1;
float e      = 0; 
float xi     = 0;
float wi     = 0;
float T      = 0.5;
float pmx    = 1408; // máximo de comando do motor
float pmi    = 1192; // mínimo de comando do motor
float u      = 0;
float up;
float ui;
float ud;
float utp;
float xd_ponto;
float k_teta_ponto = 0.0;

//EIXO Z
float teta_pz = 0;
float tetaz   = 0; // EIXO z
float xdz     = 0;
float cz      = 0.2;
float wdz     = 0;
float kpz     = -0.05; //para massa maior --> -0.013
float kiz     = 0.1;   //para massa maior --> 0.01
float kdz     = 0.0;
float xtpz    = 0;
float xtppz   = 0;
float ctpz    = 0.1;
float ez      = 0; 
float xiz     = 0;
float wiz     = 0;
float Tz      = 0.5;
float pmxz    = 1308; // máximo de comando do motor
float pmiz    = 1192; // mínimo de comando do motor
float uz      = 0;
float upz;
float uiz;
float udz;
float utpz;
float xd_pontoz;
float k_teta_pontoz = 0.0;

float setpoint0  = 0; //graus
float setpoint;       //graus

float setpoint0z = 0; //graus
float setpointz;       //graus

//FUNÇÃO THREAD TELA NEXTION
void* f_tela(void* data)
{
    int cont = 1;
    while(1)
    {
        if(cont>10)
        {
            cont=1;
        }
        else
        {
            cont++;
        }
        //ENVIA PARA TELA NEXTION
        if (cont==1)
        {
            lat_navio2.setValue(latnavio2);
        }
        if (cont==2)
        {
            lon_navio2.setValue(lonnavio2);
        }
        if (cont==3)
        {
            adc1.setValue(leituraadc1);
        }
        if (cont==4)
        {
           adc2.setValue(leituraadc2);
        } 
        if (cont==5)
        {
            error_lat.setValue(errorlat);
        }
        if (cont==6)
        {
            error_lon.setValue(errorlon);
        }
        //LEITURA DA TELA
        if (cont==7)
        {
           setp_lat.getValue(&setpoint_lat);
        }
        if (cont==8)
        {
            setp_lon.getValue(&setpoint_lon);
        }
        if (cont==9)
        {
        	flag_ganho.getValue(&flagganho);
        	if (flagganho==0)
        	{
        		lat_p.getValue(&latp);
        	}
        	if (flagganho==1)
        	{
        		lat_i.getValue(&lati);
        	}
        	if (flagganho==2)
        	{
        		lat_d.getValue(&latd);
        	}
        	if (flagganho==3)
        	{
        		lon_p.getValue(&lonp);
        	}
        	if (flagganho==4)
        	{
        		lon_i.getValue(&loni);
        	}
        	if (flagganho==5)
        	{
        		lon_d.getValue(&lond);
        	}
        }
        if (cont==10)
        {
            flag_piloto.getValue(&flagpiloto);
        }
    }
}

//FUNÇÃO SALVA VARIÁVEIS EM ARQUIVO TEXTO
void* f_texto(void* data)
{

    //CRIANDO ARQUIVO TEXTO PARA ESCRITA
    FILE *texto;
    texto = fopen("dados.txt","w");
    float tetay_texto;
    float tetaz_texto;
    float setpointy_texto;
    float setpointz_texto;
    float uy_texto;
    float uz_texto;

    while(1)
    {
        if (segundos>=100)
        {
           fclose(texto);
           return 0;
        }
        else
        {

            tetay_texto     = teta;
            tetaz_texto     = tetaz;
            setpointy_texto = setpoint;
            setpointz_texto = setpointz;
            uy_texto        = u;
            uz_texto        = uz;
               
            fprintf(texto,"%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",segundos,tetay_texto,tetaz_texto,setpointy_texto,setpointz_texto,uy_texto,uz_texto);
            usleep(20000);
        }
    }
}

//PROGRAMA PRINCIPAL
int main()
{
	//INICIALIZA TELA NEXTION
    nexInit();

    //
    lat_p.getValue(&latp);
    lat_i.getValue(&lati);
    lat_d.getValue(&latd);
    lon_p.getValue(&lonp);
    lon_i.getValue(&loni);
    lon_d.getValue(&lond);

    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }

    //AHRS
    euler.sensorinit();
    euler.setGyroOffset();

    //MULTITHREADING - TELA e TEXTO
    pthread_t tela,texto;
    pthread_create(&tela,NULL,f_tela,NULL);
    pthread_create(&texto,NULL,f_texto,NULL);

    //HABILITA LEITURA DE PORTAS ADC's
    ADC.initialize();

    //CONFIGURA 4 PORTAS DE PWM - freq 50Hz
    servo1.initialize(PWM_OUTPUT_1);
    servo1.set_frequency(PWM_OUTPUT_1, 50);
    servo1.enable(PWM_OUTPUT_1);

    servo1.initialize(PWM_OUTPUT_2);
    servo1.set_frequency(PWM_OUTPUT_2, 50);
    servo1.enable(PWM_OUTPUT_2);

    servo1.initialize(PWM_OUTPUT_3);
    servo1.set_frequency(PWM_OUTPUT_3, 50);
    servo1.enable(PWM_OUTPUT_3);

    servo1.initialize(PWM_OUTPUT_4);
    servo1.set_frequency(PWM_OUTPUT_4, 50);
    servo1.enable(PWM_OUTPUT_4);

    //MOTORES DESLIGADOS
    servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);
    servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

    //LEITURA INICIAL DE DOS POTENCIOMETROS
    ch5  = ADC.read(4);
    ch4  = ADC.read(5);
    pot1 = ch5/1000;
    pot2 = ch4/1000;

    //EIXO Y
    teta = (93.066*pot1) - 28.598 - 80;
    float tetaf = teta;

    //EIXO Z
    tetaz = (93.066*pot2) - 28.598 - 80;
    float tetafz = tetaz;

    float ativador      =  0; //ativação do controlador PID
    float rotina        =  0; //controla os setpoints numa ordem pre-determinada (-10,-10),(10,-10),(10,10),(-10,10) repetição
    float rotina_temp   =  0; //contador de tempo entre os setpoints
    float rotina_temp_y =  0; //contador de tempo entre os setpoints y
    float rotina_temp_z =  0; //contador de tempo entre os setpoints y
    float grau_max      =  10;
    float grau_min      = -10;

    while(true)
    {
    	ativador = (float)flagpiloto;
    	if(ativador == 2)
    	{	
    		//advindos do display NEXTION
        	kp  = (0.4/100)*(float)latp - 0.2;
        	ki  = (0.4/100)*(float)lati - 0.2;
        	kd  = (0.4/100)*(float)latd - 0.2;

        	kpz = (0.4/100)*(float)lonp - 0.2;
        	kiz = (0.4/100)*(float)loni - 0.2;
        	kdz = (0.4/100)*(float)lond - 0.2;

        	

        	//Contagem de tempo. Tempo total : variável segundos
        	//                   Delta       : varíavel dt     
        	gettimeofday(&tv,NULL);
        	previoustime = currenttime;
        	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        	dt = (currenttime - previoustime) / 1000000.0;
        	if (dt>100)
        	{
        	    dt = 0;
        	}

        	segundos = segundos + dt;

            //AHRS rolagem, arfagem e guinada : roll, pitch e yaw
            //de preferência o mais próximo possível da contagem de tempo
            euler.updateIMU(dt);
            euler.getEuler(&roll, &pitch, &yaw);
        
        	//PEGA SETPOINT DA TELA CASO DIFERENTE DE ZERO
        	//EIXO Y

        		setpoint0  = float(setpoint_lat) - 300;
        	//setpoint = setpoint0*(1-exp(-segundos/1));
            setpoint = setpoint0;

        	//EIXO Z
        		setpoint0z = float(setpoint_lon) - 300;

        	//setpointz = setpoint0z*(1-exp(-segundos/1));
            setpointz = setpoint0z;

        	//Leitura do canal ADC
        	ch4 = ADC.read(5); //eixo Z
        	ch5 = ADC.read(4); //eixo Y

       		//Converte para tensão
        	pot1 = ch5/1000;
        	pot2 = ch4/1000;

            //Converte para ângulo
            //UTILIZANDO AHRS PARA thetas
            //ROLL INICIALIZA EM ZERO E DEMORA EM TORNO DE 
            //8 SEGUNDOS PARA ESTABILIZAR EM 87.15 APROX.
            if (segundos<10)
            {
                tetaz = 0;
            }
            else
            {
                tetaz = roll - 87.15;
            }
        	teta = - pitch;
        	//teta  = (93.066*pot1) - 28.598 - 80;
        	//tetaz = (93.066*pot2) - 28.598 - 80;

        	//Envia para tela campo leitura ADC
        	leituraadc1 = teta  + 300;
        	leituraadc2 = tetaz + 300;

        	//ROTINA CONTROLADOR PID EIXO Y
        	xtpp   = (-1/ctp)*xtp +teta;
        	xtp    = xtp + xtpp*dt;
        	teta_p = (-1/(ctp*ctp))*xtp + (1/ctp)*teta;

        	tetaf    = tetaf + (-tetaf/T + teta/T)*dt;
        	e        = setpoint - teta;
        	errorlat = e; //enviando para tela o erro no eixo Y
        	xi       = xi + e*dt;
        	if (xi<-150)
        	{
        		xi = -150;
        	}
        	wi = xi;

        	xd_ponto = (-1/c)*xd + e;
        	xd       = xd + xd_ponto*dt;
        	wd       = (-1/(c*c))*xd + (1/c)*e;


        	up  = kp*e;
			ui  = ki*wi;
			ud  = kd*wd;

        	if(segundos>0.3)
        	{
        		u = ui + ud + up + k_teta_ponto*teta_p;

			}
			else
			{
				u = ui + up;
			}
			utp = k_teta_ponto*teta_p;

        	//LIMITADORES:
        	if(u>2.5)
        	{
        		u=2.5;
        	}
        	if(u<-2.5)
        	{
        		u=-2.5;
        	}


        	comando_2 = (pmx+pmi)/2  + (pmx-pmi)*u/2;
        	comando_4 = (pmx+pmi)/2  - (pmx-pmi)*u/2;

        	//ROTINA CONTROLADOR PID EIXO Z
        	xtppz   = (-1/ctpz)*xtpz +tetaz;
        	xtpz    = xtpz+xtppz*dt;
        	teta_pz = (-1/(ctpz*ctpz))*xtpz + (1/ctpz)*tetaz;

        	tetafz = tetafz + (-tetafz/Tz + tetaz/Tz)*dt;
        	ez = setpointz - tetaz;
        	errorlon = ez; //enviando para tela erro no eixo Z
        	xiz = xiz + ez*dt;
        	if (xiz<-150)
        	{
        		xiz = -150;
        	}
        	wiz = xiz;

        	xd_pontoz = (-1/cz)*xdz + ez;
        	xdz       = xdz + xd_pontoz*dt;
        	wdz       = (-1/(cz*cz))*xdz + (1/cz)*ez;

        	upz  = kpz*ez;
			uiz  = kiz*wiz;
			udz  = kdz*wdz;

        	if(segundos>0.3)
        	{
        		uz = uiz + udz + upz + k_teta_pontoz*teta_pz;

			}
			else
			{
				uz = uiz + upz;
			}
			utpz = k_teta_pontoz*teta_pz;

        	//LIMITADORES:
			if(uz>3.0)
        	{
        	uz=3.0;
        	}
        	if(uz<-3.0)
        	{
        		uz=-3.0;
        	}

        	comando_1 = (pmxz+pmiz)/2  + (pmxz-pmiz)*uz/2;
        	comando_3 = (pmxz+pmiz)/2  - (pmxz-pmiz)*uz/2;

        	//EIXO Z
        	servo1.set_duty_cycle(PWM_OUTPUT_1, comando_1);
        	servo1.set_duty_cycle(PWM_OUTPUT_3, comando_3);
            // servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
            // servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);            

        	//EIXO Y
        	servo1.set_duty_cycle(PWM_OUTPUT_2, comando_2);
        	servo1.set_duty_cycle(PWM_OUTPUT_4, comando_4);
            //servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
            //servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);

        	//printf("comando 1 = %.0f comando 3 = %.0f\n",comando_1,comando_3);
        }
        else
        {
        	//EIXO Z
        	servo1.set_duty_cycle(PWM_OUTPUT_1, 1000);
        	servo1.set_duty_cycle(PWM_OUTPUT_3, 1000);

        	//EIXO Y
        	servo1.set_duty_cycle(PWM_OUTPUT_2, 1000);
        	servo1.set_duty_cycle(PWM_OUTPUT_4, 1000);
        }

		usleep(20000);
		//float ch55  = ADC.read(4);
		//float ch44  = ADC.read(5);
		//float pot11 = ch55/1000;
		//float pot22 = ch44/1000;
        //printf("teta = %.2f tetaz = %.2f\n",((93.066*(pot11)) - 28.598 - 80),((93.066*(pot22)) - 28.598 - 80));
        //printf("Pwm1 = %.2f Pwm3 = %.2f pwm2 = %.2f pwm4 = %.2f up = %.2f ui = %.2f upz = %.2f uiz = %.2f ativador = %.2f\n",comando_1,comando_3,comando_2,comando_4,up,ui,upz,uiz,ativador);
		//printf("kp = %.2f ki = %.2f kd = %.2f kpz = %.2f kiz = %.2f kdz = %.2f ativador = %.2f\n",kp,ki,kd,kpz,kiz,kdz,ativador);
        //printf("wi=%.2f wiz = %.2f u=%.2f setpoint=%.2f teta=%.2f setpointz=%.2f\n",wi,wiz,u,setpoint,teta,setpointz);
        printf("y_euler=% .2f\t z_euler = % .2f\t y_pot = % .2f\t z_pot % .2f\n",teta,tetaz,(93.066*pot1) - 28.598 - 80,(93.066*pot2) - 28.598 - 80);
    }
    return 0;
}