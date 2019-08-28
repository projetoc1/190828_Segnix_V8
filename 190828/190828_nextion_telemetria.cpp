/*Este código tem como objetivo testar todas as funções disponíveis da Navio2
com todas as bibliotecas criadas para tal.
LISTA DE ITENS INCORPORADOS:

    MPU9250
    LSM9DS1
    MS4525DO
    MS5611
    AHRS

    ADC
    PWM
    TEMPO

*/

//BIBLIOTECAS
#include <cmath>
#include <math.h>
#include <MS5611.h>
#include <Util.h>
#include <unistd.h>
#include <stdio.h>
#include "Nextion.h"
#include "RCOutput_Navio2.h"
#include <RCInput_Navio2.h>
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
#include <wiringSerial.h>

//TELA NEXTION
//caixas de texto
NexText t9  = NexText( 0 , 20 , "t9"  ); //TEMPO
NexText t10 = NexText( 0 , 21 , "t10" ); //ROLL
NexText t11 = NexText( 0 , 22 , "t11" ); //PITCH
NexText t12 = NexText( 0 , 23 , "t12" ); //YAW
NexText t13 = NexText( 0 , 24 , "t13" ); //Ax
NexText t14 = NexText( 0 , 25 , "t14" ); //Ay
NexText t15 = NexText( 0 , 26 , "t15" ); //Az
NexText t16 = NexText( 0 , 27 , "t16" ); //Alt baro
NexText t17 = NexText( 0 , 28 , "t17" ); //Alt gps
//variaveis em segundo plano
NexVariable tempo_tela = NexVariable(0,10,"tempo");
NexVariable roll_tela  = NexVariable(0,3,"roll");
NexVariable pitch_tela = NexVariable(0,4,"pitch");
NexVariable yaw_tela   = NexVariable(0,5,"yaw");
NexVariable ax_tela    = NexVariable(0,7,"ax");
NexVariable ay_tela    = NexVariable(0,8,"ay");
NexVariable az_tela    = NexVariable(0,9,"az");
NexVariable alt_baro_tela = NexVariable(0,29,"alt_baro");
NexVariable alt_gps_tela  = NexVariable(0,6,"alt_gps");

//DEFINIÇÕES PARA PORTAS DE PWM
#define SERVO_MIN 1000 /*uS*/
#define SERVO_MAX 2000 /*uS*/
#define PWM_OUTPUT_1 0
#define PWM_OUTPUT_2 1

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

//PPM
RCInput_Navio2 ppm;

//PWM 
RCOutput_Navio2 servo;
float ppm_ch1;
float ppm_ch2;
float ppm_ch3;
float ppm_ch4;
float ppm_ch5;
float ppm_ch6;
float ppm_ch7;
float ppm_ch8;

//MS4525DO
MS4525DO pitot;
float pressao_pitot;
float temperatura_pitot;

//MPU9250
MPU9250 mpu1;
float axmpu,aympu,azmpu;
float gxmpu,gympu,gzmpu;
float mxmpu,mympu,mzmpu;

//LSM9DS1
LSM9DS1 lsm1;
float axlsm,aylsm,azlsm;
float gxlsm,gylsm,gzlsm;
float mxlsm,mylsm,mzlsm;

//MS5611
MS5611 barometer;
float pressao_baro,temperatura_baro;

//AHRS
AHRS euler;
float roll, pitch, yaw;

//GPS
using namespace std;
Ublox gps;
float time_of_week,Longitude,Latitude,height_e,height_msl;
float Horizontal_acc,Vertial_acc;

float time_of_week2;
float vel_norte;
float vel_leste;
float vel_descida;
float vel_3d;
float vel_solo;
float heading_gps;

void* f_thread1(void* data)
{

    while(1)
    {
        //LOOP NO THREAD1
        // BAROMETRO
        barometer.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer.readPressure();

        barometer.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer.readTemperature();

        barometer.calculatePressureAndTemperature();

        pressao_baro     = barometer.getPressure(); //milibar
        temperatura_baro = barometer.getTemperature(); //ºC
    }
}

void set_tela(uint32_t number,const char *nome)
{
    char buf[10] = {0};
    String cmd;
    
    sprintf(buf,"%u",number);
    cmd += nome;
    cmd += ".val=";
    cmd += buf;

    sendCommand(cmd.c_str());
}

void* f_thread2(void* data)
{
    int txt = 0;
    //TELA NEXTION INCIALIZAÇÃO
    nexInit();
    int cont = 1;
    //Telemetria pela serial USB0
    //int uart = serialOpen("/dev/ttyUSB0",57600);
    
    if(txt==1)
    {
    FILE *texto;

    texto = fopen("data.txt","a");

    fprintf(texto,"tempo,axmpu,aympu,azmpu,gxmpu,gympu,gzmpu,mxmpu,mympu,mzmpu,axlsm,aylsm,azlsm,");
    fprintf(texto,"gxlsm,gylsm,gzlsm,mxlsm,mylsm,mzlsm,roll,pitch,yaw,temperatura_baro,pressao_baro,temperatura_pitot,pressao_pitot");
    fprintf(texto,",RPI3 V,SERVO V,PWR V,PWR A,ADC2,ADC3,Longitude,Latitude,height_msl");
    fprintf(texto, ",ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8");
    fprintf(texto, ",vel_3d,vel_solo,heading_gps\n");

    while(1)
    {
        if (segundos>=7200) //setar depois com uma hora = 3600segundos
        {
            fclose(texto);
            return 0;
        }
        else
        {
            fprintf(texto,"%.6f,",segundos);
            fprintf(texto,"%.2f,%.2f,%.2f,",axmpu,aympu,azmpu);
            fprintf(texto,"%.2f,%.2f,%.2f,",gxmpu,gympu,gzmpu);
            fprintf(texto,"%.2f,%.2f,%.2f,",mxmpu,mympu,mzmpu);
            fprintf(texto,"%.2f,%.2f,%.2f,",axlsm,aylsm,azlsm);
            fprintf(texto,"%.2f,%.2f,%.2f,",gxlsm,gylsm,gzlsm);
            fprintf(texto,"%.2f,%.2f,%.2f,",mxlsm,mylsm,mzlsm);
            fprintf(texto,"%.2f,%.2f,%.2f,",roll,pitch,yaw);
            fprintf(texto,"%.2f,%.2f,",temperatura_baro,pressao_baro);
            fprintf(texto,"%.2f,%.2f,",temperatura_pitot,pressao_pitot);
            fprintf(texto,"%.2f,%.2f,%.2f,",ch0,ch1,ch2);
            fprintf(texto,"%.2f,%.2f,%.2f,",ch3,ch4,ch5);
            fprintf(texto,"%.8f,%.8f,%.2f,",Longitude,Latitude,height_msl);
            fprintf(texto, "%.0f,%.0f,%.0f,%.0f,",ppm_ch1,ppm_ch2,ppm_ch3,ppm_ch4);
            fprintf(texto, "%.0f,%.0f,%.0f,%.0f,",ppm_ch5,ppm_ch6,ppm_ch7,ppm_ch8);
            fprintf(texto,"%.2f,%.2f,%.2f,",vel_3d,vel_solo,heading_gps);
            fprintf(texto,"\n");


            usleep(33332); //30Hz
        }
    }
    }
    // while(1)
    // {
    //     //TELEMETRIA PELA SERIAL USB0
    //     int rolamento = (roll*100)+50000;
    //     int arfagem   = (pitch*100)+50000;
    //     int guinada   = (yaw*100)+50000;
    //     int temporal  = segundos*1000;
    //     String telemetria;
    //     telemetria+=String(((temporal)/10000000)%10);
    //     telemetria+=String(((temporal)/1000000)%10);
    //     telemetria+=String(((temporal)/100000)%10);
    //     telemetria+=String(((temporal)/10000)%10);
    //     telemetria+=String(((temporal)/1000)%10);
    //     telemetria+=String(((temporal)/100)%10);
    //     telemetria+=String(((temporal)/10)%10);
    //     telemetria+=String(((temporal)/1)%10);
    //     telemetria+=" ";
    //     telemetria+=String(((rolamento)/10000)%10);
    //     telemetria+=String(((rolamento)/1000)%10);
    //     telemetria+=String(((rolamento)/100)%10);
    //     telemetria+=String(((rolamento)/10)%10);
    //     telemetria+=String(((rolamento)/1)%10);
    //     telemetria+=" ";
    //     telemetria+=String(((arfagem)/10000)%10);
    //     telemetria+=String(((arfagem)/1000)%10);
    //     telemetria+=String(((arfagem)/100)%10);
    //     telemetria+=String(((arfagem)/10)%10);
    //     telemetria+=String(((arfagem)/1)%10);
    //     telemetria+=" ";
    //     telemetria+=String(((guinada)/10000)%10);
    //     telemetria+=String(((guinada)/1000)%10);
    //     telemetria+=String(((guinada)/100)%10);
    //     telemetria+=String(((guinada)/10)%10);
    //     telemetria+=String(((guinada)/1)%10);
    //     telemetria+="\n";
    //     const char* conv = telemetria.c_str();
    //     serialPrintf(uart, conv);
    //     //serialPrintf(uart,"ROLL = %.2f PITCH = %.2f YAW = %.2f tempo = %.6f\n",roll,pitch,yaw,segundos);
    //     usleep(20000);
    // }
    while(1)
    {
        //loop tela nextion
        // char buffer[5] = {0};
        // memset(buffer, 0, sizeof(buffer)); // limpa buffer
        // sprintf(buffer,"%.1f",segundos);   //carrega buffer com variável desejada
        // t9.setText(buffer);                //Escreve buffer na caixa de texto desejada    

        // memset(buffer, 0, sizeof(buffer)); // limpa buffer
        // sprintf(buffer,"%.1f",pitch);      //carrega buffer com variável desejada
        // t10.setText(buffer);               //Escreve buffer na caixa de texto desejada

        // memset(buffer, 0, sizeof(buffer)); // limpa buffer
        // sprintf(buffer,"%.1f",roll);       //carrega buffer com variável desejada
        // t11.setText(buffer);               //Escreve buffer na caixa de texto desejada 

        // memset(buffer, 0, sizeof(buffer)); // limpa buffer
        // sprintf(buffer,"%.1f",yaw);        //carrega buffer com variável desejada
        // t12.setText(buffer);               //Escreve buffer na caixa de texto desejada 

        //  memset(buffer, 0, sizeof(buffer)); // limpa buffer
        //  sprintf(buffer,"%.1f",axmpu);      //carrega buffer com variável desejada
        //  t13.setText(buffer);               //Escreve buffer na caixa de texto desejada 

        //  memset(buffer, 0, sizeof(buffer)); // limpa buffer
        //  sprintf(buffer,"%.1f",aympu);      //carrega buffer com variável desejada
        //  t14.setText(buffer);               //Escreve buffer na caixa de texto desejada

        // memset(buffer, 0, sizeof(buffer)); // limpa buffer
        // sprintf(buffer,"%.1f",azmpu);      //carrega buffer com variável desejada
        // t15.setText(buffer);               //Escreve buffer na caixa de texto desejada  

        // memset(buffer, 0, sizeof(buffer));  // limpa buffer
        // sprintf(buffer,"%.1f",pressao_baro);//carrega buffer com variável desejada
        // t16.setText(buffer);                //Escreve buffer na caixa de texto desejada 

        // memset(buffer, 0, sizeof(buffer));  // limpa buffer
        // sprintf(buffer,"%.1f",height_msl);  //carrega buffer com variável desejada
        // t17.setText(buffer);                //Escreve buffer na caixa de texto desejada 
        // usleep(20000);

        // if(cont>9)
        // {
        //     cont=1;
        // }
        // else
        // {
        //     cont++;
        // }
        // if(cont==1)
        // {
        //     tempo_tela.setValue(segundos*10);
        // }
        // if(cont==2)
        // {
        //     roll_tela.setValue(roll+500);
        // }
        // if(cont==3)
        // {
        //     pitch_tela.setValue(pitch+500);
        // }
        // if(cont==4)
        // {
        //     yaw_tela.setValue(yaw+500);
        // }
        // if(cont==5)
        // {
        //     ax_tela.setValue(axmpu+500);
        // }
        // if(cont==6)
        // {
        //     ay_tela.setValue(aympu+500);
        // }
        // if(cont==7)
        // {
        //     az_tela.setValue(azmpu+500);
        // }
        // if(cont==8)
        // {
        //     alt_baro_tela.setValue(pressao_baro);
        // }
        // if(cont==9)
        // {
        //     alt_gps_tela.setValue(height_msl);
        // }
        set_tela(segundos*10,"tempo");
        set_tela(roll+500,"roll");
        set_tela(pitch+500,"pitch");
        set_tela(yaw+500,"yaw");
        set_tela(axmpu+500,"ax");
        set_tela(aympu+500,"ay");
        set_tela(azmpu+500,"az");
        set_tela(pressao_baro,"alt_baro");
        set_tela(height_msl,"alt_gps");
        usleep(250000);


    }   
}

void* f_thread3(void* data)
{
    //GPS
    std::vector<double> pos_data;
    std::vector<double> vel_data;
    if(gps.testConnection())
    {
        printf("Ublox test OK\n");
    }
    if (!gps.configureSolutionRate(200)) //taxa de atualização em millisegundos
    {
        printf("Setting new rate: FAILED\n");
    }

    while(1)
    {
        //GPS -DADOS DE POSIÇÃO POSLLH = posição latitude longitude altitude
        if(gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data)==1)
        {

            time_of_week   = pos_data[0]/1000;
            Longitude      = pos_data[1]/10000000;
            Latitude       = pos_data[2]/10000000;
            height_e       = pos_data[3]/1000;
            height_msl     = pos_data[4]/1000;
            Horizontal_acc = pos_data[5]/1000;
            Vertial_acc    = pos_data[6]/1000;
        }
        //ps.decodeSingleMessage(Ublox::NAV_VELNED, vel_data);
        if(gps.decodeSingleMessage(Ublox::NAV_VELNED, vel_data)==1)
        {
            time_of_week2 = vel_data[0]/1000;
            vel_norte     = vel_data[1]/100;
            vel_leste     = vel_data[2]/100;
            vel_descida   = vel_data[3]/100;
            vel_3d        = vel_data[4]/100;
            vel_solo      = vel_data[5]/100;
            heading_gps   = vel_data[6]/100000;
            //printf("ok");
        }

        printf(" v_3d = %  .2f v_solo = %  .2f v_n = %  .2f v_l = %  .2f v_d = %  .2f heading = %  .2f\n",vel_3d,vel_solo,vel_norte,vel_leste,vel_descida,heading_gps);
        //usleep(100000);
    }
}

int main()
{
    //CHECA NAVIO2
    if (check_apm()) 
    {
        return 1;
    }
    //INICIALIZAÇÕES

    //PITOT
    pitot.initialize();

    //HABILITA LEITURA DE PORTAS ADC's
    ADC.initialize();

    //MPU9250
    mpu1.initialize();

    //LSM9DS1
    lsm1.initialize();

    //AHRS
    euler.sensorinit();
    euler.setGyroOffset();

    //BARÔMETRO MS5611
    //rodará no thread1
    barometer.initialize();

    //ARQUIVO TEXTO COM DADOS
    //rodará no thread2

    //GPS
    //rodará no thread3

    //PPM
    ppm.initialize();

    //CONFIGURA 2 PORTAS DE PWM - freq 50Hz
    servo.initialize(PWM_OUTPUT_1);
    servo.set_frequency(PWM_OUTPUT_1, 50);
    servo.enable(PWM_OUTPUT_1);

    servo.initialize(PWM_OUTPUT_2);
    servo.set_frequency(PWM_OUTPUT_2, 50);
    servo.enable(PWM_OUTPUT_2);

    //MULTITHREADING
    //RASPBERRY PI 3B+ 4 NÚCLEOS = RODA 4 THREADS
    //SENDO O MAIN O THREAD0
    //THREAD0 - main()
    //THREAD1 - F_THREAD1
    //THREAD2 - F_THREAD2
    //THREAD3 - F_THREAD3
    pthread_t thread1,thread2,thread3;
    pthread_create(&thread1,NULL,f_thread1,NULL);
    pthread_create(&thread2,NULL,f_thread2,NULL);
    pthread_create(&thread3,NULL,f_thread3,NULL);

    //LOOP PRINCIPAL CONTA COMO THREAD
    //THREAD0
    while(1)
    {
        //CONTANDO TEMPO
        //Contagem de tempo. Tempo total : variável segundos
        //                   Delta       : varíavel dt     
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
        ch0 = ADC.read(0); //TENSÃO DA PLACA
        ch1 = ADC.read(1); //TENSÃO DA LINHA DE SERVOS
        ch2 = ADC.read(2); //TENSÃO   DO POWER MODULE
        ch3 = ADC.read(3); //CORRENTE DO POWER MODULE
        ch4 = ADC.read(4); //TENSÃO DA PORTA ADC2
        ch5 = ADC.read(5); //TENSÃO DA PORTA ADC3

        //COMANDO SINAIS DE PWM. 1000 = 1000us
        servo.set_duty_cycle(PWM_OUTPUT_1, 1000);
        //servo.set_duty_cycle(PWM_OUTPUT_2, 1000);

        //PPM
        ppm_ch1 = ppm.read(0);
        ppm_ch2 = ppm.read(1);
        ppm_ch3 = ppm.read(2);
        ppm_ch4 = ppm.read(3);
        ppm_ch5 = ppm.read(4);
        ppm_ch6 = ppm.read(5);
        ppm_ch7 = ppm.read(6);
        ppm_ch8 = ppm.read(7);

        //MPU9250
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        mpu1.update();
        mpu1.read_accelerometer(&axmpu, &aympu, &azmpu);
        mpu1.read_gyroscope(&gxmpu, &gympu, &gzmpu);
        mpu1.read_magnetometer(&mxmpu, &mympu, &mzmpu);

        //LSM9DS1
        //LEITURA DE DADOS DE ACELERAÇÃO, GIRO E MAGNÉTICOS
        lsm1.update();
        lsm1.read_accelerometer(&axlsm, &aylsm, &azlsm);
        lsm1.read_gyroscope(&gxlsm, &gylsm, &gzlsm);
        lsm1.read_magnetometer(&mxlsm, &mylsm, &mzlsm);

        //MS4525DO LEITURA DE PRESSÃO DIFERENCIAL E TEMPERATURA
        temperatura_pitot = pitot.get_Temperature();
        pressao_pitot     = pitot.get_Pressure();

        servo.set_duty_cycle(PWM_OUTPUT_2, ppm_ch3);
        //printf("tensao = % .2f\t corrente = % .2f\n",ch2*0.0110344827,ch3);
        //printf("ch1 = % .0f ch2 = % .0f ",ppm_ch1,ppm_ch2);
        //printf("ch3 = % .0f ch4 = % .0f ",ppm_ch3,ppm_ch4);
        //printf("ch5 = % .0f ch6 = % .0f ",ppm_ch5,ppm_ch6);
        //printf("ch7 = % .0f ch8 = % .0f\n",ppm_ch7,ppm_ch8);

    }


    return 0;
}
