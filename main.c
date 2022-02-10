/* ////////////////////////////////////////////////// */
/*          DESCRIPCION DE BIBLIOTECAS                */
/* ////////////////////////////////////////////////// */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "Fusion.h"
#include "funciones.h"

/* ////////////////////////////////////////////////// */
/*          VARIABLES GLOBALES                        */
/* ////////////////////////////////////////////////// */

int32_t timeSystick[3];
int32_t countLoops,First, Period, lastPeriod = 0;
int16_t dataRawLSM6[6],dataRawLIS3[3],numRxUart5Recep=0,signo = 0;
float setInputControl=0, a_ang=0;
char getRxUART[10],strOut[40];
float controlPWM[2]={0,0}; //controlPWM[0]: control PWM       controlPWM[1]: filtrado control
float Hz[3]={0,0,0}; // Hz[0]: hz    Hz[1]: Filtrado     Hz[2]: last Filtrado
float wRW=0;


/* ////////////////////////////////////////////////// */
/*         SIMPLE KALMAN FILTER FOR SENSORS           */
/* ////////////////////////////////////////////////// */
float kfMagn[3], kfAccel[3], kfGyro[3];

/* ////////////////////////////////////////////////// */
/*                 STORE VALUES                       */
/* ////////////////////////////////////////////////// */
float stDevAcRW[3] = {0,0,0}; // Derivada de rad/s --> rad/s^2
float stPIDVel[6] = {0,0,0,0,0,0};

/* ////////////////////////////////////////////////// */
/*                 CONTROL PID                        */
/* ////////////////////////////////////////////////// */
const float T=0.01;
const float constPIDvel[3]={6,2.25,0.08}; // p i d controlador de velocidad
float qVEL[3]={0,0,0};

const float constPIDacRW[3]={0,23191559,0}; // p i d controlador de velocidad
float qAcRW[3]={0,0,0};

/* ////////////////////////////////////////////////// */
/*          VARIABLES GLOBALES DE AHRS                 */
/* ////////////////////////////////////////////////// */

FusionBias fusionBias;
FusionAhrs fusionAhrs;
FusionEulerAngles eulerAngles;

FusionVector3 gyroscopeSensitivity = {
    .axis.x = 0.00875,
    .axis.y = 0.00875,
    .axis.z = 0.00875,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
    .axis.x = 0.000122,
    .axis.y = 0.000122,
    .axis.z = 0.000122,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

FusionVector3 hardIronBias = {
    .axis.x = 21.251085,
    .axis.y = 9.105525,
    .axis.z = 5.912000,
}; // replace these values with actual hard-iron bias in uT if known


/* ////////////////////////////////////////////////// */
/*         PROTOTIPO DE FUNCIONES INICIALES           */
/* ////////////////////////////////////////////////// */

void TIMER1AInit(void);
void AHRSInit (void);
void SysTickIntHandler(void);
void UART5_Handler( void );
void Timer2A_Handler(void);
void Timer2AInit(void);
void TIMER1A_Handler(void);

int main(void)
 {
    PLL_Init();
    SysTickInit();
    I2Cinit();
    GPIOInit();
    Uart5Init();
    LSM6DSOXInit();
    LIS3MDLInit();
    AHRSInit ();
    PWMInit();
    Timer2AInit();
    TIMER1AInit();


    initControlPID(constPIDvel, qVEL, T);
    initControlPID(constPIDacRW, qAcRW, T);

    while(1)
    {

    }
}

void SysTickIntHandler(void)
{
    timeSystick[0] = NVIC_ST_CURRENT_R;
    /*
    LSM6DSOXgetData(dataRawLSM6);

    simpleKF_gX(dataRawLSM6[0],0.01,0.2,&kfGyro[0]);
    simpleKF_gY(dataRawLSM6[1],0.01,0.2,&kfGyro[1]);
    simpleKF_gZ(dataRawLSM6[2],0.01,0.2,&kfGyro[2]);

    // Correccion de referencia
    gyro[0] = -kfGyro[2];// * 1.5272e-04 ;  // nuevo eje X
    gyro[1] = kfGyro[1];// * 1.5272e-04 ;  // nuevo eje Y
    gyro[2] = kfGyro[0] * 1.5272e-04 ;  // nuevo eje Z --> deg/s

    // Calculo aceleracion angular cuerpo
    a_gyro = (gyro[2] - gyro_ant);
    gyro_ant = gyro[2];
    */
    countLoops++;

    // Conversion de Period del Timer2A a Hz[0].
    Hz[0] = 1/(0.0000000125*Period);
    Hz[0] = Hz[0] / 18;     // 18 taps es 1 vuelta

    // Deteccion de sentido de giro.
    if (signo == 2){ Hz[0] = Hz[0];} else if(signo == -2){Hz[0] = -Hz[0];}else{Hz[0] = 0;}

    if (-0.1 < Hz[0] && Hz[0] < 0.1){Hz[0] = 0;}

    // Filtro Kalman: Frecuencia angular de rueda
    simple_Hz(Hz[0],0.05,1,&Hz[1]);

    wRW = Hz[1]*2*PI;
    a_ang = derivada(wRW, stDevAcRW, T); // rad/s = RPS * 2pi

    controlPWM[0] = ControlPID(setInputControl, Hz[1], stPIDVel, qVEL);

    if (-6 < controlPWM[0] && controlPWM[0] < 6){controlPWM[0] = 0;}

    // Filtro Kalman: Suavizado de señal de control
    simple_cont(controlPWM[0], 0.5, 1, &controlPWM[1]);

    // Funcion que activa al Actuador
    signo = Actuator(controlPWM[1]);

    if (Period == lastPeriod){
        Period = 0;
        lastPeriod = 0;
    }

    Hz[2] = Hz[1]; // Guardado de ultimo valor

    // Medida de tiempo computacional. 800 000 ticks maximo
    timeSystick[1] = NVIC_ST_CURRENT_R;
    timeSystick[2] = timeSystick[0] - timeSystick[1];
}


void TIMER1AInit(void){
    SYSCTL_RCGCTIMER_R |= (1<<1);  // Enable clock Timer1 subtimer A in run mode

    TIMER1_CTL_R = 0;              // Disable timer1 output
    TIMER1_CFG_R = 0x0;            // Select 32-bit configuration option
    TIMER1_TAMR_R = 0x02;          // Select periodic down counter mode of timer1

    // 0.1 segundo
    //TIMER1_TAILR_R = 8000000-1 ;  // TimerA counter starting count down value from 80000000

    // 0.05 segundo
    TIMER1_TAILR_R = 4000000-1 ;  // TimerA counter starting count down value from 80000000

    // 0.025
    //TIMER1_TAILR_R = 2000000-1 ;  // TimerA counter starting count down value from 80000000

    // 1 segundo
    //TIMER1_TAILR_R = 80000000-1 ;  // TimerA counter starting count down value from 80000000

    TIMER1_ICR_R = 0x1;            // TimerA timeout flag bit clears

    // Enable timer 1 interrupt
    TIMER1_IMR_R |=(1<<0);         // Enables TimerA timeSystick-out  interrupt mask
    TIMER1_CTL_R |= 0x01;          // Enable TimerA module


    // Configuracion interrupcion
    // Timer1A=priority 5
    NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x0000A000; // 1010 -> 101 es 5--> prioridad 5 en INTB pag 152.
    NVIC_EN0_R |= (1<<21);         // enable interrupt 21 in NVIC
}

void TIMER1A_Handler(void){
    if(TIMER1_MIS_R & 0x1){
        sprintf(strOut,"R:%2.2f;V:%2.2f;A:%4.4f\n\r",setInputControl,Hz[1],a_ang);
        UART5_printString(strOut);
    }
    TIMER1_ICR_R = 0x1;
}

void UART5_Handler( void ){
    int size=10;    // tamaño buffer

    getRxUART[numRxUart5Recep] = UART5_DR_R;
    numRxUart5Recep++;

    if (numRxUart5Recep == size){
        numRxUart5Recep=0;
    }
    /*  siempre al final del string se debe enviar en este orden: \r\n  */
    /*  \r: CR o 13 o 0x0D                                              */
    /*  \n: LF o 10 o 0x0A                                              */
    /* Cuando se reciba \n, se completa con NULL el resto del string    */
    if( getRxUART[numRxUart5Recep-1] == 0x0A ){

        setInputControl = StringtoFloat(getRxUART);

        memset(getRxUART + numRxUart5Recep, 0x00, size);
        numRxUart5Recep=0;
    }

    UART5_ICR_R &= ~(0x010); // Clear receive interrupt
}

void AHRSInit (void){
    // Initialise gyroscope bias correction algorithm
    FusionBiasInitialise(&fusionBias, 2, T); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, 2); // gain = 0.5

    // Set optional magnetic field limits
    FusionAhrsSetMagneticField(&fusionAhrs, 25.0f, 65.0f); // valid magnetic field range = 25 uT to 65 uT
}
void doAHRS (void){
    // Calibrate gyroscope
    FusionVector3 uncalibratedGyroscope = {
        .axis.x = kfGyro[0], //dataRawLSM6[0], /* replace this value with actual gyroscope x axis measurement in lsb */
        .axis.y = kfGyro[1], //dataRawLSM6[1], /* replace this value with actual gyroscope y axis measurement in lsb */
        .axis.z = kfGyro[2], //dataRawLSM6[2], /* replace this value with actual gyroscope z axis measurement in lsb */
    };
    FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

    // Calibrate accelerometer
    FusionVector3 uncalibratedAccelerometer = {
        .axis.x = kfAccel[0], //dataRawLSM6[3], /* replace this value with actual accelerometer x axis measurement in lsb */
        .axis.y = kfAccel[1], //dataRawLSM6[4], /* replace this value with actual accelerometer y axis measurement in lsb */
        .axis.z = kfAccel[2], //dataRawLSM6[5], /* replace this value with actual accelerometer z axis measurement in lsb */
    };
    FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

    // Calibrate magnetometer
    FusionVector3 uncalibratedMagnetometer = {
        .axis.x = kfMagn[0], //datosMagn[0], /* replace this value with actual magnetometer x axis measurement in uT */
        .axis.y = kfMagn[1], //datosMagn[1], /* replace this value with actual magnetometer y axis measurement in uT */
        .axis.z = kfMagn[2], //datosMagn[2], /* replace this value with actual magnetometer z axis measurement in uT */
    };
    FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

    // Update gyroscope bias correction algorithm
    calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

    // Update AHRS algorithm
    FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, T);

    // Print Euler angles
    eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));
    //printf("Roll = %0.1f, Pitch = %0.1f, Yaw = %0.1f\r\n", eulerAngles.angle.roll, eulerAngles.angle.pitch, eulerAngles.angle.yaw);
}
void Timer2AInit(void){
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;  // activate timer2 *
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;    // activate port F *
                                   // allow timeSystick to finish activating *
  First = 0;                       // first will be wrong *
  GPIO_PORTF_DIR_R &= ~0x10;       // make PF4 in
  GPIO_PORTF_AFSEL_R |= 0x10;      // enable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;        // enable digital I/O on PF4
                                   // configure PF4  as T2CCP0 (Tabla 1347)
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0FFFF)+0x00070000;
  GPIO_PORTF_AMSEL_R &= ~0x10;     // disable analog functionality on PB6

  TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // disable timer2A during setup
  TIMER2_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
                                   // configure for 24-bit capture mode
  TIMER2_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);
                                   // configure for rising edge event
  TIMER2_CTL_R &= ~(TIMER_CTL_TAEVENT_POS|0xC);
  TIMER2_TAILR_R = 0x0000FFFF;// start value // GPTM TimerA Interval Load

  TIMER2_TAPR_R = 0xFF;            // activate prescale, creating 24-bit
  TIMER2_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
  TIMER2_ICR_R = TIMER_ICR_CAECINT;// clear timer2A capture match flag
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // enable timer2A 16-b, +edge timing, interrupts

  // Configuracion
  // Timer2A=priority 3
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|0x60000000; // 011 0 -> 011 es 3--> prioridad 3 en INTD pag 152.
  NVIC_EN0_R = 0x00800000;     // enable interrupt 23 in NVIC
}

void Timer2A_Handler(void){

  TIMER2_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A capture match

  Period = (First - TIMER2_TAR_R)&0xFFFFFF;// 24 bits, 12.5ns resolution
  /* 24 bits = 16777214
   * 80 MHz -> 12.5 nS
   * -------------------- Maximo 0.2097151875 Segundos
   * p = periodo
   * 16777214 ticks --->  0.2097151875 segundos
   * p              --->  ?? segundos
   * tiempo = p *  0.2097151875 / 16777214
   * tiempo = p * 0.0000000125
   * Hz[0] = 1 / tiempo
   */
  First = TIMER2_TAR_R;            // setup for next

  lastPeriod = Period;

  TIMER2_ICR_R &= ~(0x010); // Clear receive interrupt
}
