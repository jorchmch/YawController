/*
 * funciones.c
 *
 *  Created on: 15 oct. 2021
 *      Author: Jorch
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "funciones.h"

//% conversion adafruit
//% acelerometro de 4G conversion con : 0.0012   (m/s2)
//% gyroscopio de 250dps conversion con : 1.5272e-04 (rad/s)
//% magnetometro de 4gauss conversion con : 0.0146 (uT)



void LSM6DSOXgetData(int16_t *RawData){
    //Local variables

    char sensordata[12];                  // Char array

    //Read 14 Registers in a single burst
    I2C1readBytes(LSM6DS_I2CADDR_DEFAULT,LSM6DS_OUTX_L_G, 12, sensordata);
    //Turn into 16bits data
    for(char i = 0; i < 6; i++){
        RawData[i] = ((int16_t)sensordata[2*i+1] << 8) | sensordata[2*i];
    }
}
char I2C1readBytes(int slaveAddr, char memAddr, int byteCount, char* data){
    // Local variables
    char error;

    // Check for incorrect length
    if (byteCount <= 0)
        return 1;

    // Send slave address and starting address
    I2C1_MSA_R = slaveAddr << 1;            // Assign Slave Address as writing mode
    I2C1_MDR_R = memAddr;                   // Send memory Address
    I2C1_MCS_R = 3;                         // S-(saddr+w)-ACK-maddr-ACK (Start/ Run)
    error = I2C_wait_till_done();           // Wait until write is complete
    if (error) return error;                // Check for error

    // Change bus to read mode
    I2C1_MSA_R = (slaveAddr << 1) + 1;      // Assign Slave Address as reading mode
                                            // Restart: -R-(saddr+r)-ACK */
    // If it's the last Byte NonACK
    if (byteCount == 1)
        I2C1_MCS_R = 7;                     // -data-NACK-P (Start/Transmit/Stop)
    else
        I2C1_MCS_R = 0x0B;                  // -data-ACK- (Ack/Start/Transmit)
    error = I2C_wait_till_done();           // Wait until write is complete
    if (error) return error;                // Check for error

    // Store the received data
    *data++ = I2C1_MDR_R;                   // Coming data from Slave

    // Single byte read
    if (--byteCount == 0){
        while(I2C1_MCS_R & 0x40);           // Wait until bus is not busy
        return 0;                           // No error
    }

    // Read the rest of the bytes
    while (byteCount > 1){
        I2C1_MCS_R = 9;                     // -data-ACK-
        error = I2C_wait_till_done();
        if (error) return error;
        byteCount--;
        *data++ = I2C1_MDR_R;               // store data received
    }

    I2C1_MCS_R = 5;                         // -data-NACK-P
    error = I2C_wait_till_done();
    *data = I2C1_MDR_R;                     // Store data received
    while(I2C1_MCS_R & 0x40);               // Wait until bus is not busy

    return 0;                               // No error
}
void I2Cinit(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;            // Enable the clock for port A
    while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R0)==0);   // Wait until clock is initialized
    SYSCTL_RCGCI2C_R   |= SYSCTL_RCGCI2C_R1;            // Enable the clock for I2C 1
    while((SYSCTL_PRI2C_R & SYSCTL_RCGCI2C_R1)==0);      // Wait until clock is initialized
    GPIO_PORTA_DEN_R |= 0xC0;               // PA6 and PA7 as digital
                                            // 1100 00 00
    // Configure Port A pins 6 and 7 as I2C 1
    GPIO_PORTA_AFSEL_R |= 0xC0;             // Use PA6, PA7 alternate function
    GPIO_PORTA_PCTL_R  |= 0X33000000;       // Configure PA6 and PA7 as I2C
    GPIO_PORTA_ODR_R   |= 0x80;             // SDA (PA7) pin as open Drain
    I2C1_MCR_R = 0x0010;                    // Enable I2C 1 master function

    /* Configure I2C 1 clock frequency
    (1 + TIME_PERIOD ) = SYS_CLK /(2*
    ( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
    TIME_PERIOD = 80 ,000 ,000/(2(6+4) *100000) - 1 = 7 */
    I2C1_MTPR_R = 0x27; //39
}
void PLL_Init(void){

  // 0) Use RCC2
  SYSCTL_RCC2_R |=  SYSCTL_RCC2_USERCC2;  // USERCC2
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |=  SYSCTL_RCC2_BYPASS2;  // BYPASS2, PLL bypass
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R = ((SYSCTL_RCC_R & ~SYSCTL_RCC_XTAL_M)
                  | SYSCTL_RCC_XTAL_16MHZ);   // clear XTAL field, bits 10-6
                                              // 10101, configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;  // configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  // 4) set the desired system divider
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;   // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~(SYSCTL_RCC2_SYSDIV2_M | SYSCTL_RCC2_SYSDIV2LSB))  // clear system clock divider
                  | (4<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS)==0){};  // wait for PLLRIS bit
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;

}

void SysTickInit (void){
    //Configuracion 100 Hz
    NVIC_ST_CTRL_R = 0;             // 1) disable SysTick during setup

    // 100 Hz
    NVIC_ST_RELOAD_R = 800000;      // 2) maximum reload value

    // 50 Hz
    //NVIC_ST_RELOAD_R = 400000;      // 2) maximum reload value

    NVIC_ST_CURRENT_R = 0;          // 3) any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R & 0x00FFFFFF) | 0x40000000 ; // prioridad 2
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE |NVIC_ST_CTRL_INTEN;    // 4) enable SysTick with core clock

}
void Uart5Init(void){
    // Enable clock to UART5 and wait for the clock
    SYSCTL_RCGCUART_R |= 0x20;
    while((SYSCTL_PRUART_R & 0x20)==0);

    // Enable clock to PORTE for PE4/Rx and RE5/Tx and wait
    SYSCTL_RCGCGPIO_R |= 0x10;
    while((SYSCTL_PRGPIO_R & 0x10)==0);

    // UART5 initialization
    UART5_CTL_R &= ~0x01;                           // Disable UART5
    UART5_IBRD_R = (UART5_IBRD_R & ~0XFFFF)+130;    // for 38400 baud rate, integer = 104
    // IBRD=int(80000000/(16*38400)) =   int(130.2083)
    UART5_FBRD_R = (UART5_FBRD_R & ~0X3F)+13;       // for 38400 baud rate, fractional = 11
    // FBRD = round(0.2083 * 64) = 13
    UART5_CC_R = 0x00;                              // Select system clock (based on clock source and divisor factor)*/
    //UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x70;    // Data length 8-bit, not parity bit, FIFO enable
    UART5_LCRH_R = (UART5_LCRH_R & ~0XFF)| 0x60;    // Data length 8-bit, not parity bit, FIFO disable
    UART5_CTL_R &= ~0X20;                           // HSE = 0
    UART5_CTL_R |= 0X301;                           // UARTEN = 1, RXE = 1, TXE =1

    // UART5 TX5 and RX5 use PE5 and PE4.
    // Configure them as digital and enable alternate function.
    GPIO_PORTE_DEN_R = 0x30;             // Set PE4 and PE5 as digital.
    GPIO_PORTE_AFSEL_R = 0x30;           // Use PE4,PE5 alternate function.
    GPIO_PORTE_AMSEL_R = 0X00;           // Turn off analog function.
    GPIO_PORTE_PCTL_R = 0x00110000;      // Configure PE4 and PE5 for UART.

    // Enable UART5 interrupt
    UART5_ICR_R &= ~(0x0780);            // Clear receive interrupt
    UART5_IM_R  = 0x0010;                // Enable UART5 Receive interrupt

    // Configuracion interrupcion
    // Timer1A=priority 4
    NVIC_PRI15_R = (NVIC_PRI15_R&0xFFFF00FF)|0x00008000; // 100 0 -> 100 es 4--> prioridad 4 en INTB pag 152.
    NVIC_EN1_R  |= (1<<29);               // Enable IRQ61 for UART5
}

int LIS3MDLInit (void){
    //  Local variables
    char Ack;
    //  Check connection
    I2C1readBytes(LIS3MDL_I2CADDR_DEFAULT, LIS3MDL_REG_WHO_AM_I, 1, &Ack);
    if(0x3D==Ack){
        //Configure LSM6DSOX
        I2C1writeByte(LIS3MDL_I2CADDR_DEFAULT,LIS3MDL_REG_CTRL_REG1, 0x7C); // Ultra-high-performance mode 80HZ X,Y
        I2C1writeByte(LIS3MDL_I2CADDR_DEFAULT,LIS3MDL_REG_CTRL_REG2, 0x00); // 4 gauss
        I2C1writeByte(LIS3MDL_I2CADDR_DEFAULT,LIS3MDL_REG_CTRL_REG3, 0x00); // continuous mode
        I2C1writeByte(LIS3MDL_I2CADDR_DEFAULT,LIS3MDL_REG_CTRL_REG4, 0x0C); // Ultra-high-performance mode Z
        return 1;
    }
    return 0;
}
int LSM6DSOXInit(void){
    //  Local variables
    char Ack;
    //  Check connection
    I2C1readBytes(LSM6DS_I2CADDR_DEFAULT, LSM6DS_WHOAMI, 1, &Ack);
    if(0x6C==Ack){
        //Configure LSM6DSOX
        I2C1writeByte(LSM6DS_I2CADDR_DEFAULT,LSM6DS_CTRL1_XL, 0x48); //Accel: 104Hz 4G
        I2C1writeByte(LSM6DS_I2CADDR_DEFAULT,LSM6DS_CTRL2_G, 0x40); // Gyro: 104Hz 250dps
        return 1;
    }
    return 0;
}
static int I2C_wait_till_done(void){
    while(I2C1_MCS_R & (1<<0) != 0); // Wait until I2C master is not busy
    return I2C1_MCS_R & 0xE;         // Check for errors
}
char I2C1writeByte(int slaveAddr, char memAddr, char data){
    char error;
    // Send slave address and starting address
    I2C1_MSA_R = (slaveAddr << 1);      // Assign Master as writing mode
    I2C1_MDR_R = memAddr;               // Assign Starting address
    error = I2C_wait_till_done();       // Wait until write is complete
    if (error) return error;

    I2C1_MCS_R = 0x03;                  // S-(saddr+w)-ACK-maddr-ACK (Start/ Transmit)
    error = I2C_wait_till_done();       // Wait until write is complete
    if (error) return error;

    // Send data to write
    I2C1_MDR_R = data;                   // Assign data to send
    I2C1_MCS_R = 5;                      // -data-ACK-P (Transmit/ Stop)
    error = I2C_wait_till_done();        // Wait until write is complete
    while(I2C1_MCS_R & 0x40);            // Wait until bus is not busy
    error = I2C1_MCS_R & 0xE;            // Check error
    if (error) return error;
    return 0;
}
void UART5_Transmitter(unsigned char data){
    while((UART5_FR_R & (1<<5)) != 0);      // Wait until Tx buffer is not full
    UART5_DR_R = data;                      // Before giving it another byte
}

void UART5_printString(char *str){
  while(*str){
        UART5_Transmitter(*(str++));
    }
}




void GPIOInit(void){
    SYSCTL_RCGCGPIO_R |= (1<<5); //Set bit5 to enable

    //PORTF0 has special function, need to unlock to modify
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;   //Unlock commit register
    GPIO_PORTF_CR_R = 0X1F;              //Make PORTF0 configurable

    //PF4 PF0 PF1: INPUTS
    //PF3 : OUT
    //Initialize PF3 as a digital output, and PF4 as digital input pins
    GPIO_PORTF_DIR_R |= (1<<3);                         // Set PF3 as digital OUT to control green LED
    GPIO_PORTF_DEN_R |= (1<<3);    // make PORTF digital pins

    // enable interrupt in NVIC and set priority to 2
    //NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF)|0x00400000;  //Priority 2 4 = 0100 : 010 ES 2
    //NVIC_EN0_R |= (1<<30);  // enable IRQ30 (D30 of ISER[0])

}

void PWMInit(void){
    // Clock setting for PWM and GPIO PORT
    SYSCTL_RCGCPWM_R |= (1<<1);         // Enable clock to PWM1 module
    SYSCTL_RCGCGPIO_R |= (1<<5);        // Enable system clock to PORTF
    SYSCTL_RCC_R |= (1<<20);            // Enable System Clock Divisor function
    SYSCTL_RCC_R |= (7<<17);            // Use pre-divider value of 64 and after that feed clock to PWM1 module

    // Setting of PF2 pin for M1PWM6 channel output pin
    GPIO_PORTF_AFSEL_R |= (1<<2);       // PF2 sets a alternate function
    GPIO_PORTF_PCTL_R &= ~0x00000F00;   // Set PF2 as output pin
    GPIO_PORTF_PCTL_R |= 0x00000500;    // Make PF2 PWM output pin
    GPIO_PORTF_DEN_R |= (1<<2);         // set PF2 as a digital pin

    PWM1_3_CTL_R &= ~(1<<0);            // Disable Generator 3 counter
    PWM1_3_CTL_R &= ~(1<<1);            // Select down count mode of counter 3
    PWM1_3_GENA_R = 0x0000008C;         // Set PWM output when counter reloaded and clear when matches PWMCMPA
    PWM1_3_LOAD_R = 25000;               // Set load value for 50Hz 80MHz/64 = 1250kHz and (1250KHz/25000)
    PWM1_3_CMPA_R = 1;               // Set duty cyle to to minumum value
    PWM1_3_CTL_R = 1;                   // Enable Generator 3 counter
    PWM1_ENABLE_R = 0x40;               // Enable PWM1 channel 6 output
}

//--------------------Funcion para Integrar---------------------
/* float integrar(float e, float *store, float T)
 * Description: Deriva una señal e, guarda datos antiguos y se configura el periodo
 * Se necesita crear variable global:  float stor[3] = {0,0,0}
 */
float integrar( float e, float *store,float T){
    // e : señal de entrada     u : señal de salida

    //  store[0] -> e[2]: es e[-2]
    //  store[1] -> e[1]: es e[-1]
    //  store[2] -> u[0]: es u[0]
    //  store[3] -> u[1]: es u[-1]
    //  store[4] -> u[2]: es u[-2]

    store[2] = store[3] + (T * e);
    store[0] = store[1];
    store[1] = e;
    store[4] = store[3];
    store[3] = store[2];


    return store[4];
}

//--------------------Funcion para derivar---------------------
/* float derivada( float e, float *store,float T)
 * Description: Deriva una señal e, guarda datos antiguos y se configura el periodo
 * Se necesita crear variable global:  float stor[3] = {0,0,0}
 */
float derivada( float e, float *store,float T){
    // e : señal de entrada     u : señal de salida

    //  store[0] -> e[2]: es e[-2]
    //  store[1] -> e[1]: es e[-1]
    //  store[2] -> u[1]: es u[-1]
    //  store[3] -> u

    store[3] = store[2] + (1/T)*(e - 2*store[1] + store[0]);
    store[0] = store[1];
    store[1] = e;
    store[2] = store[3];
    return store[2];
}

void LIS3MDLgetData(int16_t *RawData){
    //Local variables
    char sensordata[6];                  // Char array

    //Read 14 Registers in a single burst
    I2C1readBytes(LIS3MDL_I2CADDR_DEFAULT,LIS3MDL_REG_OUT_X_L, 6, sensordata);
    //Turn into 16bits data
    for(char i = 0; i < 3; i++){
        RawData[i] = ((int16_t)sensordata[2*i+1] << 8) | sensordata[2*i];
    }
}
void simple_Hz(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_Hz==0){
    x_Hz=i;
    M_mark_Hz=1;
    P_Hz=1;
}

    xp = A*x_Hz;
    Pp = (A*P_Hz*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_Hz= xp + K*(i - H*xp);
    P_Hz = Pp - K*H*Pp;

*o = x_Hz;

}

void simple_Acc(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_Acc==0){
    x_Acc=i;
    M_mark_Acc=1;
    P_Acc=1;
}

    xp = A*x_Acc;
    Pp = (A*P_Acc*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_Acc= xp + K*(i - H*xp);
    P_Acc = Pp - K*H*Pp;

*o = x_Acc;

}
void simple_contAc(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_contAc==0){
    x_contAc=i;
    M_mark_contAc=1;
    P_contAc=1;
}

    xp = A*x_contAc;
    Pp = (A*P_contAc*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_contAc= xp + K*(i - H*xp);
    P_contAc = Pp - K*H*Pp;

*o = x_contAc;

}

void simple_cont(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_cont==0){
    x_cont=i;
    M_mark_cont=1;
    P_cont=1;
}

    xp = A*x_cont;
    Pp = (A*P_cont*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_cont= xp + K*(i - H*xp);
    P_cont = Pp - K*H*Pp;

*o = x_cont;

}

void simpleKF_mX(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_mX==0){
    x_mX=i;
    M_mark_mX=1;
    P_mX=1;
}

    xp = A*x_mX;
    Pp = (A*P_mX*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_mX= xp + K*(i - H*xp);
    P_mX = Pp - K*H*Pp;

*o = x_mX;

}

void simpleKF_mY(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_mY==0){
    x_mY=i;
    M_mark_mY=1;
    P_mY=1;
}

    xp = A*x_mY;
    Pp = (A*P_mY*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_mY= xp + K*(i - H*xp);
    P_mY = Pp - K*H*Pp;

*o = x_mY;

}

void simpleKF_mZ(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_mZ==0){
    x_mZ=i;
    M_mark_mZ=1;
    P_mZ=1;
}

    xp = A*x_mZ;
    Pp = (A*P_mZ*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_mZ= xp + K*(i - H*xp);
    P_mZ = Pp - K*H*Pp;

*o = x_mZ;

}

void simpleKF_aX(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_aX==0){
    x_aX=i;
    M_mark_aX=1;
    P_aX=1;
}

    xp = A*x_aX;
    Pp = (A*P_aX*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_aX= xp + K*(i - H*xp);
    P_aX = Pp - K*H*Pp;

*o = x_aX;

}

void simpleKF_aY(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_aY==0){
    x_aY=i;
    M_mark_aY=1;
    P_aY=1;
}

    xp = A*x_aY;
    Pp = (A*P_aY*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_aY= xp + K*(i - H*xp);
    P_aY = Pp - K*H*Pp;

*o = x_aY;

}

void simpleKF_aZ(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_aZ==0){
    x_aZ=i;
    M_mark_aZ=1;
    P_aZ=1;
}

    xp = A*x_aZ;
    Pp = (A*P_aZ*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_aZ= xp + K*(i - H*xp);
    P_aZ = Pp - K*H*Pp;

*o = x_aZ;

}

void simpleKF_gX(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_gX==0){
    x_gX=i;
    M_mark_gX=1;
    P_gX=1;
}

    xp = A*x_gX;
    Pp = (A*P_gX*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_gX= xp + K*(i - H*xp);
    P_gX = Pp - K*H*Pp;

*o = x_gX;

}

void simpleKF_gY(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_gY==0){
    x_gY=i;
    M_mark_gY=1;
    P_gY=1;
}

    xp = A*x_gY;
    Pp = (A*P_gY*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_gY= xp + K*(i - H*xp);
    P_gY = Pp - K*H*Pp;

*o = x_gY;

}

void simpleKF_gZ(float i,float Q, float R, float *o){

float A=1, H=1; float xp,Pp,K;

if (M_mark_gZ==0){
    x_gZ=i;
    M_mark_gZ=1;
    P_gZ=1;
}

    xp = A*x_gZ;
    Pp = (A*P_gZ*(1/A)) + Q;
    K = Pp*(1/H) * ( 1/(H*Pp*(1/H) + R) );
    x_gZ= xp + K*(i - H*xp);
    P_gZ = Pp - K*H*Pp;

*o = x_gZ;

}
//--------------------Funcion initControlPID---------------------
/* Description: Convierte coeficientes Kp,Ki,Kd en q1,q2,q3
 * Resultado se guarda en *q (variable global)
 */
void initControlPID(const float *constPID, float *q, const float T){
    // constPID[0] = Kp
    // constPID[1] = Ki
    // constPID[2] = Kd

    // q[0] = q0
    // q[1] = q1
    // q[2] = q2

    // T: periodo de muestreo

    volatile float Kc,Ti,Td;
    Kc = constPID[0];
    Ti = Kc/constPID[1];
    Td = constPID[2]/Kc;

    q[0] = Kc*(1+T/(2*Ti)+Td/T);
    q[1] = -Kc*(1-T/(2*Ti)+(2*Td)/T);
    q[2] = (Kc*Td)/T;
}

//--------------------Funcion PID ---------------------
/* float ControlPID(float Ref, float Salida, float *store, const float q)
 * Description: Control PID por metodo de ecuacion diferencia
 * Se necesita crear variable global:  float store[6] = {0,0,0,0,0,0}
 */
float ControlPID(float Ref, float Salida, float *store, float *q){

    // store[0] = e -> e[k]
    // store[1] = e_1 -> e[k-1]
    // store[2] = e_2 -> e[k-2]
    // store[3] = u_1 -> u[k-1]
    // store[4] = u -> u[k]
    // store[5] = delta_u

    // q[0] = q0
    // q[1] = q1
    // q[2] = q2

    // Error
    store[0] = Ref - Salida;

    // Pre- Control
    store[5] = q[0] * store[0] + q[1] * store[1] + q[2] * store[2];

    // Control
    store[4] = store[3] + store[5];

    // Almacenamiento de datos anteriores
    store[2]=store[1];
    store[1]=store[0];
    store[3]=store[4];

    return store[4];
}

float StringtoFloat(char *mensaje){ //numero entre 0 - 100

    char *token;
    char delimitador[] = "\r\n";
    float code;

    token = strtok(mensaje,delimitador);
    code=atof(token); //valores entre 0 y 100 %
    memset(token,0x00,10);

    return code;       // Para salida UART

}


//--------------------Funcion Actuator---------------------
/* Description: General señal PWM, devuelve el sentido de giro.
 */
int Actuator(float valor){
    float nf,lim=99;
    int sig;
    // Saturador
    if (abs(valor) > lim){
        nf = lim;
    }else{
        nf = abs(valor);
    }

    // Signo - Orientacion de giro
    if (valor > 0 ){  // Horario

        GPIO_PORTF_DATA_R &= (0<<3);
        sig = 2;
    }
    else if (valor < 0 ){  // Antihorario

        GPIO_PORTF_DATA_R |= (1<<3);
        sig = -2;
    } else {
        sig = 0;
    }

    // Generacion PWM
    float valueF = 25000*nf/100;
    int valueI = (int)valueF;
    PWM1_3_CMPA_R = valueI;

    return sig;

}
