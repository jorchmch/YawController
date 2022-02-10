/*
 * funciones.h
 *
 *  Created on: 15 oct. 2021
 *      Author: Jorch Mendoza Choquemamani
 */

#ifndef FUNCIONES_H_
#define FUNCIONES_H_

/* ////////////////////////////////////////////////// */
/*         LSM6DSOX ACELEROMETRO Y GIROSCOPIO         */
/* ////////////////////////////////////////////////// */

/*=========================================================================
I2C ADDRESS/BITS
---------------------------------------------------------------------------*/
#define LSM6DS_I2CADDR_DEFAULT 0x6A ///< Default breakout address
/*=========================================================================*/
#define LSM6DS_WHOAMI 0xF          ///< Chip ID register
#define LSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define LSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define LSM6DS_STATUS_REG 0X1E     ///< Status register
#define LSM6DS_OUTX_L_G 0x22       ///< First gyro data register
#define LSM6DS_OUTX_L_A 0x28       ///< First accel data register
#define PI  3.1415926535

/* ////////////////////////////////////////////////// */
/*         LIS3MDL MAGNETOMETRO                       */
/* ////////////////////////////////////////////////// */
/*=========================================================================
I2C ADDRESS/BITS
---------------------------------------------------------------------------*/
#define LIS3MDL_I2CADDR_DEFAULT (0x1C) ///< Default breakout address
/*=========================================================================*/
#define LIS3MDL_REG_WHO_AM_I 0x0F  ///< Register that contains the part ID
#define LIS3MDL_REG_CTRL_REG1 0x20 ///< Register address for control 1
#define LIS3MDL_REG_CTRL_REG2 0x21 ///< Register address for control 2
#define LIS3MDL_REG_CTRL_REG3 0x22 ///< Register address for control 3
#define LIS3MDL_REG_CTRL_REG4 0x23 ///< Register address for control 3
#define LIS3MDL_REG_OUT_X_L 0x28   ///< Register address for X axis lower byte


/* ////////////////////////////////////////////////// */
/*         PROTOTIPO DE FUNCIONES INICIALES           */
/* ////////////////////////////////////////////////// */
char I2C1writeByte(int slaveAddr, char memAddr, char data);
char I2C1readBytes(int slaveAddr, char memAddr, int byteCount, char* data);
static int I2C_wait_till_done(void);
void LSM6DSOXgetData(int16_t *RawData);
void LIS3MDLgetData(int16_t *RawData);
void doAHRS (void);
void UART5_printString(char *str);
void UART5_Transmitter(unsigned char data);

/* ////////////////////////////////////////////////// */
/*         SIMPLE KALMAN FILTER FOR SENSORS           */
/* ////////////////////////////////////////////////// */
float x_Hz,M_mark_Hz,P_Hz;
float x_Acc,M_mark_Acc,P_Acc;
float x_cont,M_mark_cont,P_cont;
float x_contAc,M_mark_contAc,P_contAc;
float M_mark_mX,M_mark_mY,M_mark_mZ, M_mark_aX,M_mark_aY,M_mark_aZ, M_mark_gX,M_mark_gY,M_mark_gZ;
float x_mX,M_mark_mX,P_mX, x_mY,M_mark_mY,P_mY, x_mZ,M_mark_mZ,P_mZ;  // kalman del magnetometro, variables globales
float x_aX,M_mark_aX,P_aX, x_aY,M_mark_aY,P_aY, x_aZ,M_mark_aZ,P_aZ;  // kalman del acelerometro, variables globales
float x_gX,M_mark_gX,P_gX, x_gY,M_mark_gY,P_gY, x_gZ,M_mark_gZ,P_gZ;  // kalman del giroscopio, variables globales




void PWMInit(void);
void GPIOInit(void);

void simple_Hz(float i,float Q, float R, float *o);
void simple_Acc(float i,float Q, float R, float *o);
void simple_cont(float i,float Q, float R, float *o);
void simple_contAc(float i,float Q, float R, float *o);
void simpleKF_mX(float i,float Q, float R, float *o);
void simpleKF_mY(float i,float Q, float R, float *o);
void simpleKF_mZ(float i,float Q, float R, float *o);
void simpleKF_aX(float i,float Q, float R, float *o);
void simpleKF_aY(float i,float Q, float R, float *o);
void simpleKF_aZ(float i,float Q, float R, float *o);
void simpleKF_gX(float i,float Q, float R, float *o);
void simpleKF_gY(float i,float Q, float R, float *o);
void simpleKF_gZ(float i,float Q, float R, float *o);
float integrar( float e, float *store,float T);
float derivada( float e, float *store,float T);
void initControlPID(const float *constPID, float *q, const float T);
float ControlPID(float Ref, float Salida, float *store, float *q);
int Actuator(float valor);
void I2Cinit(void);
void PLL_Init(void);
void SysTickInit (void);
void Uart5Init(void);
int LIS3MDLInit (void);
int LSM6DSOXInit (void);
float StringtoFloat(char *mensaje);
/* ////////////////////////////////////////////////// */
/*         SIMPLE KALMAN FILTER FOR HZ           */
/* ////////////////////////////////////////////////// */




#endif /* FUNCIONES_H_ */
