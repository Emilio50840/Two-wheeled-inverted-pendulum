#include "stdint.h"
#include "stdio.h"
#include "math.h"

#define DEF_vd 0.075

#define DEF_k1 0.05
#define DEF_k2 2.0
#define DEF_k3 0.9
#define DEF_k4 1.2

#define DEF_kp 0.04
#define DEF_kv 0.004

//Parámetros, rangos y factores
#define tauM	0.4//0.3//0.1654	//par máximo en rueda [Nm]
#define alphaM	1.4//1.57
#define omegaM	9.7
#define uM		11.0
#define uNM		11.0

#define Ts      0.01
#define ppr     22.0
#define pi_     3.141593
#define pi_s2   1.570796
#define Ra      3.0
#define NR      34.0//15.5//34.014
#define R	    0.067//0.035
#define km      0.0008//0.01186
#define Cz	    0.0485
#define Mp	    0.4580
#define b	    0.0965

#define calpha 0.10//0.2

#define deg_2_rad 			pi_/180.0
#define rad_2_deg 			180.0/pi_
#define accel_div_factor 	16384.0
#define gyro_div_factor 	131.0
#define accel_factor 		1.0/accel_div_factor
#define gyro_factor 		1.0/gyro_div_factor

#define DEF_c1 0.995

typedef struct {
    float vd, k1, k2, k3, k4;
    float Xa, Yg, alpha;
    float accelx, angulox, angulox_1, c1, c2;
    float t, iTs, esc, escs;
    float omegar, omegal;
    float kv,kp;
    float sr,sl;
    float incr, incl;
    float Ax, Gy;
    float ur, ul;
    float uWl, uWr;
    float v,alpha_1,theta,thetad,thetatil,taua,u;
    float alphap,vtil,thetap,taur,taul; 
    float Rasnkm,nkm,intvtil,v2tauM;

}control_t;

void init_control(control_t *ctrl);

void calculate_control(control_t *ctrl);

