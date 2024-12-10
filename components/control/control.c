#include <stdio.h>
#include <string.h>
#include "control.h"


void init_control(control_t *ctrl){
    memset(ctrl, 0, sizeof(control_t));
    ctrl->vd = DEF_vd;
    ctrl->k1 = DEF_k1;
    ctrl->k2 = DEF_k2;
    ctrl->k3 = DEF_k3;
    ctrl->k4 = DEF_k4;
    //ctrl->Xa = 0.0;
    //ctrl->Yg = 0.0;
    //ctrl->alpha = 0.0;
    ctrl->accelx = 0.0;
    ctrl->angulox = 0.0;
    ctrl->c1 = DEF_c1;
    ctrl->c2 = 1.0-DEF_c1;
    ctrl->t = 0.0;
    ctrl->iTs = 1/Ts;
    ctrl->esc = pi_/(2*ppr*NR);
    ctrl->escs = 127.0/uM;
    //ctrl->omegar = 0.0;
    //ctrl->omegal = 0.0;
    //ctrl->ur = 0.0;
    //ctrl->ul = 0.0;
    //ctrl->v = 0.0;
    //ctrl->alpha_1 = 0.0;
    //ctrl->theta = 0.0;
    //ctrl->thetad = 0.0;
    //ctrl->thetatil = 0.0;
    ctrl->Rasnkm = Ra/(NR*km);
    ctrl->nkm = NR*km;
    ctrl->v2tauM = 2*tauM;
}

void calculate_control(control_t *ctrl){
    ctrl->theta = ctrl->sr - ctrl->sl;
    if(ctrl->theta > 160.0)
        ctrl->theta = 160.0;
    else if(ctrl->theta < -160.0)
        ctrl->theta = -160.0;

    ctrl->theta=(-0.3*(ctrl->theta))/160.0;	//Error de seguimiento en radianes

    // Calulo velocidad derecha en radianes.
    ctrl->omegar=(ctrl->incr)*(ctrl->esc)*(ctrl->iTs);

    // Calculo velocidad izquierda en radianes.
    ctrl->omegal=(ctrl->incl)*(ctrl->esc)*(ctrl->iTs);

    // Los valores del MPU los paso a valores flotantes con sus respectivas escalas.
    ctrl->Xa=-(ctrl->Ax)*accel_factor;//inclinación en rango de -1 a 1
    ctrl->Yg=(ctrl->Gy)*gyro_factor;//Yg en grados sobre segundo
    ctrl->Yg=deg_2_rad*ctrl->Yg;//Yg en rad sobre segundo
// Calculo de filtro complementario
    ctrl->accelx=ctrl->Xa*pi_s2;//inclinación en rango de -pi/2 a pi/2 rad
    ctrl->angulox=ctrl->c1*(ctrl->angulox_1+(ctrl->Yg*Ts))+(ctrl->c2*ctrl->accelx);//ecuación del filtro complementario
    ctrl->angulox_1=ctrl->angulox;//respaldando valor pasado
    ctrl->alpha=-ctrl->angulox;
    ctrl->alpha=ctrl->alpha-calpha;//compensación de alineación de IMU
    if(ctrl->alpha>=alphaM)
        ctrl->alpha=alphaM;
    else if(ctrl->alpha<=(-alphaM))
        ctrl->alpha=-alphaM;
// Calculo velocidad traslacional
    ctrl->v=(ctrl->omegar+ctrl->omegal)*R/2;	

    ctrl->thetap=(ctrl->omegar-ctrl->omegal)*R/(2*b);
    
    ctrl->thetatil=ctrl->theta-ctrl->thetad;  				// theta tilde
    ctrl->alphap=(ctrl->alpha-ctrl->alpha_1)*ctrl->iTs;				// alpha punto
    ctrl->alpha_1=ctrl->alpha;
    ctrl->vtil=ctrl->v-ctrl->vd;								// v tilde
    if((ctrl->intvtil<ctrl->v2tauM)&&(ctrl->intvtil>(-ctrl->v2tauM)))		//Integral de v tilde
        ctrl->intvtil=ctrl->intvtil+(Ts*ctrl->vtil);
    else
    {
        if(ctrl->intvtil>=ctrl->v2tauM)
        ctrl->intvtil=0.95*ctrl->v2tauM;
        else if(ctrl->intvtil<=(-ctrl->v2tauM))
        ctrl->intvtil=-0.95*ctrl->v2tauM; 
    }
// Esfuerzos de control	
    ctrl->taua=(-ctrl->kv*ctrl->thetap-ctrl->kp*ctrl->thetatil)*2*b/R;
    ctrl->u=(ctrl->k1*ctrl->alphap)+(ctrl->k2*ctrl->alpha)+(ctrl->k3*ctrl->vtil)+(ctrl->k4*ctrl->intvtil);
    if(ctrl->u>=ctrl->v2tauM)
        ctrl->u=ctrl->v2tauM;
    else if(ctrl->u<=(-ctrl->v2tauM))
        ctrl->u=-ctrl->v2tauM; 
                    
// Pares por rueda		
    ctrl->taur=(ctrl->taua+ctrl->u)/2.0;  // par derecho
    ctrl->taul=(-ctrl->taua+ctrl->u)/2.0;  // par izquierdo
                    
// Voltaje rueda izquierda
    ctrl->ul=(ctrl->taul*ctrl->Rasnkm)+(ctrl->nkm*ctrl->omegal);	
    //ctrl->ul=uNM*sinf(0.628*ctrl->t);
    //ul=0;
    //ul=2.5;
    if(ctrl->ul>=uNM)
        ctrl->ul=uNM;
    else if(ctrl->ul<=(-uNM))
        ctrl->ul=-uNM;
    ctrl->uWl = ctrl->ul*ctrl->escs;
    
        
// Voltaje rueda derecha    
    ctrl->ur = (ctrl->taur*ctrl->Rasnkm)+(ctrl->nkm*ctrl->omegar);
    //ctrl->ur=uNM*sin(0.628*ctrl->t);
    //ur=0;
    //ur=2.5;
    if(ctrl->ur >=uNM)
        ctrl->ur = uNM;
    else if(ctrl->ur <= (-uNM))
        ctrl->ur = -uNM;
    ctrl->uWr = ctrl->ur * ctrl->escs;
    ctrl->t += Ts;
}
