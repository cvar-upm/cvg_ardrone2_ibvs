#include "stateObserver/models/EKF_model_LinkQuad.h"

//#define ACEL_G		9.81


//Tiempo de integracion
//float timeIntegration;

LinkQuadModel1::LinkQuadModel1() {
	// Valores por defecto durante 2012:
	//		Input_gains = diag([10*pi/180, 10*pi/180, 100*pi/180, -1]);
	//		Input_gains = diag([gain_pitch, gain_roll, gain_dyaw, gain_dz]);
	gain_pitch = 10.0*M_PI/180.0;
	gain_roll  = 10.0*M_PI/180.0;
	gain_dyaw  = 100.0*M_PI/180.0;
	gain_dz    = -1.0;
}

//Parrot Modelo 1
void LinkQuadModel1::processModel(Vector* Statek1, Vector* Statek, Vector* Inputs)
{
  // ************** State ************** 
  // Xestate order
  //  X1 : proportional to pitch
  //  X2 : proportional to roll
  //  X3 : internal variable related to d(yaw)/dt
  //  X4 : proportional to d(yaw)/dt
  //  X5 : yaw
  //  X6 : internal variable related to d(Z)/dt
  //  X7 : proportional to d(Z)/dt
  //  X8 : Z
  //  X9 : X
  //  X10: Y
  //  X11: VX
  //  X12: VY

	// Reading statek values

	float Paux_k = Statek->getValueData(1);
	float Raux_k = Statek->getValueData(2);
	float dYaux_k = Statek->getValueData(3);
	float dYp_k = Statek->getValueData(4);
	float Y_k = Statek->getValueData(5);
	float dZaux_k = Statek->getValueData(6);
	float dZp_k = Statek->getValueData(7);
	float Z_k = Statek->getValueData(8);
	float x_k = Statek->getValueData(9);
	float y_k = Statek->getValueData(10);
	float vx_k = Statek->getValueData(11);
	float vy_k = Statek->getValueData(12);


	// Reading input values

	float P_ref = Inputs->getValueData(1);
	float R_ref = Inputs->getValueData(2);
	float dY_ref = Inputs->getValueData(3);
	float dZ_ref = Inputs->getValueData(4);


	// Reference value gains are already included in the State Model


	// Definition of some model constants
	float g   = 9.81;

	// Definition of aerodynamic friction model constants
	float ki  = 0.0496*2.45;
	float ci  = 4.0;
	float ktr = 2.0;


	// Assignment to state in k+1

	Statek1->setValueData(Paux_k - 1.0*timeIntegration*(24.607*Paux_k - 1.0*P_ref*gain_pitch),1);
	Statek1->setValueData(Raux_k - 1.0*timeIntegration*(15.317*Raux_k - 1.0*R_ref*gain_roll),2);
	Statek1->setValueData(dYaux_k - 1.0*timeIntegration*(12.821*dYaux_k + 72.283*dYp_k - 1.0*dY_ref*gain_dyaw),3);
	Statek1->setValueData(dYp_k + dYaux_k*timeIntegration,4);
	Statek1->setValueData(Y_k + 67.213*dYp_k*timeIntegration,5);
	Statek1->setValueData(dZaux_k - 1.0*timeIntegration*(3.3651*dZaux_k + 14.598*dZp_k - 1.0*dZ_ref*gain_dz),6);
	Statek1->setValueData(dZp_k + dZaux_k*timeIntegration,7);
	Statek1->setValueData(Z_k - 14.189*dZp_k*timeIntegration,8);
	Statek1->setValueData(x_k + timeIntegration*vx_k,9);
	Statek1->setValueData(y_k + timeIntegration*vy_k,10);
	Statek1->setValueData(vx_k - 1.0*ktr*timeIntegration*(g*sin(38.575*Raux_k)*sin(Y_k) + g*sin(58.227*Paux_k)*cos(Y_k) + ki*vx_k*(ci + sqrt( pow(vx_k,2) + pow(vy_k,2) ))),11);
	Statek1->setValueData(vy_k - 1.0*ktr*timeIntegration*(g*sin(58.227*Paux_k)*sin(Y_k) + ki*vy_k*(ci + sqrt( pow(vx_k,2) + pow(vy_k,2) )) - 1.0*g*sin(38.575*Raux_k)*cos(Y_k)),12);

	return;
}


//User Observation model
void LinkQuadModel1::observationModel(Vector* Output, Vector* Statek)
{
	//  Medidas:
	//  01 pitch: C(1,1)*X1
	//  02 roll : C(2,2)*X2
	//  03 dYaw : C(3,4)*X4
	//  04 Yaw  : X5
	//  05 dZ   : C(5,7)*X7
	//  06 Z    : X8
	//  07 X    : X9
	//  08 Y    : X10
	//  09 Vx   : X11
	//  10 Vy   : X12
	//  11 [Vxm;: [R_Y R_Y]' * [X11;
	//  12 Vym]: [R_Y R_Y]     X12]

	// Reading statek values

	float Paux_k = Statek->getValueData(1);
	float Raux_k = Statek->getValueData(2);
	float dYaux_k = Statek->getValueData(3);
	float dYp_k = Statek->getValueData(4);
	float Y_k = Statek->getValueData(5);
	float dZaux_k = Statek->getValueData(6);
	float dZp_k = Statek->getValueData(7);
	float Z_k = Statek->getValueData(8);
	float x_k = Statek->getValueData(9);
	float y_k = Statek->getValueData(10);
	float vx_k = Statek->getValueData(11);
	float vy_k = Statek->getValueData(12);


	// Filling in Output/measurements vector

	Output->setValueData(58.227*Paux_k,1);
	Output->setValueData(38.575*Raux_k,2);
	Output->setValueData(67.213*dYp_k,3);
	Output->setValueData(Y_k,4);
	Output->setValueData(-14.189*dZp_k,5);
	Output->setValueData(Z_k,6);
	Output->setValueData(x_k,7);
	Output->setValueData(y_k,8);
	Output->setValueData(vx_k,9);
	Output->setValueData(vy_k,10);
	Output->setValueData(vx_k*cos(Y_k) + vy_k*sin(Y_k),11);
	Output->setValueData(vy_k*cos(Y_k) - 1.0*vx_k*sin(Y_k),12);
	Output->setValueData(Y_k,13);

	return;
}


//User Jacobian process model
void LinkQuadModel1::jacobiansProcessModel(Matrix* MatJacFx, Matrix* MatJacFu, Vector* Statek, Vector* Inputs)
{

	  // ************** State **************
	  // Xestate order
	  //  X1 : proportional to pitch
	  //  X2 : proportional to roll
	  //  X3 : internal variable related to d(yaw)/dt
	  //  X4 : proportional to d(yaw)/dt
	  //  X5 : yaw
	  //  X6 : internal variable related to d(Z)/dt
	  //  X7 : proportional to d(Z)/dt
	  //  X8 : Z
	  //  X9 : X
	  //  X10: Y
	  //  X11: VX
	  //  X12: VY

	// Sample code from Jose Luis
	//MatJacFx->setZeros();
	//MatJacFu->setZeros();

	// Reading statek values

	float Paux_k = Statek->getValueData(1);
	float Raux_k = Statek->getValueData(2);
	float dYaux_k = Statek->getValueData(3);
	float dYp_k = Statek->getValueData(4);
	float Y_k = Statek->getValueData(5);
	float dZaux_k = Statek->getValueData(6);
	float dZp_k = Statek->getValueData(7);
	float Z_k = Statek->getValueData(8);
	float x_k = Statek->getValueData(9);
	float y_k = Statek->getValueData(10);
	float vx_k = Statek->getValueData(11);
	float vy_k = Statek->getValueData(12);


	// Definition of some model constants
	float g   = 9.81;

	// Definition of aerodynamic friction model constants
	float ki  = 0.0496*2.45;
	float ci  = 4.0;
	float ktr = 2.0;
	double eps_jpp = 1e-6;


	// Filling in Process Jacobian, MatJacFx

	MatJacFx->setValueData(1.0 - 24.607*timeIntegration,1,1);
	MatJacFx->setValueData(1.0 - 15.317*timeIntegration,2,2);
	MatJacFx->setValueData(1.0 - 12.821*timeIntegration,3,3);
	MatJacFx->setValueData(-72.283*timeIntegration,3,4);
	MatJacFx->setValueData(timeIntegration,4,3);
	MatJacFx->setValueData(1.0,4,4);
	MatJacFx->setValueData(67.213*timeIntegration,5,4);
	MatJacFx->setValueData(1.0,5,5);
	MatJacFx->setValueData(1.0 - 3.3651*timeIntegration,6,6);
	MatJacFx->setValueData(-14.598*timeIntegration,6,7);
	MatJacFx->setValueData(timeIntegration,7,6);
	MatJacFx->setValueData(1.0,7,7);
	MatJacFx->setValueData(-14.189*timeIntegration,8,7);
	MatJacFx->setValueData(1.0,8,8);
	MatJacFx->setValueData(1.0,9,9);
	MatJacFx->setValueData(timeIntegration,9,11);
	MatJacFx->setValueData(1.0,10,10);
	MatJacFx->setValueData(timeIntegration,10,12);
	MatJacFx->setValueData(-58.227*g*ktr*timeIntegration*cos(58.227*Paux_k)*cos(Y_k),11,1);
	MatJacFx->setValueData(-38.575*g*ktr*timeIntegration*cos(38.575*Raux_k)*sin(Y_k),11,2);
	MatJacFx->setValueData(ktr*timeIntegration*(g*sin(58.227*Paux_k)*sin(Y_k) - 1.0*g*sin(38.575*Raux_k)*cos(Y_k)),11,5);
	if (sqrt( pow(vx_k,2) + pow(vy_k,2) ) < eps_jpp) {
	MatJacFx->setValueData(1.0 - 1.0*ki*ktr*timeIntegration*sqrt( pow(vx_k,2) + pow(vy_k,2) ) - 1.0*ci*ki*ktr*timeIntegration,11,11);
	} else {
	MatJacFx->setValueData(1.0 - 1.0*ki*ktr*timeIntegration*sqrt( pow(vx_k,2) + pow(vy_k,2) ) - (1.0*ki*ktr*timeIntegration*pow(vx_k,2))/sqrt( pow(vx_k,2) + pow(vy_k,2) ) - 1.0*ci*ki*ktr*timeIntegration,11,11);
	}
	if (sqrt( pow(vx_k,2) + pow(vy_k,2) ) < eps_jpp) {
	MatJacFx->setValueData(0.0f,11,12);
	} else {
	MatJacFx->setValueData(-(1.0*ki*ktr*timeIntegration*vx_k*vy_k)/sqrt( pow(vx_k,2) + pow(vy_k,2) ),11,12);
	}
	MatJacFx->setValueData(-58.227*g*ktr*timeIntegration*cos(58.227*Paux_k)*sin(Y_k),12,1);
	MatJacFx->setValueData(38.575*g*ktr*timeIntegration*cos(38.575*Raux_k)*cos(Y_k),12,2);
	MatJacFx->setValueData(-1.0*ktr*timeIntegration*(g*sin(38.575*Raux_k)*sin(Y_k) + g*sin(58.227*Paux_k)*cos(Y_k)),12,5);
	if (sqrt( pow(vx_k,2) + pow(vy_k,2) ) < eps_jpp) {
	MatJacFx->setValueData(0.0f,12,11);
	} else {
	MatJacFx->setValueData(-(1.0*ki*ktr*timeIntegration*vx_k*vy_k)/sqrt( pow(vx_k,2) + pow(vy_k,2) ),12,11);
	}
	if (sqrt( pow(vx_k,2) + pow(vy_k,2) ) < eps_jpp) {
	MatJacFx->setValueData(1.0 - 1.0*ki*ktr*timeIntegration*sqrt( pow(vx_k,2) + pow(vy_k,2) ) - 1.0*ci*ki*ktr*timeIntegration,12,12);
	} else {
	MatJacFx->setValueData(1.0 - 1.0*ki*ktr*timeIntegration*sqrt( pow(vx_k,2) + pow(vy_k,2) ) - (1.0*ki*ktr*timeIntegration*pow(vy_k,2))/sqrt( pow(vx_k,2) + pow(vy_k,2) ) - 1.0*ci*ki*ktr*timeIntegration,12,12);
	}


	// Filling in Process Jacobian, MatJacFu

	MatJacFu->setValueData(gain_pitch*timeIntegration,1,1);
	MatJacFu->setValueData(gain_roll*timeIntegration,2,2);
	MatJacFu->setValueData(gain_dyaw*timeIntegration,3,3);
	MatJacFu->setValueData(gain_dz*timeIntegration,6,4);

//    MatJacFx->mostrar();
//    MatJacFu->mostrar();
	return;
}


//User Jacobian of observation model
void LinkQuadModel1::jacobiansObservationModel(Matrix* MatJacHx, Vector* Statek)
{
	//  Medidas:
	//  01 pitch: C(1,1)*X1
	//  02 roll : C(2,2)*X2
	//  03 dYaw : C(3,4)*X4
	//  04 Yaw  : X5
	//  05 dZ   : C(5,7)*X7
	//  06 Z    : X8
	//  07 X    : X9					"VISION"
	//  08 Y    : X10					"VISION"
	//  09 Vx   : X11
	//  10 Vy   : X12
	//  11 [Vxm;: [R_Y R_Y]' * [X11;
	//  12 Vym] : [R_Y R_Y]     X12]
	//  13 Yaw  : X5					"VISION"

	//Hx, Ejemplo Jose Luis
	//MatJacHx->setValueData(1.0f,1,1);
	
	//MatJacHx->setZeros();

	// Reading statek values

	float Paux_k = Statek->getValueData(1);
	float Raux_k = Statek->getValueData(2);
	float dYaux_k = Statek->getValueData(3);
	float dYp_k = Statek->getValueData(4);
	float Y_k = Statek->getValueData(5);
	float dZaux_k = Statek->getValueData(6);
	float dZp_k = Statek->getValueData(7);
	float Z_k = Statek->getValueData(8);
	float x_k = Statek->getValueData(9);
	float y_k = Statek->getValueData(10);
	float vx_k = Statek->getValueData(11);
	float vy_k = Statek->getValueData(12);


	// Filling in Observation Jacobian

	MatJacHx->setValueData(58.227,1,1);
	MatJacHx->setValueData(38.575,2,2);
	MatJacHx->setValueData(67.213,3,4);
	MatJacHx->setValueData(1.0,4,5);
	MatJacHx->setValueData(-14.189,5,7);
	MatJacHx->setValueData(1.0,6,8);
	MatJacHx->setValueData(1.0,7,9);
	MatJacHx->setValueData(1.0,8,10);
	MatJacHx->setValueData(1.0,9,11);
	MatJacHx->setValueData(1.0,10,12);
	MatJacHx->setValueData(vy_k*cos(Y_k) - 1.0*vx_k*sin(Y_k),11,5);
	MatJacHx->setValueData(cos(Y_k),11,11);
	MatJacHx->setValueData(sin(Y_k),11,12);
	MatJacHx->setValueData(- 1.0*vx_k*cos(Y_k) - 1.0*vy_k*sin(Y_k),12,5);
	MatJacHx->setValueData(-1.0*sin(Y_k),12,11);
	MatJacHx->setValueData(cos(Y_k),12,12);
	MatJacHx->setValueData(1.0,13,5);

  //MatJacHx->mostrar();

	return;
}
