
#ifndef _MODEL_H
#define _MODEL_H

#include "matrixLib.h"


class Model
{
public:
	void processModelCalculation(Vector* Statek1, Vector* Statek, Vector* Inputs);
	//Continuous
	virtual void processModelCalculation(Vector* Statek1, Vector* Statek, Vector* Inputs, double time)
	{
		return;
	}
	void observationModelCalculation(Vector* Output, Vector* State);
	void jacobiansProcessModelCalculation(Matrix* MatJacFx, Matrix* MatJacFu, Vector* State, Vector* Inputs);
	void jacobiansObservationModelCalculation(Matrix* MatJacHx, Vector* State);
	//Set Integration Times
	virtual void setMaxIntegrationTime(double maxTimeIntegrationIn)
	{return;}
	virtual void setTimeIntegration(double timeIntegrationIn)
	{return;}
	//Get max integration time
	virtual double getMaxIntegrationTime(void)
	{return 0.0;}

public:
	///// model user functions ////
	//Process model
	virtual void processModel(Vector* Statek1, Vector* Statek, Vector* Inputs)
	{
		return;
	}
	//Observation model
	virtual void observationModel(Vector* Output, Vector* State)
	{
		return;
	}
	//Jacobian process model
	virtual void jacobiansProcessModel(Matrix* MatJacFx, Matrix* MatJacFu, Vector* State, Vector* Inputs)
	{
		return;
	}
	//Jacobian process model
	virtual void jacobiansObservationModel(Matrix* MatJacHx, Vector* State)
	{
		return;
	}
};


//It has time management and integration time
class ContinuousModel : public Model
{
protected:
	//Max integration time
	double maxTimeIntegration;
	//Model integration times
	double timeIntegration;
public:
	//Set Integration Times
	void setMaxIntegrationTime(double maxTimeIntegrationIn);
	void setTimeIntegration(double timeIntegrationIn);
	//Get max integration time
	double getMaxIntegrationTime(void);
	//Calculations
	void processModelCalculation(Vector* Statek1, Vector* Statek, Vector* Inputs, double integrationInterval);
};


class DiscreteModel : public Model
{
	//Nothing new
};



#endif