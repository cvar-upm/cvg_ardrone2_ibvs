#include "stateObserver/EKF_lib/model.h"



void Model::processModelCalculation(Vector* Statek1, Vector* Statek, Vector* Inputs)
{
	Model::processModel(Statek1,Statek,Inputs);
}

void Model::observationModelCalculation(Vector* Output, Vector* State)
{
	Model::observationModel(Output,State);
}

void Model::jacobiansProcessModelCalculation(Matrix* MatJacFx, Matrix* MatJacFu, Vector* State, Vector* Inputs)
{
	Model::jacobiansProcessModel(MatJacFx,MatJacFu,State,Inputs);
}

void Model::jacobiansObservationModelCalculation(Matrix* MatJacHx, Vector* State)
{
	Model::jacobiansObservationModel(MatJacHx,State);
}





void ContinuousModel::setMaxIntegrationTime(double maxTimeIntegrationIn)
{
	maxTimeIntegration=maxTimeIntegrationIn;
	return;
}

void ContinuousModel::setTimeIntegration(double timeIntegrationIn)
{
	timeIntegration=timeIntegrationIn;
	return;
}

double ContinuousModel::getMaxIntegrationTime(void)
{
	return maxTimeIntegration;
}

void ContinuousModel::processModelCalculation(Vector* Statek1, Vector* Statek, Vector* Inputs, double integrationInterval)
{
	int numIntegrationSteps=1;

	if(integrationInterval>maxTimeIntegration)
	{
		numIntegrationSteps=floor(integrationInterval/maxTimeIntegration);
		timeIntegration=integrationInterval/numIntegrationSteps;
	}
	else
	{
		numIntegrationSteps=1;
		timeIntegration=integrationInterval;
	}

//	printf("numsteps=%d, intInt=%f, maxTime=%f\n",numIntegrationSteps,integrationInterval,maxTimeIntegration);

	for(int i=1;i<=numIntegrationSteps;i++)
	{
		//Model::processModel(Statek1,Statek,Inputs);
		processModel(Statek1,Statek,Inputs);
		if(i<=numIntegrationSteps)
		{
			Statek->copy(Statek1);
			//printf("step %d",i);
			//Statek1->mostrar();
		}
	}
	return;
}
