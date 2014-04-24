#include "stateObserver/EKF_lib/extendedKalmanFilter.h"


ExtendedKalmanFilter::ExtendedKalmanFilter(void)
{
	creation(0,0,0);
	return;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn)
{
	creation(numSystemStatesIn,numSystemInputsIn,numSystemMeasuresIn);
	return;
}

ExtendedKalmanFilter::~ExtendedKalmanFilter(void)
{
	deletion();
	return;
}

int ExtendedKalmanFilter::creation(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn)
{
	//Set Configuration variables
	numSystemStates=numSystemStatesIn;
	numSystemInputs=numSystemInputsIn;
	numSystemMeasures=numSystemMeasuresIn;

	//Alocate memory for the variables
	//Models
	matJacFx.creation(numSystemStates,numSystemStates);
	matJacFu.creation(numSystemStates,numSystemInputs);
	matJacHx.creation(numSystemMeasures,numSystemStates);
	matJacHxp.creation(numSystemMeasures,numSystemStates);
	//Inputs
	systemInputs.creation(numSystemInputs);
	matVarQ.creation(numSystemInputs,numSystemInputs);
	//State
	estimatedState.estimatedState_kk.creation(numSystemStates);
	estimatedState.estimatedState_k1k.creation(numSystemStates);
	estimatedState.estimatedState_k1k1.creation(numSystemStates);
	matVarP.matVarP_kk.creation(numSystemStates,numSystemStates);
	matVarP.matVarP_k1k.creation(numSystemStates,numSystemStates);
	matVarP.matVarP_k1k1.creation(numSystemStates,numSystemStates);
	matVarM.creation(numSystemStates,numSystemStates);
	//Measure-Observation
	systemMeasures.realMeasure.creation(numSystemMeasures);
	systemMeasures.estimatedMeasure.creation(numSystemMeasures);
	measuresEnabled.creation(numSystemMeasures);
	matMeasuresEnabled.creation(numSystemMeasures,numSystemMeasures);
	matVarR.creation(numSystemMeasures,numSystemMeasures);
	matVarRp.creation(numSystemMeasures,numSystemMeasures);
	//Measure-Innovation
	measureInnovation.creation(numSystemMeasures);
	matVarS.creation(numSystemMeasures,numSystemMeasures);
	//Kalman filter Gain
	matKFGain.creation(numSystemStates,numSystemMeasures);
	//Estimated Output
	estimatedOutput.creation(numSystemMeasures);
	estimatedOutputVariances.creation(numSystemMeasures,numSystemMeasures);

	//End
	return 1;
}

int ExtendedKalmanFilter::deletion(void)
{
	//Set Configuration variables
	numSystemStates=0;
	numSystemInputs=0;
	numSystemMeasures=0;

	//Delete memory
	//Models
	matJacFx.deletion();
	matJacFu.deletion();
	matJacHx.deletion();
	matJacHxp.deletion();
	//Inputs
	systemInputs.deletion();
	matVarQ.deletion();
	//State
	estimatedState.estimatedState_kk.deletion();
	estimatedState.estimatedState_k1k.deletion();
	estimatedState.estimatedState_k1k1.deletion();
	matVarP.matVarP_kk.deletion();
	matVarP.matVarP_k1k.deletion();
	matVarP.matVarP_k1k1.deletion();
	matVarM.deletion();
	//Measure-Observation
	systemMeasures.realMeasure.deletion();
	systemMeasures.estimatedMeasure.deletion();
	measuresEnabled.deletion();
	matMeasuresEnabled.deletion();
	matVarR.deletion();
	matVarRp.deletion();
	//Measure-Innovation
	measureInnovation.deletion();
	matVarS.deletion();
	//Kalman filter Gain
	matKFGain.deletion();
	//Estimated Output
	estimatedOutput.deletion();
	estimatedOutputVariances.deletion();

	//End
	return 1;
}

int ExtendedKalmanFilter::init(void)
{
	ExtendedKalmanFilter::measuresEnabled.setOnes();
	return 1;
}

int ExtendedKalmanFilter::init(Vector* initialState, Vector* varActuation, Vector* varObservation, Matrix* matPinit)
{
	ExtendedKalmanFilter::setInitialState(initialState);
	ExtendedKalmanFilter::setVarActuation(varActuation);
	ExtendedKalmanFilter::setVarObservation(varObservation);
	ExtendedKalmanFilter::setVarInitState(matPinit);
	return ExtendedKalmanFilter::init();
}

void ExtendedKalmanFilter::initModel(Model* userModel)
{
	systemModel=userModel;
	return;
}

void ExtendedKalmanFilter::setInitialState(Vector* initialState)
{
	ExtendedKalmanFilter::estimatedState.estimatedState_kk.copy(initialState);
	return;
}

void ExtendedKalmanFilter::setVarActuation(Vector* varActuation)
{
	ExtendedKalmanFilter::matVarQ.setDiagonalMatrix(varActuation);
	return;
}

void ExtendedKalmanFilter::setVarActuation(Matrix* varActuation)
{
	ExtendedKalmanFilter::matVarQ.copy(varActuation);
	return;
}

void ExtendedKalmanFilter::setVarObservation(Vector* varObservation)
{
	ExtendedKalmanFilter::matVarR.setDiagonalMatrix(varObservation);
	return;
}

void ExtendedKalmanFilter::setVarObservation(Matrix* varObservation)
{
	ExtendedKalmanFilter::matVarR.copy(varObservation);
	return;
}

void ExtendedKalmanFilter::setVarInitState(Vector* varInitState)
{
	ExtendedKalmanFilter::matVarP.matVarP_kk.setDiagonalMatrix(varInitState);
	return;
}

void ExtendedKalmanFilter::setVarInitState(Matrix* varInitState)
{
	ExtendedKalmanFilter::matVarP.matVarP_kk.copy(varInitState);
	return;
}

void ExtendedKalmanFilter::setVarState(Vector* varState)
{
	ExtendedKalmanFilter::matVarM.setDiagonalMatrix(varState);
	return;
}

void ExtendedKalmanFilter::setVarState(Matrix* varState)
{
	ExtendedKalmanFilter::matVarM.copy(varState);
	return;
}

int ExtendedKalmanFilter::setSystemInputs(Vector* inputsIn)
{
	ExtendedKalmanFilter::systemInputs.copy(inputsIn);
	return 1;
}

int ExtendedKalmanFilter::getSystemInputs(Vector* inputsOut)
{
	inputsOut->copy(&(systemInputs));
	return 1;
}

int ExtendedKalmanFilter::setSystemMeasures(Vector* measuresIn)
{
	ExtendedKalmanFilter::systemMeasures.realMeasure.copy(measuresIn);
	return 1;
}

int ExtendedKalmanFilter::getEstimatedState(Vector* estimatedStateOut)
{
	estimatedStateOut->copy(&(ExtendedKalmanFilter::estimatedState.estimatedState_k1k1));
	return 1;
}

int ExtendedKalmanFilter::getEstimatedOutput(Vector* estimatedOutputOut)
{
	systemModel->observationModel(&(estimatedOutput),&(estimatedState.estimatedState_k1k1));
	estimatedOutputOut->copy(&(ExtendedKalmanFilter::estimatedOutput));
	return 1;
}

int ExtendedKalmanFilter::getEstimatedStateVariances(Matrix* estimatedStateVariances)
{
	estimatedStateVariances->copy(&(matVarP.matVarP_k1k1));
	return 1;
}

int ExtendedKalmanFilter::getEstimatedOutputVariances(Matrix* estimatedOutputVariancesOut)
{
	Matrix MatHxTrans(numSystemStates,numSystemMeasures);
	MatHxTrans.transpose(&matJacHx);
	estimatedOutputVariances.multiplication(&matJacHx,&(matVarP.matVarP_k1k1),&MatHxTrans);
	estimatedOutputVariancesOut->copy(&estimatedOutputVariances);
	return 1;
}

int ExtendedKalmanFilter::activateSystemMeasure(int numInputActivate)
{
	if(numInputActivate<=numSystemMeasures)
	{
		measuresEnabled.setValueData(1.0,numInputActivate);
		return 1;
	}
	else
		return 0;
}

int ExtendedKalmanFilter::deactivateSystemMeasure(int numInputDeactivate)
{
	if(numInputDeactivate<=numSystemMeasures)
	{
		measuresEnabled.setValueData(0.0,numInputDeactivate);
		return 1;
	}
	else
		return 0;
}


int ExtendedKalmanFilter::evaluateProcessModel(void)
{
	//Process Model
	systemModel->processModel(&(estimatedState.estimatedState_k1k),&(estimatedState.estimatedState_kk),&(systemInputs));
	//End
	return 1;
}

int ExtendedKalmanFilter::evaluateJacobProcessModel(void)
{
	//Jacobians of the process Model
	systemModel->jacobiansProcessModel(&(matJacFx),&(matJacFu),&(estimatedState.estimatedState_kk),&(systemInputs));
	//End
	return 1;
}

int ExtendedKalmanFilter::evaluateObservationModel(void)
{
	//Observation model
	systemModel->observationModel(&(systemMeasures.estimatedMeasure),&(estimatedState.estimatedState_k1k));
	//End
	return 1;
}

int ExtendedKalmanFilter::evaluateJacobObservationModel(void)
{
	systemModel->jacobiansObservationModel(&(matJacHx),&(estimatedState.estimatedState_k1k));
	//End
	return 1;
}

int ExtendedKalmanFilter::statePrediction(void)
{
	//Get x(k+1|k)
	evaluateProcessModel();
	//Get Fx(k+1) & Fu(k+1)
	evaluateJacobProcessModel();
	//Update matrix P(k+1|k)
	Matrix matFxTrans(numSystemStates,numSystemStates);
	matFxTrans.transpose(&(ExtendedKalmanFilter::matJacFx));
	matVarP.matVarP_k1k.multiplication(&(ExtendedKalmanFilter::matJacFx),&(ExtendedKalmanFilter::matVarP.matVarP_kk),&matFxTrans);
	if(numSystemStates>0 && numSystemInputs>0)
	{
		Matrix matFuTrans(numSystemStates,numSystemInputs);
		matFuTrans.transpose(&(ExtendedKalmanFilter::matJacFu));
		Matrix matAuxProd(numSystemStates,numSystemStates);
		matAuxProd.multiplication(&(ExtendedKalmanFilter::matJacFu),&(ExtendedKalmanFilter::matVarQ),&matFuTrans);
		matVarP.matVarP_k1k.addition(&(matVarP.matVarP_k1k),&matAuxProd);
	}
	else if(numSystemStates>0 && numSystemInputs==0)
	{
		//Do nothing
	}
	else
	{
		return 0;
	}
	matVarP.matVarP_k1k.addition(&(matVarP.matVarP_k1k),&matVarM);
	//End
	return 1;
}

int ExtendedKalmanFilter::measurementsPrediction(void)
{
	//Get ye(k+1)
	evaluateObservationModel();
	//Get Hx(k+1)
	evaluateJacobObservationModel();
	return 1;
}


int ExtendedKalmanFilter::measuresMatching(void)
{
	//Get v(k+1)
	ExtendedKalmanFilter::measureInnovation.substraction(&(ExtendedKalmanFilter::systemMeasures.realMeasure),&(ExtendedKalmanFilter::systemMeasures.estimatedMeasure));
	ExtendedKalmanFilter::measureInnovation.elementMultiplication(&(ExtendedKalmanFilter::measuresEnabled),&(ExtendedKalmanFilter::measureInnovation));
	//Get S(k+1)
	//Aux
	ExtendedKalmanFilter::matMeasuresEnabled.setDiagonalMatrix(&(ExtendedKalmanFilter::measuresEnabled));
	ExtendedKalmanFilter::matJacHxp.multiplication(&(ExtendedKalmanFilter::matMeasuresEnabled),&(ExtendedKalmanFilter::matJacHx));
	ExtendedKalmanFilter::matVarRp.multiplication(&(ExtendedKalmanFilter::matMeasuresEnabled),&(ExtendedKalmanFilter::matVarR));
	//Calculo
	Matrix HxpTrans(numSystemStates,numSystemMeasures);
	HxpTrans.transpose(&matJacHxp);
	ExtendedKalmanFilter::matVarS.multiplication(&(matJacHxp),&(matVarP.matVarP_k1k),&(HxpTrans));
	ExtendedKalmanFilter::matVarS.addition(&(matVarS),&(matVarRp));
	//Fin
	return 1;
}


int ExtendedKalmanFilter::stateCorrection(float mahalanobisDistance)
{
	Matrix distance(1,1);
	Matrix innovTrans(1,numSystemMeasures);
	innovTrans.transpose(&measureInnovation);
	distance.multiplication(&innovTrans,&matVarS,&measureInnovation);
	if(distance.getValueData(1,1)<=mahalanobisDistance || mahalanobisDistance<0.0)
	{
		//W(k+1)
		Matrix pinvS(numSystemMeasures,numSystemMeasures);
		pinvS.pseudoinverse(&matVarS);
		Matrix HxpTrans(numSystemStates,numSystemMeasures);
		HxpTrans.transpose(&matJacHxp);
		matKFGain.multiplication(&(matVarP.matVarP_k1k),&HxpTrans,&pinvS);
		estimatedState.estimatedState_k1k1.multiplication(&matKFGain,&measureInnovation);
		estimatedState.estimatedState_k1k1.addition(&(estimatedState.estimatedState_k1k),&(estimatedState.estimatedState_k1k1));
		//P(k+1|k+1)
		matVarP.matVarP_k1k1.multiplication(&(matKFGain),&matJacHxp,&(matVarP.matVarP_k1k));
		matVarP.matVarP_k1k1.substraction(&(matVarP.matVarP_k1k),&(matVarP.matVarP_k1k1));
		//End
		return 1;
	}
	else
	{
		//Do not update
		return 0;
	}
}


int ExtendedKalmanFilter::updateDataState(void)
{
	ExtendedKalmanFilter::estimatedState.estimatedState_kk.copy(&(ExtendedKalmanFilter::estimatedState.estimatedState_k1k1));
	ExtendedKalmanFilter::matVarP.matVarP_kk.copy(&(ExtendedKalmanFilter::matVarP.matVarP_k1k1));
	return 1;
}

int ExtendedKalmanFilter::stateEstimation(float mahalanobisDistance)
{
	int stateCorrected;
	statePrediction();
	measurementsPrediction();
	measuresMatching();
	stateCorrected=stateCorrection(mahalanobisDistance);
	updateDataState();
	return stateCorrected;
}


int ExtendedKalmanFilter::stateEstimation(Vector* mahalanobisDistanceSensors)
{
	//aux
	int stateCorrected;
	int someMeasureDeactivated=0;
	Vector measuresEnabledPast(numSystemMeasures);
	measuresEnabledPast.copy(&measuresEnabled);

	//Corremos todo
	statePrediction();
	measurementsPrediction();
	measuresMatching();
	//Comprobamos si la medida de los sensores es buena
	for(int i=1;i<=numSystemMeasures;i++)
	{
		if(measureInnovation.getValueData(i)*matVarS.getValueData(i,i)*measureInnovation.getValueData(i)>=mahalanobisDistanceSensors->getValueData(i) && mahalanobisDistanceSensors->getValueData(i)>=0.0)
		{
			//Deshabilitamos las entradas malas
			deactivateSystemMeasure(i);
			someMeasureDeactivated++;
		}
	}
	//Volvemos a correr el matching
	if(someMeasureDeactivated)
	{
		//measurementsPrediction();
		measuresMatching();
	}
	//Estimamos lo que nos falta
	stateCorrected=stateCorrection(-1.0);
	updateDataState();
	//Deshacemos
	measuresEnabled.copy(&measuresEnabledPast);
	//Acabamos
	if(stateCorrected && someMeasureDeactivated)
		return 2;
	else
		return stateCorrected;
}



//////// Continuous EKF ///////
ContinuousExtendedKalmanFilter::ContinuousExtendedKalmanFilter(void)
{
	creation(0,0,0);
	initTime=0.0;
	actualTime=0.0;
	return;
}
ContinuousExtendedKalmanFilter::ContinuousExtendedKalmanFilter(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn)
{
	creation(numSystemStatesIn,numSystemInputsIn,numSystemMeasuresIn);
	initTime=0.0;
	actualTime=0.0;
	return;
}
ContinuousExtendedKalmanFilter::~ContinuousExtendedKalmanFilter(void)
{
	deletion();
	return;
}

void ContinuousExtendedKalmanFilter::initModel(ContinuousModel* userModel)
{
	systemModel=userModel;
	return;
}
//Set Init time
void ContinuousExtendedKalmanFilter::setInitTime(double initTimeIn)
{
	initTime=initTimeIn;
	actualTime=initTime;
	return;
}

//Set actual time
void ContinuousExtendedKalmanFilter::setActualTime(double actualTimeIn)
{
	initTime=actualTime;
	actualTime=actualTimeIn;
	return;
}

int ContinuousExtendedKalmanFilter::stateEstimation(float mahalanobisDistance)
{
	int stateCorrected;
	//We save the user measures enabled
	Vector measuresEnabledUser(numSystemMeasures);
	measuresEnabledUser.copy(&measuresEnabled);
	//We put in blind mode the EKF, but we save the user measures
	for(int i=1;i<=numSystemMeasures;i++)
	{
		deactivateSystemMeasure(i);
	}
	//We calculate the number of steps and the value of the time integration
	int numIntegrationSteps=1;
	double integrationInterval=actualTime-initTime;
	double maxTimeIntegration=systemModel->getMaxIntegrationTime();
	double timeIntegration;
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
	systemModel->setTimeIntegration(timeIntegration);
	for(int i=1;i<numIntegrationSteps;i++)
	{
		statePrediction();
		measurementsPrediction();
		measuresMatching();
		stateCorrected=stateCorrection(-1.0);
		updateDataState();
	}
	measuresEnabled.copy(&measuresEnabledUser);
	//No blind mode
	statePrediction();
	measurementsPrediction();
	measuresMatching();
	stateCorrected=stateCorrection(mahalanobisDistance);
	updateDataState();
	//End
	return stateCorrected;
}


int ContinuousExtendedKalmanFilter::stateEstimation(Vector* mahalanobisDistanceSensors)
{
	int stateCorrected;
	//We save the user measures enabled
	Vector measuresEnabledUser(numSystemMeasures);
	measuresEnabledUser.copy(&measuresEnabled);
	//We put in blind mode the EKF, but we save the user measures
	for(int i=1;i<=numSystemMeasures;i++)
	{
		deactivateSystemMeasure(i);
	}
	//We calculate the number of steps and the value of the time integration
	int numIntegrationSteps=1;
	double integrationInterval=actualTime-initTime;
	double maxTimeIntegration=systemModel->getMaxIntegrationTime();
	double timeIntegration;
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
	systemModel->setTimeIntegration(timeIntegration);
	for(int i=1;i<numIntegrationSteps;i++)
	{
		statePrediction();
		measurementsPrediction();
		measuresMatching();
		stateCorrected=stateCorrection(-1.0);
		updateDataState();
	}
	measuresEnabled.copy(&measuresEnabledUser);
	//No blind mode
	int someMeasureDeactivated=0;
	//Vector measuresEnabledPast(numSystemMeasures);
	//measuresEnabledPast.copy(&measuresEnabled);
	//Corremos todo
	statePrediction();
	measurementsPrediction();
	measuresMatching();
	//Comprobamos si la medida de los sensores es buena
	for(int i=1;i<=numSystemMeasures;i++)
	{
		if(measureInnovation.getValueData(i)*matVarS.getValueData(i,i)*measureInnovation.getValueData(i)>=mahalanobisDistanceSensors->getValueData(i) && mahalanobisDistanceSensors->getValueData(i)>=0.0)
		{
			//Deshabilitamos las entradas malas
			deactivateSystemMeasure(i);
			someMeasureDeactivated++;
		}
	}
	//Volvemos a correr el matching
	if(someMeasureDeactivated)
	{
		//measurementsPrediction();
		measuresMatching();
	}
	//Estimamos lo que nos falta
	stateCorrected=stateCorrection(-1.0);
	updateDataState();
	//Deshacemos
	measuresEnabled.copy(&measuresEnabledUser);
	//Acabamos
	if(stateCorrected && someMeasureDeactivated)
		return 2;
	else
		return stateCorrected;
}


//////////// Discrete EKF /////////////
DiscreteExtendedKalmanFilter::DiscreteExtendedKalmanFilter(void)
{
	creation(0,0,0);
	return;
}
DiscreteExtendedKalmanFilter::DiscreteExtendedKalmanFilter(int numSystemStatesIn,int numSystemInputsIn,int numSystemMeasuresIn)
{
	creation(numSystemStatesIn,numSystemInputsIn,numSystemMeasuresIn);
	return;
}
DiscreteExtendedKalmanFilter::~DiscreteExtendedKalmanFilter(void)
{
	deletion();
	return;
}

void DiscreteExtendedKalmanFilter::initModel(DiscreteModel* userModel)
{
	systemModel=userModel;
	return;
}
