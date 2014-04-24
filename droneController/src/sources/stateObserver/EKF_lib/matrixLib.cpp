#include "stateObserver/EKF_lib/matrixLib.h"


Matrix::Matrix(void)
{
	creation(0,0);
}

Matrix::Matrix(int numFilasIn,int numColumnasIn)
{
	creation(numFilasIn,numColumnasIn);
}

Matrix::~Matrix(void)
{
//	std::cout << "Matrix::~Matrix(void)" << std::endl;
	deletion();
}


int Matrix::creation(int numFilasIn,int numColumnasIn)
{
	if(numFilasIn>0 && numColumnasIn>0)
	{
		data=new float[numFilasIn*numColumnasIn];
		numFilas=numFilasIn;
		numColumnas=numColumnasIn;
		setZeros();
	}
	else
	{
		data=0;
		numColumnas=0;
		numFilas=0;
	}
	return 1;
}


int Matrix::deletion(void)
{
	if(data!=0)
	{
		delete[] data;
	}
	creation(0,0);
	return 1;
	
}

void Matrix::mostrar(void)
{
	printf("[");
	for(int fila=1;fila<=Matrix::numFilas;fila++)
	{
		for(int columna=1;columna<=Matrix::numColumnas;columna++)
		{
			printf("%f",getValueData(fila,columna));
			if(columna<numColumnas)
				printf(" ");
			else if(fila<numFilas)
				printf(";\n ");
			else
				printf("]\n");
		}
	}
}


int Matrix::setValueData(float value,int numFilaIn,int numColumnaIn)
{
	if(numFilaIn<=numFilas && numColumnaIn<=numColumnas)
	{
		data[(numFilaIn-1)*(numColumnas)+(numColumnaIn-1)]=value;
		return 1;
	}
	else
	{
		return 0;
	}
}

float Matrix::getValueData(int numFilaIn,int numColumnaIn) const
{
	if(numFilaIn<=numFilas && numColumnaIn<=numColumnas)
	{
		return data[(numFilaIn-1)*(numColumnas)+(numColumnaIn-1)];
	}
	else
	{
		return 0;
	}
}


int Matrix::copy(const Matrix* matrixOriginal)
{
	//Comprobaciones

	//bucle
	for(int fila=1;fila<=Matrix::numFilas;fila++)
	{
		for(int columna=1;columna<=Matrix::numColumnas;columna++)
		{
			Matrix::setValueData(matrixOriginal->getValueData(fila,columna),fila,columna);
		}
	}
	//end
	return 0;
}


int Matrix::transpose(Matrix* matrixOriginal)
{
	//Previous checks
	//To do!!
	//printf("%dx%d\n",numFilas,numColumnas);
	//Transposition
	for(int i=1;i<=numFilas;i++)
	{
		for(int j=1;j<=numColumnas;j++)
		{
			setValueData(matrixOriginal->getValueData(j,i),i,j);
			//printf("%f\n",getValueData(i,j));
		}
	}
	//printf("end");
	//End
	return 1;
}


int Matrix::setInAllElements(float value)
{
	for(int i=1;i<=numFilas;i++)
	{
		for(int j=1;j<=numColumnas;j++)
		{
			Matrix::setValueData(value,i,j);
		}
	}
	//End
	return 1;
}


int Matrix::setZeros(void)
{
	Matrix::setInAllElements(0.0f);
	return 1;
}


int Matrix::setOnes(void)
{
	Matrix::setInAllElements(1.0f);
	return 1;
}


int Matrix::setDiagonalMatrix(Vector* vectorDiagonal)
{
	//Comprobaciones previas
	//To do
	//Creation
	for(int i=1;i<=numFilas;i++)
	{
		for(int j=1;j<=numColumnas;j++)
		{
			if(i==j)
				Matrix::setValueData(vectorDiagonal->getValueData(i),i,j);
			else
				Matrix::setValueData(0.0f,i,j);
		}
	}
	//end
	return 1;
}


int Matrix::elementsOperation(Matrix* matA, Matrix* matB,int operation)
{
	//Comprobaciones previas
	//To do!!
	//Addition
	for(int i=1;i<=numFilas;i++)
	{
		for(int j=1;j<=numColumnas;j++)
		{
			switch(operation)
			{
			case 1: //Add
			default:
				setValueData((matA->getValueData(i,j)+matB->getValueData(i,j)),i,j);
				break;
			case 2: //Substract
				setValueData((matA->getValueData(i,j)-matB->getValueData(i,j)),i,j);
				break;
			case 3: //Element multiplication
				setValueData((matA->getValueData(i,j)*matB->getValueData(i,j)),i,j);
				break;
			}
		}
	}
	//End
	return 1;
}

int Matrix::addition(Matrix* matA, Matrix* matB)
{
	elementsOperation(matA,matB,1);
	return 1;
}

int Matrix::substraction(Matrix* matA, Matrix* matB)
{
	elementsOperation(matA,matB,2);
	return 1;
}

int Matrix::elementMultiplication(Matrix* matA, Matrix* matB)
{
	elementsOperation(matA,matB,3);
	return 1;
}


int Matrix::multiplication(Matrix* matA, Matrix* matB)
{
	//Comprobaciones previas
	//To do!!
	//Multiplication
	float aux;
	for(int i=1;i<=matA->numFilas;i++)
	{
		for(int j=1;j<=matB->numColumnas;j++)
		{
			aux=0;
			for(int k=1;k<=matA->numColumnas;k++)
			{
				aux+=(matA->getValueData(i,k)*matB->getValueData(k,j));
			}
			setValueData(aux,i,j);
		}
	}
	//End
	return 1;
}


int Matrix::multiplication(Matrix* matA, Matrix* matB, Matrix* matC)
{
	//Comprobaciones previas
	//To do
	//Multiplicacion
	Matrix Aux(matA->numFilas,matB->numColumnas);
	Aux.multiplication(matA,matB);
	multiplication(&Aux,matC);
	Aux.deletion();
	//End
	return 1;
}



int Matrix::pseudoinverse(Matrix* matA)
{
	//Comprobaciones previas
	//To do
	//Pinv calulation
	CvMat* pinvAux=cvCreateMat(Matrix::numFilas,Matrix::numColumnas,CV_32FC1);
	CvMat* sourceAux=cvCreateMat(Matrix::numFilas,Matrix::numColumnas,CV_32FC1);
	for(int fila=1;fila<=numFilas;fila++)
	{
		for(int columna=1;columna<=numColumnas;columna++)
		{
			//printf("%f\n",sourceAux->data.fl[1]);
			sourceAux->data.fl[(fila-1)*numColumnas+columna-1]=matA->getValueData(fila,columna);
			//printf("%f\n",sourceAux->data.fl[(fila-1)*numColumnas+columna-1]);
		}
	}
    cvInvert(sourceAux,pinvAux,CV_SVD);
	for(int fila=1;fila<=numFilas;fila++)
	{
		for(int columna=1;columna<=numColumnas;columna++)
		{
			Matrix::setValueData(pinvAux->data.fl[(fila-1)*numColumnas+columna-1],fila,columna);
		}
	}

	//End
	return 1;
}





Vector::Vector(void)
{
	creation(0);
	return;
}

Vector::Vector(int numFilasIn)
{
	creation(numFilasIn);
	return;
}

Vector::~Vector(void)
{
	deletion();
	return;
}

int Vector::creation(int numFilasIn)
{
	Matrix::creation(numFilasIn,1);
	return 1;
}

int Vector::deletion(void)
{
	Matrix::deletion();
	return 1;
}

int Vector::setValueData(float value,int numFilaIn)
{
	Matrix::setValueData(value,numFilaIn,1);
	return 1;
}

float Vector::getValueData(int numFilaIn) const
{
	return Matrix::getValueData(numFilaIn,1);
}


int Vector::copy(const Vector* matrixOriginal)
{
	//Comprobaciones

	//bucle
	for(int fila=1;fila<=numFilas;fila++)
	{
		Vector::setValueData(matrixOriginal->getValueData(fila),fila);
	}
	//end
	return 0;
}

int Vector::length() {
	return numFilas;
}
