/************************************
* Extended Kalman Filter Class
*
*
* Author:					Jose Luis Sanchez Lopez
* Property:				CVG-CAR-UPM
*
*
* Version:					1.5
*
* Creation Date:			22/02/2012
* Last Modification Date:	23/06/2012
*
****************************************/

/************************************
* Modified by:				Jesus Pestana Puerta
* Property:				CVG-CAR-UPM
*
* added: int length(); // to class Vector
* added deep-copy to Matrix class (instead of shallow copy)
* added deep-copy to Vector class (instead of shallow copy)
*
****************************************/

#ifndef _MATRIX_LIB_H
#define _MATRIX_LIB_H


#include <math.h>
#include <stdio.h>

//#include <opencv/cv.h>		// changed to make it work with ubuntu 12.04 opencv software center installaion
//#include <opencv/cxcore.h>	// changed to make it work with ubuntu 12.04 opencv software center installaion
//#include <opencv2/opencv.hpp>
//#include "highgui.h"

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O


class Vector;

class Matrix
{
	protected:
		int numFilas;
		int numColumnas;
		float* data;

	public:
		Matrix(void);
		Matrix(int numFilasIn,int numColumnasIn);
		~Matrix(void);
		int creation(int numFilasIn,int numColumnasIn);
		int deletion(void);

		void mostrar(void);

		int setValueData(float value,int numFilaIn,int numColumnaIn);
		float getValueData(int numFilaIn,int numColumnaIn) const;

		int copy(const Matrix* matrixOriginal);

		int transpose(Matrix* matrixOriginal);

		int setInAllElements(float value);
		int setZeros(void);
		int setOnes(void);
		int setDiagonalMatrix(Vector* vectorDiagonal);

	protected:
		int elementsOperation(Matrix* matA, Matrix* matB, int operation);
	public:
		int addition(Matrix* matA, Matrix* matB);
		int substraction(Matrix* matA, Matrix* matB);
		int elementMultiplication(Matrix* matA, Matrix* matB);
		int multiplication(Matrix* matA, Matrix* matB);
		int multiplication(Matrix* matA, Matrix* matB, Matrix* matC);

		int pseudoinverse(Matrix* matA);

		/* ***** Funciones aniadidas por Jesus ***** */
		// Copy-constructor
		Matrix( const Matrix& other) : numFilas(other.numFilas), numColumnas(other.numColumnas),
				data( ((numFilas>0)&&(numColumnas>0)) ? new float[numFilas*numColumnas] : 0 ) {
			if ( data != 0) {
				copy(&other);
			} else { // data == 0
				creation(0,0);
			}
		}

		// Assignment operator
		Matrix& operator=(Matrix other) { // other is copy-constructed from the rhs term
			swap((*this), other);

			return (*this);
			// other is destroyed using the destructor (which is called at the end of this function)
		}

		friend void swap(Matrix& first, Matrix& second) { // nothrow

			// Alternativa 1: usar los swap nativos de std::swap (o algo así)
			// enable ADL (not necessary in our case, but good practice)
			 using std::swap;
			 swap( first.numFilas, second.numFilas);
			 swap( first.numColumnas, second.numColumnas);
			 swap( first.data, second.data);

			// Alternativa 2: hacer el swap manualmente
			// swap operation
//			int auxFilas, auxColumnas;
//			float* auxData;
//			auxFilas    = first.numFilas;
//			auxColumnas = first.numColumnas;
//			auxData		= first.data;
//
//			first.numFilas    = second.numFilas;
//			first.numColumnas = second.numColumnas;
//			first.data        = second.data;
//
//			second.numFilas    = auxFilas;
//			second.numColumnas = auxColumnas;
//			second.data        = auxData;
		}

		inline float *getPData() { return data; } // This function should only be used for debugging purposes
		/* END: ***** Funciones aniadidas por Jesus ***** */

};

class Vector : public Matrix
{
	private:

	public:
		Vector(void);
		Vector(int numFilasIn);
		~Vector(void);
		int creation(int numFilasIn);
		int deletion(void);

		int setValueData(float value,int numFilaIn);
		float getValueData(int numFilaIn) const;

		int length();

		int copy(const Vector* matrixOriginal);

		/* ***** Funciones aniadidas por Jesus ***** */
		// Copy-constructor
		Vector( const Vector& other) : Matrix(other) /* copy constructor of parent class*/ {}

		// Assignment operator
		Vector& operator=(Vector other) {
			Matrix::operator=(other); /* assignment operator constructor of parent class*/

			return (*this);
			// other is destroyed using the destructor (which is called at the end of this function)
		}
		/* END: ***** Funciones aniadidas por Jesus ***** */
};






#endif
