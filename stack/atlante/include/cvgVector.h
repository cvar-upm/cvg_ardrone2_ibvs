/**
 * Description: cvgVector class.
 * Author: Ignacio Mellado (CVG)
 * Last revision date: 24/11/2010
 */

#ifndef CVG_VECTOR_INCLUDED__
#define CVG_VECTOR_INCLUDED__

#include <vector>
#include "cvg_types.h"
#include "cvgString.h"

template<class _T> class cvgVector : public std::vector<_T> {
};

class cvgString;
typedef cvgVector<cvgString>	cvgStringVector;

#endif	/* CVG_VECTOR_INCLUDED__ */
