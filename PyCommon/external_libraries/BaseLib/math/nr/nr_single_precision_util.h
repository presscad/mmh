#ifndef _NR_single_precision_UTIL_H_
#define _NR_single_precision_UTIL_H_

#include <string>
#include <cmath>
#include <complex>
#include <iostream>
using namespace std;

#include "nrutil_nr.h"

namespace NR_single_precision {
	inline void nrerror(const string error_text)
	// Numerical Recipes standard error handler
	{
		cerr << "Numerical Recipes run-time error..." << endl;
		cerr << error_text << endl;
		cerr << "...now exiting to system..." << endl;
		Msg::error(error_text.c_str());
	}
}

#endif