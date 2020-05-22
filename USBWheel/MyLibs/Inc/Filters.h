
#ifndef FILTERS_H
#define FILTERS_H

#include "cppmain.h"
#include "math.h"

enum ORDER {OD1, OD2, OD3, OD4};
enum TYPE {LOWPASS, HIGHPASS};

#define MAX_ORDER (uint8_t)5
#define EPSILON (float)0.00001
#define WEPSILON (float)0.00010
#define KM (float)100
#ifndef PI
	#define PI                      (float)3.14159265359 //consider using library defined value of PI
#endif
#define SQRT2 (float)1.41421356237
#define SQRT3 (float)1.73205080757
#define SQRT5 (float)2.2360679775

class Filters {
public:
	Filters(float hz_, float ts_, ORDER od_, TYPE ty_ = TYPE::LOWPASS);
	Filters();
	~Filters();

	float filterIn(float input);

	void flush();
	void init(uint8_t doFlush=true);

	void setSamplingTime(float ts_, uint8_t doFlush=true) { ts = ts_; init(doFlush); }
	void setCutoffFreqHZ(float hz_, uint8_t doFlush=true) { hz = hz_; init(doFlush); }
	void setOrder(ORDER od_, uint8_t doFlush=true)          { od = od_; init(doFlush); }

	uint8_t isInErrorState() { return f_err;  }
	uint8_t isInWarnState()  { return f_warn; }

private:
	float ts;
	float hz;
	ORDER od;
	TYPE  ty;

	// Helper variables during coefficient calcutions
	float a, b, c, d, e;
	// Filter coefficients
	float b0, b1, b2, b3, b4, a0, a1, a2;
	// Difference equation terms
	float k0, k1, k2, k3, k4, k5;
	float j0, j1, j2;
	// Filter buffer
	float y[MAX_ORDER], u[MAX_ORDER];

	uint8_t f_err, f_warn; ///< Numerical error or warning; only relevant for 8-bit micros

	float ap(float p); ///< Assert Parameter

	inline float computeLowPass(float input);
	inline float computeHighPass(float input);

	/** \brief Computes the discrete coefficients for a Butterworth low-pass filter via pole-zero matching.
	 *  Up to order 4.
	 */
	inline void  initLowPass();

	/** \brief Computes the discrete coefficients for a Butterworth high-pass filter via Bilinear-Transformation.
	 *  Up to order 2 (I was lazy).
	 */
	inline void  initHighPass();

};

#endif /* EXTIHANDLER_H_ */
