//
//  TimeMapper.h
//  kresse
//
//  Created by Hansi on 01.05.20.
//

#ifndef TimeMapper_h
#define TimeMapper_h

#define FLT_EPSILON 0.0001

// was ofMap() from openframeworks https://openframeworks.cc/documentation/math/ofMath/
static inline float scale_lin(float value, float inputMin, float inputMax, float outputMin, float outputMax, bool clamp = false) {
	if (fabsf(inputMin - inputMax) < FLT_EPSILON){
		return outputMin;
	} else {
		float outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);
		
		if( clamp ){
			if(outputMax < outputMin){
				if( outVal < outputMax )outVal = outputMax;
				else if( outVal > outputMin )outVal = outputMin;
			}else{
				if( outVal > outputMax )outVal = outputMax;
				else if( outVal < outputMin )outVal = outputMin;
			}
		}
		return outVal;
	}
}


struct TimeMapper{
	double t, startTime, endTime;
	
	TimeMapper() : t(-1), startTime(0), endTime(1){}
	TimeMapper( double t, double startTime, double endTime ) : t(t), startTime(startTime), endTime(endTime){}
	
	bool active(){
		return t >= startTime && t <= endTime;
	}
	
	double easeIn( double from, double to ){
		double t = scale_lin(this->t, startTime, endTime, 0, 1, true);
		return (to-from)*t*t + from;
	}
	
	double easeOut( double from, double to ){
		double t = scale_lin(this->t, startTime, endTime, 0, 1, true);
		return -(to-from)*t*(t-2) + from;
	}
	
	double easeInOut( double from, double to ){
		double t = scale_lin(this->t, startTime, endTime, 0, 2, true);
		if (t < 1) return (to-from)/2*t*t + from;
		t--;
		return -(to-from)/2 * (t*(t-2) - 1) + from;
	}
	
	double cubicSpline( double a, double b, double c, double d ){
		double t = scale_lin(this->t, startTime, endTime, 0, 1, true);
		return a + b*t + c*t*t + d*t*t*t;
	}
	
	double linear( double from, double to ){
		double t = scale_lin(this->t, startTime, endTime, 0, 1, true );
		return from + (to-from)*t;
	}
	
	TimeMapper fromStart( double duration ){
		return TimeMapper(t,startTime,startTime+duration);
	}
	
	TimeMapper fromEnd( double duration ){
		return TimeMapper(t,endTime-duration,endTime);
	}
	
};


#endif /* TimeMapper_h */
