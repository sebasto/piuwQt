#ifndef COMPASS_H
#define COMPASS_H

#include <iostream>
#include <math.h>

#define SMOOTHFACTOR 0.95

struct vector{
	float x;
	float y;
	float z;
};

class Compass {
	private :	
		float _heading;
		float _smoothHeading;
		vector _frameOrientation;
		
		float vectorDot(vector *a,vector* b);
		vector vectorCross(const vector *a,const vector *b);
		void vectorNormalize(vector *V);
		void updateHeading(float heading);
		
	public:
		Compass(void);
		~Compass(void);
		
		void update(float ax, float ay, float az, float mx, float my, float mz);
		float getHeading(void);
		float getSmoothHeading(void);
};
#endif