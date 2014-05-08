#ifndef COMPASS_H
#define COMPASS_H

#include <math.h>

struct vector{
	float x;
	float y;
	float z;
};

class Compass {
	private :	
		float _heading;
		vector _frameOrientation;
		
		float vectorDot(vector *a,vector* b);
		vector vectorCross(const vector *a,const vector *b);
		void vectorNormalize(vector *V);
		
	public:
		Compass(void);
		~Compass(void);
		
		void update(float ax, float ay, float az, float mx, float my, float mz);
		float getHeading(void);
};
#endif