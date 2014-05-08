#include "Compass.h"

Compass::Compass() 
{
	_heading = 0;
	_frameOrientation.x = 1;
	_frameOrientation.y = 0;
	_frameOrientation.z = 0;
}

Compass::~Compass(void)
{
}

vector Compass::vectorCross(const vector *a,const vector *b)
{
	vector tmpVector;
	tmpVector.x = (a->y * b->z) - (a->z * b->y);
	tmpVector.y = (a->z * b->x) - (a->x * b->z);
	tmpVector.z = (a->x * b->y) - (a->y * b->x);
	
	return tmpVector;
}

float Compass::vectorDot(vector *a,vector* b)
{
	return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void Compass::vectorNormalize(vector *v)
{
	float mag = sqrt(vectorDot(v, v));
	v->x /= mag;
	v->y /= mag;
	v->z /= mag;
}

void Compass::update(float ax, float ay, float az, float mx, float my, float mz)
{
	vector temp_m;
	temp_m.x = mx;
	temp_m.y = my;
	temp_m.z = mz;
	vector temp_a;
	temp_a.x = ax;
	temp_a.y = ay;
	temp_a.z = az;
	
	
    // compute E and N
    vector E;
    vector N;
    E = vectorCross(&temp_m, &temp_a);
    vectorNormalize(&E);
    N = vectorCross(&temp_a, &E);
    vectorNormalize(&N);

    // compute heading
    _heading = atan2(vectorDot(&E, &_frameOrientation), vectorDot(&N, &_frameOrientation)) * 180 / M_PI;
    if (_heading < 0) _heading += 360;
}

float Compass::getHeading(void)
{
	return _heading;
}