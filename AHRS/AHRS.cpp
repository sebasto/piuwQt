#include "AHRS.h"
#include <math.h>
#include <iostream>

MadgwickAHRS::MadgwickAHRS()
{
	_Beta = BETA;
	_Quaternion[0] = 1.0f; 
	_Quaternion[1] = 0.0f;
	_Quaternion[2] = 0.0f;
	_Quaternion[3] = 0.0f;
	
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
}

MadgwickAHRS::~MadgwickAHRS()
{
}

void MadgwickAHRS::PrintQuat(void) {
	std::cout << "q0 : " << _Quaternion[0] << " q1 : " << _Quaternion[1] << " q2 : " << _Quaternion[2] << " q3 : " << _Quaternion[3] <<"\n";
}

void MadgwickAHRS::printYawPitchRoll(void)
{
	float yaw,pitch,roll;
	
	getYawPitchRoll(&yaw,&pitch,&roll );
	std::cout << " yaw : " << yaw << " pitch : " << pitch << " roll : " << roll << "\n" ; 
}

void MadgwickAHRS::getYawPitchRoll(float* yaw, float* pitch, float* roll )
{
	*yaw = _yaw;
	*pitch = _pitch;
	*roll = _roll;
	return;

  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (_Quaternion[1]*_Quaternion[3] - _Quaternion[0]*_Quaternion[2]);
  gy = 2 * (_Quaternion[0]*_Quaternion[1] + _Quaternion[2]*_Quaternion[3]);
  gz = _Quaternion[0]*_Quaternion[0] - _Quaternion[1]*_Quaternion[1] - _Quaternion[2]*_Quaternion[2] + _Quaternion[3]*_Quaternion[3];
  
  *yaw = atan2(2 * _Quaternion[1] * _Quaternion[2] - 2 * _Quaternion[0] * _Quaternion[3], 2 * _Quaternion[0]*_Quaternion[0] + 2 * _Quaternion[1] * _Quaternion[1] - 1) * 180/M_PI;
  *pitch = atan(gx / sqrt(gy*gy + gz*gz))  * 180/PI;
  *roll = atan(gy / sqrt(gx*gx + gz*gz))  * 180/PI;
}

/// <param name="gx">
/// Gyroscope x axis measurement in radians/s.
/// </param>
/// <param name="gy">
/// Gyroscope y axis measurement in radians/s.
/// </param>
/// <param name="gz">
/// Gyroscope z axis measurement in radians/s.
/// </param>
/// <param name="ax">
/// Accelerometer x axis measurement in any calibrated units.
/// </param>
/// <param name="ay">
/// Accelerometer y axis measurement in any calibrated units.
/// </param>
/// <param name="az">
/// Accelerometer z axis measurement in any calibrated units.
/// </param>
/// <param name="mx">
/// Magnetometer x axis measurement in any calibrated units.
/// </param>
/// <param name="my">
/// Magnetometer y axis measurement in any calibrated units.
/// </param>
/// <param name="mz">
/// Magnetometer z axis measurement in any calibrated units.
/// </param>
/// <param name="samplePeriod">
/// Time spent between this measure and the last one (unit = seconds)
/// </param>
/// <remarks>

void MadgwickAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod)
{
	float q1 = _Quaternion[0], q2 = _Quaternion[1], q3 = _Quaternion[2], q4 = _Quaternion[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _2q1q3 = 2.0 * q1 * q3;
	float _2q3q4 = 2.0 * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = (float)sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = (float)sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0 * q1 * mx;
	_2q1my = 2.0 * q1 * my;
	_2q1mz = 2.0 * q1 * mz;
	_2q2mx = 2.0 * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = (float)sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0 * _2bx;
	_4bz = 2.0 * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	norm = 1.0 / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of _Quaternion
	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - _Beta * s1;
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - _Beta * s2;
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - _Beta * s3;
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - _Beta * s4;

	// Integrate to yield _Quaternion
	q1 += qDot1 * samplePeriod;
	q2 += qDot2 * samplePeriod;
	q3 += qDot3 * samplePeriod;
	q4 += qDot4 * samplePeriod;
	norm = 1.0 / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise _Quaternion
	
	_Quaternion[0] = q1 * norm;
	_Quaternion[1] = q2 * norm;
	_Quaternion[2] = q3 * norm;
	_Quaternion[3] = q4 * norm;
}

void MadgwickAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az,float samplePeriod)
{
	float q1 = _Quaternion[0], q2 = _Quaternion[1], q3 = _Quaternion[2], q4 = _Quaternion[3];   // short name local variable for readability
	float norm;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1 = 2.0 * q1;
	float _2q2 = 2.0 * q2;
	float _2q3 = 2.0 * q3;
	float _2q4 = 2.0 * q4;
	float _4q1 = 4.0 * q1;
	float _4q2 = 4.0 * q2;
	float _4q3 = 4.0 * q3;
	float _8q2 = 8.0 * q2;
	float _8q3 = 8.0 * q3;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = (float)sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0) return; // handle NaN
	norm = 1 / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Gradient decent algorithm corrective step
	s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
	s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
	s3 = 4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
	s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay;
	norm = 1.0 / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of _Quaternion
	qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - _Beta * s1;
	qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - _Beta * s2;
	qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - _Beta * s3;
	qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - _Beta * s4;

	// Integrate to yield _Quaternion
	q1 += qDot1 * samplePeriod;
	q2 += qDot2 * samplePeriod;
	q3 += qDot3 * samplePeriod;
	q4 += qDot4 * samplePeriod;
	norm = 1.0 / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise _Quaternion
	_Quaternion[0] = q1 * norm;
	_Quaternion[1] = q2 * norm;
	_Quaternion[2] = q3 * norm;
	_Quaternion[3] = q4 * norm;
}

void MadgwickAHRS::AHRSupdateFreeIMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod) 
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	float qa, qb, qc;
	float q0 = _Quaternion[0];
	float q1 = _Quaternion[1];
	float q2 = _Quaternion[2];
	float q3 = _Quaternion[3];
	
	float twoKp = (2.0f * 0.5f); // 2 * proportional gain
	float twoKi = (2.0f * 0.1f); // 2 * integral gain
	//twoKi = 0; // 2 * integral gain
	//twoKp = 0;

	
	// Auxiliary variables to avoid repeated arithmetic
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

	// Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
	if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
		float hx, hy, bx, bz;
		float halfwx, halfwy, halfwz;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of magnetic field
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (my * halfwz - mz * halfwy);
		halfey = (mz * halfwx - mx * halfwz);
		halfez = (mx * halfwy - my * halfwx);
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
		float halfvx, halfvy, halfvz;

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex += (ay * halfvz - az * halfvy);
		halfey += (az * halfvx - ax * halfvz);
		halfez += (ax * halfvy - ay * halfvx);
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			//std::cout << "integralFBx" << integralFBx << " integralFBy " << integralFBy << " integralFBz " << integralFBz << " gx " << gx << " gy " << gy << " gz " << gz << "\n";
			integralFBx += twoKi * halfex * samplePeriod;  // integral error scaled by Ki
			integralFBy += twoKi * halfey * samplePeriod;
			integralFBz += twoKi * halfez * samplePeriod;
			
			//std::cout << " towki " << twoKi << " halfex " << halfex << " halfey " << halfey << " halfez " << halfez << "\n";
			//std::cout << integralFBx << " " << integralFBy << " " << integralFBz << " " << gx << " " << gy << " " << gz << "\n";
			gx += integralFBx;  // apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f; // prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * samplePeriod);   // pre-multiply common factors
	gy *= (0.5f * samplePeriod);
	gz *= (0.5f * samplePeriod);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	_Quaternion[0] = q0;
	_Quaternion[1] = q1;
	_Quaternion[2] = q2;
	_Quaternion[3] = q3;
}

/**
 * Fast inverse square root implementation. Compatible both for 32 and 8 bit microcontrollers (taken from FreeIMU project)
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float MadgwickAHRS::invSqrt(float number) {
/*
  union {
    float f;
    unsigned long int i;
  } y;

  y.f = number;
  y.i = 0x5f375a86 - (y.i >> 1);
  y.f = y.f * ( 1.5f - ( number * 0.5f * y.f * y.f ) );
  return y.f;
  */
  return 1/sqrt(number);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void MadgwickAHRS::rotateV(Vector *v,float* delta) {
  Vector v_tmp = *v;
  v->z -= delta[0]  * v_tmp.x + delta[1] * v_tmp.y;
  v->x += delta[1]  * v_tmp.z - delta[2]   * v_tmp.y;
  v->y += delta[1] * v_tmp.z + delta[2]   * v_tmp.x;
}

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

void MadgwickAHRS::AHRSupdateMultiWiiIMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod) 
{
	float accMag = 0;
	float deltaGyroAngle[3];

	static Vector EstG,EstM;
	
	std::cout << gx <<" "<<  gy  <<" "<<  gz<<" " << ax<<" "<< ay<<" "<<az<<" "<< mx<<" "<< my<<" "<< mz<<" "<<samplePeriod<<"\n";
	
	// Initialization
	deltaGyroAngle[0] = gx * samplePeriod;
	deltaGyroAngle[1] = gy * samplePeriod;
	deltaGyroAngle[2] = gz * samplePeriod;

	accMag = sqrt(ax * ax + ay * ay + az * az);

	rotateV(&EstG,deltaGyroAngle);
	rotateV(&EstM,deltaGyroAngle);

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
	if (  0.85 < accMag && accMag < 1.15 ) {
		EstG.x = (EstG.x * GYR_CMPF_FACTOR + ax) * INV_GYR_CMPF_FACTOR;
		EstG.y = (EstG.y * GYR_CMPF_FACTOR + ay) * INV_GYR_CMPF_FACTOR;
		EstG.z = (EstG.z * GYR_CMPF_FACTOR + az) * INV_GYR_CMPF_FACTOR;
	}
	
	EstM.x = (EstM.x * GYR_CMPFM_FACTOR + mx) * INV_GYR_CMPFM_FACTOR;
	EstM.y = (EstM.y * GYR_CMPFM_FACTOR + my) * INV_GYR_CMPFM_FACTOR;
	EstM.z = (EstM.z * GYR_CMPFM_FACTOR + mz) * INV_GYR_CMPFM_FACTOR;
	
	//std::cout << "EstG : " << EstG.x << " " << EstG.y << " " << EstG.z << "\n";
	//std::cout << "Acc " << ax << " " << ay << " " << az << "\n";
	//std::cout << "EstM : " << EstM.x << " " << EstM.y << " " << EstM.z << "\n";
	//std::cout << "Mag : " << mx << " " << my << " " << mz << "\n";
	  
	// Attitude of the estimated vector
	float sqGX = EstG.x * EstG.x;
	float sqGY = EstG.y * EstG.y;
	float sqGZ = EstG.z * EstG.z;
	float sqGX_sqGZ = sqGX + sqGZ;
	float invmagXZ  = 1/sqrt(sqGX_sqGZ);
	float invG = 1/sqrt(sqGX_sqGZ + sqGY);
	_roll  = atan2(EstG.x , EstG.z) * 180/M_PI;
	_pitch = atan2(EstG.y , invmagXZ*sqGX_sqGZ) * 180/M_PI;

    _yaw = atan2(EstM.z * EstG.x - EstM.x * EstG.z , EstM.y * invG * sqGX_sqGZ  - (EstM.x * EstG.x + EstM.z * EstG.z) * invG * EstG.y ) * 180/M_PI; 
	std::cout << "roll pitch yaw " << _roll << " " << _pitch << " " << _yaw << "\n";
}
