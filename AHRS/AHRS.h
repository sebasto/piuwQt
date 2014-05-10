#ifndef AHRS_H
#define AHRS_H

#define BETA 0.1
#define EPSILON		0.0001f
#define PI		3.14159265359f
struct Vector{
	float x;
	float y;
	float z;
};

class MadgwickAHRS {
	private :	
		float _SamplePeriod;
		float _Beta; //algorithm gain beta (for Madgwick algorithm)
		float _Quaternion[4];
		float _roll,_pitch,_yaw;
		
		float integralFBx,integralFBy,integralFBz; //needed by FreeIMU

		public:
		MadgwickAHRS(void);
		~MadgwickAHRS(void);
		
		void getYawPitchRoll(float* yaw, float* pitch, float* roll);
		void printYawPitchRoll(void);
		void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod);
		void Update(float gx, float gy, float gz, float ax, float ay, float az,float samplePeriod);
		void AHRSupdateFreeIMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod);
		void rotateV(Vector *v,float* delta);
		void AHRSupdateMultiWiiIMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float samplePeriod);
		void PrintQuat();
		float invSqrt(float number);
};
#endif