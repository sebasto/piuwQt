#include "mpu9150.h"

extern "C" {
#include "linux_glue.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
}

#include <iostream>
#include <stdio.h>

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from InvenSense's MPL.
 */

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*

       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

int MPU9150AHRS::load_cal(void)
{
	int i;
	FILE *f;
	char buff[32];
	long val[6];

	f = fopen("./magcal.txt", "r");

	if (!f) {
		std::cout << "./magcal.txt not found\n";
		//TODO : handle this case with default values
		return -1;
	}
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	_magOffset[0] = (val[0] + val[1]) / 2;
	_magOffset[1] = (val[2] + val[3]) / 2;
	_magOffset[2] = (val[4] + val[5]) / 2;

	_magRange[0] = val[1] - _magOffset[0];
	_magRange[1] = val[3] - _magOffset[1];
	_magRange[2] = val[5] - _magOffset[2];
	
	
	f = fopen("./accelcal.txt", "r");

	if (!f) {
		std::cout << "accelcal.txt not found\n";
		//TODO : handle this case with default values
		return -1;
	}
	
	for (i = 0; i < 6; i++) {
		if (!fgets(buff, 20, f)) {
			printf("Not enough lines in calibration file\n");
			break;
		}

		val[i] = atoi(buff);

		if (val[i] == 0) {
			printf("Invalid cal value: %s\n", buff);
			break;
		}
	}

	fclose(f);

	if (i != 6) 
		return -1;

	_accOffset[0] = (val[0] + val[1]) / 2;
	_accOffset[1] = (val[2] + val[3]) / 2;
	_accOffset[2] = (val[4] + val[5]) / 2;
	
	return 0;
}

MPU9150AHRS::MPU9150AHRS() {
#if DEBUG
	std::cout << "Init MPU9150AHRS\n";
#endif

	_ahrs = new(MadgwickAHRS);
	_compass = new(Compass);
	
	_lastMeasureTimestamp = 0;
	
	_gyroOffset[0] = -8.7;
	_gyroOffset[1] = -22.7;
	_gyroOffset[2] = -4.7;
		
	if (mpu_init(NULL)) {
		std::cout << "\nmpu_init() failed\n";
	}
	
	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) {
		std::cout << "\nmpu_set_sensors() failed\n";
	}
		
	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		std::cout << ("\nmpu_configure_fifo() failed\n");
	}
	
	if (mpu_set_sample_rate(MPUSAMPLERATE)) {
		std::cout << ("\nmpu_set_sample_rate() failed\n");
	}

	if (mpu_set_compass_sample_rate(MAGSAMPLERATE)) {
		std::cout << ("\nmpu_set_compass_sample_rate() failed\n");
	}
	
	if (dmp_load_motion_driver_firmware()) {
		std::cout << ("\ndmp_load_motion_driver_firmware() failed\n");
	}

	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
		std::cout << ("\ndmp_set_orientation() failed\n");
	}

  	if (dmp_enable_feature(DMP_FEATURE_SEND_RAW_ACCEL 
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
		std::cout << ("\ndmp_enable_feature() failed\n");
	}
 
	if (dmp_set_fifo_rate(MPUSAMPLERATE)) {
		std::cout << ("\ndmp_set_fifo_rate() failed\n");
	}

	if (mpu_set_dmp_state(1)) {
		std::cout << ("\nmpu_set_dmp_state(1) failed\n");
	}

	if (LOW_PASS_FILTER != 0)
		mpu_set_lpf(LOW_PASS_FILTER);                                                // set the low pass filter
  
	mpu_get_gyro_sens(&_gyroSens); // get gyro sensibility (in deg/s)
	_gyroSens = _gyroSens * (180 / PI); // convert to rad/s
	mpu_get_accel_sens(&_accSens);
	if (load_cal() != 0) { //load calibration files for acc and mag
		std::cout << ("\nCan't read calibration files\n");
		exit(1);
	}
    initGyroOffsets();
	
	//set_cal(0, null); //lit le fichier de configuration par défaut pour 
	//set_cal(1, null);
	
	std::cout << (" done\n\n");
#if DEBUG
	std::cout << "mpu9150 initialized\n";
#endif
}

MPU9150AHRS::~MPU9150AHRS() {
	delete (_ahrs);
	delete (_compass);
	
	// turn off the DMP on exit 
	if (mpu_set_dmp_state(0))
		printf("Turn off mpu9150 DMP failed\n");

	// TODO: Should turn off the sensors too
}

// this function is intended to find the gyro offsets
void MPU9150AHRS::initGyroOffsets(void){
	int nbSamples = 1000;
	int i,j;
	
	
	std::cout << "Initialising Gyros (" << nbSamples << " samples) \n";
	for (i = 0; i < 3; i++){
		_gyroOffset[i] = 0;
	}
	
	for (j = 0; j < nbSamples; j++){
		delay_ms(1000/MPUSAMPLERATE);
		if (mpu_get_gyro_reg(_rawGyro, NULL)) {
			std::cout << "\nmpu_get_gyro_reg() failed\n";
		}
		
		for (i = 0; i < 3; i++){
			_gyroOffset[i] += _rawGyro[i];
		}
	}
	
	
	for (i = 0; i < 3; i++){
		_gyroOffset[i] = _gyroOffset[i] / (float) nbSamples;
		std::cout << "gyroOffset[" << i <<"]=" << _gyroOffset[i] << "\n";
	}
}

void MPU9150AHRS::updateData(){
	unsigned long currentTimestamp;
	int i;
	
	/*
	//Try to use FIFO, but with no success so far ...
    short sensors;
    unsigned char more;
	long * mpuquat; //value not used but needed to call dmp_read_fifo
	
	
	 if ((result = dmp_read_fifo(__rawGyro, __rawAcc, mpuquat, &currentTimestamp, &sensors, &more)) != 0) {
      std::cout << "Error reading fifo" << "\n";
    } 
	*/	
	
#if DEBUG
	std::cout << "mpu_get_gyro_reg() \n" ;
#endif
	if (mpu_get_gyro_reg(_rawGyro, NULL)) {
		std::cout << "\nmpu_get_gyro_reg() failed\n";
	}
	
	for (i = 0; i < 3; i++) {
		_Gyro[i] = ((float)_rawGyro[i] - _gyroOffset[i]) / _gyroSens;
		if (_Gyro[i] < 0.01) {
			_Gyro[i] = 0;
		}
	}
#if DEBUG
	std::cout << "Gyro X " << _rawGyro[0]-_gyroOffset[0] << " Y " << _rawGyro[1]-_gyroOffset[1] << " Z " << _rawGyro[2]-_gyroOffset[2] << "\n";
	
	std::cout << "mpu_get_accel_reg() \n" ;
#endif

	if (mpu_get_accel_reg(_rawAcc, NULL)){
		std::cout << "\nmpu_get_accel_reg() failed\n";
	}
	
	for (i = 0; i < 3; i++) {
		_Acc[i] = (float) (_rawAcc[i] - _accOffset[i]) / (float) _accSens;
	}
	
#if DEBUG
	std::cout << "mpu_get_compass_reg() \n" ;
#endif
	if (mpu_get_compass_reg(_rawMag, NULL)){
		std::cout << "\nmpu_get_compass_reg() failed\n";
	}
	//the magnetometer does not have the same orientaion as accel and gyro on the MPU9150, here we realign magnetometer
	// (see http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/IMU/PS-MPU-9150A.pdf chapter 7)
	/*
	for (i = 0; i < 3; i++) {
		_Mag[i] = (float) (_rawMag[i] - _magOffset[i]) / (float) _magRange[i]; //calibrated mag (arbitrary unit)
	}
	*/
	_Mag[0] = (float) (_rawMag[1] - _magOffset[1]) / (float) _magRange[1];
	_Mag[1] = (float) (_rawMag[0] - _magOffset[0]) / (float) _magRange[0];
	_Mag[2] =  -(float) (_rawMag[2] - _magOffset[2]) / (float) _magRange[2];

#if DEBUG
	std::cout << "mpu_get_temperature() \n" ;
#endif
	if (mpu_get_temperature(&_rawTemp, NULL)){
		std::cout << "\nmpu_get_temperature() failed\n";
	}
	
	if (_lastMeasureTimestamp == 0) {
		get_ms(&_lastMeasureTimestamp);
	}
	get_ms(&currentTimestamp);
	
#if DEBUG
	std::cout << "_ahrs->Update("<<_Gyro[0]<<","<<_Gyro[1]<<","<<_Gyro[2]<<","<<_Acc[0]<<","<<_Acc[1]<<","<<_Acc[2]<<","<<_Mag[0]<<","<<_Mag[1]<<","<<_Mag[2]<<","<<(currentTimestamp - _lastMeasureTimestamp) / 1000.0f <<"); \n" ;
#endif

	_compass->update(_Acc[0],_Acc[1],_Acc[2],_Mag[0],_Mag[1],_Mag[2]);
	//_ahrs->AHRSupdateFreeIMU(_Gyro[0],_Gyro[1],_Gyro[2],_Acc[0],_Acc[1],_Acc[2],_Mag[0],_Mag[1],_Mag[2],(currentTimestamp - _lastMeasureTimestamp) / 1000.0f);
	//_ahrs->Update(_Gyro[0],_Gyro[1],_Gyro[2],_Acc[0],_Acc[1],_Acc[2],_Mag[0],_Mag[1],_Mag[2],(currentTimestamp - _lastMeasureTimestamp) / 1000.0f);
	_ahrs->AHRSupdateMultiWiiIMU(_Gyro[0],_Gyro[1],_Gyro[2],_Acc[0],_Acc[1],_Acc[2],_Mag[0],_Mag[1],_Mag[2],(currentTimestamp - _lastMeasureTimestamp) / 1000.0f);
	_lastMeasureTimestamp = currentTimestamp;
	
	//printRawData();
	//_ahrs->printYawPitchRoll();
}

void MPU9150AHRS::printRawData(){
	std::cout << "Raw Gyro X " << _rawGyro[0] << " Y " << _rawGyro[1] << " Z " << _rawGyro[2] << "\n";
	std::cout << "Raw Acc  X " << _rawAcc[0] << " Y " << _rawAcc[1] << " Z " << _rawAcc[2] <<"\n";
	std::cout << "Raw Mag X " << _rawMag[0] << " Y " << _rawMag[1] << " Z " << _rawMag[2] << "\n";
	std::cout << "Raw Temp : " << _rawTemp << "\n";
}

void MPU9150AHRS::printData(){
	std::cout << "Gyro X " << _Gyro[0] << " Y " << _Gyro[1] << " Z " << _Gyro[2] << "\n";
	std::cout << "Acc  X " << _Acc[0] << " Y " << _Acc[1] << " Z " << _Acc[2] <<"\n";
	std::cout << "Mag X " << _Mag[0] << " Y " << _Mag[1] << " Z " << _Mag[2] << "\n";
	std::cout << "Raw Temp : " << _rawTemp << "\n";
}

void MPU9150AHRS::printQuat(){
	std::cout << _lastMeasureTimestamp << "\n";
	_ahrs->PrintQuat();
}

void MPU9150AHRS::getYawPitchRoll(float * yaw,float * pitch, float * roll) {
	_ahrs->getYawPitchRoll(yaw,pitch,roll);
}

float MPU9150AHRS::getCompassHeading(void) {
	return _compass->getSmoothHeading();
}