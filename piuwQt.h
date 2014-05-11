#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QKeyEvent>
#include "MS5803_14BA.h"
#include "mpu9150.h"
#include "RTIMULib.h"

class MainScreen : public QWidget
{
Q_OBJECT

	public:
		MainScreen(QWidget *parent = 0);
		
	private :
		void keyPressEvent(QKeyEvent *event);
		
		QHBoxLayout *hbox; //main layout
		QVBoxLayout *leftCol; //left col layout
		QLabel *yawLabel;
		QLabel *yawValue;
		QLabel *depthLabel;
		QLabel *depthValue;
		QLabel *tempLabel;
		QLabel *tempValue;
		QVBoxLayout *rightCol; // right col layout
		QLabel *nbStationsLabel;
		QLabel *nbStationsValue;
		QLabel *distLabel;
		QLabel *distValue;
		QLabel *timeLabel;
		QLabel *timeValue;
		
		QTimer *imuTimer;
		QTimer *depthSensorTimer;
		QTimer *clockTimer;
		
		///////////////////////////
		///////////////////////////
		QTimer *timer;
		int timerId;
		MS5803_14BA profsensor;
		//MPU9150AHRS mpu;
		RTIMU *_imu;
		float dist;
		int nbStations;
		
	private slots:
		void updateIMU();	
		void updateDepthSensor();	
		void updateClock();
};
