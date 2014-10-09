#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QKeyEvent>
#include <QFile>
#include <QTextStream>
#include "MS5803_14BA.h"
#include "RTIMULib.h"
#include "GPIOClass.h"

class openXmlTopoFile{
	public : 
		openXmlTopoFile();
		void write(float distance, float cap, float profondeur);
		
	private :
		QFile _openXmlFile;
};

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
		QLabel *batteryLabel;
		QLabel *batteryValue;
		
		QTimer *buttonCheckTimer;
		QTimer *imuTimer;
		QTimer *depthSensorTimer;
		QTimer *clockTimer;
		QTimer *batteryStatusTimer;
		QTimer *buttonsTimer;
		QTimer *screenRefreshTimer;
		
		///////////////////////////
		///////////////////////////
		QTimer *timer;
		int timerId;
#ifndef NOSENSOR
		MS5803_14BA _profsensor;
		RTIMU *_imu;
#endif
		float _yaw;
		float _depth;
		float _temperature;
		float _dist;
		int _nbStations;
		openXmlTopoFile _topoFile;
		int _batteryStatus;
		int _batteryCharging;
		
		GPIOClass* _button1;
		GPIOClass* _button2;
		GPIOClass* _button3;
		
		
	private slots:
		void updateIMU();	
		void updateDepthSensor();	
		void updateClock();
		void updateBatteryStatus();
		void updateButtons();
		void updateScreen();
};
