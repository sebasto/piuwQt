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

#include "piuwQt.h"

void MainScreen::updateIMU() {
	float yaw,pitch,roll;
	QString tmpString;
	
	if (!_imu->IMURead()) {
		return;
	}
    RTIMU_DATA imuData = _imu->getIMUData();
	
	yaw = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
	if (yaw < 0) {
		yaw += 360;
	}
	tmpString.setNum(yaw,'f',0);
	yawValue->setText(tmpString);
}

//update depth/T°
void MainScreen::updateDepthSensor() {
	QString tmpString;
		
	profsensor.updateData();
	tmpString.setNum(profsensor.getTemperature(),'f',1);
	tempValue->setText(tmpString);
	tmpString.setNum(profsensor.getPressure(),'f',1);
	depthValue->setText(tmpString);
}


MainScreen::MainScreen(QWidget *parent)
    : QWidget(parent)
{
	//create parent horizontal box
	hbox = new QHBoxLayout(this);
	hbox->setSpacing(1);

	//create left col
	leftCol = new QVBoxLayout();
	leftCol->setSpacing(1);
	hbox->addLayout(leftCol);

	//// LEFT COL LABELS ////
	// AHRS YAW
	yawLabel = new QLabel("AHRS Yaw :", this);
	leftCol->addWidget(yawLabel,1, Qt::AlignCenter);
	yawValue = new QLabel("0", this);
	leftCol->addWidget(yawValue,1, Qt::AlignCenter);
	
	//DEPTH
	depthLabel = new QLabel("Depth :", this);
	leftCol->addWidget(depthLabel,1, Qt::AlignCenter);
	depthValue = new QLabel("0", this);
	leftCol->addWidget(depthValue,1, Qt::AlignCenter);
	
	//TEMPERATURE
	tempLabel = new QLabel("T° :", this);
	leftCol->addWidget(tempLabel,1, Qt::AlignCenter);
	tempValue = new QLabel("0", this);
	leftCol->addWidget(tempValue,1, Qt::AlignCenter);
	
	//// RIGHT COL ////
	QVBoxLayout *rightCol = new QVBoxLayout();
	rightCol->setSpacing(1);
	hbox->addLayout(rightCol);

	//NBSTATIONS
	nbStationsLabel = new QLabel("Nb Stations :", this);
	rightCol->addWidget(nbStationsLabel,1, Qt::AlignCenter);
	nbStationsValue = new QLabel("0", this);
	rightCol->addWidget(nbStationsValue,1, Qt::AlignCenter);
	
	//DIST
	distLabel = new QLabel("Dist :", this);
	rightCol->addWidget(distLabel,1, Qt::AlignCenter);
	distValue = new QLabel("0", this);
	rightCol->addWidget(distValue,1, Qt::AlignCenter);
	
	//TIME
	timeLabel = new QLabel("Time :", this);
	rightCol->addWidget(timeLabel,1, Qt::AlignCenter);
	QTime qtime = QTime::currentTime();
	QString stime = qtime.toString(Qt::TextDate);
	timeValue = new QLabel(stime, this);
	rightCol->addWidget(timeValue,1, Qt::AlignCenter);
	
	////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////
	dist = 0;
	nbStations = 0;
	
	//Initialize IMU 
	RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
    _imu = RTIMU::createIMU(settings);
	if ((_imu == NULL) || (_imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }
    _imu->IMUInit();
	
	//Launch IMU refresh timer
    imuTimer = new QTimer(this);
    connect(imuTimer, SIGNAL(timeout()), this, SLOT(updateIMU()));
    imuTimer->start(10);
	
	//Launch Depth Sensor refresh timer
    depthSensorTimer = new QTimer(this);
    connect(depthSensorTimer, SIGNAL(timeout()), this, SLOT(updateDepthSensor()));
    depthSensorTimer->start(100);
	
	//Launch clock refresh timer
    clockTimer = new QTimer(this);
    connect(clockTimer, SIGNAL(timeout()), this, SLOT(updateClock()));
    clockTimer->start(1000);
}

void MainScreen::keyPressEvent(QKeyEvent *event)
{
	QString tmpString;
	
    if(event->key() == Qt::Key_Plus)
    {
		dist += 0.1;
		tmpString.setNum(dist,'f',1);
        distValue->setText(tmpString);
    }
    if(event->key() == Qt::Key_Minus)
    {
		dist -= 0.1;
		tmpString.setNum(dist,'f',1);
        distValue->setText(tmpString);
    }
    if((event->key() == Qt::Key_Return) || (event->key() == Qt::Key_Enter))
    {
		nbStations += 1;
		tmpString.setNum(nbStations);
        nbStationsValue->setText(tmpString);
    }
}

void MainScreen::updateClock(void){
	QString tmpString;
	
	//update clock
	QTime qtime = QTime::currentTime();
	QString stime = qtime.toString(Qt::TextDate);
	timeValue->setText(stime);
	
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
	
    MainScreen window;

    window.resize(250, 150);
    window.setWindowTitle("Pi under water");
    window.show();

    return app.exec();
}
