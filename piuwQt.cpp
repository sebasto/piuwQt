#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QKeyEvent>
#include "piuwQt.h"

//separate timer to update screen info
void MainScreen::updateScreen() {
	static float yaw = 0;
	static float temperature = 0;
	static float depth = 0;
	static float dist = 0;
	static int nbStations = 0;
	
	QString tmpString;
	
	if (yaw != _yaw) {
		yaw = _yaw;
		tmpString.setNum(yaw,'f',0);
		yawValue->setText(tmpString);
	}
	
	if (temperature != _temperature) {
		temperature = _temperature;
		tmpString.setNum(temperature,'f',1);
		tempValue->setText(tmpString);
	}
	
	if (depth != _depth) {
		depth = _depth;
		tmpString.setNum(depth,'f',1);
		depthValue->setText(tmpString);
	}
	
	if (dist != _dist) {
		dist = _dist;
		tmpString.setNum(dist,'f',1);
        distValue->setText(tmpString);
	}
	
	if (nbStations != _nbStations) {	
		nbStations = _nbStations;
		tmpString.setNum(nbStations);
        nbStationsValue->setText(tmpString);
	}
}

void MainScreen::updateIMU() {

#ifndef NOSENSOR	
	if (!_imu->IMURead()) {
		return;
	}
    RTIMU_DATA imuData = _imu->getIMUData();
	
	_yaw = imuData.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
	if (_yaw < 0) {
		_yaw += 360;
	}
#else
	_yaw = 12;
#endif
	
}

//update depth/T°
void MainScreen::updateDepthSensor() {
	
#ifndef NOSENSOR
	_profsensor.updateData();
	_temperature = _profsensor.getTemperature();
	_depth = _profsensor.getPressure();
#else
	_temperature = 12;
	_depth = 120;
#endif
}

void MainScreen::checkButtons() {
	string inputstate;
	QString tmpString;

	_button1->getval_gpio(inputstate);
	if (inputstate == "1") {
		_dist += 0.1;
	}
	
	_button2->getval_gpio(inputstate);
	if (inputstate == "1") {
		_dist -= 0.1;
	}
	
	_button3->getval_gpio(inputstate);
	if (inputstate == "1") {
		//_nbStations += 1;
	}
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
	_dist = 0;
	_nbStations = 0;
	//Initialize Buttons
	_button1 = new GPIOClass("22");
	_button1->export_gpio();
	_button1->setdir_gpio("in");
	
	_button2 = new GPIOClass("27");
	_button2->export_gpio();
	_button2->setdir_gpio("in");
	
	_button3 = new GPIOClass("18");
	_button3->export_gpio();
	_button3->setdir_gpio("in");
	
	
	//Launch button check timer
    buttonCheckTimer = new QTimer(this);
    connect(buttonCheckTimer, SIGNAL(timeout()), this, SLOT(checkButtons()));
    buttonCheckTimer->start(100);
	
#ifndef NOSENSOR
	//Initialize IMU 
	RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
	
	_imu = RTIMU::createIMU(settings);
	if ((_imu == NULL) || (_imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }
    _imu->IMUInit();
#endif
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
	
	//Launch screen refresh timer
    screenRefreshTimer = new QTimer(this);
    connect(screenRefreshTimer, SIGNAL(timeout()), this, SLOT(updateScreen()));
    screenRefreshTimer->start(100);
	
}

void MainScreen::keyPressEvent(QKeyEvent *event)
{
	QString tmpString;
	
    if(event->key() == Qt::Key_Plus)
    {
		_dist += 0.1;
		tmpString.setNum(_dist,'f',1);
        distValue->setText(tmpString);
    }
    if(event->key() == Qt::Key_Minus)
    {
		_dist -= 0.1;
		tmpString.setNum(_dist,'f',1);
        distValue->setText(tmpString);
    }
    if((event->key() == Qt::Key_Return) || (event->key() == Qt::Key_Enter))
    {
		_nbStations += 1;
		tmpString.setNum(_nbStations);
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
