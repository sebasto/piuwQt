#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include "MS5803_14BA.h"
#include "mpu9150.h"

class MainScreen : public QWidget
{
	public:
		MainScreen(QWidget *parent = 0);
		void timerEvent(QTimerEvent *event);
		
	private :
		QHBoxLayout *hbox; //main layout
		QVBoxLayout *leftCol; //left col layout
		QLabel *yawLabel;
		QLabel *yawValue;
		QLabel *compassLabel;
		QLabel *compassValue;
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
		
		QTimer *timer;
		int timerId;
		MS5803_14BA profsensor;
		MPU9150AHRS mpu;
};

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
	// COMPASS YAW
	compassLabel = new QLabel("Compass Yaw :", this);
	leftCol->addWidget(compassLabel,1, Qt::AlignCenter);
	compassValue = new QLabel("0", this);
	leftCol->addWidget(compassValue,1, Qt::AlignCenter);
	
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
 
    timerId = startTimer(10);
}

void MainScreen::timerEvent(QTimerEvent *event){
	QString tmpString;
	float yaw,pitch,roll;
	
	//update time
	QTime qtime = QTime::currentTime();
	QString stime = qtime.toString(Qt::TextDate);
	timeValue->setText(stime);
	//update depth/T°
	profsensor.updateData();
	tmpString.setNum(profsensor.getTemperature(),'f',1);
	tempValue->setText(tmpString);
	tmpString.setNum(profsensor.getPressure(),'f',1);
	depthValue->setText(tmpString);
	
	mpu.updateData();
	mpu.getYawPitchRoll(&yaw,&pitch,&roll);
	tmpString.setNum(yaw,'f',1);
	yawValue->setText(tmpString);
	
	tmpString.setNum(mpu.getCompassHeading(),'f',1);
	compassValue->setText(tmpString);
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
