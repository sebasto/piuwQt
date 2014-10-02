#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>
#include <QTime>
#include <QTimer>
#include <QKeyEvent>
#include "piuwQt.h"

openXmlTopoFile::openXmlTopoFile() {
	QString tmpString;
	QString str;
	
	int counter = 0;
	int flag = 0;
	
	// open XML file for writing
	while (flag ==0) {
		tmpString = "topo_";
		str.setNum(counter);
		tmpString += str;
		tmpString += ".xml";
		_openXmlFile.setFileName(tmpString);
		
		if (_openXmlFile.exists()) {
			counter++;
		}
		else {
			if (!_openXmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
				printf("An error occured while trying to open Xml file for writing");
				exit(-1);
			}
			flag = 1;
		}
	  }
}

void openXmlTopoFile::write(float distance, float cap, float profondeur) {
    static int countMesure = 1;
	QTextStream openXmlFileTextStream(&_openXmlFile);
	
    if (countMesure == 1) {
		openXmlFileTextStream << "<?xml version=\"1.0\" standalone=\"yes\"?>" << endl;
		openXmlFileTextStream << "<DataSetOAS xmlns=\"https://www.arianesline.com/XML/DataSetOAS.xsd\">" << endl;
		openXmlFileTextStream << "<xs:schema id=\"DataSetOAS \" targetNamespace=  \"https://www.arianesline.com/XML/DataSetOAS.xsd\" xmlns:mstns=\"https://www.arianesline.com/XML/DataSetOAS.xsd\" xmlns=\"https://www.arianesline.com/XML/DataSetOAS.xsd\" xmlns:xs=\"http://www.w3.org/2001/XMLSchema\" xmlns:msdata=\"urn:schemas-microsoft-com:xml-msdata\" attributeFormDefault=\"qualified\" elementFormDefault=\"qualified\">" << endl;
		openXmlFileTextStream << "    <xs:element name=\"DataSetOAS\" msdata:IsDataSet=\"true\" msdata:UseCurrentLocale=\"true\">" << endl;
		openXmlFileTextStream << "      <xs:complexType>" << endl;
		openXmlFileTextStream << "        <xs:choice minOccurs=\"0\" maxOccurs=\"unbounded\">" << endl;
		openXmlFileTextStream << "          <xs:element name=\"SurveyData\">" << endl;
		openXmlFileTextStream << "            <xs:complexType>" << endl;
		openXmlFileTextStream << "              <xs:sequence>" << endl;
		openXmlFileTextStream << "                <xs:element name=\"ID\" msdata:AutoIncrementSeed=\"-1\" msdata:AutoIncrementStep=\"-1\" type=\"xs:int\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Name\" type=\"xs:string\" default=\"&quot;&quot;\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"FromID\" type=\"xs:int\" default=\"-1\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"ClosureToID\" type=\"xs:int\" default=\"-1\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Section\" type=\"xs:string\" default=\"&quot;Main Section&quot;\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Type\" type=\"xs:string\" default=\"&quot;Real&quot;\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Explorer\" type=\"xs:string\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Date\" type=\"xs:dateTime\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Color\" type=\"xs:string\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Length\" type=\"xs:double\" default=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Azimut\" type=\"xs:double\" default=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Inclination\" type=\"xs:double\" default=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Depth\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Left\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Right\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Down\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Up\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"DepthIn\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Longitude\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Latitude\" type=\"xs:double\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Comment\" type=\"xs:string\" minOccurs=\"0\" />" << endl;
		openXmlFileTextStream << "              </xs:sequence>" << endl;
		openXmlFileTextStream << "            </xs:complexType>" << endl;
		openXmlFileTextStream << "          </xs:element>" << endl;
		openXmlFileTextStream << "          <xs:element name=\"Infos\">" << endl;
		openXmlFileTextStream << "            <xs:complexType>" << endl;
		openXmlFileTextStream << "              <xs:sequence>" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Key\" type=\"xs:string\" />" << endl;
		openXmlFileTextStream << "                <xs:element name=\"Value\" type=\"xs:string\" default=\"&quot;&quot;\" />" << endl;
		openXmlFileTextStream << "              </xs:sequence>" << endl;
		openXmlFileTextStream << "            </xs:complexType>" << endl;
		openXmlFileTextStream << "          </xs:element>" << endl;
		openXmlFileTextStream << "        </xs:choice>" << endl;
		openXmlFileTextStream << "      </xs:complexType>" << endl;
		openXmlFileTextStream << "      <xs:unique name=\"Constraint1\">" << endl;
		openXmlFileTextStream << "        <xs:selector xpath=\".//mstns:SurveyData\" />" << endl;
		openXmlFileTextStream << "        <xs:field xpath=\"mstns:ID\" />" << endl;
		openXmlFileTextStream << "      </xs:unique>" << endl;
		openXmlFileTextStream << "      <xs:unique name=\"Infos_Constraint1\" msdata:ConstraintName=\"Constraint1\" msdata:PrimaryKey=\"true\">" << endl;
		openXmlFileTextStream << "        <xs:selector xpath=\".//mstns:Infos\" />" << endl;
		openXmlFileTextStream << "        <xs:field xpath=\"mstns:Key\" />" << endl;
		openXmlFileTextStream << "      </xs:unique>" << endl;
		openXmlFileTextStream << "    </xs:element>" << endl;
		openXmlFileTextStream << "  </xs:schema>" << endl;
		openXmlFileTextStream << "    <Infos>" << endl;
		openXmlFileTextStream << "      <Key>Project Name</Key>" << endl;
		openXmlFileTextStream << "      <Value>ProjectName</Value>" << endl;
		openXmlFileTextStream << "    </Infos>" << endl;
		openXmlFileTextStream << "    <Infos>" << endl;
		openXmlFileTextStream << "      <Key>Unit</Key>" << endl;
		openXmlFileTextStream << "      <Value>m</Value>" << endl;
		openXmlFileTextStream << "    </Infos>" << endl;
		openXmlFileTextStream << "      <SurveyData>" << endl;
		openXmlFileTextStream << "        <ID>0</ID>" << endl;
		openXmlFileTextStream << "        <Name>FS00</Name>" << endl;
		openXmlFileTextStream << "        <FromID>-1</FromID>" << endl;
		openXmlFileTextStream << "        <ClosureToID>-1</ClosureToID>" << endl;
		openXmlFileTextStream << "        <Section>FirstSection</Section>" << endl;
		openXmlFileTextStream << "        <Type>Start</Type>" << endl;
		openXmlFileTextStream << "        <Explorer>Ardutopo</Explorer>" << endl;
		openXmlFileTextStream << "        <Date>2013-09-21T23:18:41.0381791+02:00</Date>" << endl;
		openXmlFileTextStream << "        <Color>Red</Color>" << endl;
		openXmlFileTextStream << "        <Length>0</Length>" << endl;
		openXmlFileTextStream << "        <Azimut>0</Azimut>" << endl;
		openXmlFileTextStream << "        <Inclination>0</Inclination>" << endl;
		openXmlFileTextStream << "        <Depth>0</Depth>" << endl;
		openXmlFileTextStream << "        <Left>0.1</Left>" << endl;
		openXmlFileTextStream << "        <Right>0.1</Right>" << endl;
		openXmlFileTextStream << "        <Down>0.1</Down>" << endl;
		openXmlFileTextStream << "        <Up>0.1</Up>" << endl;
		openXmlFileTextStream << "        <DepthIn>0</DepthIn>" << endl;
		openXmlFileTextStream << "        <Longitude>0</Longitude>" << endl;
		openXmlFileTextStream << "        <Latitude>0</Latitude>" << endl;
		openXmlFileTextStream << "        <Comment>GPS Lat:20.634687 Long: -87.084124</Comment>" << endl;
		openXmlFileTextStream << "      </SurveyData>" << endl;
    }
	openXmlFileTextStream << "      <SurveyData>" << endl;
	openXmlFileTextStream << "    <ID>" << countMesure << "</ID>" << endl;
	openXmlFileTextStream << "    <Name>FS" << countMesure << "</Name>" << endl;
	openXmlFileTextStream << "    <FromID>" << countMesure - 1 << "</FromID>" << endl;
	openXmlFileTextStream << "    <ClosureToID>-1</ClosureToID>" << endl;
	openXmlFileTextStream << "    <Section>FirstSection</Section>" << endl;
	openXmlFileTextStream << "    <Type>Real</Type>" << endl;
	openXmlFileTextStream << "    <Explorer>Ardutopo</Explorer>" << endl;
	openXmlFileTextStream << "    <Date>2013-09-21T23:18:41.0381791+02:00</Date>" << endl;
	openXmlFileTextStream << "    <Color>#FFFF0000</Color>" << endl;
	openXmlFileTextStream << "    <Length>" << distance << "</Length>" << endl;
	openXmlFileTextStream << "    <Azimut>" << cap << "</Azimut>" << endl;
	openXmlFileTextStream << "    <Inclination>" << 0 << "</Inclination>" << endl;
	openXmlFileTextStream << "    <Depth>" << profondeur << "</Depth>" << endl;
	openXmlFileTextStream << "    <Left>0.1</Left>" << endl;
	openXmlFileTextStream << "    <Right>0.1</Right>" << endl;
	openXmlFileTextStream << "    <Down>0.1</Down>" << endl;
	openXmlFileTextStream << "    <Up>0.1</Up>" << endl;
	openXmlFileTextStream << "    <DepthIn>-1</DepthIn>" << endl;
	openXmlFileTextStream << "    <Longitude>0</Longitude>" << endl;
	openXmlFileTextStream << "    <Latitude>0</Latitude>" << endl;
	openXmlFileTextStream << "    <Comment />" << endl;
	openXmlFileTextStream << "  </SurveyData>" << endl;
}

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
/*
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
		_nbStations += 1;
	}
*/
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
		_topoFile.write(_dist,_yaw,_depth);
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
