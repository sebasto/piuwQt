
#ifndef GPIO_CLASS_H
#define GPIO_CLASS_H

#include <string>
#include <QSocketNotifier>
#include <QFile>

using namespace std;
/* GPIO Class
 * Purpose: Each object instantiated from this class will control a GPIO pin
 * The GPIO pin number must be passed to the overloaded class constructor
 */
class GPIOClass
{
public:
	GPIOClass();
	GPIOClass(string gnum);
	int export_gpio();
	int unexport_gpio();
    int setdir_gpio(string dir);
	int setedge_gpio(string edge);
    int setval_gpio(string val);
    int getval_gpio(string& val);
    string get_gpionum();
	string get_gpiopath();
	
private:
	string gpionum;
};

#endif
