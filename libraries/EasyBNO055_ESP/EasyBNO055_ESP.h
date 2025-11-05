#ifndef LIBRARIES_EASYBNO055_ESP_EASYBNO_H_
#define LIBRARIES_EASYBNO055_ESP_EASYBNO_H_

#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


class EasyBNO055_ESP{
public:
	EasyBNO055_ESP();
	void (*sync) (void);
	void start(void (*mysync) (void));
	void start(){
		start(nullptr);
	}
	double orientationX=0;
	double orientationY=0;
	double orientationZ=0;


};

#endif /* LIBRARIES_EASYBNO055_ESP_EASYBNO_H_ */
