/*
 * EasyBNO055_ESP.cpp
 *
 *  Created on: Nov 14, 2023
 *      Author: hephaestus
 */

#include <EasyBNO055_ESP.h>

static EasyBNO055_ESP * ref =nullptr;
static TaskHandle_t complexHandlerTaskUS;
static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
static int started=0;
static double staticOffset=0;
void EasyBNO055Thread(void *param){
	bool bnoStarted = false;
	bnoStarted=bno.begin();
	double revolutions=0;
	bool start=false;
	double lastOrent=0;
	double lastDiff=0;
	while(true){
		if(ref!=nullptr){
			if(started>=10)
				if(ref->sync!=nullptr){
					ref->sync();
				}
			if(bnoStarted){
				sensors_event_t orientationData;
				bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
				ref->orientationX=orientationData.orientation.y;
				ref->orientationY=orientationData.orientation.z;
				double tmp=orientationData.orientation.x;
				lastDiff = tmp-lastOrent;
				if(abs(lastDiff)>180){
					if(tmp>180)
						revolutions-=1;
					else
						revolutions+=1;
					//Serial.println("Revolutions = "+String(revolutions));
				}
				lastOrent=tmp;
				ref->orientationZ = (revolutions*360)+tmp+staticOffset;
				if(started <11)
					started++;
				if(started==10){
					staticOffset = -ref->orientationZ;
				}
			}else{
				Serial.println("EasyBNO055 error, resetting");
				bnoStarted=bno.begin();
			}
		}
		vTaskDelay(10); //sleep 10ms

	}
}

EasyBNO055_ESP::EasyBNO055_ESP(){


}
void EasyBNO055_ESP::start(void (*mysync) (void)){
	sync = mysync;
	ref=this;

	xTaskCreatePinnedToCore(EasyBNO055Thread, "EasyBNO055", 8192, NULL, 2, // low priority timout thread
					&complexHandlerTaskUS, 0);
	while(started<10){
		vTaskDelay(10); //sleep 10ms

	}
}
