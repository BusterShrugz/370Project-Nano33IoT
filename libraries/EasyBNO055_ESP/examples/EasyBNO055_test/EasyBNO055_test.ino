
#include <EasyBNO055_ESP.h>


EasyBNO055_ESP bno;

void otherI2CUpdate(){
	//Serial.println("otherI2CUpdate");
}

void setup(){
	Serial.begin(115200);
	Serial.println("Starting EasyBNO055 test");

	// Start EasyBNO055 with a function pointer to call the update of all other I2C devices
	bno.start(&otherI2CUpdate);

	// If you have no other I2C device to synchronize, you can use the other start method
	//bno.start();
}

void loop(){
	delay(100);
    Serial.print("\tx= ");
    Serial.print(bno.orientationX);
    Serial.print(" |\ty= ");
    Serial.print(bno.orientationY);
    Serial.print(" |\tz= ");
    Serial.println(bno.orientationZ);
}
