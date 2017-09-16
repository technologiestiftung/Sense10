// SDS011 dust sensor example
// -----------------------------
//
// By R. Zschiegner (rz@madavi.de).
// April 2016
// edited by Hannes 09/2017

#include "SDS011.h"

#define D0 0
#define D1 1

float p10,p25;
int error;

SDS011 my_sds;

void setup() {
	Serial.begin(9600);
  Serial.println("starting setup()");
  my_sds.begin(D0,D1);
  my_sds.wakeup();
}

void loop() {
	error = my_sds.read(&p25,&p10);
	if (! error) {
		Serial.println("P2.5: "+String(p25));
		Serial.println("P10:  "+String(p10));
	}
 else
 {
  Serial.println("Error");
 }
	delay(1000);
}
