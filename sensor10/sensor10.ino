/*
 * sensor10 - test program
 * originated @ cyclehack berlin 2017 
 * 
 */

// include all libraries
#include "SDS011.h"

// define PINS

#define RX 0
#define TX 1


// define values for sensor data

float p10,p25;
int sds_error


// define sensors
SDS011 sds;

void setup() {
  // put your setup code here, to run once:

  // init SDS
  sds.begin(RX, TX);

  // init GPS

  // init LoRa

}

void loop() {
  // put your main code here, to run repeatedly:
  /* pseudo-code for defining program-log */

  /*
  read_gps_data();
  if(gps_accuracy > accurate)
  {
    sds_error = sds.read(&p25,&p10);
    if(!sds_error)
    {
      send_data_over_lorra();
      maybe_wait_for_response();
    }
    else
    {
      Log("error reading SDS");
    }
  }
  else
  {
    Log("GPS accuracy too low!");
  }
  delay("10000");
  */ //end of pseudo code
}
