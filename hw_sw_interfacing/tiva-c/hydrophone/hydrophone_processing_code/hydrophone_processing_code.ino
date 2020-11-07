/********************sliding_dft.hpp**************************
This code efficiently computes discrete Fourier transforms (DFTs)
from a continuous sequence of input values. It is a recursive algorithm 
that updates the DFT when each new time-domain measurement arrives, 
effectively applying a sliding window over the last N samples. This 
implementation applies the Hanning window in order to minimise spectral leakage
*/
#include <complex.h>
#include <ros.h>
#include <std_msgs/float.h>
#include "sliding_dft.hpp"

float calc_heading();

ros::NodeHandle nh;
std_msgs::float heading_angle;
ros::Publisher hydrophone("pinger_heading", &heading_angle);

const int hydrophone1_pin = A0;    
const int hydrophone2_pin = A1; 
const int hydrophone3_pin = A2; 
     



void setup() 
{

  nh.initNode();
  nh.advertise(hydrophone.);

}

void loop() 
{
  static SlidingDFT<double, 512> dft1, dft2, dft2;

  for(int sample = 0; sample < 512; sample++)
  {
  	dft1.update(analogRead(hydrophone1_pin));
  	dft1.update(analogRead(hydrophone2_pin));
  	dft1.update(analogRead(hydrophone3_pin));
  }

  std::complex<double> hydrophone1_signal = dft1.dft[0];
  std::complex<double> hydrophone2_signal = dft2.dft[0];
  std::complex<double> hydrophone3_signal = dft3.dft[0];
  if(//check if there is a ping and other conditions are satisfied)
  {
  	float angle = calc_heading();
  	hydrophone.publish( &heading_angle );
  }
     
  nh.spinOnce();
}

