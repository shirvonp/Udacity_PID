/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  
  kp = Kpi;
  ki = Kii;
  kd = Kdi;
  minLim = output_lim_mini;
  maxLim = output_lim_maxi;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  if(dt>0){
    d_error = (cte - p_error)/dt;
  } 
  else{
    d_error = 0.0;
  }
  p_error = cte;
  i_error += cte*dt;
  
}


double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
  
    control = (kp*p_error + kd*d_error + ki*i_error);
  
    if (control < minLim) {
    control = minLim;
    }
    else if (control > maxLim){
    control = maxLim;
    }       
  
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
	dt = new_delta_time;
    return dt;
}