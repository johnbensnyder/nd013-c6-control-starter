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
  this->Kpi = Kpi;
  this->Kii = Kii;
  this->Kdi = Kdi;
  this->output_lim_maxi = output_lim_maxi;
  this->output_lim_mini = output_lim_mini;
  diff_cte = 0.0;
  prev_cte = 0.0;
  sigma_cte = 0.0;
  //delta_time = 1.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  if(delta_time==0){
  	  diff_cte = 0.0;
  	}
  else{
  	  diff_cte = (cte - prev_cte) / delta_time;
  	}
  prev_cte = cte;
  sigma_cte += cte * delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    //double control;
    double control = Kpi * prev_cte + Kii * sigma_cte + Kdi * diff_cte;
    control = min(output_lim_maxi, max(output_lim_mini, control));
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}