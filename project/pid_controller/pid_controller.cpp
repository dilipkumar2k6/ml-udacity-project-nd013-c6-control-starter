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

    output_lim_min = output_lim_mini;
    output_lim_max = output_lim_maxi;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
    p_error = cte;
    i_error = cte * delta_time;
    d_error = (cte - p_error) / delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = (kp * p_error) - (ki * i_error) - (kd * d_error);

    return max(output_lim_min, min(control, output_lim_max));
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
    delta_time = new_delta_time;

    return delta_time;
}