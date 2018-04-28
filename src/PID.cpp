#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

PID::~PID() {
}

void PID::Init(double InKp, double InKi, double InKd){
    Kp = InKp;
    Ki = InKi;
    Kd = InKd;
}

void PID::UpdateError(double cte) {
    i_error += cte;
    d_error = cte - p_error;
    p_error = cte;
}

double PID::TotalError() {
    return -Kp * p_error - Ki * i_error - Kd * d_error;
}

