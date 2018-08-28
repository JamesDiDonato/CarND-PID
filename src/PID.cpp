#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}


/*
* Initialize PID.
*/
void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Kd = Kd; 
	this->Ki = Ki;

	p_error = 0;
  i_error = 0;
  d_error = 0;

}

/*
* Update the PID error variables given cross track error.
*/
void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

/*
* Calculate the total PID error.
*/
double PID::TotalError() {
	return - (Kp*p_error + Ki*i_error + Kd*d_error);
}

