// Pid.cpp
#include <eagle_one_test/PIDController.h>

Pid::Pid()
    :last_time(0),
     dt(0),
     kp(0.5),
     ki(0),
     kd(0),
     current_time(0),
 	 error(0),
     update(0)
{
	// TODO Auto-generated constructor stub
}

Pid::~Pid() {
	// TODO Auto-generated destructor stub
}

void Pid::calcDt(double ct)
{
  	setDt(ct - getLastTime());
  	setLastTime(ct);
}

void Pid::calcError(double e)
{
  	setError(e + getError());
}

double Pid::calcP()
{
  	return getKp() * getError();
}

double Pid::calcI()
{
  	return getKi() * getError() * getDt();
}

double Pid::calcD(double ce)
{
  	return getKd() * (ce - getError()) / getDt();
}

double Pid::calcUpdate(double ce, double ct)
{
  	calcDt(ct);
  	float d_update = calcD(ce);
  	calcError(ce);
  	update = calcP()+ calcI() + d_update ;
}

void Pid::zeroize()
{
 	setError(0);
}

double Pid::getCurrentTime() const {
	return current_time;
}

void Pid::setCurrentTime(double currentTime) {
	current_time = currentTime;
}

double Pid::getError() const {
	return error;
}

void Pid::setError(double error) {
	this->error = error;
}

double Pid::getKd() const {
	return kd;
}

void Pid::setKd(double kd) {
	this->kd = kd;
}

double Pid::getKi() const {
	return ki;
}

void Pid::setKi(double ki) {
	this->ki = ki;
}

double Pid::getKp() const {
	return kp;
}

void Pid::setKp(double kp) {
	this->kp = kp;
}

double Pid::getLastTime() const {
	return last_time;
}

void Pid::setLastTime(double lastTime) {
	last_time = lastTime;
}

void Pid::setDt(double t)
{
    dt = t;
}

double Pid::getDt()
{
    return (double) dt;
}

double Pid::getUpdate()
{
    return update;
}
