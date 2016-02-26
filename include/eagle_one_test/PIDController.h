/*
This is the header file as well as class for the pid controller
*/
#ifndef PID_H //This is used to create the header
#define PID_H

#include <ros/ros.h> //the libraries we are going to compile from as well
#include <ardrone_autonomy/Navdata.h> //include the following library at location
#include <eagle_one_test/Pid_send.h>

class Pid
{
public:
    Pid(); //pid function

		//take care of the derivative
		//getters and setters
		//int getXposition();
		//void setXposition();
		//int
    ~Pid();

  	void calcDt(double t);
  	void calcError(double e);
  	double calcP();
  	double calcI();
  	double calcD(double ce);
  	double calcUpdate(double ce, double ct);

  	void zeroize();
    void pid_set_param(const eagle_one_test::Pid_send::ConstPtr& data);

	double getCurrentTime() const;
	void setCurrentTime(double currentTime);
	double getError() const;
	void setError(double error);
	double getKd() const;
	void setKd(double kd);
	double getKi() const;
	void setKi(double ki);
	double getKp() const;
	void setKp(double kp);
	double getLastTime() const;
	void setLastTime(double lastTime);
    void setDt(double t);
    double getDt();
    double getUpdate();

private: //here we are setting the attributes
    double last_time;
    double dt;
    double kd;
    double kp;
    double ki;
    double current_time;
	double error;
    double update;

};
#endif /*PID_H*/
