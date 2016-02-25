#ifndef TAG_H
#define TAG_H

#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>

class Tag
{
public:
  	Tag();

    int getXPosition();		   // getters and setters are public
	void setXPosition(int);   // void is for set
  	int getYPosition();
  	void setYPosition(int);		//pass type through setters
  	int getOrientation();
  	void setOrientation(int);
  	bool isAcquired();
  	void setAcquired(bool);

private:					//attributes are private.
	int x_position_;
	int y_position_;
  	double orientation_;	//member attributes underscore at end
    bool acquired_;
};

#endif /* TAG_H */
