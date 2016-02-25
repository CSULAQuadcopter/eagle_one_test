#include <eagle_one_test/Tag.h>

Tag::Tag()
    :x_position_(0),
	 y_position_(0),
  	 orientation_(0.0),
     acquired_(false)
     {};

int Tag::getXPosition()
{
  	return x_position_;
}

void Tag::setXPosition(int posx)
{
  	x_position_ = posx;
}

int Tag::getYPosition()
{
	return y_position_;
}

void Tag::setYPosition(int posy)
{
	y_position_= posy;
}

int Tag::getOrientation()
{
	return orientation_;
}

void Tag::setOrientation(int orient)
{

  	orientation_= orient;
}

bool Tag::isAcquired()
{
	return acquired_;
}

void Tag::setAcquired(bool acquired)
{
 	 acquired_ = acquired;
}
