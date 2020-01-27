#include <math.h>

#include "car.h"

Car::Car(
		double vx,
		double vy,
		double s
	) :
    vx(vx),
    vy(vy),
    s(s) {
    	speed = sqrt(vx * vx + vy * vy);
    }