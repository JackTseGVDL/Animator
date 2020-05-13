#ifndef PARTICLE_H
#define PARTICLE_H


#include "vec.h"


class Particle {
public:
	Vec3<double> position;
	Vec3<double> velocity;

public:
	Particle() {
		position = Vec3<double>();
		velocity = Vec3<double>();
	}

	Particle(Vec3<double> pos, Vec3<double> velocity):
		position(pos), velocity(velocity)
	{}


	void reset() {
		position[0] = 0;
		position[1] = 0;
		position[2] = 0;
	}


	// update with no acceleration
	void update(float interval) {
		position += velocity * (double)interval;
	}
};


#endif
