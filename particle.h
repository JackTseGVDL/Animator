#ifndef PARTICLE_H
#define PARTICLE_H


#include "vec.h"


class Particle {
public:
	Vec3<float> position;
	Vec3<float> velocity;

public:
	Particle() {
		position = Vec3<float>();
		velocity = Vec3<float>();
	}

	Particle(Vec3<float> pos, Vec3<float> velocity):
		position(pos), velocity(velocity)
	{}


	void reset() {
		position[0] = 0;
		position[1] = 0;
		position[2] = 0;
	}


	// update with no acceleration
	void update(float interval) {
		position += velocity * interval;
	}


	// update with acceleration
	void update(float interval, Vec3<float> force) {
		velocity += force;
		update(interval);
	}
};


#endif
