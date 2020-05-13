#pragma warning(disable : 4786)

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>
#include <cstdlib>

#include "particleSystem.h"
#include "modelerdraw.h"


ParticleSystem::ParticleSystem() {
	bake_start_time = -1;
	bake_end_time = -1;
}


ParticleSystem::~ParticleSystem() {

}


// start the simulation
void ParticleSystem::startSimulation(float t) {
	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	clearBaked();

	bake_start_time = t;
	bake_end_time = -1;
	simulate = true;
	dirty = true;
}


// stop the simulation
void ParticleSystem::stopSimulation(float t) {
	// These values are used by the UI
	simulate = false;
	dirty = true;
}


// reset the simulation
void ParticleSystem::resetSimulation(float t) {
	// These values are used by the UI
	simulate = false;
	dirty = true;

	clearBaked();
}


// compute forces and update particles
void ParticleSystem::computeForcesAndUpdateParticles(float t) {
	if (!simulate) return;

	// make a new frame
	ParticleFrame* frame = new ParticleFrame();
	if (particle_frame.size() != 0) frame->copy(particle_frame.back());
	
	// update old particles velocity
	float interval = 0;
	if (particle_frame.size() != 0) interval = t - particle_frame.back()->time;

	for (auto* particle : frame->particles) {
		particle->velocity[0] += 0.5;
		particle->velocity[1] -= 0.1;
	}

	// get necessary data for generate new particles
	Mat4d mat = point_object.getMatrix();
	Vec3<GLdouble> position = Vec3<GLdouble>(mat.n[3], mat.n[7], mat.n[11]);
	Vec3<GLdouble> velocity;
	const GLdouble emit_count = point_object.getEmitNumber();

	// add new particle
	Particle* particle = nullptr;
	printf("%i \n", (int)emit_count);
	for (int i = 0; i < (int)emit_count; i++) {

		const GLdouble rand_x = (GLdouble)((int)(rand() % 200) - 100) / 100;
		const GLdouble rand_y = (GLdouble)((int)(rand() % 200) - 100) / 100;
		const GLdouble rand_z = (GLdouble)((int)(rand() % 200) - 100) / 100;
		velocity = Vec3<GLdouble>(rand_x + 1, rand_y + 10, rand_z + 1);

		particle = new Particle();
		particle->position = position;
		particle->velocity = velocity;
		frame->particles.push_back(particle);
	}

	// update all particles position
	for (auto* particle : frame->particles) {
		particle->update(interval);
	}

	cur_frame = frame;
	bakeParticles(t);
}


// render particles
void ParticleSystem::drawParticles(float t) {
	if (bake_start_time < 0) return;
	if (t < bake_start_time || t > bake_end_time) return;

	// find the closest frame
	for (auto* frame : particle_frame) {
		if (t > frame->time) continue;

		for (auto& particle : frame->particles) {
			glPushMatrix();
			glTranslated(
				particle->position[0],
				particle->position[1],
				particle->position[2]);
			drawSphere(0.1);
			glPopMatrix();
		}
		return;
	}
}


// adds the current configuration of particles to 
// the data structure for storing backed particles
void ParticleSystem::bakeParticles(float t) {
	if (cur_frame == nullptr) return;
	particle_frame.push_back(cur_frame);
	cur_frame->time = t;
	bake_end_time = t;
}


// clears out the data structure of backed particles
void ParticleSystem::clearBaked() {
	particle_frame.clear();
	bake_start_time = -1;
	bake_end_time = -1;
}
