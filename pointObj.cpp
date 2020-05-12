#include "ModelerDraw.h"
#include "PointObj.h"


// Operation Handling
// tree
// no child is allowed
bool PointObject::add(ModelObject* child, uint32_t index) {
	return false;
}


// model
void PointObject::modelSelf() {
	for (auto& particle : particles) {
		glPushMatrix();
		glTranslated(
			particle.position[0],
			particle.position[1],
			particle.position[2]);
		drawSphere(1);
		glPopMatrix();
	}
}


// no child is allowed
void PointObject::modelChild(int32_t depth) {
	return;
}


// control
void PointObject::controlSelf(std::vector<ModelControl*>* controls) {
	ModelControl* control_0 = new ModelControl(&count, 0, 5);
	control_0->appendName(name);
	control_0->appendName(": Count");
	controls->push_back(control_0);
}


// no child is allowed
void PointObject::controlChild(std::vector<ModelControl*>* controls, int32_t depth) {
	return;
}


// particle
void PointObject::setParticleCount(int count) {
	this->count = (GLdouble)count;
}


void PointObject::resetParticle() {
	for (auto& particle : particles) particle.reset();
}


void PointObject::updateParticle(float interval) {
	for (auto& particle : particles) particle.update(interval, force);
}
