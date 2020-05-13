#include "ModelerDraw.h"
#include "PointObj.h"


// Operation Handling
PointObject::PointObject():
	ModelObject::ModelObject() {

	// attachment
	attach_size = 0;

	// point object
	emit = 1;
}

// tree
// no child is allowed
bool PointObject::add(ModelObject* child, uint32_t index) {
	return false;
}


// model
void PointObject::modelSelf() {
	return;
}


// control
void PointObject::controlSelf(std::vector<ModelControl*>* controls) {
	ModelControl* control_0 = new ModelControl(&emit, 0, 20);
	control_0->appendName(name);
	control_0->appendName(": Count");
	controls->push_back(control_0);
}


// particle
void PointObject::setParticles(std::list<Particle*> *particles) {
	this->particles = particles;
}


GLdouble PointObject::getEmitNumber() {
	return emit;
}
