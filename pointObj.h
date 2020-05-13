#ifndef POINTOBJ_H
#define POINTOBJ_H


#include <vector>
#include <list>
#include "Particle.h"
#include "ModelObject.h"


class PointObject : public ModelObject {

protected:
	std::list<Particle*>	*particles = nullptr;
	GLdouble				emit;

public:
	PointObject();

	// tree
	bool add(ModelObject* child, uint32_t index) override;

	// model
	void modelSelf() override;

	// control
	void controlSelf(std::vector<ModelControl*>* controls) override;

	// particle
	void setParticles(std::list<Particle*> *particles);
	GLdouble getEmitNumber();
};


#endif
