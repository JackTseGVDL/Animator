#ifndef POINTOBJ_H
#define POINTOBJ_H


#include <vector>
#include "Particle.h"
#include "ModelObject.h"


class PointObject : public ModelObject {

protected:
	std::vector<Particle>		particles;
	Vec3<float>					force;
	GLdouble					count;  // particle number

public:
	// tree
	bool add(ModelObject* child, uint32_t index) override;

	// model
	void modelSelf() override;
	void modelChild(int32_t depth) override;
	
	// control
	void controlSelf(std::vector<ModelControl*>* controls) override;
	void controlChild(std::vector<ModelControl*>* controls, int32_t depth) override;

	// particle
	void setParticleCount	(int count);
	void resetParticle		();
	void updateParticle		(float interval);
};


#endif
