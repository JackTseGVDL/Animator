#ifndef MODELOBJECT_TORUS_H
#define MODELOBJECT_TORUS_H


#include "ModelObject.h"


class ModelObject_Torus : public ModelObject {

// Data
protected:
	GLdouble torus_c = 10;
	GLdouble torus_t = 10;
	GLdouble torus_r_1 =1;
	GLdouble torus_r_2 = 0.25;

// Operation
public:
	ModelObject_Torus();

	// model
	void modelSelf() override;
	void controlSelf(std::vector<ModelControl*>* controls) override;
};


#endif