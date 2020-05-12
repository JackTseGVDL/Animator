#ifndef MODELOBJECT_SPHERE_H
#define MODELOBJECT_SPHERE_H


#include "ModelObject.h"


class ModelObject_Sphere : public ModelObject {

// Data
protected:
	// ...

// Operation
public:
	ModelObject_Sphere();

	// model
	void modelSelf() override;
	void controlSelf(std::vector<ModelControl*>* controls) override;

};


#endif
