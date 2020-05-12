#ifndef MODELOBJECT_PRISM_H
#define MODELOBJECT_PRISM_H


#include "ModelObject.h"


class ModelObject_Prism : public ModelObject {

// Data
protected:
	// ...

// Operation
public:
	ModelObject_Prism();

	// model
	void modelSelf() override;
	void controlSelf(std::vector<ModelControl*>* controls) override;
};


#endif
