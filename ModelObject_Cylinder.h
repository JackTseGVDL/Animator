#ifndef MODELOBJECT_CYLINDER_H
#define MODELOBJECT_CYLINDER_H


#include "ModelObject.h"


class ModelObject_Cylinder : public ModelObject {

// Data
protected:
	// attachment
	ModelAttachment_Dynamic attachment[6];
	ModelAttachment* attach_list[6];
	GLdouble buffer[3];

// Operation
public:
	ModelObject_Cylinder();

	// model
	void modelSelf() override;
	void controlSelf(std::vector<ModelControl*>* controls) override;

// Static Function
protected:
	static GLdouble* Ops_getPoint_top(void* mm);
	static GLdouble* Ops_getRotation_top(void* mm);

	static GLdouble* Ops_getPoint_bottom(void* mm);
	static GLdouble* Ops_getRotation_bottom(void* mm);

	static GLdouble* Ops_getPoint_left(void* mm);
	static GLdouble* Ops_getRotation_left(void* mm);

	static GLdouble* Ops_getPoint_right(void* mm);
	static GLdouble* Ops_getRotation_right(void* mm);

	static GLdouble* Ops_getPoint_front(void* mm);
	static GLdouble* Ops_getRotation_front(void* mm);

	static GLdouble* Ops_getPoint_back(void* mm);
	static GLdouble* Ops_getRotation_back(void* mm);
};


# endif
