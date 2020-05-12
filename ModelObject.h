#ifndef MODELOBJECT_H
#define MODELOBJECT_H


#include <FL/gl.h>
#include <vector>
#include "stdint.h"
#include "ModelAttachment.h"
#include "ModelControl.h"


class ModelObject {

// Data
protected:
	// geo
	GLdouble origin[3] = { 0 };
	GLdouble dimension[3] = { 1, 1, 1 };
	GLdouble rotation[3] = { 0 };

	ModelAttachment** attach = nullptr;
	uint32_t attach_size = 0;  // number of attachment

	// tree
	uint32_t attach_index = 0;
	ModelObject* parent = nullptr;
	std::vector<ModelObject*> children;

	// control
	const char* name = "";

// Operation
public:
	ModelObject();

	// geo
	void setOrigin(GLdouble x, GLdouble y, GLdouble z);
	void setDimension(GLdouble x, GLdouble y, GLdouble z);
	void setRotation(GLdouble x, GLdouble y, GLdouble z);

	GLdouble* getOrigin();
	GLdouble* getDimension();
	GLdouble* getRotation();

	// tree
	virtual bool add(ModelObject *child, uint32_t index);

	// model
	void model();
	void model(int32_t depth);
	virtual void modelSelf() = 0;
	virtual void modelChild(int32_t depth);

	// control
	void setName(const char* name);
	void control(std::vector<ModelControl*>* controls);
	void control(std::vector<ModelControl*>* controls, int32_t depth);
	void controlChild(std::vector<ModelControl*>* controls);
	virtual void controlSelf(std::vector<ModelControl*>* controls) = 0;
	virtual void controlChild(std::vector<ModelControl*>* controls, int32_t depth);

	// helper
	static void Helper_addControl_dimension(ModelObject* model, std::vector<ModelControl*>* controls, GLdouble min, GLdouble max);
	static void Helper_addControl_rotation(ModelObject* model, std::vector<ModelControl*>* controls, GLdouble min, GLdouble max);
	static void Helper_addControl_origin(ModelObject* model, std::vector<ModelControl*>* controls, GLdouble min, GLdouble max);
};


#endif
