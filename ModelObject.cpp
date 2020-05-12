#include "ModelObject.h"


ModelObject::ModelObject() {
	origin[0] = 0.0f;
	origin[1] = 0.0f;
	origin[2] = 0.0f;

	attach = nullptr;
	attach_size = 0;

	attach_index = 0xFFFFFFFF;
	// parent = nullptr;

	this->name = name;
}


void ModelObject::setOrigin(GLdouble x, GLdouble y, GLdouble z) {
	origin[0] = x;
	origin[1] = y;
	origin[2] = z;
}


void ModelObject::setDimension(GLdouble x, GLdouble y, GLdouble z) {
	dimension[0] = x;
	dimension[1] = y;
	dimension[2] = z;
}


void ModelObject::setRotation(GLdouble x, GLdouble y, GLdouble z) {
	rotation[0] = x;
	rotation[1] = y;
	rotation[2] = z;
}


GLdouble* ModelObject::getOrigin() {
	return origin;
}


GLdouble* ModelObject::getDimension() {
	return dimension;
}


GLdouble* ModelObject::getRotation() {
	return rotation;
}


// TODO: currently no checking on tree structure
bool ModelObject::add(ModelObject *child, uint32_t index) {
	if (child == nullptr || index >= attach_size) return false;
	children.push_back(child);
	child->parent = this;
	child->attach_index = index;
	return true;
}


void ModelObject::model() {
	model(-1);
}


void ModelObject::model(int32_t depth) {
	if (depth == 0) return;

	// attachment rotation and translation
	if (origin[0] != 0 || origin[1] != 0 || origin[2] != 0) glTranslated(origin[0], origin[1], origin[2]);
	if (rotation[0] != 0) glRotated(rotation[0], 1, 0, 0);
	if (rotation[1] != 0) glRotated(rotation[1], 0, 1, 0);
	if (rotation[2] != 0) glRotated(rotation[2], 0, 0, 1);

	// self
	glPushMatrix();
	modelSelf();
	glPopMatrix();

	// child
	modelChild(depth);
}


void ModelObject::modelChild(int32_t depth) {
	for (ModelObject* child : children) {
		glPushMatrix();

		// joint translation and rotation
		if (attach[child->attach_index]->getPoint()[0] != 0 ||
			attach[child->attach_index]->getPoint()[1] != 0 ||
			attach[child->attach_index]->getPoint()[2] != 0)
			glTranslated(
				attach[child->attach_index]->getPoint()[0],
				attach[child->attach_index]->getPoint()[1],
				attach[child->attach_index]->getPoint()[2]);

		if (attach[child->attach_index]->getRotation()[0] != 0) glRotated(attach[child->attach_index]->getRotation()[0], 1, 0, 0);
		if (attach[child->attach_index]->getRotation()[1] != 0) glRotated(attach[child->attach_index]->getRotation()[1], 0, 1, 0);
		if (attach[child->attach_index]->getRotation()[2] != 0) glRotated(attach[child->attach_index]->getRotation()[2], 0, 0, 1);

		child->model(depth <= 0 ? depth : depth - 1);
		glPopMatrix();
	}
}


void ModelObject::setName(const char* s) {
	name = s;
}


void ModelObject::control(std::vector<ModelControl*>* controls) {
	control(controls, -1);
}


void ModelObject::control(std::vector<ModelControl*>* controls, int32_t depth) {
	if (depth == 0) return;
	controlSelf(controls);
	controlChild(controls, depth);
}


void ModelObject::controlChild(std::vector<ModelControl*>* controls) {
	controlChild(controls, -1);
}


void ModelObject::controlChild(std::vector<ModelControl*>* controls, int32_t depth) {
	for (ModelObject* child : children) child->control(controls, depth <= 0 ? depth : depth - 1);
}


void ModelObject::Helper_addControl_dimension(ModelObject* model, std::vector<ModelControl*>* controls, GLdouble min, GLdouble max) {
	ModelControl* control_0 = new ModelControl(model->dimension + 0, min, max);
	ModelControl* control_1 = new ModelControl(model->dimension + 1, min, max);
	ModelControl* control_2 = new ModelControl(model->dimension + 2, min, max);

	control_0->appendName(model->name);
	control_1->appendName(model->name);
	control_2->appendName(model->name);

	control_0->appendName(": Dimension-X");
	control_1->appendName(": Dimension-Y");
	control_2->appendName(": Dimension-Z");

	controls->push_back(control_0);
	controls->push_back(control_1);
	controls->push_back(control_2);
}


void ModelObject::Helper_addControl_rotation(ModelObject* model, std::vector<ModelControl*>* controls, GLdouble min, GLdouble max) {
	ModelControl* control_0 = new ModelControl(model->rotation + 0, min, max);
	ModelControl* control_1 = new ModelControl(model->rotation + 1, min, max);
	ModelControl* control_2 = new ModelControl(model->rotation + 2, min, max);

	control_0->appendName(model->name);
	control_1->appendName(model->name);
	control_2->appendName(model->name);

	control_0->appendName(": Rotation-X");
	control_1->appendName(": Rotation-Y");
	control_2->appendName(": Rotation-Z");

	controls->push_back(control_0);
	controls->push_back(control_1);
	controls->push_back(control_2);
}


void ModelObject::Helper_addControl_origin(ModelObject* model, std::vector<ModelControl*>* controls, GLdouble min, GLdouble max) {
	ModelControl* control_0 = new ModelControl(model->origin + 0, min, max);
	ModelControl* control_1 = new ModelControl(model->origin + 1, min, max);
	ModelControl* control_2 = new ModelControl(model->origin + 2, min, max);

	control_0->appendName(model->name);
	control_1->appendName(model->name);
	control_2->appendName(model->name);

	control_0->appendName(": Origin-X");
	control_1->appendName(": Origin-Y");
	control_2->appendName(": Origin-Z");

	controls->push_back(control_0);
	controls->push_back(control_1);
	controls->push_back(control_2);
}
