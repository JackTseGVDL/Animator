#include "modelerdraw.h"
#include "ModelObject_Cylinder.h"


ModelObject_Cylinder::ModelObject_Cylinder() :
	ModelObject::ModelObject()
{
	// geo
	dimension[0] = 1;
	dimension[1] = 1;
	dimension[2] = 1;

	// attachment
	attach_size = sizeof(attachment) / sizeof(ModelAttachment_Dynamic);

	attach = attach_list;
	for (int i = 0; i < attach_size; i++) attach_list[i] = attachment + i;

	attachment[0].setMM(this);
	attachment[0].setOps_point(Ops_getPoint_top);
	attachment[0].setOps_rotation(Ops_getRotation_top);

	attachment[1].setMM(this);
	attachment[1].setOps_point(Ops_getPoint_bottom);
	attachment[1].setOps_rotation(Ops_getRotation_bottom);

	attachment[2].setMM(this);
	attachment[2].setOps_point(Ops_getPoint_left);
	attachment[2].setOps_rotation(Ops_getRotation_left);

	attachment[3].setMM(this);
	attachment[3].setOps_point(Ops_getPoint_right);
	attachment[3].setOps_rotation(Ops_getRotation_right);

	attachment[4].setMM(this);
	attachment[4].setOps_point(Ops_getPoint_front);
	attachment[4].setOps_rotation(Ops_getRotation_front);

	attachment[5].setMM(this);
	attachment[5].setOps_point(Ops_getPoint_back);
	attachment[5].setOps_rotation(Ops_getRotation_back);

	// buffer
	for (int i = 0; i < 3; i++) buffer[i] = 0;
}


void ModelObject_Cylinder::modelSelf() {
	// glTranslated(0, 0, 0);
	glRotated(-90, 1, 0, 0);
	glScaled(dimension[0], dimension[2], dimension[1]);
	drawPolygon(16, 1, 1, 1);
}


void ModelObject_Cylinder::controlSelf(std::vector<ModelControl*>* controls) {
	ModelControl* control_0 = new ModelControl(rotation + 0, -180, 180);
	ModelControl* control_1 = new ModelControl(rotation + 1, -180, 180);
	ModelControl* control_2 = new ModelControl(rotation + 2, -180, 180);

	control_0->appendName(name);
	control_1->appendName(name);
	control_2->appendName(name);

	control_0->appendName(": Rotation-X");
	control_1->appendName(": Rotation-Y");
	control_2->appendName(": Rotation-Z");

	controls->push_back(control_0);
	controls->push_back(control_1);
	controls->push_back(control_2);
}


// static function
GLdouble* ModelObject_Cylinder::Ops_getPoint_top(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = model->dimension[1];
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getRotation_top(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = 0;
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getPoint_bottom(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = 0;
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getRotation_bottom(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 180;
	model->buffer[1] = 0;
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getPoint_left(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = -(model->dimension[0]) / 2;
	model->buffer[1] = model->dimension[1] / 2;
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getRotation_left(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = 0;
	model->buffer[2] = 90;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getPoint_right(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = model->dimension[0] / 2;
	model->buffer[1] = model->dimension[1] / 2;
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getRotation_right(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = 0;
	model->buffer[2] = -90;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getPoint_front(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = model->dimension[1] / 2;
	model->buffer[2] = model->dimension[2] / 2;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getRotation_front(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 90;
	model->buffer[1] = 0;
	model->buffer[2] = 0;
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getPoint_back(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = 0;
	model->buffer[1] = model->dimension[1] / 2;
	model->buffer[2] = -(model->dimension[2] / 2);
	return model->buffer;
}


GLdouble* ModelObject_Cylinder::Ops_getRotation_back(void* mm) {
	ModelObject_Cylinder* model = (ModelObject_Cylinder*)mm;
	model->buffer[0] = -90;
	model->buffer[1] = 0;
	model->buffer[2] = 0;
	return model->buffer;
}

