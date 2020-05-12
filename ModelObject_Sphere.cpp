#include "modelerdraw.h"
#include "ModelObject_Sphere.h"


ModelObject_Sphere::ModelObject_Sphere():
	ModelObject::ModelObject()
{
	dimension[0] = 1;
	dimension[1] = 1;
	dimension[2] = 1;
}


void ModelObject_Sphere::modelSelf() {
	glTranslated(0, dimension[1] / 2, 0);
	glScaled(dimension[0] / 2, dimension[1] / 2, dimension[2] / 2);
	drawSphere(1);
}


void ModelObject_Sphere::controlSelf(std::vector<ModelControl*>* controls) {
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
