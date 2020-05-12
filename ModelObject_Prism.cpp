#include "ModelObject_Prism.h"
#include "modelerdraw.h"
#include "stdio.h"


// Operation
ModelObject_Prism::ModelObject_Prism():
ModelObject::ModelObject()
{
}


void ModelObject_Prism::modelSelf() {
	// draw the base
	drawTriangle(
		-0.5, 0, -0.5,
		0.5, 0, -0.5,
		0.5, 0, 0.5);
	drawTriangle(
		-0.5, 0, -0.5,
		0.5, 0, 0.5,
		-0.5, 0, 0.5);

	// draw upper
	drawTriangle(
		-0.5, 0, -0.5,
		0.5, 0, -0.5,
		0, 1, 0);
	drawTriangle(
		0.5, 0, -0.5,
		0.5, 0, 0.5,
		0, 1, 0);
	drawTriangle(
		0.5, 0, 0.5,
		-0.5, 0, 0.5,
		0, 1, 0);
	drawTriangle(
		-0.5, 0, 0.5,
		-0.5, 0, -0.5,
		0, 1, 0);
}


void ModelObject_Prism::controlSelf(std::vector<ModelControl*>* controls) {
	return;
}
