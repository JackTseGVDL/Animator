#include "ModelControl.h"


// Operation Handling
ModelControl::ModelControl(GLdouble* v, GLdouble min, GLdouble max) {
	value = v;
	limit[0] = min;
	limit[1] = max;
}


void ModelControl::setLimit(GLdouble min, GLdouble max) {
	limit[0] = min;
	limit[1] = max;
}


void ModelControl::setValue(GLdouble v) {
	*value = v;
}


void ModelControl::appendName(const char* s) {
	name.append(s);
}


std::string* ModelControl::getName() {
	return &(name);
}


GLdouble ModelControl::getValue() {
	return *value;
}


GLdouble ModelControl::getLimit_min() {
	return limit[0];
}


GLdouble ModelControl::getLimit_max() {
	return limit[1];
}
