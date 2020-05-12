#include "ModelAttachment.h"


// Static Data
static GLdouble default_array[3] = { 0, 0, 0 };


// ModelAttachment
ModelAttachment::ModelAttachment()
{
}


// Model Attachment: Static
ModelAttachment_Static::ModelAttachment_Static():
ModelAttachment::ModelAttachment()
{
}


void ModelAttachment_Static::setPoint(GLdouble x, GLdouble y, GLdouble z) {
	point[0] = x;
	point[1] = y;
	point[2] = z;
}


void ModelAttachment_Static::setRotation(GLdouble x, GLdouble y, GLdouble z) {
	rotation[0] = x;
	rotation[1] = y;
	rotation[2] = z;
}

GLdouble* ModelAttachment_Static::getPoint() {
	return point;
}


GLdouble* ModelAttachment_Static::getRotation() {
	return rotation;
}


// Model Attachemnt: Dynamic
ModelAttachment_Dynamic::ModelAttachment_Dynamic():
ModelAttachment::ModelAttachment()
{
}


void ModelAttachment_Dynamic::setMM(void* m) {
	mm = m;
}


void ModelAttachment_Dynamic::setOps_point(GLdouble* (*ops)(void* mm)) {
	ops_get_point = ops;
}


void ModelAttachment_Dynamic::setOps_rotation(GLdouble* (*ops)(void* mm)) {
	ops_get_rotation = ops;
}


// do nothing
void ModelAttachment_Dynamic::setPoint(GLdouble x, GLdouble y, GLdouble z) {
}


// do nothing
void ModelAttachment_Dynamic::setRotation(GLdouble x, GLdouble y, GLdouble z) {
}


GLdouble* ModelAttachment_Dynamic::getPoint() {
	if (ops_get_point == nullptr) return default_array;
	return ops_get_point(mm);
}


GLdouble* ModelAttachment_Dynamic::getRotation() {
	if (ops_get_rotation == nullptr) return default_array;
	return ops_get_rotation(mm);
}
