#ifndef MODELATTACHMENT_H
#define MODELATTACHMENT_H


#include <FL/gl.h>


class ModelAttachment {

// Operation
public:
	ModelAttachment();

	// set
	virtual void setPoint(GLdouble x, GLdouble y, GLdouble z) = 0;
	virtual void setRotation(GLdouble x, GLdouble y, GLdouble z) = 0;

	// get
	virtual GLdouble* getPoint() = 0;
	virtual GLdouble* getRotation() = 0;
};


class ModelAttachment_Static : public ModelAttachment {

// Data
protected:
	GLdouble point[3] = { 0 };
	GLdouble rotation[3] = { 0 };

// Operation
public:
	ModelAttachment_Static();

	// set
	void setPoint(GLdouble x, GLdouble y, GLdouble z) override;
	void setRotation(GLdouble x, GLdouble y, GLdouble z) override;

	// get
	GLdouble* getPoint() override;
	GLdouble* getRotation() override;
};


class ModelAttachment_Dynamic : public ModelAttachment {

// Data
protected:
	void*		mm = nullptr;
	GLdouble*	(*ops_get_point)(void *mm)		= nullptr;
	GLdouble*	(*ops_get_rotation)(void *mm)	= nullptr;

// Operation
public:
	ModelAttachment_Dynamic();

	// set
	void setMM(void* m);
	void setOps_point(GLdouble* (*ops)(void* mm));
	void setOps_rotation(GLdouble* (*ops)(void* mm));

	void setPoint(GLdouble x, GLdouble y, GLdouble z) override;
	void setRotation(GLdouble x, GLdouble y, GLdouble z) override;

	// get
	GLdouble* getPoint() override;
	GLdouble* getRotation() override;
};


#endif
