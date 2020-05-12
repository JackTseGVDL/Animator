#ifndef MODELCONTROL_H
#define MODELCONTROL_H


#include <FL/gl.h>
#include <string>


class ModelControl {

// Data
protected:
	std::string name;
	GLdouble* value;
	GLdouble  limit[2];

// Operation
public:
	ModelControl(GLdouble* value, GLdouble min, GLdouble max);

	// set
	void setLimit(GLdouble min, GLdouble max);
	void setValue(GLdouble value);
	void appendName(const char* s);

	// get
	std::string* getName();
	GLdouble getValue();
	GLdouble getLimit_min();
	GLdouble getLimit_max();
};


#endif
