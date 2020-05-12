// The sample model.  You should build a file
// very similar to this for when you make your model.
#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "modelerglobals.h"
#include <FL/gl.h>

#include "ModelObject.h"
#include "ModelObject_Box.h"
#include "ModelObject_Sphere.h"
#include "ModelObject_Cylinder.h"
#include "ModelObject_Prism.h"
#include "ModelObject_Torus.h"


// Data
// Feature
ModelObject_Box model_body;
ModelObject_Box model_head;
ModelObject_Box model_RUA;		// upper arm
ModelObject_Box model_RLA;
ModelObject_Box model_LUA;		// lower arms
ModelObject_Box model_LLA;
ModelObject_Box model_LUL;		// upper leg
ModelObject_Box model_LLL;
ModelObject_Box model_RUL;		// lower leg
ModelObject_Box model_RLL;
ModelObject_Box model_LF;		// foot
ModelObject_Box model_RF;
ModelObject_Prism model_hat;
ModelObject_Torus model_torus;

GLdouble control_global_buffer[4];
ModelControl control_global_x(control_global_buffer + 0, -5, 5);
ModelControl control_global_y(control_global_buffer + 1, -5, 5);
ModelControl control_global_z(control_global_buffer + 2, -5, 5);
ModelControl control_global_scale(control_global_buffer + 3, 0, 1);

GLdouble control_multi_buffer[1];
ModelControl control_multi_action(control_multi_buffer + 0, -90, 90);

std::vector<ModelControl*> controls;
int control_size = 0;
ModelerControl* control_table = nullptr;


// Static Function
static void callback_valChanged();


// To make a SampleModel, we inherit off of ModelerView
class SampleModel : public ModelerView {
public:
    SampleModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) { }

    virtual void draw();
};


// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createSampleModel(int x, int y, int w, int h, char *label) { 
    return new SampleModel(x, y, w, h, label); 
}


// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out SampleModel
void SampleModel::draw() {
    // This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();

	// set color
	setAmbientColor(.1f, .1f, .1f);
	setDiffuseColor(COLOR_GREEN);

	// global movement: push
	glPushMatrix();
	glScaled(
		control_global_buffer[3],
		control_global_buffer[3],
		control_global_buffer[3]);
	glTranslated(
		control_global_buffer[0],
		control_global_buffer[1], 
		control_global_buffer[2]);

	// model
	// Feature
	// Example of Creating a Minecraft Man
	// model_body.model();

	model_body.model();

	// global movement: pop
	glPopMatrix();
}


int main() {
	// model
	// Feature
	// Example of Creating a Minecraft Man
	model_body.setName("Body");
	model_head.setName("Head");
	model_LUA.setName("LUA");
	model_LLA.setName("LLA");
	model_RUA.setName("RUA");
	model_RLA.setName("RLA");
	model_LUL.setName("LUL");
	model_LLL.setName("LLL");
	model_RUL.setName("RUL");
	model_RLL.setName("RLL");
	model_hat.setName("Hat");
	model_LF.setName("LF");
	model_RF.setName("RF");
	model_torus.setName("Torus");

	model_body.setDimension(1, 2, 1);
	model_head.setDimension(0.8, 0.8, 0.8);
	model_LUA.setDimension(0.5, 1, 0.8);
	model_LLA.setDimension(0.5, 1, 0.8);
	model_RUA.setDimension(0.5, 1, 0.8);
	model_RLA.setDimension(0.5, 1, 0.8);
	model_LUL.setDimension(0.45, 1, 0.8);
	model_LLL.setDimension(0.45, 1, 0.8);
	model_RUL.setDimension(0.45, 1, 0.8);
	model_RLL.setDimension(0.45, 1, 0.8);
	model_LF.setDimension(0.5, 0.25, 0.8);
	model_RF.setDimension(0.5, 0.25, 0.8);

	model_LUA.setOrigin(0.5, 0, 0);
	model_RUA.setOrigin(-0.5, 0, 0);
	model_LUL.setOrigin(-0.25, 0, 0);
	model_RUL.setOrigin(0.25, 0, 0);

	model_body.add(&model_head, 0);
	model_body.add(&model_LUA, 2);
	model_body.add(&model_RUA, 3);
	model_body.add(&model_LUL, 1);
	model_body.add(&model_RUL, 1);
	model_head.add(&model_hat, 0);
	model_LUA.add(&model_LLA, 0);
	model_RUA.add(&model_RLA, 0);
	model_LUL.add(&model_LLL, 0);
	model_RUL.add(&model_RLL, 0);
	model_LLL.add(&model_LF, 0);
	model_RLL.add(&model_RF, 0);
	model_head.add(&model_torus, 0);

	// control
	control_global_x.appendName("Global X");
	control_global_y.appendName("Global Y");
	control_global_z.appendName("Global Z");
	control_global_scale.appendName("Global Scale");
	control_multi_action.appendName("Mutli Part Action");
	control_multi_action.setLimit(0, 2);
	control_global_buffer[3] = 1;

	controls.push_back(&control_global_x);
	controls.push_back(&control_global_y);
	controls.push_back(&control_global_z);
	controls.push_back(&control_global_scale);
	controls.push_back(&control_multi_action);

	// Feature:
	// can automatically add control widget
	// but as it grab all the control widget, it wil be too messy and should only be used for debug
	//
	model_body.control(&controls);

	control_size = (int)controls.size();
	control_table = new ModelerControl[control_size];
	
	int i = 0;
	for (ModelControl *control : controls) {
		control_table[i].SetVals(
			control->getName()->c_str(),
			control->getLimit_min(),
			control->getLimit_max(), 
			(control->getLimit_max() - control->getLimit_max()) / 100,
			control->getValue());
		i++;
	}

	// mainloop
    ModelerApplication::Instance()->Init(&createSampleModel, control_table, control_size);
	ModelerApplication::Instance()->setExtCallback_slider(callback_valChanged);
	return ModelerApplication::Instance()->Run();
}


// Static Function Implementation
static void callback_valChanged() {
	for (int i = 0; i < control_size; i++) {
		controls[i]->setValue(ModelerApplication::Instance()->GetControlValue(i));
	}

	// for mutli action
	// hat and torus
	GLdouble* origin_hat = model_hat.getOrigin();
	GLdouble* origin_torus = model_torus.getOrigin();
	model_hat.setOrigin(origin_hat[0], control_multi_buffer[0], origin_hat[2]);
	model_torus.setOrigin(origin_torus[0], control_multi_buffer[0] * 2, origin_torus[2]);
}
