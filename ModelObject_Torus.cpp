#include "ModelObject_Torus.h"
#include <cmath>
#include <math.h>


// Static Function Prototype
static void torus(int c, int t, GLdouble r_1, GLdouble r_2);


// Operation
ModelObject_Torus::ModelObject_Torus() :
	ModelObject::ModelObject()
{
}


void ModelObject_Torus::modelSelf() {
    glRotated(90, 1, 0, 0);
    torus((int)torus_c, (int)torus_t, torus_r_1, torus_r_2);
}


void ModelObject_Torus::controlSelf(std::vector<ModelControl*>* controls) {
    ModelControl* control_0 = new ModelControl(&torus_c, 1, 50);
    ModelControl* control_1 = new ModelControl(&torus_t, 1, 50);
    ModelControl* control_2 = new ModelControl(&torus_r_1, 0.1, 10);
    ModelControl* control_3 = new ModelControl(&torus_r_2, 0.1, 10);

    control_0->appendName(name);
    control_1->appendName(name);
    control_2->appendName(name);
    control_3->appendName(name);

    control_0->appendName(": Torus-C");
    control_1->appendName(": Torus-T");
    control_2->appendName(": Torus-R1");
    control_3->appendName(": Torus-R2");

    controls->push_back(control_0);
    controls->push_back(control_1);
    controls->push_back(control_2);
    controls->push_back(control_3);
}


// Static Function Implementation
static void torus(int numc, int numt, GLdouble r_1, GLdouble r_2) {
    int i, j, k;
    double s, t, x, y, z, twopi;

    twopi = 2 * (double)3.14159265358979353;
    for (i = 0; i < numc; i++) {
        glBegin(GL_QUAD_STRIP);
        for (j = 0; j <= numt; j++) {
            for (k = 1; k >= 0; k--) {
                s = (i + k) % numc + 0.5;
                t = j % numt;

                x = (r_1 + r_2 * cos(s * twopi / numc)) * cos(t * twopi / numt);
                y = (r_1 + r_2 * cos(s * twopi / numc)) * sin(t * twopi / numt);
                z = r_2 * sin(s * twopi / numc);
                glVertex3f(x, y, z);
            }
        }
        glEnd();
    }
}
