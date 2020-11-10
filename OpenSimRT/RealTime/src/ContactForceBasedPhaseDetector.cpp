#include "ContactForceBasedPhaseDetector.h"

#include "GRFMPrediction.h"

#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <SimTKcommon/internal/Stage.h>
#include <Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(
        const Model& otherModel,
        const GRFMPrediction::Parameters& otherParameters)
        : GaitPhaseDetector(otherModel, otherParameters) {

    // add elements to a copy of the original model....

    // platform
    auto platform = new OpenSim::Body("Platform", 1.0, Vec3(0), Inertia(0));
    model.addBody(platform);

    // weld joint
    auto platformToGround = new WeldJoint(
            "PlatformToGround", model.getGround(), Vec3(0), Vec3(0), *platform,
            -parameters.contact_plane_origin, Vec3(0));
    model.addJoint(platformToGround);

    // contact half-space
    auto platformContact = new ContactHalfSpace();
    platformContact->setName("PlatformContact");
    platformContact->set_location(Vec3(0));
    platformContact->set_orientation(Vec3(0.0, 0.0, -Pi / 2.0));
    platformContact->setFrame(*platform);
    model.addContactGeometry(platformContact);

    // contact spheres
    auto rightHeelContact = new ContactSphere();
    auto leftHeelContact = new ContactSphere();
    auto rightToeContact = new ContactSphere();
    auto leftToeContact = new ContactSphere();
    rightHeelContact->set_location(Vec3(0.012, -0.0015, -0.005));
    leftHeelContact->set_location(Vec3(0.012, -0.0015, 0.005));
    rightToeContact->set_location(Vec3(0.055, 0.01, -0.01));
    leftToeContact->set_location(Vec3(0.055, 0.01, 0.01));
    rightHeelContact->setFrame(model.getBodySet().get("calcn_r"));
    leftHeelContact->setFrame(model.getBodySet().get("calcn_l"));
    rightToeContact->setFrame(model.getBodySet().get("toes_r"));
    leftToeContact->setFrame(model.getBodySet().get("toes_l"));
    rightHeelContact->setRadius(0.01);
    leftHeelContact->setRadius(0.01);
    rightToeContact->setRadius(0.01);
    leftToeContact->setRadius(0.01);
    rightHeelContact->setName("RHeelContact");
    leftHeelContact->setName("LHeelContact");
    rightToeContact->setName("RToeContact");
    leftToeContact->setName("LToeContact");
    model.addContactGeometry(rightHeelContact);
    model.addContactGeometry(leftHeelContact);
    model.addContactGeometry(rightToeContact);
    model.addContactGeometry(leftToeContact);

    // contact parameters
    double stiffness = 2e6;
    double dissipation = 1.0;
    double staticFriction = 0.9;
    double dynamicFriction = 0.8;
    double viscousFriction = 0.6;

    auto rightContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            stiffness, dissipation, staticFriction, dynamicFriction,
            viscousFriction);
    auto leftContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            stiffness, dissipation, staticFriction, dynamicFriction,
            viscousFriction);

    rightContactParams->addGeometry("PlatformContact");
    rightContactParams->addGeometry("RHeelContact");
    rightContactParams->addGeometry("RToeContact");
    leftContactParams->addGeometry("PlatformContact");
    leftContactParams->addGeometry("LHeelContact");
    leftContactParams->addGeometry("LToeContact");

    // contact forces
    rightContactForce = new OpenSim::HuntCrossleyForce(rightContactParams);
    leftContactForce = new OpenSim::HuntCrossleyForce(leftContactParams);
    rightContactForce->setName("RightContactForce");
    leftContactForce->setName("LeftContactForce");
    model.addForce(rightContactForce.get());
    model.addForce(leftContactForce.get());

    // initialize system
    state = model.initSystem();
}

void ContactForceBasedPhaseDetector::updDetector(
        const GRFMPrediction::Input& input) {
    updateState(input, model, state, Stage::Dynamics);

    // compute contact forces
    auto rightContactWrench = rightContactForce.get()->getRecordValues(state);
    Vec3 rightContactForce(-rightContactWrench[0], -rightContactWrench[1],
                           -rightContactWrench[2]);

    auto leftontactWrench = leftContactForce.get()->getRecordValues(state);
    Vec3 leftContactForce(-leftontactWrench[0], -leftontactWrench[1],
                          -leftontactWrench[2]);

    // update detector internal state
    updDetectorState(state.getTime(),
                     rightContactForce.norm() - parameters.threshold,
                     leftContactForce.norm() - parameters.threshold);
}
