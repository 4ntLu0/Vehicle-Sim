// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.
// =============================================================================


//system includes
#include <memory>
#include <iostream>

//chrono includes
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;

// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

class SimpleCar {
public:
	double throttle;
	double conic_tau;
	double gar_tau;
	double max_motor_torque;
	double max_motor_speed;

	// .. chassis:
	std::shared_ptr<ChBody> chassis;
	// .. right front suspension:
	std::shared_ptr<ChBody> spindleRF;
	std::shared_ptr<ChBody> wheelRF;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteRF;
	std::shared_ptr<ChLinkDistance> link_distRFU1;
	std::shared_ptr<ChLinkDistance> link_distRFU2;
	std::shared_ptr<ChLinkDistance> link_distRFL1;
	std::shared_ptr<ChLinkDistance> link_distRFL2;
	std::shared_ptr<ChLinkDistance> link_distRSTEER;
	std::shared_ptr<ChLinkTSDA> link_springRF;
	// .. left front suspension:
	std::shared_ptr<ChBody> spindleLF;
	std::shared_ptr<ChBody> wheelLF;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteLF;
	std::shared_ptr<ChLinkDistance> link_distLFU1;
	std::shared_ptr<ChLinkDistance> link_distLFU2;
	std::shared_ptr<ChLinkDistance> link_distLFL1;
	std::shared_ptr<ChLinkDistance> link_distLFL2;
	std::shared_ptr<ChLinkDistance> link_distLSTEER;
	std::shared_ptr<ChLinkTSDA> link_springLF;
	// .. right back suspension:
	std::shared_ptr<ChBody> spindleRB;
	std::shared_ptr<ChBody> wheelRB;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteRB;
	std::shared_ptr<ChLinkDistance> link_distRBU1;
	std::shared_ptr<ChLinkDistance> link_distRBU2;
	std::shared_ptr<ChLinkDistance> link_distRBL1;
	std::shared_ptr<ChLinkDistance> link_distRBL2;
	std::shared_ptr<ChLinkDistance> link_distRBlat;
	std::shared_ptr<ChLinkTSDA> link_springRB;
	std::shared_ptr<ChLinkMotorRotationTorque> link_motorL;
	// .. left back suspension:
	std::shared_ptr<ChBody> spindleLB;
	std::shared_ptr<ChBody> wheelLB;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteLB;
	std::shared_ptr<ChLinkDistance> link_distLBU1;
	std::shared_ptr<ChLinkDistance> link_distLBU2;
	std::shared_ptr<ChLinkDistance> link_distLBL1;
	std::shared_ptr<ChLinkDistance> link_distLBL2;
	std::shared_ptr<ChLinkDistance> link_distLBlat;
	std::shared_ptr<ChLinkTSDA> link_springLB;
	std::shared_ptr<ChLinkMotorRotationTorque> link_motorR;

	SimpleCar(ChSystemNSC &my_system, ISceneManager *msceneManager, IVideoDriver *mdriver) {
		throttle = 0;
		conic_tau = 0.2;
		gear_tau = 0.3;
		max_motor_torque = 80;
		max_motor_speed = 800;

		auto texture = chrono_types::make_shared<ChTexture>();
		texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
	}
};

/* ignore this main function - we will write our own.
int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"A simple project template", core::dimension2d<u32>(800, 600),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(2, 2, -5),
                                 core::vector3df(0, 1, 0));  // to change the position of camera
    application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    //======================================================================

    // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
    //
    // An example: a pendulum.

    // 1-Create a floor that is fixed (that is used also to represent the absolute reference)

    auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
                                                     3000,       // density
                                                     false,      // no contact geometry
                                                     true        // enable visualization geometry
                                                     );
    floorBody->SetPos(ChVector<>(0, -2, 0));
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

    // 2-Create a pendulum

    auto pendulumBody = std::make_shared<ChBodyEasyBox>(0.5, 20, 0.5,  // x, y, z dimensions
                                                        3000,         // density
                                                        false,        // no contact geometry
                                                        true          // enable visualization geometry
                                                        );
    pendulumBody->SetPos(ChVector<>(0, 3, 0));
    pendulumBody->SetPos_dt(ChVector<>(1, 0, 0));

    mphysicalSystem.Add(pendulumBody);

    // 3-Create a spherical constraint.
    //   Here we'll use a ChLinkMateGeneric, but we could also use ChLinkLockSpherical

    auto sphericalLink =
        std::make_shared<ChLinkMateGeneric>(true, true, true, false, false, false);  // x,y,z,Rx,Ry,Rz constrains
    ChFrame<> link_position_abs(ChVector<>(0, 4, 0));

    sphericalLink->Initialize(pendulumBody,        // the 1st body to connect
                              floorBody,           // the 2nd body to connect
                              false,               // the two following frames are in absolute, not relative, coords.
                              link_position_abs,   // the link reference attached to 1st body
                              link_position_abs);  // the link reference attached to 2nd body

    mphysicalSystem.Add(sphericalLink);

    // Optionally, attach a RGB color asset to the floor, for better visualization
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.52f, 0.25f, 0.25f));
    floorBody->AddAsset(color);

    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));  // texture in ../data
    pendulumBody->AddAsset(texture);

    //======================================================================

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(0.005);
    application.SetTryRealtime(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        application.DoStep();

        application.EndScene();
    }

    return 0;
}
*/