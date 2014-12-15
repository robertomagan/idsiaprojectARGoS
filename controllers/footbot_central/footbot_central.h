/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around. The LEDs are used to visually show to
 * the user the angle where closest obstacle was detected.
 *
 * This controller is meant to be used with the XML files:
 *    xml/diffusion_1.xml
 *    xml/diffusion_10.xml
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos2/common/control_interface/ci_controller.h>
/* Definition of the foot-bot wheel actuator */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
/* Definition of the foot-bot LEDs actuator */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

#include <argos2/common/control_interface/ci_wifi_sensor.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/common/utility/math/vector2.h>
#include <iostream>
#include "includes/lcm/lcmthread.h"
#include <list>
#include <vector>
#include <sys/time.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotCentral: public CCI_Controller {

public:

	/* Class constructor. */
	CFootBotCentral();

	/* Class destructor. */
	virtual ~CFootBotCentral() {
	}

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML file
	 * in the <controllers><footbot_diffusion_controller> section.
	 */
	virtual void Init(TConfigurationNode& t_node);

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	virtual void ControlStep();

	/*
	 * This function resets the controller to its state right after the Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything, so
	 * the function could have been omitted. It's here just for completeness.
	 */
	virtual void Reset() {
	}

	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 * In this example controller there is no need for clean anything up, so
	 * the function could have been omitted. It's here just for completeness.
	 */
	virtual void Destroy();

private:

	CSpace& m_cSpace;

	/* LCM engine */
	LCMHandler * lcmHandler;
	map<UInt8, Node> m_prevListNodes;


};

#endif
