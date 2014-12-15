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
#include <iostream>
#include <math.h>

/** LCM engine */
#include "includes/lcm/lcmthread.h"

/** NAVIGATION AND AVOIDING COLLISION */
/* Navigations agents */
#include "includes/navigation/Agent.h"
#include "includes/navigation/ORCAAgent.h"
#include "includes/navigation/HRVOAgent.h"
#include "includes/navigation/HLAgent.h"
/* Additional sensors*/
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
/* Obstacle radius */
#define OBSTACLE_RADIUS 0.12
/* State of the robot*/
enum {
	MOVING, ARRIVED_AT_TARGET, TURN
} state;

/*  */
#define ODOMETRY_CORRECTION_FACTOR_RIGHT 1
#define ODOMETRY_CORRECTION_FACTOR_LEFT 1

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotAN: public CCI_Controller {

public:

	/* Class constructor. */
	CFootBotAN();

	/* Class destructor. */
	virtual ~CFootBotAN() {
	}

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML file
	 * in the <controllers><footbot_diffusion_controller> section.
	 */
	virtual void Init(TConfigurationNode& t_node);

	/** NAVIGATION AND AVOIDING COLLISION */
	/* Navigation agents */
	void setAgent(Agent &a);
	void updateAgent(Agent *a);

	/* Local navigation */
	void initLocalNavigation(TConfigurationNode& t_tree);
	void initGlobalNavigation(TConfigurationNode& t_tree);
	void initOdometry();

	/* Obstacles */
	void addNodesAsObstacles(map<UInt8, Node> listNodeObstacles);

	/* Add fixed obstacles in a square or rectangular way */
	std::vector<CVector2> addAreaBounds(CVector2 bottomLeftCoord, CVector2 upperRightCoord);
	void setFixedObstacles(std::vector<CVector2> obstaclesPoints);

	/* Update navigation */
	void updateNavigation();
	void updateDesideredVelocity();
	void updateVelocity();

	/** PROXIMITY SENSORS */
	void updateVelocityProximitySensors();

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

	/* Pointer to the foot-bot wheels actuator */
	CCI_FootBotWheelsActuator* m_pcWheels;
	/* Pointer to the foot-bot LEDs actuator */
	CCI_FootBotLedsActuator* m_pcLEDs;
	/* Pointer to the foot-bot proximity sensor */
	CCI_FootBotProximitySensor* m_pcProximity;

	CSpace& m_cSpace;
	UInt32 numberOfRobots;
	std::string str_Me;
	std::string str_Dest;
	bool amIaGenerator;
	bool skipFirstSend;
	UInt16 m_iSendOrNotSend;
	CCI_WiFiSensor* m_pcWifiSensor;
	CCI_WiFiActuator* m_pcWifiActuator;
	static UInt32 sendPackets;
	static UInt32 receivedPackets;

	/*
	 * The following variables are used as parameters for the
	 * algorithm. You can set their value in the <parameters> section
	 * of the XML configuration file, under the
	 * <controllers><footbot_diffusion_controller> section.
	 */

	/* Maximum tolerance for the angle between
	 * the robot heading direction and
	 * the closest obstacle detected. */
	CDegrees m_cAlpha;
	/* Maximum tolerance for the proximity reading between
	 * the robot and the closest obstacle.
	 * The proximity reading is 0 when nothing is detected
	 * and grows exponentially to 1 when the obstacle is
	 * touching the robot.
	 */
	Real m_fDelta;
	/* Wheel speed. */
	Real m_fWheelVelocity;
	/* Angle tolerance range to go straight.
	 * It is set to [-alpha,alpha]. */
	CRange<CRadians> m_cGoStraightAngleRange;

	/* Communication Parameters */
	Real m_numberOfGenerators;		// There are numberOfGenerators*numberOfRobots generators, where  0.0 < numberOfGenerators < 1.0
	UInt16 m_generatePacketInterval;	// Send a packet every m_generatePacketInterval simulation steps

	/** LCM engine */
	// LCM thread
	LCMThread lcmThread;
	// LCM thread command channel
	LCMThread lcmThreadCommand;
	// List of node obstacles retrieved by LCM-tracking system. The other nodes are an obstacles for me.
	map<UInt8, Node> listNodeObstacles;
	// Myself Node
	Node mySelf;

	/** NAVIGATION AND AVOIDING COLLISION */
	/* Additional sensors */
	CCI_FootBotEncoderSensor* encoderSensor;

	/* Navigation agents */
	Agent * agent;
	HLAgent hlAgent;
	ORCAAgent orcaAgent;
	HRVOAgent hrvoAgent;

	/* Local navigation */
	std::string localNavigationType;
	int localNavigationIndex;

	/* Mobility parameters */
	Real axisLength;
	CVector2 position;
	CVector2 velocity;
	CRadians angle;
	CRadians angularSpeed;
	CVector2 deltaPosition;
	Real speed;
	Real m_targetMinPointDistance;

	/* Target -> next (x,y) point */
	CVector2 targetPosition;

	// Area bounds as fixed obstacles
	CVector2 originAreaCoord;
	CVector2 destinationAreaCoord;

	Real originAreaX;
	Real originAreaY;
	Real destinationAreaX;
	Real destinationAreaY;


};

#endif
