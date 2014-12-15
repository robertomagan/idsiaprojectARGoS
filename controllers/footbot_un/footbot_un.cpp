/* Include the controller definition */
#include "footbot_un.h"

#include <argos2/common/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos2/common/utility/math/vector2.h>

#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/entities/footbot_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>

#include <string>

// static emmbers initialization
UInt32 CFootBotUN::sendPackets = 0;
UInt32 CFootBotUN::receivedPackets = 0;

/****************************************/
/****************************************/

CFootBotUN::CFootBotUN() :
		m_pcWheels(NULL), m_pcLEDs(NULL), m_pcProximity(NULL), m_cAlpha(10.0f), m_fDelta(0.5f), m_fWheelVelocity(2.5f), m_targetMinPointDistance(0.12f), m_numberOfGenerators(0.2f), m_generatePacketInterval(
				10), m_cSpace(CSimulator::GetInstance().GetSpace()), m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {
}

/****************************************/
/****************************************/

void CFootBotUN::Init(TConfigurationNode& t_node) {
	/*
	 * Get sensor/actuator handles
	 *
	 * The passed string (ex. "footbot_wheels") corresponds to the XML tag of the device
	 * whose handle we want to have. For a list of allowed values, type at the command
	 * prompt:
	 *
	 * $ launch_argos -q actuators
	 *
	 * to have a list of all the possible actuators, or
	 *
	 * $ launch_argos -q sensors
	 *
	 * to have a list of all the possible sensors.
	 *
	 * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
	 *       the lists provided the XML file at the <controllers><footbot_diffusion><actuators>
	 *       and <controllers><footbot_diffusion><sensors> sections. If you forgot to
	 *       list a device in the XML and then you request it here, an error occurs.
	 */
	m_pcWheels = dynamic_cast<CCI_FootBotWheelsActuator*>(GetRobot().GetActuator("footbot_wheels"));
	m_pcLEDs = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
	m_pcProximity = dynamic_cast<CCI_FootBotProximitySensor*>(GetRobot().GetSensor("footbot_proximity"));

	//Min distance to the current position to the target point to determine it this is reached (meters)
	//GetNodeAttributeOrDefault(t_node, "targetMinPointDistance", m_targetMinPointDistance, m_targetMinPointDistance);
	std::string text;

	if (NodeExists(t_node, "targetMinPointDistance")) {
		GetNodeText(GetNode(t_node, "targetMinPointDistance"), text);
		sscanf(text.c_str(), "%f", &m_targetMinPointDistance);
		//m_targetMinPointDistance = fmin(MAX_RESOLUTION, m_targetMinPointDistance);
	}

	printf("Min distance to target point %f in robot %s\n", m_targetMinPointDistance, GetRobot().GetRobotId().c_str());

//////////////////////////////////////////////////////////////
// Initialize things required by the communications
//////////////////////////////////////////////////////////////
	m_pcWifiSensor = dynamic_cast<CCI_WiFiSensor*>(GetRobot().GetSensor("wifi"));
	m_pcWifiActuator = dynamic_cast<CCI_WiFiActuator*>(GetRobot().GetActuator("wifi"));
	str_Me = GetRobot().GetRobotId();
	m_iSendOrNotSend = 0;

	// How many robots are there ?
	CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType("wifi_equipped_entity");
	numberOfRobots = tEntityMap.size();
	printf("%s: There are %d robots [RMAGAN]\n", str_Me.c_str(), numberOfRobots);

	// Am I a generator?
	amIaGenerator = false;
	skipFirstSend = true;
	int numGenerators = m_numberOfGenerators * numberOfRobots;
	int myID = atoi(str_Me.c_str() + 3);
	if (myID < numGenerators)
		amIaGenerator = true;
	if (amIaGenerator)
		printf("%s: There are %d generators, I am on of them\n", str_Me.c_str(), numGenerators);
	else
		printf("%s: There are %d generators, but not me :-(\n", str_Me.c_str(), numGenerators);

	if (amIaGenerator) {
		UInt32 id = CSimulator::GetInstance().GetRNG()->Uniform(CRange<UInt32>(numGenerators, numberOfRobots));
		//UInt32 id = myID + numGenerators;
		std::ostringstream str_tmp(ostringstream::out);
		str_tmp << "fb_" << id;
		str_Dest = str_tmp.str();
		printf(" (dest=%s).\n", str_Dest.c_str());
	}

	//////////////////////////////////////////////////////////////
	// end of communications things here
	//////////////////////////////////////////////////////////////

	/** LCM engine */
	// LCM Thread
	lcmThreadCommand.setLCMEngine("udpm://239.255.76.67:7667?ttl=1", "TARGET");
	lcmThreadCommand.startInternalThread();

	lcmThread.setLCMEngine("udpm://239.255.76.67:7667?ttl=1", "TRACK");
	lcmThread.startInternalThread();
	/** NAVIGATION AND AVOIDING COLLISION */
	/* Additional sensors */
	encoderSensor = dynamic_cast<CCI_FootBotEncoderSensor*>(GetRobot().GetSensor("footbot_encoder"));

	/* Init navigation methods */
	initOdometry();
	initLocalNavigation(t_node);

}

/****************************************/
/****************************************/

void CFootBotUN::ControlStep() {

	printf("\n --------- UN %s - Simulation step %d -----------\n", GetRobot().GetRobotId().c_str(), CSimulator::GetInstance().GetSpace().GetSimulationClock());

	m_pcLEDs->SetAllColors(CColor::BLUE);

	/** Wifi communications section */
	std::ostringstream str(ostringstream::out);
	if (amIaGenerator) {
		if (m_iSendOrNotSend % m_generatePacketInterval == 0) {
			if (skipFirstSend) {
				skipFirstSend = false;
			} else {
				sendPackets++;
				str << "Hi I'm " << str_Me << " and I say \"Hello " << str_Dest << "\"";
				str << ",Hi I'm " << str_Me << " and I say \"Hello " << str_Dest << "\"";
				m_pcWifiActuator->SendMessageTo(str_Dest, str.str());
				//std::cout << str_Me << " sending\n";
			}
		}
	}

	m_iSendOrNotSend++;

	//searching for the received msgs
	TMessageList t_incomingMsgs;
	m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
	for (TMessageList::iterator it = t_incomingMsgs.begin(); it != t_incomingMsgs.end(); it++) {
		receivedPackets++;
		std::cout << str_Me << " received: " << it->Payload << " from " << it->Sender << " (total received=)" << receivedPackets << std::endl;
	}
	/** End of wifi*/

	/** LCM for getting current positions and obtaining the next target point */
	UInt8 robotID = atoi(GetRobot().GetRobotId().substr(3).c_str());

	/* New target point from the COMMAND engine*/
	if (lcmThreadCommand.getLcmHandler()->existNode(robotID)) {
		Node nodeCommand = lcmThreadCommand.getLcmHandler()->getNodeById(robotID);
		targetPosition.Set(nodeCommand.getPosition().GetX(), nodeCommand.getPosition().GetY());
		printf("ID %d - TARGET: (%f,%f)\n", robotID, nodeCommand.getPosition().GetX(), nodeCommand.getPosition().GetY());
	} else {
		if (lcmThread.getLcmHandler()->existNode(robotID)) {
			Node nodeCommand = lcmThread.getLcmHandler()->getNodeById(robotID);
			targetPosition.Set(nodeCommand.getPosition().GetX(), nodeCommand.getPosition().GetY());
			printf("ID %d - TARGET: (%f,%f)\n", robotID, nodeCommand.getPosition().GetX(), nodeCommand.getPosition().GetY());
		}
	}

	if (lcmThread.getLcmHandler()->existNode(robotID)) {

		//lcmThread.getLcmHandler()->printNodeListElements();

		/** LCM related Node information */
		//To get the other nodes locations through LCM
		listNodeObstacles = lcmThread.getLcmHandler()->retrieveNodeList();

		//Get myself
		mySelf = lcmThread.getLcmHandler()->getNodeById(robotID);

		printf("ID %d\n", mySelf.getId());
		printf("POS (%f,%f)\n", mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
		printf("QUAT (%f,%f,%f,%f)\n", mySelf.getOrientation().GetW(), mySelf.getOrientation().GetX(), mySelf.getOrientation().GetY(), mySelf.getOrientation().GetZ());
		printf("VEL %d\n", mySelf.getVelocity());

		/** From the simulator */
		CVector2 oldPosition = position;
		CSpace& space = CSimulator::GetInstance().GetSpace();
		const std::string stringIdA = GetRobot().GetRobotId();
		CFootBotEntity *robotEntityA = static_cast<CFootBotEntity*>(&space.GetEntity(stringIdA));
		CVector3 cFootBotPositionA = robotEntityA->GetEmbodiedEntity().GetPosition();
		CQuaternion cFootBotOrientationA = robotEntityA->GetEmbodiedEntity().GetOrientation();

		CVector3 axis;
		CRadians oA;
		cFootBotOrientationA.ToAngleAxis(oA, axis);
		if (axis.GetZ() < 0)
			oA = -oA;

		// Position
		position = CVector2(cFootBotPositionA.GetX(), cFootBotPositionA.GetY());
		// Angle
		angle = oA;

		// Velocity
		//velocity = CVector2(speed, 0);
		velocity = CVector2(mySelf.getVelocity(), 0);

		/** NAVIGATION AND AVOIDING COLLISION */

		updateAgent(agent);

		agent->clearObstacles();

		addNodesAsObstacles(listNodeObstacles);

		if ((targetPosition - position).Length() < m_targetMinPointDistance) {	// 2 cm
			state = ARRIVED_AT_TARGET;
			printf("State: STOPPED\n");
		} else {
			state = MOVING;
			printf("State: MOVING\n");
		}

		updateNavigation();

	}

}

void CFootBotUN::initLocalNavigation(TConfigurationNode& t_tree) {
	hlAgent.Init(t_tree);
	hlAgent.axisLength = axisLength;
	orcaAgent.Init(t_tree);
	orcaAgent.axisLength = axisLength;
	hrvoAgent.Init(t_tree);
	hrvoAgent.axisLength = axisLength;

	localNavigationType = "HL";

	if (NodeExists(t_tree, "local_navigation")) {
		TConfigurationNode node = GetNode(t_tree, "local_navigation");
		if (node.HasAttribute("type"))
			GetNodeAttribute(node, "type", localNavigationType);
	}

	if (localNavigationType == "HL") {
		localNavigationIndex = 2;
		setAgent(hlAgent);
	} else if (localNavigationType == "ORCA") {
		localNavigationIndex = 0;
		setAgent(orcaAgent);
	} else if (localNavigationType == "HRVO") {
		localNavigationIndex = 0;
		setAgent(hrvoAgent);
	} else {
		throw "Navigation type not defined!";
		return;
	}

	printf("INIT local nav %.3f %.3f", axisLength, agent->axisLength);

}

void CFootBotUN::initOdometry() {
	position = CVector2(0, 0);
	angle = CRadians(0);
	velocity = CVector2(0, 0);
	axisLength = encoderSensor->GetReading().WheelAxisLength * 0.01;

	printf("INIT axis length %.3f", axisLength);

}

void CFootBotUN::setAgent(Agent &a) {
	if (agent == &a)
		return;
	agent = &a;
}

void CFootBotUN::updateAgent(Agent *a) {
	a->position = position;
	a->velocity = velocity;
	a->angularSpeed = angularSpeed;
	//a->leftWheelSpeed=leftWheelSpeed;
	//a->rightWheelSpeed=rightWheelSpeed;
	a->angle = angle;
}

void CFootBotUN::Destroy() {
	if (str_Me == "an") {
		CSimulator& sim = CSimulator::GetInstance();
		//sim.sendPackets = sendPackets;
		//sim.receivedPackets = receivedPackets;
		//sim.avgDelay = 0.0;
		std::cout << str_Me << " DESTROY: received=" << receivedPackets << " out of sended=" << sendPackets << std::endl;
	}
}

void CFootBotUN::addNodesAsObstacles(map<UInt8, Node> listNodeObstacles) {

	//Aux variables
	CVector2 auxPosition;
	CVector2 auxVelocity;

	//Create a list of Nodes
	for (map<UInt8, Node>::iterator it = listNodeObstacles.begin(); it != listNodeObstacles.end(); it++) {

		//printf("Node %d\n", (it->second).getId());

		// I'm not and obstacle for myself!
		if ((it->second).getId() != mySelf.getId()) {

			//printf("Agent \n");
			auxPosition.Set((it->second).getPosition().GetX(), (it->second).getPosition().GetY());
			auxVelocity.Set((it->second).getVelocity(), 0);

			// For a dynamic obstacle we need to use the addObstacleAtPoint(position,velocity,radius)
			//agent->addObstacleAtPoint(auxPosition, OBSTACLE_RADIUS);
			agent->addObstacleAtPoint(auxPosition, auxVelocity, OBSTACLE_RADIUS);

			printf("Adding obstacle : %d\n", (it->second).getId());
		}

	}

}

void CFootBotUN::updateNavigation() {
	printf("UPDATE Navigation with state %d\n", state);

	if (state == ARRIVED_AT_TARGET) {
		agent->desideredAngle = CRadians::ZERO;
		agent->desideredSpeed = 0;
		agent->desideredVelocity = CVector2(0, 0);
	} else {
		updateDesideredVelocity();
	}
	agent->updateVelocity();
	updateVelocity();
}

void CFootBotUN::updateDesideredVelocity() {

	CVector2 agentToTarget = targetPosition - position;

	CRadians a0 = agentToTarget.Angle() - angle;

	DEBUG_CONTROLLER("updateDesideredVelocity to %.2f, state = %d \r\n",a0.GetValue(),state);

	agent->targetPosition = targetPosition;
	agent->updateDesideredVelocity();

	printf("=> desidered speed %.2f, desidered angle %.2f \n", agent->desideredSpeed, agent->desideredAngle.GetValue());
}

void CFootBotUN::updateVelocity() {

	printf("%p: target wheel speed (%.3f %.3f) r %.3f\n", agent, agent->leftWheelTargetSpeed, agent->rightWheelTargetSpeed, agent->radius);

	m_pcWheels->SetLinearVelocity(100 * agent->leftWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_LEFT, 100 * agent->rightWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_RIGHT);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotUN, "footbot_un_controller")
