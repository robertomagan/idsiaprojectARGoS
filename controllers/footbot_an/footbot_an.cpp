/* Include the controller definition */
#include "footbot_an.h"
/* Function definitions for XML parsing */
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
#include <math.h>

// static emmbers initialization
UInt32 CFootBotAN::sendPackets = 0;
UInt32 CFootBotAN::receivedPackets = 0;

/****************************************/
/****************************************/

CFootBotAN::CFootBotAN() :
		m_pcWheels(NULL), m_pcLEDs(NULL), m_pcProximity(NULL), m_cAlpha(10.0f), m_fDelta(0.001f), m_fWheelVelocity(5.0f), m_targetMinPointDistance(0.12f), m_numberOfGenerators(0.2f), m_generatePacketInterval(
				10), m_cSpace(CSimulator::GetInstance().GetSpace()), m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {
}

/****************************************/
/****************************************/

void CFootBotAN::Init(TConfigurationNode& t_node) {
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

//	/*
//	 * Parse the XML
//	 *
//	 * The user defines this part. Here, the algorithm accepts three parameters and it's nice to
//	 * put them in the XML so we don't have to recompile if we want to try other settings.
//	 */
	GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
	m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
	GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
//	GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
	GetNodeAttributeOrDefault(t_node, "generators", m_numberOfGenerators, m_numberOfGenerators);
	//GetNodeAttributeOrDefault(t_node, "targetMinPointDistance", m_targetMinPointDistance, m_targetMinPointDistance);

	std::string text;

	if (NodeExists(t_node, "targetMinPointDistance")) {
		GetNodeText(GetNode(t_node, "targetMinPointDistance"), text);
		sscanf(text.c_str(), "%f", &m_targetMinPointDistance);
		//m_targetMinPointDistance = fmin(MAX_RESOLUTION, m_targetMinPointDistance);
	}

	printf("\nMin distance to target point %f in robot %s\n", m_targetMinPointDistance, GetRobot().GetRobotId().c_str());

	if (NodeExists(t_node, "originAreaX")) {
		GetNodeText(GetNode(t_node, "originAreaX"), text);
		sscanf(text.c_str(), "%f", &originAreaX);
	}

	if (NodeExists(t_node, "originAreaY")) {
		GetNodeText(GetNode(t_node, "originAreaY"), text);
		sscanf(text.c_str(), "%f", &originAreaY);
	}

	if (NodeExists(t_node, "destinationAreaX")) {
		GetNodeText(GetNode(t_node, "destinationAreaX"), text);
		sscanf(text.c_str(), "%f", &destinationAreaX);
	}

	if (NodeExists(t_node, "destinationAreaY")) {
		GetNodeText(GetNode(t_node, "destinationAreaY"), text);
		sscanf(text.c_str(), "%f", &destinationAreaY);
	}

	printf("Area bounds-> ORIGIN (%f,%f), DESTINATION (%f,%f)\n", originAreaX, originAreaY, destinationAreaX, destinationAreaY);
	originAreaCoord.SetX(originAreaX);
	originAreaCoord.SetY(originAreaY);
	destinationAreaCoord.SetX(destinationAreaX);
	destinationAreaCoord.SetY(destinationAreaY);

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

void CFootBotAN::ControlStep() {

	printf("\n --------- AN %s - Simulation step %d -----------\n", GetRobot().GetRobotId().c_str(), CSimulator::GetInstance().GetSpace().GetSimulationClock());

	m_pcLEDs->SetAllColors(CColor::RED);

	/** Wifi communication section */
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

	/** End of wifi */

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

	/*  */
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

		if ((targetPosition - position).Length() < m_targetMinPointDistance) {
			state = ARRIVED_AT_TARGET;
			printf("State: STOPPED\n");
		} else {
			state = MOVING;
			printf("State: MOVING\n");
		}

		updateNavigation();

		/** END OF NAVIGATION */

	}

	/** PROXIMITY SENSORS  */
	//updateVelocityProximitySensors();

}

void CFootBotAN::initLocalNavigation(TConfigurationNode& t_tree) {
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

	/* Init fixed obstacles */
	//Does not work here :(
	//agent->clearObstacles();
	//setFixedObstacles(addAreaBounds(originAreaCoord, destinationAreaCoord));

	printf("INIT local nav %.3f %.3f", axisLength, agent->axisLength);

}

void CFootBotAN::initOdometry() {
	position = CVector2(0, 0);
	angle = CRadians(0);
	velocity = CVector2(0, 0);
	axisLength = encoderSensor->GetReading().WheelAxisLength * 0.01;

	printf("INIT axis length %.3f", axisLength);

}

void CFootBotAN::setAgent(Agent &a) {
	if (agent == &a)
		return;
	agent = &a;
}

void CFootBotAN::updateAgent(Agent *a) {
	a->position = position;
	a->velocity = velocity;
	a->angularSpeed = angularSpeed;
	//a->leftWheelSpeed=leftWheelSpeed;
	//a->rightWheelSpeed=rightWheelSpeed;
	a->angle = angle;
}

void CFootBotAN::Destroy() {
	if (str_Me == "an") {
		CSimulator& sim = CSimulator::GetInstance();
		//sim.sendPackets = sendPackets;
		//sim.receivedPackets = receivedPackets;
		//sim.avgDelay = 0.0;
		std::cout << str_Me << " DESTROY: received=" << receivedPackets << " out of sended=" << sendPackets << std::endl;
	}
}

void CFootBotAN::addNodesAsObstacles(map<UInt8, Node> listNodeObstacles) {

	//Aux variables
	CVector2 auxPosition;
	CVector2 auxVelocity;

	//Create a list of Nodes
	for (map<UInt8, Node>::iterator it = listNodeObstacles.begin(); it != listNodeObstacles.end(); it++) {

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

	/* Init fixed obstacles */
	//setFixedObstacles(addAreaBounds(originAreaCoord, destinationAreaCoord));

}

std::vector<CVector2> CFootBotAN::addAreaBounds(CVector2 bottomLeftCoord, CVector2 upperRightCoord) {

	//All obstacle points
	std::vector<CVector2> obstaclePoints;

	CVector2 upperLeftCoord;
	upperLeftCoord.Set(bottomLeftCoord.GetX(), upperRightCoord.GetY());

	CVector2 bottomRightCoord;
	bottomRightCoord.Set(upperRightCoord.GetX(), bottomLeftCoord.GetY());

	printf("\n Area bounds vertices (%f,%f),(%f,%f),(%f,%f),(%f,%f)\n", bottomLeftCoord.GetX(), bottomLeftCoord.GetY(), upperLeftCoord.GetX(), upperLeftCoord.GetY(), upperRightCoord.GetX(), upperRightCoord.GetY(),
			bottomRightCoord.GetX(), bottomRightCoord.GetY());

	//Distance from bottom left to upper left.
	CVector2 d1;
	d1 = upperLeftCoord - bottomLeftCoord;

//	printf("D1 %f\n", d1.Length());

	double numObsD1;
	modf(d1.Length() / (2 * OBSTACLE_RADIUS), &numObsD1);
	double interDistanceD1 = d1.Length() / numObsD1;

	CVector2 pointAux;
	pointAux.SetX(bottomLeftCoord.GetX());
	for (int i = 0; i < numObsD1; i++) {
		pointAux.SetY(bottomLeftCoord.GetY() + i * interDistanceD1);

//		printf("D1, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
		obstaclePoints.push_back(pointAux);
	}

	//Distance from upper left to upper right
	CVector2 d2;
	d2 = upperLeftCoord - upperRightCoord;
	//printf("D2 %f\n", d2.Length());

	double numObsD2;
	modf(d2.Length() / (2 * OBSTACLE_RADIUS), &numObsD2);
	double interDistanceD2 = d2.Length() / numObsD2;

	pointAux.SetY(upperLeftCoord.GetY());
	for (int i = 0; i < numObsD2; i++) {
		pointAux.SetX(upperLeftCoord.GetX() + i * interDistanceD2);

		//printf("D2, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
		obstaclePoints.push_back(pointAux);
	}

	//Distance from upper right to bottom right
	CVector2 d3;
	d3 = upperRightCoord - bottomRightCoord;
	//printf("D3 %f\n", d3.Length());

	double numObsD3;
	modf(d3.Length() / (2 * OBSTACLE_RADIUS), &numObsD3);
	double interDistanceD3 = d3.Length() / numObsD3;

	pointAux.SetX(upperRightCoord.GetX());
	for (int i = 0; i < numObsD3; i++) {
		pointAux.SetY(upperRightCoord.GetY() - i * interDistanceD3);

		//printf("D3, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
		obstaclePoints.push_back(pointAux);
	}

	//Distance from bottom right to bottom left
	CVector2 d4;
	d4 = bottomRightCoord - bottomLeftCoord;
	//printf("D4 %f\n", d4.Length());

	double numObsD4;
	modf(d4.Length() / (2 * OBSTACLE_RADIUS), &numObsD4);
	double interDistanceD4 = d4.Length() / numObsD4;

	pointAux.SetY(bottomRightCoord.GetY());
	for (int i = 0; i < numObsD4; i++) {
		pointAux.SetX(bottomRightCoord.GetX() - i * interDistanceD4);

		//printf("D4, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
		obstaclePoints.push_back(pointAux);
	}

	return obstaclePoints;

}

void CFootBotAN::setFixedObstacles(std::vector<CVector2> obstaclesPoints) {

	std::vector<CVector2>::iterator it;
	for (int i = 0; i < obstaclesPoints.size(); i++) {
		agent->addObstacleAtPoint(obstaclesPoints[i], OBSTACLE_RADIUS);
	}

}

void CFootBotAN::updateNavigation() {
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

void CFootBotAN::updateDesideredVelocity() {

	agent->targetPosition = targetPosition;
	agent->updateDesideredVelocity();

	printf("=> desidered speed %.2f, desidered angle %.2f \n", agent->desideredSpeed, agent->desideredAngle.GetValue());
}

void CFootBotAN::updateVelocity() {

	printf("%p: target wheel speed (%.3f %.3f) r %.3f\n", agent, agent->leftWheelTargetSpeed, agent->rightWheelTargetSpeed, agent->radius);

	m_pcWheels->SetLinearVelocity(100 * agent->leftWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_LEFT, 100 * agent->rightWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_RIGHT);
}

void CFootBotAN::updateVelocityProximitySensors() {
	/* Get readings from proximity sensor */
	const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
	/* Sum them together */
	CVector2 cAccumulator;
	for (size_t i = 0; i < tProxReads.size(); ++i) {
		cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
	}
	cAccumulator /= tProxReads.size();
	/* If the angle of the vector is small enough and the closest obstacle is far enough,
	 continue going straight, otherwise curve a little */
	CRadians cAngle = cAccumulator.Angle();
	if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) && cAccumulator.Length() < m_fDelta) {
		/* Go straight */
		//m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
	} else {
		/* Turn, depending on the sign of the angle */
		if (cAngle.GetValue() > 0.0f) {
			m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity / 2.0f);
		} else {
			m_pcWheels->SetLinearVelocity(-m_fWheelVelocity / 2.0f, m_fWheelVelocity);
		}
	}
	/* Set LED colors */
	m_pcLEDs->SetAllColors(CColor::BLACK);
	cAngle.UnsignedNormalize();
	UInt32 unIndex = static_cast<UInt32>(cAngle * 12.0f / CRadians::TWO_PI);
	m_pcLEDs->SetSingleColor(unIndex, CColor::RED);
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
REGISTER_CONTROLLER(CFootBotAN, "footbot_an_controller")
