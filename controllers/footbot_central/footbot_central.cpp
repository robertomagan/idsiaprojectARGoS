/* Include the controller definition */
#include "footbot_central.h"

#include <argos2/common/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos2/common/utility/math/vector2.h>

#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/entities/footbot_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/general.h>
#include <string>
#include <stdlib.h>
#include <vector>
#include <map>

/****************************************/
/****************************************/

CFootBotCentral::CFootBotCentral() :
		m_cSpace(CSimulator::GetInstance().GetSpace()) {
}

/****************************************/
/****************************************/

void CFootBotCentral::Init(TConfigurationNode& t_node) {

	/* LCM Engine */
	lcmHandler = new LCMHandler("udpm://239.255.76.67:7667?ttl=1", "TRACK");

}

/****************************************/
/****************************************/

void CFootBotCentral::ControlStep() {

	printf("\n --------- CENTRAL - Simulation step %d -----------\n", CSimulator::GetInstance().GetSpace().GetSimulationClock());

	/* Getting all NODE positions */

	CVector3 auxPositions;
	CQuaternion auxOrientations;
	UInt8 robotID = 0;
	string robotName;

	map<UInt8, CFootBotEntity&> ans;

	/*Update the positions of the robot*/
	CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType("footbot_entity");

	/*Iterate over all robots with wifi and ask their position*/
	for (CSpace::TAnyEntityMap::iterator it = tEntityMap.begin(); it != tEntityMap.end(); ++it) {
		CFootBotEntity& footBotEntity = *(any_cast<CFootBotEntity*>(it->second));

		/* Robot's name */
		robotName = footBotEntity.GetControllableEntity().GetController().GetRobot().GetRobotId();

		/* Robot's location */
		auxPositions = footBotEntity.GetWiFiEquippedEntity().GetPosition();

		/* Get ID */
		robotID = atoi(robotName.substr(3).c_str());

		/* Store the footbots ANS */
		ans.insert(pair<UInt8, CFootBotEntity&>(robotID, footBotEntity));

		printf("Robot %s, ID %d,  (x=%f - y=%f - z=%f)\n", robotName.c_str(), robotID, auxPositions[0], auxPositions[1], auxPositions[2]);

	}

	map<UInt8, Node> listAllNodes;
	// To set the timestamp
	struct timeval tp;

	//Create a list of Nodes
	for (map<UInt8, CFootBotEntity&>::iterator it = ans.begin(); it != ans.end(); it++) {

		Node node;

		//printf("Current node : %d\n", node.getId());

		node.setId(it->first);

		CFootBotEntity& footBotEntity = static_cast<CFootBotEntity&>(it->second);

		CVector3 positions(footBotEntity.GetWiFiEquippedEntity().GetPosition().GetX() * 1000.0f, footBotEntity.GetWiFiEquippedEntity().GetPosition().GetY() * 1000.0f,
				footBotEntity.GetWiFiEquippedEntity().GetPosition().GetZ() * 1000.0f);

		node.setPosition(positions);

		CQuaternion quat(footBotEntity.GetWiFiEquippedEntity().GetOrientation().GetW() * 1000.0f, footBotEntity.GetWiFiEquippedEntity().GetOrientation().GetX() * 1000.0f,
				footBotEntity.GetWiFiEquippedEntity().GetOrientation().GetY() * 1000.0f, footBotEntity.GetWiFiEquippedEntity().GetOrientation().GetZ() * 1000.0f);

		node.setOrientation(quat);

		gettimeofday(&tp, NULL);
		UInt64 ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
		node.setTimestamp(ms);

		// To compute each node velocity
		if (m_prevListNodes.find(it->first) != m_prevListNodes.end()) {
			CVector2 prevPos(m_prevListNodes[it->first].getPosition().GetX(), m_prevListNodes[it->first].getPosition().GetY());
			CVector2 currentPos(positions.GetX(), positions.GetY());
			CVector2 deltaPosition = currentPos - prevPos;
			UInt16 speed = deltaPosition.Length() / 0.1;//->mm/s
			node.setVelocity(speed);//-> m/s
			printf("Velocity %d\n",node.getVelocity());
			// Velocity
			//velocity = CVector2(speed, 0);

		}
		listAllNodes[node.getId()] = node;
	}

	//Remove myself
	robotID = atoi(GetRobot().GetRobotId().substr(3).c_str());
	listAllNodes.erase(robotID);

	printf("# of nodes = %d\n", listAllNodes.size());

	// Publish the information of all nodes to the channel
	lcmHandler->publish(listAllNodes);
	//lcmHandler->getAvailableMessages();
	//lcmHandler->printNodeListElements();
	m_prevListNodes = listAllNodes;

	// To print the messages
	//lcmHandler->getAvailableMessages();

}

void CFootBotCentral::Destroy() {

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
REGISTER_CONTROLLER(CFootBotCentral, "footbot_central_controller")
