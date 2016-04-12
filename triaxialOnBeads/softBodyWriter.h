#ifndef SOFTBODYWRITER_H
#define SOFTBODYWRITER_H
#include <iostream>
#include <fstream>
#include <cstdio>		//needed for formatted file outputs
#include <iomanip>
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftBodyHelpers.h"							//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftRigidDynamicsWorld.h"					//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btDefaultSoftBodySolver.h"						//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"	//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftSoftCollisionAlgorithm.h"		//ADDED


// For writing the CSV format data (soft body position/displacement vectors) of soft body in "/softbodyPosition/softbodyPosition.xx.csv"
void softBodyPositionWriter(btSoftBody* softBody, int numberOfNodes, btScalar simTime, int outputIndex){
	//outputIndex is provided to manage the output stream for various softbodies or various phases of stress application

	checkpointTime3 = floor(simTime/(writeOutputInterval));
	if (checkpointTime3 > checkpointTime4 || simTime == 0 ){
		std::ofstream 	softbodyPosition("softbodyPosition/softbodyPosition" + std::to_string(outputIndex) + "." + std::to_string(checkpointTime3) + ".csv");

		softbodyPosition << "xCoord" << "," << "yCoord" << "," << "zCoord" << "," << "Velocity" << std::endl;
	
		for (int k=0; k < numberOfNodes; k++){
			softbodyPosition << softBody->m_nodes[k].m_x.x() << "," << softBody->m_nodes[k].m_x.y() << "," << softBody->m_nodes[k].m_x.z() << "," << softBody->m_nodes[k].m_v.length() << std::endl;
		}


		softbodyPosition.close();
	}
	checkpointTime4 = floor(simTime/(writeOutputInterval));
}

//For writing the CSV data of the relative deformation of the nodes of the softbodies (amount of stretching and shrinkage) in "/softBodyDeformations/softBodyDeformation.xx.csv"
void softBodyDeformationWriter(btSoftBody* softBody, int numberOfLinks, btScalar simTime, int outputIndex){
	//outputIndex is provided to manage the output stream for various softbodies or various phases of stress application


	checkpointTime5 = floor(simTime/(writeOutputInterval));
	if (checkpointTime5 > checkpointTime6 || simTime == 0 ){

		std::ofstream	softBodyDeformation("softbodyDeformation/softbodyDeformation" + std::to_string(outputIndex) + "." + std::to_string(checkpointTime5) + ".csv");

		softBodyDeformation << "xCoord" << "," << "yCoord" << "," << "zCoord" << "," << "Deformation" << "," << "Strain(percent)" << std::endl;
		for (int k=0; k < numberOfLinks; k++){

			btScalar deformation = btFabs(btDistance(softBody->m_links[k].m_n[1]->m_x, softBody->m_links[k].m_n[0]->m_x)) - softBody->m_links[k].m_rl;

			softBodyDeformation << lerp(softBody->m_links[k].m_n[0]->m_x,softBody->m_links[k].m_n[1]->m_x,0.5).getX() << "," << lerp(softBody->m_links[k].m_n[0]->m_x,softBody->m_links[k].m_n[1]->m_x,0.5).getY() << "," << lerp(softBody->m_links[k].m_n[0]->m_x,softBody->m_links[k].m_n[1]->m_x,0.5).getZ() << "," << deformation/scaleFactor << "," << deformation*100/softBody->m_links[k].m_rl << std::endl;
		}

		softBodyDeformation.close();
	}
	checkpointTime6 = floor(simTime/(writeOutputInterval));
}



#endif