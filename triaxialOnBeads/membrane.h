#ifndef MEMBRANE_H
#define MEMBRANE_H
#include <iostream>

	//A function to determine which nodes should be under confinement stress. It writes the data into a "btAlignedObjectArray<int>"
	void determineTheNodesUnderStress(btSoftBody* softBody, btScalar membraneRadius, btAlignedObjectArray<int> &nodeArray){
		for(int i=0; i < softBody->m_nodes.size(); i++){

			if (	(softBody->m_nodes[i].m_x.x() <= membraneRadius * 1.01) 	&&
					(softBody->m_nodes[i].m_x.x() >= -membraneRadius * 1.01) 	&&
					(softBody->m_nodes[i].m_x.y() <= membraneRadius * 1.01)		&&
					(softBody->m_nodes[i].m_x.y() >= -membraneRadius * 1.01)
				) {
				nodeArray.push_back(i);
			}
		}
	}

	//The function needed for application of radial force to the membrane (linear with respect to consolidation time)
	void confiningStressOnMembrane(btSoftBody* softBody, btRigidBody* lowerPlaten, btRigidBody* upperPlaten, btScalar confiningStress, btScalar consolidationTime, btScalar simTime){

		btScalar totalConfiningForce = confiningStress*(membraneHeight*membraneRadius*SIMD_PI*-2) / btPow(scaleFactor,2); //total force on all of the nodes of the softbody

		btScalar nodalConfiningForceInDeltaTime = (simTime /consolidationTime) * totalConfiningForce / softBody->m_nodes.size(); // the nodal force on each node of the softbody (membrane)

		for (int i = 0; i < softBody->m_nodes.size(); i++){
			if ((softBody->m_nodes[i].m_x.z() > (lowerPlaten->getWorldTransform().getOrigin().getZ() - plateThickness/2)) &&
				(softBody->m_nodes[i].m_x.z() < (upperPlaten->getWorldTransform().getOrigin().getZ() + plateThickness/2))) {

				btScalar xyLength			= btSqrt(btPow(softBody->m_nodes[i].m_x.x(),2) + btPow(softBody->m_nodes[i].m_x.y(),2));
				btScalar xForceComponent 	= nodalConfiningForceInDeltaTime * softBody->m_nodes[i].m_x.x() / xyLength;
				btScalar yForceComponent 	= nodalConfiningForceInDeltaTime * softBody->m_nodes[i].m_x.y() / xyLength;

				softBody->addForce(btVector3(xForceComponent, yForceComponent,0), i);
			}
		}
	}

	//The function needed for keeping of radial force to the membrane (independent of time)
	void keepStressOnMembrane(btSoftBody* softBody, btRigidBody* lowerPlaten, btRigidBody* upperPlaten, btScalar confiningStress){

		btScalar totalConfiningForce = confiningStress*(membraneHeight*membraneRadius*SIMD_PI*-2) / (softBody->m_nodes.size() * btPow(scaleFactor,2)); //total force on all of the nodes of the softbody


		for (int i = 0; i < softBody->m_nodes.size(); i++){
			if ((softBody->m_nodes[i].m_x.z() > (lowerPlaten->getWorldTransform().getOrigin().getZ() - plateThickness/2)) &&
				(softBody->m_nodes[i].m_x.z() < (upperPlaten->getWorldTransform().getOrigin().getZ() + plateThickness/2))) {

				btScalar xyLength			= btSqrt(btPow(softBody->m_nodes[i].m_x.x(),2) + btPow(softBody->m_nodes[i].m_x.y(),2));
				btScalar xForceComponent 	= totalConfiningForce * softBody->m_nodes[i].m_x.x() / xyLength;
				btScalar yForceComponent 	= totalConfiningForce * softBody->m_nodes[i].m_x.y() / xyLength;

				softBody->addForce(btVector3(xForceComponent, yForceComponent,0), i);
			}
		}
	}

#endif