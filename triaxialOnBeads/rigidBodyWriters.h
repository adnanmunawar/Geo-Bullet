#ifndef RIGIDBODYWRITERS_H
#define RIGIDBODYWRITERS_H
#include <iostream>
#include <fstream>
#include <cstdio>		//needed for formatted file outputs


// For writing the CSV format data (beads position/rotation/displacement vectors) of beads in "/beadsPositions/beadsPositions.xx.csv"
void beadPositionWriter(btAlignedObjectArray<btRigidBody*> rigidBody, int NumberOfRigidBodies, btScalar simTime,  int outputIndex){

	checkpointTime1 = floor(simTime/(writeOutputInterval));
	if (checkpointTime1 > checkpointTime2 || simTime == 0 ){

		std::ofstream 	beadsPositions("beadsPositions/beadsPositions." + std::to_string(outputIndex) + "."  + std::to_string(checkpointTime1) + ".csv");

		beadsPositions << "xCoord" << "," << "yCoord" << "," << "zCoord" << "," << "rotation(deg)" << "," << "xDisp" << "," << "yDisp" << "," << "zDisp" << std::endl;
	

		for (int j =0; j < NumberOfRigidBodies; j++){
			beadsPositions << rigidBody[j]->getWorldTransform().getOrigin().getX() << "," << rigidBody[j]->getWorldTransform().getOrigin().getY() << "," << rigidBody[j]->getWorldTransform().getOrigin().getZ() << "," << btDegrees(rigidBody[j]->getWorldTransform().getRotation().getAngle()) << "," << (rigidBody[j]->getWorldTransform().getOrigin().getX() - xDispBuffer[j]) << "," << (rigidBody[j]->getWorldTransform().getOrigin().getY() - yDispBuffer[j]) << "," << (rigidBody[j]->getWorldTransform().getOrigin().getZ() - zDispBuffer[j]) << std::endl;
		
		xDispBuffer[j] = rigidBody[j]->getWorldTransform().getOrigin().getX();
		yDispBuffer[j] = rigidBody[j]->getWorldTransform().getOrigin().getY();
		zDispBuffer[j] = rigidBody[j]->getWorldTransform().getOrigin().getZ();
		}

		beadsPositions.close();
	}
	checkpointTime2 = floor(simTime/(writeOutputInterval));

}

// Vertical force detection
void detectVerticalForce(btSoftRigidDynamicsWorld* dynamicsWorld, btRigidBody* topPlateRigidBody, btScalar simTime, btScalar totalTime, btScalar timeInterval, std::ofstream &phaseOutput){
	if (simTime == 0){
		if (phaseOutput == consolidationOutput) {
			phaseOutput.open("consolidation.txt");
		} else if(phaseOutput == shearingOutput){
			phaseOutput.open("shearing.txt");
		}
		phaseOutput << "time(s)" << " " << "verticalDisp(mm)" << " " << "strain" << " " << "verticalForce(kN)" << " " << "verticalStress(kPa)" << " " << "mobilizedAngleOfFriction(deg)" << " " << "numManifolds" << " " << "numContacts" << " " << "averageCoordinationNum" << std::endl;
	}

	checkpointTime7 = floor(simTime/(timeInterval));
	if (checkpointTime7 > checkpointTime8 || simTime == 0 ){
		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();	// getting the number of manifolds (derived by broadphase)
		int numContacts=0;
		btScalar	verticalForce=0;

		btAlignedObjectArray<btPersistentManifold*> manifold;
		manifold.resize(numManifolds);

		for (int i=0; i < numManifolds; i++){
			manifold[i] = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			numContacts += manifold[i]->getNumContacts();
			//finding the contact contact between bottom platen and bottom spherical support
			if (manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() < (-plateThickness/2.f)) {
				verticalForce = manifold[i]->getContactPoint(0).m_appliedImpulse / deltaTime;;
			}
		}

		btScalar verticalDisp = (sampleHeight + plateThickness/2) - topPlateRigidBody->getWorldTransform().getOrigin().getZ();

		phaseOutput << std::fixed  << simTime / timeFactor << " " << verticalDisp << " " << verticalDisp/sampleHeight << " " << verticalForce/1000 << " " << (btPow(scaleFactor,2)*verticalForce/(SIMD_PI*btPow(sampleRadius,2)))/1000 << " " << btDegrees(btAsin(((btPow(scaleFactor,2)*verticalForce/(SIMD_PI*btPow(sampleRadius,2)))-confiningStress)/(((btPow(scaleFactor,2)*verticalForce/(SIMD_PI*btPow(sampleRadius,2)))+confiningStress)))) << " " << numManifolds << " " << numContacts << " " << btScalar(numContacts)*2/btScalar(exactNumBead) << std::endl;
	}
	checkpointTime8 = floor(simTime/(timeInterval));

	if (simTime == totalTime){
		phaseOutput.close();
	}
}


//Application of stress on the top plate - linear application with respect to time
void stressOnPlate(btRigidBody* plateRigidBody, btScalar totalStress, btScalar sampleRadius, btScalar totalTime, btScalar simTime){
	btScalar		totalForcePerStep = -(totalStress * (SIMD_PI * btPow(sampleRadius,2)) / btPow(scaleFactor,2)) * (simTime /totalTime);
	plateRigidBody->applyCentralForce(btVector3(0,0,totalForcePerStep));
}

//Keeping the stress on the top plate - time independent
void keepStressOnPlate(btRigidBody* plateRigidBody, btScalar totalStress, btScalar sampleRadius){
	btScalar		totalForcePerStep = -( totalStress * (SIMD_PI * btPow(sampleRadius,2)) / btPow(scaleFactor,2));
	plateRigidBody->applyCentralForce(btVector3(0,0,totalForcePerStep));
}

void applyDeviatoricStrain(btRigidBody* plateRigidBody, btScalar maxDisplacement, btScalar totalTime, btScalar deltaTime){
	
	btScalar topPlateAltitude = plateRigidBody->getWorldTransform().getOrigin().getZ();
	topPlateAltitude -= maxDisplacement * (deltaTime/totalTime);
	btTransform topPlateBTtransform = btTransform(upright,btVector3(0,0,topPlateAltitude));
	plateRigidBody->setWorldTransform(topPlateBTtransform);
}

#endif