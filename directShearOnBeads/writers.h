#ifndef WRITERS_H
#define WRITERS_H
#include <iostream>
#include <fstream>
#include <cstdio>		//needed for formatted file outputs


// For writing the data of the consolidation-time curve to "consolidation.txt"
void consolidationOutputWriter(btScalar simTime, btScalar consolidationTime,btRigidBody* topPlateRB){

	if (simTime == 0){
		consolidationOutput.open("consolidation.txt");
		consolidationOutput << "Time(s)" << " " << "topPlatePositionX" << " " << "topPlatePositionY" << " " << "topPlatePositionZ" << " " << "forceOnTopPlate(N)" << " " << "normalStress(kPa)" << std::endl;
	}

	consolidationOutput << std::fixed << simTime / timeFactor << " " << topPlateRB->getWorldTransform().getOrigin().getX() << " "<< topPlateRB->getWorldTransform().getOrigin().getY() << " " << topPlateRB->getWorldTransform().getOrigin().getZ() << " " << -1*topPlateRB->getTotalForce().getZ() << " " << (-1*topPlateRB->getTotalForce().getZ())/(btPow(boxSize/scaleFactor,2)*1000) << std::endl;
	
	if (simTime == consolidationTime){
		consolidationOutput.close();
	}
}



// For writing the data of the shear stress-shear strain curve to "shearing.txt"
void shearOutputWriter_stressControlled(btScalar simTime, btScalar shearingTime){

	if (simTime == 0){
		shearingOutput.open("shearing.txt");
		shearingOutput << "time(s)" << " " << "lowerBoxDisp" << " " << "upperBoxDisp" << " " << "shearinDisp" << " " << "topPlateVerticalDisp" << " " << "springForce(N)" << " " << "shearStress(kPa)" << std::endl;
	}

	checkpointTime1 = floor(simTime/writeOutputInterval);
	if (checkpointTime1 > checkpointTime2 || simTime == 0 ){

		double 		springForceDoubleBuffer		=	(upperBoxRBBuffer.getOrigin().getX()) * springStiffness;
		double 		springStressDoubleBuffer	=	springForceDoubleBuffer/((boxSize/scaleFactor)*((boxSize-(lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX()))/scaleFactor)*1000);

		shearingOutput << std::fixed  << simTime / timeFactor << " " << lowerBoxBuffer.getOrigin().getX() << " " << upperBoxRBBuffer.getOrigin().getX() << " " << lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX() << " " << topPlateBuffer.getOrigin().getZ() - topPlateBasePos.getOrigin().getZ() << " " << springForceDoubleBuffer << " " << springStressDoubleBuffer << std::endl;

		std::cout << std::fixed << std::setprecision(2) << (simTime*100/shearingTime) << "% of shearing implemented..." << std::endl;

		if (simTime == shearingTime){
			shearingOutput.close();
		}
	}

	checkpointTime2 = floor(simTime/writeOutputInterval);

}



// For writing the CSV format data (beads position/rotation/displacement vectors) of beads in "/beadsPositions/beadsPositions.xx.csv"
void beadPositionWriter(btAlignedObjectArray<btRigidBody*> rigidBody, int NumberOfRigidBodies, btScalar simTime, btRigidBody* lowerBoxRB){

	checkpointTime3 = floor(simTime/(writeOutputInterval*10));
	if (checkpointTime3 > checkpointTime4 || simTime == 0 ){

		std::ofstream 	beadsPositions("beadsPositions/beadsPositions." + std::to_string(checkpointTime3) + ".csv");

		beadsPositions << "xCoord" << "," << "yCoord" << "," << "zCoord" << "," << "rotation(deg)" << "," << "xDisp" << "," << "yDisp" << "," << "zDisp" << std::endl;

		lowerBoxXPos2 = lowerBoxRB->getWorldTransform().getOrigin().getX();
		deltaLowerBoxPos = lowerBoxXPos2 - lowerBoxXPos1;
	

		for (int j =0; j < NumberOfRigidBodies; j++){
			beadsPositions << rigidBody[j]->getWorldTransform().getOrigin().getX() << "," << rigidBody[j]->getWorldTransform().getOrigin().getY() << "," << rigidBody[j]->getWorldTransform().getOrigin().getZ() << "," << btDegrees(rigidBody[j]->getWorldTransform().getRotation().getAngle()) << "," << (rigidBody[j]->getWorldTransform().getOrigin().getX() - xDispBuffer[j]- deltaLowerBoxPos) << "," << (rigidBody[j]->getWorldTransform().getOrigin().getY() - yDispBuffer[j]) << "," << (rigidBody[j]->getWorldTransform().getOrigin().getZ() - zDispBuffer[j]) << std::endl;
		
		xDispBuffer[j] = rigidBody[j]->getWorldTransform().getOrigin().getX();
		yDispBuffer[j] = rigidBody[j]->getWorldTransform().getOrigin().getY();
		zDispBuffer[j] = rigidBody[j]->getWorldTransform().getOrigin().getZ();
		lowerBoxXPos1 = lowerBoxRB->getWorldTransform().getOrigin().getX();

		}

		beadsPositions.close();
	}
	checkpointTime4 = floor(simTime/(writeOutputInterval*10));

}

// For writing the data of the shear stress-shear strain curve to "shearing.txt"
void shearOutputWriter_strainControlled(btScalar simTime, btScalar shearingTime, btDiscreteDynamicsWorld* dynamicsWorld, btScalar deltaTime){

	if (simTime == 0){
		shearingOutput.open("shearing.txt");
		shearingOutput << "time(s)" << " " << "shearinDisp" << " " << "topPlateVerticalDisp" << " " << "forwardSphereForce(N)" << " " << "rearSphereForce(N)" << " " << "totalShearForce(N)" << " " << "shearStress(kPa)" << " " << "numManifolds" << " " << "numContacts" << " " << "averageCoordinationNum" << std::endl;
	}

	checkpointTime1 = floor(simTime/writeOutputInterval);
	if (checkpointTime1 > checkpointTime2 || simTime == 0 ){

		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();	// getting the number of manifolds (derived by broadphase)
		int numContacts=0;

		btScalar 	forwardSphereForce 	= 0;
		btScalar	rearSphereForce 	= 0;	

		btAlignedObjectArray<btPersistentManifold*> manifold;
		manifold.resize(numManifolds);

		for (int i=0; i < numManifolds; i++){
			manifold[i] = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			numContacts += manifold[i]->getNumContacts();
			//finding the forward sphere contact id
			if ((manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() > (boxSize + boxThickness)/2.f)  && (manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() > (boxHeight * 0.75))) {
				forwardSphereForce = manifold[i]->getContactPoint(0).m_appliedImpulse / deltaTime;;
			}

			//finding the rear sphere contact id
			if ((manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() < (-boxSize - boxThickness)/2.f)  && (manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() > (boxHeight * 0.75))) {
				rearSphereForce = manifold[i]->getContactPoint(0).m_appliedImpulse / deltaTime;;
			}
		}


		shearingOutput << std::fixed  << simTime / timeFactor << " " << lowerBoxBuffer.getOrigin().getX() << " " << topPlateBuffer.getOrigin().getZ() - topPlateBasePos.getOrigin().getZ() << " " << forwardSphereForce << " " << rearSphereForce << " " << (forwardSphereForce - rearSphereForce) << " " << (forwardSphereForce - rearSphereForce)/((boxSize/scaleFactor)*((boxSize-lowerBoxBuffer.getOrigin().getX())/scaleFactor)*1000) << " " << numManifolds << " " << numContacts << " " << btScalar(numContacts)*2/btScalar(exactNumBead) << std::endl;

		std::cout << std::fixed  << std::setprecision(2) << (simTime*100/shearingTime) << "% of shearing implemented..." << std::endl;

		if (simTime == shearingTime){
			shearingOutput.close();
		}
	}

	checkpointTime2 = floor(simTime/writeOutputInterval);

}


// For writing the CSV format data (contact position/rotations and normal force chain and friction force chain ) of beads in "/contactData/contactData.xx.csv"
void contactDataWriter(btDiscreteDynamicsWorld* dynamicsWorld, btScalar simTime, btScalar deltaTime){

	checkpointTime5 = floor(simTime/(writeOutputInterval*10));
	if (checkpointTime5 > checkpointTime6 || simTime == 0 ){


		std::ofstream 	contactData("contactData/contactData." + std::to_string(checkpointTime5) + ".csv");

		contactData << "xCoord" << "," << "yCoord" << "," << "zCoord" << "," << "normalForce(N)" << "," << "xContactNormal" << "," << "yContactNormal" << "," << "zContactNormal" << "," << "lateralForce(N)" << "," << "xLateralDir" << "," << "yLateralDir" << "," << "zLateralDir" << std::endl;

		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
		btAlignedObjectArray<btPersistentManifold*> manifold;
		manifold.resize(numManifolds);

		for (int i=0; i < numManifolds; i++){
			manifold[i] = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			if (manifold[i]->getNumContacts() >0 && manifold[i]->getContactPoint(0).m_appliedImpulse >0 && manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() != (boxSize/2 + boxThickness) && manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() != -(boxSize/2 + boxThickness)){
				btScalar	lateralForce = btSqrt( btPow((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getX()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getX()),2) + btPow((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getY()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getY()),2) + btPow((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getZ()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getZ()),2))  / deltaTime ;
				btScalar 	xLateralDir = ((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getX()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getX())) /lateralForce ;
				btScalar 	yLateralDir = ((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getY()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getY())) /lateralForce ;
				btScalar 	zLateralDir = ((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getZ()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getZ())) /lateralForce ;

	 
				contactData << manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() << "," << manifold[i]->getContactPoint(0).m_positionWorldOnB.getY() << "," << manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() << "," << manifold[i]->getContactPoint(0).m_appliedImpulse / deltaTime << "," << manifold[i]->getContactPoint(0).m_normalWorldOnB.getX() << "," << manifold[i]->getContactPoint(0).m_normalWorldOnB.getY() << "," << manifold[i]->getContactPoint(0).m_normalWorldOnB.getZ() << "," << lateralForce << "," << xLateralDir << "," << yLateralDir << "," << zLateralDir << std::endl;
			}
		}

		contactData.close();
	}
	checkpointTime6 = floor(simTime/(writeOutputInterval*10));

}

// For writring the number of contacts in specific volume (RVE), useful for finding out what occurrs in the shear band. 
void contactNumberRVE(btScalar simTime, btScalar shearingTime, btDiscreteDynamicsWorld* dynamicsWorld, btScalar Xmin, btScalar Xmax, btScalar Ymin, btScalar Ymax, btScalar Zmin, btScalar Zmax){

	if (simTime==0){
		contactRVEOutput.open("contactRVEOutput.txt");
		contactRVEOutput << "time(s)" << " " << "shearinDisp" << " " << "numManifolds" << " " << "numContacts" << std::endl;
	}
	
	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	int numContacts=0;
	btAlignedObjectArray<btPersistentManifold*> manifold;
	manifold.resize(numManifolds);

	for (int i=0; i < numManifolds; i++){
		manifold[i] = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		if (manifold[i]->getContactPoint(0).m_appliedImpulse >0 && 
			manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() > Xmin && 
			manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() < Xmax &&
			manifold[i]->getContactPoint(0).m_positionWorldOnB.getY() > Ymin &&
			manifold[i]->getContactPoint(0).m_positionWorldOnB.getY() < Ymax &&
			manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() > Zmin &&
			manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() < Zmax){

			numContacts++;
		}

	}
	
	checkpointTime7 = floor(simTime/writeOutputInterval);
	if (checkpointTime7 > checkpointTime8 || simTime == 0 ){
		contactRVEOutput << simTime/timeFactor << " " << (lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX()) << " " << numManifolds << " " << numContacts << std::endl;
	}
	checkpointTime8 = floor(simTime/writeOutputInterval);


	if (simTime==shearingTime){
		contactRVEOutput.close();
	}

}

// For writing the data of the total normal and frictional force and the ratio between those
void forceRatios(btScalar simTime, btScalar shearingTime, btDiscreteDynamicsWorld* dynamicsWorld, btScalar Xmin, btScalar Xmax, btScalar Ymin, btScalar Ymax, btScalar Zmin, btScalar Zmax){

	if (simTime==0){
		forceRatioOutput.open("forceRatioOutput.txt");
		forceRatioOutput << "time(s)" << " " << "shearinDisp" << " " << "totalNormalForce(N)" << " " << "totalFrictionalForce(N)" << " " << "forceRatio" << std::endl; 
	}

	checkpointTime9 = floor(simTime/writeOutputInterval);
	if (checkpointTime8 > checkpointTime10 || simTime == 0 ){
		btScalar 	totalNormalForce=0;
		btScalar 	totalFrictionalForce=0;

		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
		btAlignedObjectArray<btPersistentManifold*> manifold;
		manifold.resize(numManifolds);

		for (int i=0; i < numManifolds; i++){
			manifold[i] = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			if (manifold[i]->getContactPoint(0).m_appliedImpulse >0 && 
				manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() > Xmin &&
				manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() < Xmax &&
				manifold[i]->getContactPoint(0).m_positionWorldOnB.getY() > Ymin &&
				manifold[i]->getContactPoint(0).m_positionWorldOnB.getY() < Xmax &&
				manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() > Zmin &&
				manifold[i]->getContactPoint(0).m_positionWorldOnB.getZ() < Zmax){
				btScalar	lateralForce = btSqrt( btPow((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getX()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getX()),2) + btPow((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getY()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getY()),2) + btPow((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getZ()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getZ()),2))  / deltaTime ;
				btScalar 	xLateralDir = ((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getX()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getX())) /lateralForce ;
				btScalar 	yLateralDir = ((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getY()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getY())) /lateralForce ;
				btScalar 	zLateralDir = ((manifold[i]->getContactPoint(0).m_appliedImpulseLateral1 * manifold[i]->getContactPoint(0).m_lateralFrictionDir1.getZ()) + (manifold[i]->getContactPoint(0).m_appliedImpulseLateral2 * manifold[i]->getContactPoint(0).m_lateralFrictionDir2.getZ())) /lateralForce ;

				totalNormalForce 		+= 	btFabs(manifold[i]->getContactPoint(0).m_appliedImpulse / deltaTime);
				totalFrictionalForce 	+=	btFabs(lateralForce);
			}
		}

		forceRatioOutput << simTime << " " << (lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX()) << " " << totalNormalForce << " " << totalFrictionalForce << " " << totalFrictionalForce/totalNormalForce << std::endl;
	}
	checkpointTime10 = floor(simTime/writeOutputInterval);


	if (simTime==shearingTime){
		forceRatioOutput.close();
	}
}

//to write the data of the normal force of each contact and its direction projected to x-plane => to draw normal force polar graph
void polarForceData(btDiscreteDynamicsWorld* dynamicsWorld, btScalar simTime, btScalar deltaTime){
	checkpointTime11 = floor(simTime/(writeOutputInterval*10));
	if (checkpointTime11 > checkpointTime12 || simTime == 0 ){
		std::ofstream 	polarForce("polarForceData/polarForceData." + std::to_string(checkpointTime11) + ".csv");

		polarForce << "shearDisp" << " " << "angle(XZ)(deg)" << " " << "contactForce(N)" << std::endl;

		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
		btAlignedObjectArray<btPersistentManifold*> manifold;
		manifold.resize(numManifolds);
		btScalar	contactForce[18]={0};
		for (int i=0; i < numManifolds; i++){
			manifold[i] = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			btScalar	forceBuffer=0;
			
			btScalar	angleInXZ=0;
			if (manifold[i]->getNumContacts() >0 && manifold[i]->getContactPoint(0).m_appliedImpulse >0 && manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() != (boxSize/2 + boxThickness) && manifold[i]->getContactPoint(0).m_positionWorldOnB.getX() != -(boxSize/2 + boxThickness)){
			
				forceBuffer	=	btFabs(manifold[i]->getContactPoint(0).m_appliedImpulse / deltaTime);
				angleInXZ 		=	btDegrees(btAtan((manifold[i]->getContactPoint(0).m_normalWorldOnB.getZ())/(manifold[i]->getContactPoint(0).m_normalWorldOnB.getX())));
				if (angleInXZ < 0 ){
					angleInXZ += 180;
				}
				if(angleInXZ >= 180){
					angleInXZ -= 180;
				}
				// if (manifold[i]->getContactPoint(0).m_normalWorldOnB.getX() == 0) {
				// 	angleInXZ = 90;
				// }

				for (int i=0 ; i < 18 ; i++){
					if ((angleInXZ >= i*10) && (angleInXZ < (i+1)*10)){
						contactForce[i] += forceBuffer;
					}
				}
			}
		}

		for (int i=0 ; i < 18; i++){
			polarForce << (lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX()) << " " << i*10 << " " << contactForce[i] << std::endl;
			polarForce << (lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX()) << " " << (i+1)*10 << " " << contactForce[i] << std::endl;
		}




		polarForce.close();
	}
	checkpointTime12 = floor(simTime/(writeOutputInterval*10));
}

//For getting the amount of rotations of each bead in total sample and in RVE
void beadRotationWriter(btAlignedObjectArray<btRigidBody*> rigidBody, int NumberOfRigidBodies, btScalar simTime, btScalar shearingTime, btScalar Xmin, btScalar Xmax, btScalar Ymin, btScalar Ymax, btScalar Zmin, btScalar Zmax){

	if (simTime == 0){
		beadRotationOutput.open("beadRotation.txt");
		beadRotationOutput << "time(s)" << " " << "shearinDisp" << " " << "totalRot(deg)" << " " << "RVErot(deg)" << " " << "numRVEBeds" << std::endl;
	}

	btScalar 	totalRotation=0;
	btScalar 	RVErotation=0;
	btScalar	numRVEBeds=0;

	checkpointTime13 = floor(simTime/writeOutputInterval);
	if (checkpointTime13 > checkpointTime14 || simTime == 0 ){

		for (int j =0; j < NumberOfRigidBodies; j++){

			totalRotation += btFabs(btDegrees(rigidBody[j]->getWorldTransform().getRotation().getAngle()));

			if ( (rigidBody[j]->getWorldTransform().getOrigin().getX() > Xmin) &&
				(rigidBody[j]->getWorldTransform().getOrigin().getX() < Xmax) &&
				(rigidBody[j]->getWorldTransform().getOrigin().getY() > Ymin) &&
				(rigidBody[j]->getWorldTransform().getOrigin().getY() < Ymax) &&
				(rigidBody[j]->getWorldTransform().getOrigin().getZ() > Zmin) &&
				(rigidBody[j]->getWorldTransform().getOrigin().getZ() < Zmax)
				){

				numRVEBeds ++;
				RVErotation +=  btFabs(btDegrees(rigidBody[j]->getWorldTransform().getRotation().getAngle()));
			}
		}

		beadRotationOutput << simTime << " " << (lowerBoxBuffer.getOrigin().getX() - upperBoxRBBuffer.getOrigin().getX()) << " " << totalRotation << " " << RVErotation << " " << numRVEBeds << std::endl;

	}
	checkpointTime14 = floor(simTime/(writeOutputInterval));

	if (simTime == shearingTime){
		beadRotationOutput.close();
	}


}

#endif