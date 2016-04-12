#include <iostream>
#include <iomanip>
#include <fstream>	// for output writing
#include <chrono>	// for timing
//#define BT_USE_DOUBLE_PRECISION			// to use double preciison, you have to uncomment this line "before" bullet header
#include <btBulletDynamicsCommon.h>
// My own headers
#include "variables.h"			// contains the global variables needed for direct shear test
#include "writers.h"			//  contains the writers functions for visualisations in Paraview
// For stable and efficient spring simulation
#include "../.internalLibs/btGeneric6DofSpring2Constraint.h"


/*######	TODO 	#####
	1)	Effect of using different solvers, MLCP/SI/FeatherStone/etc.
	2)	Effect of parallelization / multithreading.
*/


int main() {

	// starting time point.
	const auto start_time = std::chrono::steady_clock::now();

	//########## BULLET SETTINGS ##########

		//Broadphase
		btBroadphaseInterface* broadphase=new btDbvtBroadphase(); 	

		//Narrowphase (dispatcher)
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

		//Solver
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		//Dynamics world
		btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
		dynamicsWorld -> setGravity(gravity);

			//In the case that user change the number of solver iterations (default is 20)
			btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
			info.m_numIterations = numberOfIterations;

		std::cout << "The Bullet settings are made successfully..." << std::endl;

	//########## OBJECTS ##########

	
	
	//****	1) Top plate setup
		btCollisionShape* topPlateShape	= new btBoxShape(btVector3(boxSize/2.f - topPlateGap, boxSize/2.f - topPlateGap, topPlateThickness/2.f));
		btDefaultMotionState* topPlateState = new btDefaultMotionState(btTransform(upright, btVector3(0, 0, boxHeight + topPlateThickness)));
		topPlateShape->calculateLocalInertia(topPlateMass, topPlateInertia);
		btRigidBody* topPlateRB = new btRigidBody(topPlateMass, topPlateState, topPlateShape, topPlateInertia);
		dynamicsWorld->addRigidBody(topPlateRB);
		topPlateRB->setFriction(btSqrt(boundaryFrict));
		topPlateRB->setRestitution(restitutionCoef);
		topPlateRB->setActivationState(4);
		topPlateRB->setLinearFactor(topPlateLinearFactor);
		topPlateRB->setAngularFactor(topPlateAngularFactor);
		topPlateRB->setRollingFriction(btSqrt(rollingFriction));

		std::cout << "The top plate is generated..." << std::endl;

	//****	2) Metal beads setup
		btCollisionShape* metalBeadShape	= 	new btSphereShape(metalBeadDiameter/2.f);
		metalBeadShape->calculateLocalInertia(metalBeadMass, metalBeadInertia);

		btAlignedObjectArray<btDefaultMotionState*> metalBeadState;
		metalBeadState.resize(approxNumBeads);

		btAlignedObjectArray<btRigidBody*>	metalBeadRB;
		metalBeadRB.resize(approxNumBeads);

		// a loop for generation of metal beads in their correct position, in touching state, without any overlapping.
		for (zBeadPos; zBeadPos<=verBoxLimit;) {

			for (yBeadPos; yBeadPos <= horBoxLimit;) {

				for(xBeadPos; xBeadPos <= horBoxLimit;) {
					beadIndex++;								//adds one to the number of beads index
					metalBeadState[beadIndex]	= new 	btDefaultMotionState(btTransform(upright, btVector3(xBeadPos, yBeadPos, zBeadPos)));
					metalBeadRB[beadIndex] 		= new 	btRigidBody(metalBeadMass, metalBeadState[beadIndex], metalBeadShape, metalBeadInertia);
					dynamicsWorld->addRigidBody(metalBeadRB[beadIndex]);

					metalBeadRB[beadIndex]->setFriction(btSqrt(internalFrict));
					metalBeadRB[beadIndex]->setRollingFriction(btSqrt(rollingFriction));
					metalBeadRB[beadIndex]->setRestitution(restitutionCoef);
					metalBeadRB[beadIndex]->setActivationState(4);
					//std::cout << beadIndex << " ### " << xBeadPos << " ### " << yBeadPos << " ### " << zBeadPos <<  std::endl;

					xBeadPos= xBeadPos + metalBeadDiameter;		//x coordination will be set for the next bead in x direction

					if (yBeadPos > boxFittingYDir ) {
						boxFittingYDir = yBeadPos;
					}

				}
				// shift control for x coordination of beads positions
				if (xCoordControl == false) {(xCoordControl= true);} else {(xCoordControl = false);}
				xBeadPos	= -boxSize/2.f + metalBeadDiameter/2.f + ((xCoordControl==true) ? (metalBeadDiameter/2.f):(0) );//bringing back the x coordination for the next row of beads
				yBeadPos	= yBeadPos + pyramidSlant;	//y increment for y coordination of beads
			}

			if (zCoordControl == false) {(zCoordControl= true);} else {(zCoordControl = false);}

			xBeadPos = -boxSize/2.f + metalBeadDiameter/2.f;
			yBeadPos = -boxSize/2.f + metalBeadDiameter/2.f + ((zCoordControl==true) ? (pyramidSlant - (btTan(btRadians(30)) * metalBeadDiameter/2.f) ):(0) );
			xCoordControl=false;
			zBeadPos = zBeadPos + pyramidHeight;		// z increment for z coordination of beads
		}

		boxFittingYDir = boxSize/2 - (boxFittingYDir + metalBeadDiameter/2);
		//std::cout << "last y coor : " << boxFittingYDir<< std::endl;

		exactNumBead = beadIndex +1;	// exact number of beads

		std::cout << "Total number of " << exactNumBead << " metal beads are generated." << std::endl; 

	//****	3) ShearBox setup
		btCompoundShape* 	lowerBoxShape	=	new btCompoundShape();
		btCompoundShape* 	upperBoxShape	=	new btCompoundShape();	

			// an array of collision shapes named"shearBoxShape" is defined with 9 members
		btAlignedObjectArray<btCollisionShape*> shearBoxShape;
		shearBoxShape.resize(10);
		//btCollisionShape*	shearBoxShape[8];
		// note that for upper box shapes, a height of 1.5 times of lower box height is considered.
		// bottom box shape size: index 0
		shearBoxShape[0]	= new	btBoxShape(btVector3(boxSize/2.f + boxThickness, boxSize/2.f + boxThickness, boxThickness/2.f));
		// x-side box shape size: for both side boxes: 1 (lower box front); 2 (lower box rear); 5 (upper box front); 6 (upper box rear)
		shearBoxShape[1]	=	new	btBoxShape(btVector3(boxThickness/2.f, boxSize/2.f + boxThickness, boxHeight/4.f));
		shearBoxShape[2]	=	new	btBoxShape(btVector3(boxThickness/2.f, boxSize/2.f + boxThickness, boxHeight/4.f));
		shearBoxShape[3]	=	new	btBoxShape(btVector3(boxSize/2.f, boxThickness/2.f, boxHeight/4.f));
		shearBoxShape[4]	=	new	btBoxShape(btVector3(boxSize/2.f, boxThickness/2.f, boxHeight/4.f));
		shearBoxShape[5]	=	new	btBoxShape(btVector3(boxThickness/2.f, boxSize/2.f + boxThickness, 1.25 * boxHeight/4.f));
		shearBoxShape[6]	=	new	btBoxShape(btVector3(boxThickness/2.f, boxSize/2.f + boxThickness, 1.25 * boxHeight/4.f));
		shearBoxShape[7]	=	new	btBoxShape(btVector3(boxSize/2.f, boxThickness/2.f, 1.25 * boxHeight/4.f));
		shearBoxShape[8]	=	new	btBoxShape(btVector3(boxSize/2.f, boxThickness/2.f, 1.25 * boxHeight/4.f));

		
		shearBoxOffset.resize(10);
		// fill all the arrays with zero magnitudes:
		for (int i=0; i<9; i++){
			shearBoxOffset[i].setIdentity();
		}
		//lowerBox offsets
		shearBoxOffset[0].setOrigin(btVector3(0, 0, -((boxThickness/2.f) + (boxHeight/4.f)) ));
		shearBoxOffset[1].setOrigin(btVector3(boxSize/2.f + boxThickness/2.f, 0 , 0));
		shearBoxOffset[2].setOrigin(btVector3(-(boxSize/2.f + boxThickness/2.f), 0, 0));
		shearBoxOffset[3].setOrigin(btVector3(0, boxSize/2.f + boxThickness/2.f - boxFittingYDir, 0));
		shearBoxOffset[4].setOrigin(btVector3(0, -(boxSize/2.f + boxThickness/2.f), 0));
		//upperBox offsets
		shearBoxOffset[5].setOrigin(btVector3(boxSize/2.f + boxThickness/2.f, 0, 0));
		shearBoxOffset[6].setOrigin(btVector3(-(boxSize/2.f + boxThickness/2.f), 0, 0));
		shearBoxOffset[7].setOrigin(btVector3(0, boxSize/2.f + boxThickness/2.f - boxFittingYDir, 0));
		shearBoxOffset[8].setOrigin(btVector3(0, -(boxSize/2.f + boxThickness/2.f), 0));

		// assigning the child shapes of lowerBox and upperBox
		for (int i=0 ; i<9; i++){
			if(i<5){
				lowerBoxShape->addChildShape(shearBoxOffset[i], shearBoxShape[i]);
			}else{
				upperBoxShape->addChildShape(shearBoxOffset[i], shearBoxShape[i]);
			}

		}

		//motionStates of lower and upper boxes
		btDefaultMotionState* lowerBoxState	=	new btDefaultMotionState(btTransform(upright, btVector3(0, 0, boxHeight/4.f)));
		btDefaultMotionState* upperBoxState	=	new btDefaultMotionState(btTransform(upright, btVector3(0, 0, 3.25 * boxHeight/4.f + boxGap)));

		// rigid body construction infos, definitions, adding them to the world, and the settings
		upperBoxShape->calculateLocalInertia(upperBoxMass, upperBoxInertia);
		btRigidBody* lowerBoxRB = new btRigidBody(0, lowerBoxState, lowerBoxShape, btVector3(0,0,0));
		btRigidBody* upperBoxRB = new btRigidBody(upperBoxMass, upperBoxState, upperBoxShape, upperBoxInertia);
		dynamicsWorld->addRigidBody(lowerBoxRB);
		dynamicsWorld->addRigidBody(upperBoxRB);

		lowerBoxRB->setRestitution(restitutionCoef);
		upperBoxRB->setRestitution(restitutionCoef);

		lowerBoxRB->setFriction(btSqrt(boundaryFrict));	// in Bullet friction ratio of a contact is a multiplication of both frictions
		upperBoxRB->setFriction(btSqrt(boundaryFrict));

		lowerBoxRB->setActivationState(4);
		upperBoxRB->setActivationState(4);

		upperBoxRB->setRollingFriction(btSqrt(rollingFriction));
		lowerBoxRB->setRollingFriction(btSqrt(rollingFriction));


		//restrains:
		upperBoxRB->setLinearFactor(upperBoxLinearFactor);		//upper box only can move in linear x direction 
		upperBoxRB->setAngularFactor(upperBoxAngularFactor);

		// dynamicsWorld->updateSingleAabb(lowerBoxRB);		// update the compound shape 
		// dynamicsWorld->updateSingleAabb(upperBoxRB);		// update the compound shape 

		std::cout << "The upper and lower boxes are generated..." << std::endl;

	//****	4) Support setup
		btCollisionShape* supportShape = new btBoxShape(btVector3(boxThickness/2.f, boxSize/2.f, 1.25 * boxHeight/4.f));
		btDefaultMotionState* supportState = new btDefaultMotionState(btTransform(upright, btVector3(2 * boxSize + boxThickness, 0, 3.25 * boxHeight/4.f + boxGap)));
		btRigidBody* supportRB = new btRigidBody(0.0, supportState, supportShape, btVector3(0,0,0));
		if (strainControlledTest == false){
			dynamicsWorld->addRigidBody(supportRB);
		}
		supportRB->setActivationState(4);

	//****	5) Spring setup
		btGeneric6DofSpring2Constraint* springRB = new btGeneric6DofSpring2Constraint(*supportRB, *upperBoxRB, springToSupport, springToUpperBox);
		//setting the spring ranges
		springRB->setLinearUpperLimit(btVector3(boxSize, boxSize, 0.f));
		springRB->setLinearLowerLimit(btVector3(-boxSize, -boxSize, 0.f));
		springRB->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
		springRB->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));

		if (strainControlledTest == false){
			dynamicsWorld->addConstraint(springRB, true);	//2nd parameter: true means collision between two bodies is disabled.
		}
		springRB->enableSpring(0, true);				// 0 means in x direction spring is enabled
		springRB->setStiffness(0, springStiffness);	//change the units from [N/m] to [N/mm]
		springRB->setDamping(0, springDamping);				//change the units from [N.s/m] to [N.s/mm]
		springRB->setEquilibriumPoint(); 			//the current length of the spring will be considered as the natural length.

	//****	6) Dual fixed spheres setup (to make two-sided contact between upperBox and dual spheres in x direction):: for strain controlled test
		btCollisionShape* fixedSphereShape = new btSphereShape(fixedSphereRadius);

		//Firstly the fixed spheres are simulated without any gap compared to upperBox
		btDefaultMotionState* forwardFixedSphereState = new btDefaultMotionState(btTransform(upright, btVector3(boxSize/2.f + boxThickness + fixedSphereRadius, 0, 3.25 * boxHeight/4.f + boxGap)));
		btDefaultMotionState* rearFixedSphereState = new btDefaultMotionState(btTransform(upright, btVector3(-boxSize/2.f - boxThickness - fixedSphereRadius, 0, 3.25 * boxHeight/4.f + boxGap)));

		btRigidBody* forwardFixedSphereRB = new btRigidBody(0.0, forwardFixedSphereState, fixedSphereShape, btVector3(0,0,0));
		btRigidBody* rearFixedSphereRB = new btRigidBody(0.0, rearFixedSphereState, fixedSphereShape, btVector3(0,0,0));

		if (strainControlledTest == true){
			dynamicsWorld->addRigidBody(forwardFixedSphereRB);
			dynamicsWorld->addRigidBody(rearFixedSphereRB);

		}
		forwardFixedSphereRB->setActivationState(4);
		rearFixedSphereRB->setActivationState(4);


	std::cout << "Starting Bullet simulation..." << std::endl;

	//Run


		std::cout << "Consolidation phase started ... " << std::endl;
		//## Consolidation phase
		for (int i=0; i<consolidationSteps ; i++){

			//real gravitational force on beads
			for (int i=0; i< exactNumBead; i++){
				metalBeadRB[i]->applyCentralForce(btVector3(0,0,-metalBeadMass*9.81f/1000));
			}

			simTime = deltaTime*i;

			topPlateRB->applyCentralForce(btVector3(0,0, (normalForce/consolidationTime)*simTime ));

			//to make sure that upperbox is fixed during consolidation
			upperBoxRB->setWorldTransform(btTransform(upright,btVector3(0,0,3.25 * boxHeight/4.f + boxGap)));

			consolidationOutputWriter(simTime, consolidationTime, topPlateRB);	// write the data of the consolidation curve

			dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
		}

		topPlateBasePos = 	topPlateRB->getWorldTransform();	//state of the top plate is recorded for the next phase

		
		std::cout << "Shearing phase started ... " << std::endl;
		//## Shearing phase
		for (int i=0; i<shearingSteps ; i++){

			simTime = deltaTime*i;
			topPlateRB->applyCentralForce(btVector3(0,0,normalForce));

			//real gravitational force on beads
			for (int i=0; i< exactNumBead; i++){
				metalBeadRB[i]->applyCentralForce(btVector3(0,0,-metalBeadMass*9.81f/1000));
			}

			//application of shearing displacement
			lowerBoxBuffer.setIdentity();
			lowerBoxBuffer.setOrigin(btVector3(simTime*(shearDisplacement/shearingTime), 0, boxHeight/4.f));
			lowerBoxRB->setWorldTransform(lowerBoxBuffer);

			//Buffers for positions and rotations defined here:
			upperBoxRBBuffer 	=	upperBoxRB->getWorldTransform();
			topPlateBuffer		=	topPlateRB->getWorldTransform();

			beadPositionWriter(metalBeadRB, exactNumBead, simTime, lowerBoxRB); // writes the CSV data for paraview visualisation
			contactDataWriter(dynamicsWorld,simTime, deltaTime);	// writes the CSV data of contact positions and their normal and frictional forces

			if (strainControlledTest == false){
				shearOutputWriter_stressControlled(simTime, shearingTime);		// writes the data of the stress strain curve
			} else {
				shearOutputWriter_strainControlled(simTime, shearingTime, dynamicsWorld, deltaTime);		// writes the data of the stress strain curve
			}

			contactNumberRVE(simTime, shearingTime, dynamicsWorld, 
				-boxSize/2 - boxThickness, 
				boxSize/2 + boxThickness, 
				-boxSize/2 - boxThickness, 
				boxSize/2 + boxThickness, 
				boxHeight/2 + metalBeadDiameter/10, 
				boxHeight/2 + boxGap);

			forceRatios(simTime,shearingTime,dynamicsWorld,
				-boxSize/2 - boxThickness, 
				boxSize/2 + boxThickness, 
				-boxSize/2 - boxThickness, 
				boxSize/2 + boxThickness, 
				boxHeight/2 + metalBeadDiameter/10, 
				boxHeight/2 + boxGap);
			polarForceData(dynamicsWorld, simTime, deltaTime);

			beadRotationWriter(metalBeadRB, exactNumBead, simTime, shearingTime, 
				-boxSize/2, 
				boxSize/2, 
				-boxSize/2, 
				boxSize/2, 
				boxHeight/2 - metalBeadDiameter/2, 
				boxHeight/2 + boxGap + metalBeadDiameter/2);

			dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
		}
	

		std::cout << "Simulation finished...\nDeallocating..." << std::endl;


	//cleaning up allocated memories

		delete 		fixedSphereShape;

		delete 		forwardFixedSphereState;
		delete 		rearFixedSphereState;

		delete 		forwardFixedSphereRB;
		delete 		rearFixedSphereRB;

		delete 		springRB;

		delete 		supportRB;
		delete 		supportState;
		delete 		supportShape;

		for (int i=0; i<approxNumBeads; i++){
			delete 	metalBeadRB[i];
		}

		for (int i=0; i<approxNumBeads; i++){
			delete 	metalBeadState[i];
		}

		delete 		metalBeadShape;

		delete 		topPlateRB;
		delete 		topPlateState;
		delete 		topPlateShape;

		delete 		upperBoxRB;
		delete 		lowerBoxRB;

		delete 		upperBoxState;
		delete 		lowerBoxState;

		delete 		upperBoxShape;
		delete 		lowerBoxShape;

		for (int i=0; i<9; i++){
			delete 	shearBoxShape[i];
		}

		delete 		dynamicsWorld;
		delete 		solver;
		delete 		dispatcher;
		delete 		collisionConfiguration;
		delete 		broadphase;
	
		std::cout << "Bullet simulation ended successfully!" << std::endl;

		//end point time + timing information at the end.
		double time_in_seconds = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now() - start_time).count() 	/ 1000.0;
		std::cout << "=====================================" << std::endl;
		std::cout << "Total elapsed time : " << time_in_seconds << "s" << std::endl;
		std::cout << "Simulation time: " << endTime << "s" << std::endl;
		std::cout << "Number of rigid bodies: " << exactNumBead + 5 << std::endl;

	return 0;
}