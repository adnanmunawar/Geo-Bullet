#include <iostream>
#include <chrono>	// for timing
#include <btBulletDynamicsCommon.h>
#include "variables.h"					// variables
#include "softBodyWriter.h"				// contains the writers functions for visualisations in Paraview
#include "cylindricalBeadPacking.h"		// the function for generation of rhombic sphere packing in cylindrical shape of the triaxial sample
#include "rigidBodyWriters.h"			// contains the writers needed for rigid bodies
#include "../.internalLibs/utility.h"					// some c++ functions such as progress monitor
#include "../.internalLibs/STLSoftImporter.h"	// contains the functions needed to import STLmesh as a soft body
#include "membrane.h"					// membrane functions (maker and radial force applicator)




/*###### IMPORTANT POINT ######
	The linear stiffness of the softbody depends both on mass of the particles (more
	mass, more stiffness) and the amount of linear stiffness. Note that the springs are
	connected to each other in series.
#############################*/

int main() {

	// starting time point.
	const auto start_time = std::chrono::steady_clock::now();
	
	//Bullet Basic Settings
		//Broadphase
		btBroadphaseInterface* broadphase=new btDbvtBroadphase(); 	

		//Narrowphase (dispatcher)
		btDefaultCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration(); // CHANGED!

		btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

		//Solver
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
		btSoftBodySolver*	softBodySolver = new btDefaultSoftBodySolver();		//CHANGED!

		//Dynamics world
		btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration,softBodySolver); //ADDED!
		dynamicsWorld->setGravity(gravity);


		//In the case that user change the number of solver iterations (default is 20)
			btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
			info.m_numIterations = numberOfIterations;

	std::cout << "The Bullet settings are made successfully..." << std::endl;

	//###### Objects:

		//	1- Membrane

			btSoftBody* membrane;
			STLImporterToSoftBody(dynamicsWorld, membrane, "membrane-80x80-radius25.8.stl");

			/* Sort of collision between soft body and rigid body 
			Options are: 
							1- RVSmask	: Rigid versus soft mask
							2- SDF_RS	: SDF based rigid vs soft
							3- CL_RS 	: Cluster vs convex rigid vs soft

							4- SVSmask	: Soft versus soft mask	
							5- VF_SS	: Vertex vs face soft vs soft handling
							6- CL_SS 	: Cluster vs cluster soft vs soft handling
							7- CL_SELF	: Cluster soft body self collision

			Attention! in the case of using clusters, you have to:
			'softbody->generateClusters(0);'
			Note 1: 	generateClusters with k=0 will create a convex cluster for each tetrahedron or triangle
						otherwise an approximation will be used (better performance)


			Note 2: 	You can use a combination of collision algorithms to gain better collision detection.

			Note 3: 	Based on the experience, the best method for membrane shaped softbodies is SDF_RS. Use it.

			*/
			membrane->m_cfg.collisions		=	btSoftBody::fCollision::SDF_RS;

			dynamicsWorld->addSoftBody(membrane);

			membrane->getWorldInfo()->m_gravity	=	gravity;

			membrane->m_cfg.viterations = 	membraneVelocityIteration;
			membrane->m_cfg.piterations = 	membranePositionIteration;
			membrane->m_cfg.diterations	= 	membraneDriftSolverIteration;
			membrane->m_cfg.citerations	= 	membraneClusterSolverIteration;

			// do not uncomment the block below, because it makes double links, the bending stiffness is recommended for closed shape soft bodies
			/* btSoftBody::Material* 	pm = membrane->appendMaterial();
			pm->m_kLST		=	membraneBendingStiffness;
			membrane->generateBendingConstraints(2,pm);
			*/
			membrane->m_materials[0]->m_kLST	= 	membraneLinearStiffness;
			membrane->m_materials[0]->m_kAST	= 	membraneAngularStiffness;
			membrane->m_materials[0]->m_kVST	= 	membraneVolumetricStiffness;
			membrane->m_cfg.kDP					= 	membraneDampingFactor;
			membrane->m_cfg.kDF 				=	 membraneDynamicFriction;


			membrane->setTotalMass(membraneTotalMass);

			std::cout << "The softbody-based membrane is created with radius of " << membraneRadius << " and Height  of " << membraneHeight << " and total nodes of " << membrane->m_nodes.size()  << std::endl; 			



		//	2- Beads
			btCollisionShape* metalBeadShape	= 	new btSphereShape(metalBeadDiameter/2.f);
			metalBeadShape->calculateLocalInertia(metalBeadMass, metalBeadInertia);

			btAlignedObjectArray<btVector3*>	beadsCenter;
			beadsCenter.resize(approxNumBeads);

			for (int i = 0 ; i < approxNumBeads ; i++){
				beadsCenter[i] = new btVector3(0,0,0);
			}

			//Here the sphere packing function goes ...
			cylindricalSphereClosePacking(beadsCenter, sampleRadius, sampleHeight, metalBeadDiameter, beadIndex, topmostBeadCenter);

			exactNumBead = beadIndex +1;	// exact number of beads

			btAlignedObjectArray<btDefaultMotionState*> metalBeadState;
			metalBeadState.resize(beadIndex);

			btAlignedObjectArray<btRigidBody*>	metalBeadRB;
			metalBeadRB.resize(beadIndex);


			//loop for creation of rigid bodies and their corresponding characteristics
			for(int i=0; i < beadIndex; i++){
				metalBeadState[i]	= new 	btDefaultMotionState(btTransform(upright, *beadsCenter[i]));
				metalBeadRB[i] 		= new 	btRigidBody(metalBeadMass, metalBeadState[i], metalBeadShape, metalBeadInertia);
				dynamicsWorld->addRigidBody(metalBeadRB[i]);

				metalBeadRB[i]->setFriction(btSqrt(internalFrict));
				metalBeadRB[i]->setRollingFriction(btSqrt(rollingFriction));
				metalBeadRB[i]->setRestitution(restitutionCoef);
				metalBeadRB[i]->setActivationState(4);
			}

			std::cout << "Total number of " << exactNumBead << " metal beads are generated." << std::endl; 

			sampleHeight = topmostBeadCenter + metalBeadDiameter/2; // correcting the sample height (for strain calculation)

		//	3-	Circular top and bottom plates
			btCollisionShape* plateShape = new btCylinderShapeZ(btVector3(sampleRadius, sampleRadius, plateThickness/2));
			btDefaultMotionState* topPlateState = new btDefaultMotionState(btTransform(upright, btVector3(0 , 0, sampleHeight +  plateThickness/2)));
			btDefaultMotionState* bottomPlateState = new btDefaultMotionState(btTransform(upright, btVector3(0, 0, -plateThickness/2)));
			plateShape->calculateLocalInertia(plateMass, plateInertia);

			btRigidBody* topPlateRB = new btRigidBody(plateMass, topPlateState, plateShape, plateInertia);		//top plate is a dynamic object
			btRigidBody* bottomPlateRB = new btRigidBody(plateMass, bottomPlateState, plateShape, plateInertia);	//bottom plate is a dynamic object

			dynamicsWorld->addRigidBody(topPlateRB);
			dynamicsWorld->addRigidBody(bottomPlateRB);
			//setting parameters
			topPlateRB->setRestitution(restitutionCoef);
			topPlateRB->setFriction(internalFrict);
			topPlateRB->setActivationState(4);
			bottomPlateRB->setActivationState(4);
			bottomPlateRB->setRestitution(restitutionCoef);
			bottomPlateRB->setFriction(internalFrict);
			//DOFs
			topPlateRB->setLinearFactor(plateLinearFactor);
			topPlateRB->setAngularFactor(plateAngularFactor);
			bottomPlateRB->setLinearFactor(plateLinearFactor);
			bottomPlateRB->setAngularFactor(plateAngularFactor);

			std::cout << "The top and bottom plates are generated ..." << std::endl;

		//	4-	Spherical support (for detecting the global force)
			btCollisionShape* fixedSphereShape = new btSphereShape(fixedSphereRadius);
			btDefaultMotionState* fixedSphereState = new btDefaultMotionState(btTransform(upright, btVector3(0, 0, -(plateThickness + fixedSphereRadius))));
			btRigidBody* fixedSphereRB = new btRigidBody(0.0, fixedSphereState, fixedSphereShape, btVector3(0,0,0));
			dynamicsWorld->addRigidBody(fixedSphereRB);
			fixedSphereRB->setActivationState(4);

			btDefaultMotionState* fixedPlusSphereState = new btDefaultMotionState(btTransform(upright, btVector3(0, 0, topmostBeadCenter + metalBeadDiameter/2 +  plateThickness + fixedSphereRadius)));
			btRigidBody* fixedPlusSphereRB = new btRigidBody(0.0, fixedPlusSphereState, fixedSphereShape, btVector3(0,0,0));
			dynamicsWorld->addRigidBody(fixedPlusSphereRB);
			fixedPlusSphereRB->setActivationState(4);

			std::cout << "The spherical bottom support is generated ..." << std::endl;

	//Run
	std::cout << "Starting Bullet simulation ..." << std::endl;

		//## Circumferential Consolidation phase
		std::cout << "Circumferential consolidation phase started ... " << std::endl;
		for (int i=1; i <= consolidationSteps ; i++){
			
			softBodyPositionWriter(membrane, membrane->m_nodes.size(), simTime, 1);
			softBodyDeformationWriter(membrane, membrane->m_links.size(), simTime, 1);
			beadPositionWriter(metalBeadRB, beadIndex, simTime, 1);
			confiningStressOnMembrane(membrane, bottomPlateRB, topPlateRB, confiningStress, consolidationTime, simTime);
			stressOnPlate(topPlateRB, confiningStress, sampleRadius, consolidationTime, simTime);
			detectVerticalForce(dynamicsWorld, topPlateRB, simTime, consolidationTime, 0.1, consolidationOutput);
			//membrane->setPose(true,true);
			simTime	=	i*deltaTime;
			dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
			showProgress(simTime, consolidationTime);	// to show the progress of application of load in whole the consolidation

		}

		std::cout << "\nConverting top platen from a dynamic object to kinematic object for shearing ... " << std::endl;
		// transforming the top plate from a dynamic object to a kinematic object (for application of deviatoric strain).
			btTransform topPlatePosition = topPlateRB->getWorldTransform();

			dynamicsWorld->removeRigidBody(topPlateRB);

			topPlateState = new btDefaultMotionState(topPlatePosition);
			topPlateRB = new btRigidBody(0.0, topPlateState, plateShape, btVector3(0,0,0));
			dynamicsWorld->addRigidBody(topPlateRB);

		// Appending head and tail of membrane to top and bottom platens, respectively
		std::cout << "Constraining head and tail of membrane to top and bottom platens ... " << std::endl;

			// Constraining head of membrane
			for (int i=0; i < membrane->m_nodes.size(); i++){
				if (membrane->m_nodes[i].m_x.z() > (topPlateRB->getWorldTransform().getOrigin().getZ()) &&
					membrane->m_nodes[i].m_x.z() < (topPlateRB->getWorldTransform().getOrigin().getZ() + plateThickness/2)) {
					membrane->appendAnchor(i, topPlateRB);
				}
			}

			// Constraining head of membrane
			for (int i=0; i < membrane->m_nodes.size(); i++){
				if (membrane->m_nodes[i].m_x.z() < (bottomPlateRB->getWorldTransform().getOrigin().getZ()) &&
					membrane->m_nodes[i].m_x.z() > (bottomPlateRB->getWorldTransform().getOrigin().getZ() - plateThickness/2)) {
					membrane->appendAnchor(i, bottomPlateRB);
				}
			}

		writeOutputInterval = writeOutputInterval*10;	// the length of recording set to be every second
		simTime = 0; 									//setting the simTime again to zero fro starting the next phase

		std::cout << "Shearing phase started ... " << std::endl;		
		for (int i=1; i <= shearingSteps ; i++){
			keepStressOnMembrane(membrane, bottomPlateRB, topPlateRB, confiningStress);
			//membrane->setPose(true,true);
			//keepStressOnPlate(topPlateRB, confiningStress, sampleRadius);
			applyDeviatoricStrain(topPlateRB, maxShearDisplacement, shearingTime, deltaTime);
			softBodyPositionWriter(membrane, membrane->m_nodes.size(), simTime, 2);
			softBodyDeformationWriter(membrane, membrane->m_links.size(), simTime, 2);
			beadPositionWriter(metalBeadRB, beadIndex, simTime, 2);
			detectVerticalForce(dynamicsWorld, topPlateRB, simTime, shearingTime, 0.25, shearingOutput);
			simTime	=	i*deltaTime;
			dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
			showProgress(simTime, shearingTime);
		}
		
		std::cout << "Bullet simulation is finalized successfully ... " << std::endl;

	//Deallocation
		std::cout << "Memory deallocation ... " << std::endl;
		delete 	fixedPlusSphereRB;
		delete 	fixedSphereRB;
		delete 	fixedPlusSphereState;
		delete 	fixedSphereState;
		delete 	fixedSphereShape;

		delete 	bottomPlateRB;
		delete 	topPlateRB;
		delete 	bottomPlateState;
		delete 	topPlateState;
		delete 	plateShape;

		for (int i=0; i<beadIndex; i++){
			delete 	metalBeadRB[i];
		}

		for (int i=0; i<beadIndex; i++){
			delete 	metalBeadState[i];
		}

		for (int i=0; i<approxNumBeads; i++){
			delete 	beadsCenter[i];
		}

		delete 	metalBeadShape;
		delete 	membrane;
		delete 	dynamicsWorld;
		delete 	softBodySolver;
		delete 	solver;
		delete 	dispatcher;
		delete 	collisionConfiguration;
		delete 	broadphase;

		std::cout << "Bullet simulation ended successfully!" << std::endl;

	//end point time + timing information at the end.
		double time_in_seconds = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now() - start_time).count() 	/ 1000.0;
		std::cout << "=====================================" << std::endl;
		std::cout << "Total elapsed time : " << time_in_seconds << "s" << std::endl;
		std::cout << "Simulation time: " << endTime << "s" << std::endl;
		std::cout << "Number of rigid bodies: " << exactNumBead + 4 << std::endl;

	return 0;
}