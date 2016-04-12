#include <iostream>
#include <btBulletDynamicsCommon.h>

#include <cmath> // needed for sqrt (for frequency calculation)
// libraries for using btGeneric6DofSpring2Constraint (the recent spring)
#include "../../../Bullet/bullet3/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

/*#######	Important note 	#######
	The 'btGeneric6DofSpring2Constraint' seems more suitable for engineering simulations.
	Because the 'btGeneric6DofSpringConstraint' code defines damping as a dependent 
	parameter to the size of time increment which makes it very very unsuitable for 
	engineering simulations.
*/

int main() {
	//Broadphase
	btBroadphaseInterface* broadphase=new btDbvtBroadphase(); 	

	//Narrowphase (dispatcher)
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	//Solver
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	//Dynamics world
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	dynamicsWorld -> setGravity(btVector3(0,0,0));

	//########## OBJECTS ##########
	//Base Box (fixed)
		btCollisionShape* baseBoxShape = new btBoxShape(btVector3(0.5,0.5,0.5));
		btDefaultMotionState* baseBoxState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(-5,0,0))); //the surface of the tilting plane has zero altitude
		btRigidBody::btRigidBodyConstructionInfo baseBoxRigidBodyCI(0, baseBoxState, baseBoxShape, btVector3(0,0,0));
		btRigidBody* baseBoxRigidBody = new btRigidBody(baseBoxRigidBodyCI);
		dynamicsWorld->addRigidBody(baseBoxRigidBody);
		baseBoxRigidBody->setFriction(0.0);
		baseBoxRigidBody->setRestitution(0.0);

	//Vibrating Box (moving)
		btCollisionShape* movingBoxShape = new btBoxShape(btVector3(0.5,0.5,0.5));
		btCollisionShape* movingBoxShape2= new btBoxShape(btVector3(0.2,0.2,0.2));
		btDefaultMotionState* movingBoxState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
		btDefaultMotionState* movingBoxState2 = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,-2)));

		
		btScalar 	boxMass=5.0;
		btVector3 	boxInertia(0, 0, 0);
		movingBoxShape->calculateLocalInertia(boxMass, boxInertia);
		movingBoxShape2->calculateLocalInertia(boxMass, boxInertia);

		
		btRigidBody::btRigidBodyConstructionInfo movingBoxRigidBodyCI(boxMass, movingBoxState, movingBoxShape, boxInertia);
		btRigidBody* movingBoxRigidBody = new btRigidBody(movingBoxRigidBodyCI);
		dynamicsWorld->addRigidBody(movingBoxRigidBody);
		btRigidBody::btRigidBodyConstructionInfo movingBoxRigidBodyCI2(boxMass, movingBoxState2, movingBoxShape2, boxInertia);
		btRigidBody* movingBoxRigidBody2 = new btRigidBody(movingBoxRigidBodyCI2);
		dynamicsWorld->addRigidBody(movingBoxRigidBody2);

		movingBoxRigidBody->setFriction(0.0);
		movingBoxRigidBody->setRestitution(0.0);
		movingBoxRigidBody->setActivationState(4);					//make the object always alive, so it will not sleep.
		movingBoxRigidBody->setAngularFactor(btVector3(0,0,0));
		movingBoxRigidBody->setLinearFactor(btVector3(1,0,0)); 		//all rotations and translations in y and z restrained, so, the box can move only in x direction. This reduces the calculation process a lot, and it is more efficient than restraining the DOFs by conventional constraints.
		movingBoxRigidBody2->setFriction(0.0);
		movingBoxRigidBody2->setRestitution(0.0);
		movingBoxRigidBody2->setActivationState(4);	

		btTransform A, B;
		A.setIdentity();
		B.setOrigin(btVector3(0,0,2));

		btFixedConstraint* fixedConst =  new btFixedConstraint(*movingBoxRigidBody, *movingBoxRigidBody2, B, A);
		dynamicsWorld->addConstraint(fixedConst, true);

	//Spring definition
		btTransform frameInA, frameInB;
		frameInA = btTransform::getIdentity();	//position of center of spring with respect to the base box
		frameInA.setOrigin(btVector3(5,0,0));
		frameInB = btTransform::getIdentity();	//position of center of spring with respect to the moving box
		frameInB.setOrigin(btVector3(0,0,0));
		//Spring definition
		//btGeneric6DofSpringConstraint* spring = new btGeneric6DofSpringConstraint(*baseBoxRigidBody, *movingBoxRigidBody, frameInA, frameInB, true);
		//For using the "btGeneric6DofSpring2Constraint" (the newer but maybe buggy) spring implementation, you can comment the above line and uncomment the below line.
		btGeneric6DofSpring2Constraint* spring = new btGeneric6DofSpring2Constraint(*baseBoxRigidBody, *movingBoxRigidBody, frameInA, frameInB);
		//setting the limits of the spring
		spring->setLinearUpperLimit(btVector3(5., 0., 0.));
		spring->setLinearLowerLimit(btVector3(-5., 0., 0.));
		spring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
		spring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));

		dynamicsWorld->addConstraint(spring, true);	// 1st parameter: the name of spring; 2nd parameter: if collision between two bodies to be neglected or not=> true means collision is disabled.

		btScalar 	springStiffness = 50 ; 	// units in N/m
		btScalar 	springDamping = 3;		// when "btGeneric6DofSpring2Constraint" is used, the unit is [N.s/m]

		//now the spring should be enabled in the desired direction:
		spring->enableSpring(0, true);		// 0 means in x, 1 means in y, 2: z, 3: rotation about x and so on..
		spring->setStiffness(0, springStiffness);
		spring->setDamping(0, springDamping);
		spring->setEquilibriumPoint(); 		//this function makes the spring at its equilibrium state at the current position. In other words, the current length of the spring will be considered as the natural length.

		//now lets move the box to some extent
		btScalar amplitude = 3.f;
		movingBoxRigidBody->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(amplitude,0,0)));
		movingBoxRigidBody2->setWorldTransform(btTransform(btQuaternion(0,0,0,1),btVector3(amplitude,0,0)));


	//Simulation parameters
		btScalar 	simTime;
		btScalar 	deltaTime = 	0.001; 	// Normally 0.01 is enough, the safe range is between 1/60 and smaller, depending on the case.
		btScalar 	endTime = 		20; 	// Units in seconds.
		btScalar 	numberOfSteps =	endTime/deltaTime;

	/*
		IMPORTANT NOTICE: by default the bullet sequentialImpulseSolver (LCP) does the simulation
		with the projected Gauss Seidel constraint solver with 20 as the maximum number of 
		iterations. You can always change it by:
			btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
			info.m_numIterations = 20;
		Please keep it in mind.
	*/

	//output header:
		std::cout << "  amplitude(m)= " << amplitude << std::endl;
		std::cout << "  boxMass(kg)= " << boxMass << std::endl;
		std::cout << "  springStiffness(N/m)= " << springStiffness << std::endl;
		std::cout << "  springFrequency= " << sqrt(springStiffness/boxMass) << std::endl;
		std::cout << "  springDamping= " << springDamping << std::endl;
		std::cout << "time(s)" << " " << "boxMovementX(m)" << " " << "secondBoxMovement(m)" << std::endl;

	//Run
		for (int i=0; i<numberOfSteps ; i++){
			dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
			
			simTime = deltaTime*i;

			//Buffers for positions and rotations defined here:
			btTransform 	boxBuffer 	=	movingBoxRigidBody->getWorldTransform();
			btTransform 	box2Buffer 	=	movingBoxRigidBody2->getWorldTransform();

			std::cout << simTime << " " << boxBuffer.getOrigin().getX() << " " << box2Buffer.getOrigin().getX() << std::endl;
		}
	
	
	//cleaning up allocated memories

		delete 		spring;
		delete 		movingBoxRigidBody;
		delete 		movingBoxState;
		delete 		movingBoxShape;
		delete 		baseBoxRigidBody;
		delete 		baseBoxState;
		delete 		baseBoxShape;
		delete 		dynamicsWorld;
		delete 		solver;
		delete 		dispatcher;
		delete 		collisionConfiguration;
		delete 		broadphase;
		
	return 0;
}