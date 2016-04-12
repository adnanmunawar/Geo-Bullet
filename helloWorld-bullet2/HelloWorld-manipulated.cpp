#include <iostream>
#include <btBulletDynamicsCommon.h>


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
	dynamicsWorld -> setGravity(btVector3(0,0,-10));

	//Collision shapes
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,50,1));
	btCollisionShape* barrier = new btBoxShape(btVector3(2,2,2));
	btCollisionShape* sphereShape = new btSphereShape(1);

	// in order to use a btCompoundShape made out  of 2 box shapes which make a barrier form, you can uncomment the following
	// btCompoundShape* whole = new btCompoundShape();

	// btTransform offset;
	// offset.setIdentity();
	// whole->addChildShape(offset, groundShape);
	// offset.setOrigin(btVector3(5,0,2));
	// whole->addChildShape(offset, barrier);

	//Adding ground	
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(btVector3(0,0,1),btScalar(0)),btVector3(0,0,-1)));
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
	//btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,whole,btVector3(0,0,0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);
	groundRigidBody->setActivationState(4);
	groundRigidBody->setFriction(1);
	groundRigidBody->setRestitution(1);

	//Adding sphere
	btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(btVector3(0,1,0),btScalar(0)),btVector3(0,0,5)));
	btScalar mass=1;
	btVector3 fallInertia(0,0,0);
	sphereShape->calculateLocalInertia(mass,fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, sphereShape,fallInertia);
	btRigidBody* fallRigidBody = new btRigidBody (fallRigidBodyCI);
	dynamicsWorld-> addRigidBody(fallRigidBody);
	fallRigidBody->setActivationState(4);
	fallRigidBody->setFriction(0.7);
	fallRigidBody->setRestitution(1);


	//How to set the restitution -> "Rigid body name->setRestitution(x)"
	//fallRigidBody->setRestitution(0.8f);
	//groundRigidBody->setRestitution(1.0f);

	
	/***************************************btRigidBody functions:
	Its helpful to look at the btRigidBody functions in the source code.
	For example the following command "releases" the movement of the rigid body only in y direction. "0" means locked, and "1" means released.
	fallRigidBody->setLinearFactor(btVector3(0,1,0));
	And this only releases the rotation of the rigid body around y and z directions.
	fallRigidBody->setAngularFactor(btVector3(0,1,1));
	and the following command, makes the environment viscous. It acts like "air drag" rather than damping! Its a misnomer.
	fallRigidBody->setDamping(btScalar(0.8f),btScalar(0.8f));
	and the two following commands are used to set linear and angular velocities, respectively:
	fallRigidBody->setLinearVelocity(btVector3(0,0,4));
	fallRigidBody->setAngularVelocity(btVector3(0,0,0));
	------------------------------------------------
	You can get the force applied from external sources to an object using:
	fallRigidBody->getTotalForce();
	You can also apply forces using the command below (first parameter is force magnitude and the second is position, so it consider the rotation effect if there are some offset between the center mass and the line of action of the force):
	fallRigidBoyd->applyForce(btVector3(fx,fy,fz),btVector3(x,y,z));
	[or]
	fallRigidBody->applyCentarlForce(btVector3(fx,fy,fz));
	!!!NOTE!!!
	When you apply forces they will vanish after the first "stepSimulation()", so, you need to keep them in the stepSimulation loop. There must be the same case for apply impulses or even velocities or anything else that user sets.
	------------------------------------------------
	 */
	

	/***************************************btCollisionShape functions:
	Its helpful to look at the btCollisionShape functions (setters and getters) in the source code
	For example the following command sets the coefficient of friction:
	fallRigidBody->setFriction(btScalar(0.8));
	Even you can specify anisotropic friction.
	And by the following command, you can set restitution:
	fallRigidBody->setRestitution(0.8f);
	 */

std::cout << "time(s)" << " " << "sphereHeight(m)" << " " << "impulse" << " " << "Force(N)" << std::endl;

	//btScalar ehsan=0;

	btScalar deltaTime = 0.02;

	btScalar endTime = 5;

	//Run
	for (int i=0; i<endTime/deltaTime ; i++){
		dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
		
		

		//btTransform trans; //defining new btTransform typedef to work with
		//fallRigidBody->getMotionState()->getWorldTransform(trans);
		
		//***** HOW TO ROTATE/TRANSFORM A RIGIDBODY (DYNAMIC OR STATIC)
			// btTransform ntrans = groundRigidBody->getWorldTransform(); //Another btTransform is defined as buffer
		
			// ehsan += 0.002f;
		
			// ntrans = btTransform(btQuaternion(btVector3(0,1,0),btScalar(ehsan)));
			// groundRigidBody->setWorldTransform(ntrans);
			//std::cout<<"Rotation: " << groundRigidBody->getWorldTransform().getRotation().getAngle() <<std::endl;

		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();	//gets number of "possible contacts" derived from broadphase query
		btScalar totalImpact = 0.f;
		if (numManifolds>0){
			btPersistentManifold* manifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(numManifolds-1);

			int p = manifold->getNumContacts();		//gets number of "definite contacts" derived from narrow phase (dispatcher) query
			
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				// std::cout << " m_appliedImpulse: " << manifold->getContactPoint(p).m_appliedImpulse << std::endl;
				// std::cout << " m_appliedImpulseLateral1: " << manifold->getContactPoint(p).m_appliedImpulseLateral1 << std::endl;
				// std::cout << " m_appliedImpulseLateral2: " << manifold->getContactPoint(p).m_appliedImpulseLateral2 << std::endl;
				// std::cout << " m_localPointA:  " << manifold->getContactPoint(p).m_localPointA.getX() << " / " << manifold->getContactPoint(p).m_localPointA.getY() << " / " << manifold->getContactPoint(p).m_localPointA.getZ() << std::endl;
				// std::cout << " m_localPointB:  " << manifold->getContactPoint(p).m_localPointB.getX() << " / " << manifold->getContactPoint(p).m_localPointB.getY() << " / " << manifold->getContactPoint(p).m_localPointB.getZ() << std::endl;
				// std::cout << " m_positionWorldOnB:  " << manifold->getContactPoint(p).m_positionWorldOnB.getX() << " / " << manifold->getContactPoint(p).m_positionWorldOnB.getY() << " / " << manifold->getContactPoint(p).m_positionWorldOnB.getZ() << std::endl;
				// std::cout << " m_positionWorldOnA:  " << manifold->getContactPoint(p).m_positionWorldOnA.getX() << " / " << manifold->getContactPoint(p).m_positionWorldOnA.getY() << " / " << manifold->getContactPoint(p).m_positionWorldOnA.getZ() << std::endl;
				// std::cout << " m_normalWorldOnB:  " << manifold->getContactPoint(p).m_normalWorldOnB.getX() << " / " << manifold->getContactPoint(p).m_normalWorldOnB.getY() << " / " << manifold->getContactPoint(p).m_normalWorldOnB.getZ() << std::endl;
				// std::cout << " m_distance1:  " << manifold->getContactPoint(p).m_distance1 << std::endl;
				// std::cout << " m_combinedFriction:  " << manifold->getContactPoint(p).m_combinedFriction << std::endl;
				// std::cout << " m_combinedRollingFriction:  " << manifold->getContactPoint(p).m_combinedRollingFriction << std::endl;
				// std::cout << "######################################" << std::endl;
				totalImpact=manifold->getContactPoint(p).m_appliedImpulse;




			}

			
		}
		std::cout << i*deltaTime << " " << fallRigidBody->getWorldTransform().getOrigin().getZ() << " " << totalImpact << " " << totalImpact/deltaTime << std::endl;


		//std::cout << "sphere height: " << trans.getOrigin().getX() << std::endl;
		//std::cout << "Fz= " << fallRigidBody->getTotalForce().getZ() << std::endl;
		//std::cout << "Impulse denominator: " << fallRigidBody->computeImpulseDenominator(btVector3(0,0,0),btVector3(1,0,0)) << std::endl;
		//std::cout << "Velocity: " << fallRigidBody->getLinearVelocity().getZ() << std::endl;
		//std::cout << "center of mass postion: " << fallRigidBody->getCenterOfMassPosition().getZ() << std::endl;
	}
	
	
	
	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
	delete groundShape;
	delete sphereShape;

	return 0;
}