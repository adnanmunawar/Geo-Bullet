//Original HelloWorld example of bullet 2
#include <iostream>

#include <btBulletDynamicsCommon.h>

int main (void) {

	//Broadphase definition: the assigned name is "broadphase". "btDbvtBroadphase" is so efficient and fast. Try to keep using it. ALthough there is another broadphase algorithm entitled "btAxisSweep3" (maybe still under development) but sometimes it works better than btDbvtBroadphase. A benchmark test is recommended when a large number of objects exist in the scene.
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();

	//Narrowphase (also called dispatcher) is assigned with default configurations. "collisionConfiguration" is a set of parameters and rules for collision resolution and collision detection. Here, the default sets are accepted for simplicity. For the collision matrix refer to the user's manual. There you will find a table stating that on which circumstances, what collision algorithm is in use by default. Only if you wanetd to change this default algorithm, pass something other than btDefaultCollisionConfiguration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	//Default collisionConfiguration is passed to the new dispatcher entitled "dispatcher"
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	//Solver definition with the name "solver". Normally, "btSequentialImpulseConstraintSolver" does a good job (i.e. realtime response) as long as you don't push it to the extremes. It is the default solver of Bullet, however, for parallel processing you can use a different solver (btParallelConstraintSolver).
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	//Now the components of the world is set (broadphase, dispatcher, collision configuration and solver), we can instantiate the dynamics world. Notice to the parameters that dynamicsWorld needs and their sequence.
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	//Setting gravity. The function "setGravity" can be called using "->" syntax, since the class was defined on the heap memory.
	dynamicsWorld->setGravity(btVector3(0, -10, 0));
	//---------------------------------------------------------------------------------------------------------

	//Creating collision shapes for rigid bodies. Collision shapes are just physical shape of the object, and it does not contain any concept of mass, inertia, friction and restitution.
	//Attention! If you have many rigid bodies with an identical physical shape, the best is to pass a same collision shape to all of them, and not to define a lot of collision shapes.
	
	//The static plane shape is defined with the coordinates below, the last parameter is "collision margin". And the first parameter is the normal vector of the plane. This introduces an infinite plane collision shape.
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
	//The falling shape that falls is a sphere with the radius of 1. So, here the only parameter is "radius".
	btCollisionShape* fallShape = new btSphereShape(1);
	//An alternative can be a definition of a box shape as following: (Note that the three parameters passed are the dimensions in x, y and z) 
	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)))

	//-------------------------------------------
	//Characterizing the ground rigid body:
	//"MotionStates" are just for the dynamic objects which are subjected to movements, They record the movements. You can use it for getting positions and velocities of objects. First parameter is the orientation and the second is the position. btQuaternion is used for setting the alignment of the object, the whole structure of the btTransform is like below:
	//btTransform(  btQuaternion(btVector3(axis,axis,axis),btScalar(angle)),   btVector(position))
	btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(btVector3(0,1,0),btScalar(0)), btVector3(0, -1, 0)));
	// btRigidBodyConstructionInfo is a the set of characteristics of the rigid body. It simplifies passing parameters to a rigid body.
	//Example: btRigidBodyConstructionInfo(mass, motionState, collisionShape, inertia). Static shapes has no mass and no inertia.
	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	//Now rigid body is defined with the relevant parameters:
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
	//Adding rigid body to the world:
	dynamicsWorld->addRigidBody(groundRigidBody);


	//Characterizing the falling rigid body:
	btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
	//btScalar is a typedef
	btScalar mass = 1;
	//Since we don't know the inertia (or too lazy to calculate), first we define an empty btVector3 (buffer) and...
	btVector3 fallInertia(0, 0, 0);
	//... then call "calculateLocalInertia" function (from collisionShape) to calculate its inertia. the second parameter will be filled with the calculated inertia.
	fallShape->calculateLocalInertia(mass, fallInertia);
	// Now the inertia is calculated and the parameters "fallInertia" is filled with correct magnitudes.
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
	dynamicsWorld->addRigidBody(fallRigidBody);
	//-------------------------------------------


	//Stepping the simulation:
	for (int i = 0; i < 300; i++) {
		//The original function is stepSimulation(A,B,C). A is the rough time increment; B is maxSubSteps and C is the fixed time increment.
		//For engineering purposes just use "stepSimulation(timeIncrement,1,timeIncrement)"
		//When you pass Bullet B > 1 (maxSubSteps more than 1), it will interpolate movement for you
    	dynamicsWorld->stepSimulation(1/60.f,1,1/60.f);

    	//btTransform is a typedef and we just have defined an empty container to fill our data in that.
    	//In principle "btTransform" is a container which is capable of holding orientation and position values, respectively, and all the transformation and rotations of dynamic/static/kinematic objects can be carried out by manipulating this buffer. For all the objects, there is a function called "setIdentity()" which sets all the position and orientation values to zero (at origin). Also you can use it to empty the buffer. Here we name the container (buffer) as "trans".
    	btTransform trans;
    	//This is the way you can get the positions of the rigid bodies. "getWorldTransform" will be written to "trans" variable.
    	fallRigidBody->getMotionState()->getWorldTransform(trans);

    	std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
	}

	//----------------------------------------------
	//All the things defined on the heap memory should be cleaned in "reverse" order of definition.
	dynamicsWorld->removeRigidBody(fallRigidBody);
	delete fallRigidBody->getMotionState();
	delete fallRigidBody;

	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;


	delete fallShape;

	delete groundShape;


	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;

	return 0;
}