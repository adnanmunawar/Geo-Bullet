#include <iostream>
#include <btBulletDynamicsCommon.h>

// 	Trimesh shape libs
#include "../../../Bullet/bullet3/src/BulletCollision/Gimpact/btGImpactShape.h"
#include "../../../Bullet/bullet3/src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "../../../Bullet/bullet3/src/BulletCollision/Gimpact/btCompoundFromGimpact.h"


//Data for btConvexHullShape
	#define convexVertexCount 8
	static btScalar convexVertex[] = {
	-1.0f, -1.0f, -0.5f,
	-1.0f, -1.0f, 0.5f,
	1.0f, -1.0f, -0.5f,
	1.0f, -1.0f, 0.5f,
	1.0f, 1.0f, -0.5f,
	1.0f, 1.0f, 0.5f,
	-1.0f, 1.0f, -0.5f,
	-1.0f, 1.0f, 0.5f
	};

//Data for Trimesh definition: look through MovingConcaveDemo file for implementation.
	#define REAL btScalar
	const int NUM_TRIANGLES =12;
	const int NUM_VERTICES = 8;
	const int NUM_INDICES  = NUM_TRIANGLES * 3;


	REAL gVertices[NUM_VERTICES * 3] = {
		REAL(-1.0), REAL(-1.0), REAL(-0.5), // #0
		REAL(1.0), REAL(-1.0), REAL(-0.5),	// #1
		REAL(1.0), REAL(-1.0), REAL(0.5),	// #2
		REAL(-1.0), REAL(-1.0), REAL(0.5),	
		REAL(-1.0), REAL(1.0), REAL(-0.5),
		REAL(1.0), REAL(1.0), REAL(-0.5),
		REAL(1.0), REAL(1.0), REAL(0.5),
		REAL(-1.0), REAL(1.0), REAL(0.5)	// #7
	};

	int gIndices[NUM_TRIANGLES][3] = {
		{0,1,2},
		{2,0,3},
		{1,5,6},
		{6,2,1},
		{5,4,7},
		{5,7,6},
		{4,0,3},
		{4,3,7},
		{0,1,4},
		{1,5,4},
		{3,2,6},
		{3,6,7}
	};

//Data for tilting plane based on trimesh shape (the purpose is to check the accuracy of the trimesh-sphere collision when friction is calculated. The results will help to choose best collision pairs for simulation of direct shear test. For example using trimesh for direct shear box or btBoxShape?)
	const int NUM_TRIANGLES_PLANE =2;
	const int NUM_VERTICES_PLANE = 4;
	const int NUM_INDICES_PLANE  = NUM_TRIANGLES_PLANE * 3;

	REAL gVertices_plane[NUM_VERTICES_PLANE * 3] = {
		REAL(50.0), REAL(50.0), REAL(0.0), 		// #0
		REAL(50.0), REAL(-50.0), REAL(0.0),		// #1
		REAL(-50.0), REAL(-50.0), REAL(0.0),	// #2
		REAL(-50.0), REAL(50.0), REAL(0.0)		// #3
	};

	int gIndices_plane[NUM_TRIANGLES_PLANE][3] = {
		{0,1,2},
		{0,2,3}
	};

int main() {
	//Broadphase
	btBroadphaseInterface* broadphase = new btDbvtBroadphase(); 	

	//Narrowphase (dispatcher)
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	//Solver
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	//Dynamics world
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
	dynamicsWorld -> setGravity(btVector3(0,0,-9.81));

	//########## OBJECTS ##########
	//Tilting plane
		//btCollisionShape* tiltingPlaneShape = new btBoxShape(btVector3(50,50,1));
		btTriangleIndexVertexArray* trimeshVertexArrays_plane = new btTriangleIndexVertexArray(NUM_TRIANGLES_PLANE, &gIndices_plane[0][0], 3*sizeof(int),NUM_VERTICES_PLANE,(REAL*) &gVertices_plane[0], sizeof(REAL)*3);

		btGImpactMeshShape* trimeshPlaneShape = new btGImpactMeshShape(trimeshVertexArrays_plane);
		trimeshPlaneShape->setLocalScaling(btVector3(1.f,1.f,1.f));
		trimeshPlaneShape->setMargin(0.0f);
		trimeshPlaneShape->updateBound();
		btCollisionShape* compoundTrimeshPlaneShape;
		compoundTrimeshPlaneShape = btCreateCompoundFromGimpactShape(trimeshPlaneShape, 0.f);


		btDefaultMotionState* tiltingPlaneState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0))); //the surface of the tilting plane has zero altitude
		btRigidBody::btRigidBodyConstructionInfo tiltingPlaneRigidBodyCI(0,tiltingPlaneState,compoundTrimeshPlaneShape,btVector3(0,0,0));
		btRigidBody* tiltingPlaneRigidBody = new btRigidBody(tiltingPlaneRigidBodyCI);
		dynamicsWorld->addRigidBody(tiltingPlaneRigidBody);
		tiltingPlaneRigidBody->setFriction(1.0);
		tiltingPlaneRigidBody->setRestitution(0.0);

	//[1] box
		// Note: for generation of Box shape, btBoxShape function takes half extends of the box,
		btCollisionShape* boxShape = new btBoxShape(btVector3(1.0, 1.0, 0.25));
		btDefaultMotionState* boxState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0.25)));
		
		btScalar 	boxMass=2.0;
		btVector3 	boxInertia(0, 0, 0);
		boxShape->calculateLocalInertia(boxMass, boxInertia);
		
		btRigidBody::btRigidBodyConstructionInfo boxRigidBodyCI(boxMass, boxState, boxShape, boxInertia);
		btRigidBody* boxRigidBody = new btRigidBody(boxRigidBodyCI);
		dynamicsWorld->addRigidBody(boxRigidBody);

		boxRigidBody->setFriction(0.7);
		boxRigidBody->setRestitution(0.0);
		boxRigidBody->setActivationState(4);					//make the object always alive, so it will not sleep.

	//[2] sphere [rotation-locked]
		btCollisionShape* sphereShape = new btSphereShape(0.5);
		btDefaultMotionState* sphereState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,5,0.5)));

		btScalar 	sphereMass = 2.0;
		btVector3 	sphereInertia(0,0,0);
		sphereShape->calculateLocalInertia(sphereMass, sphereInertia);
		
		btRigidBody::btRigidBodyConstructionInfo sphereRigidBodyCI(sphereMass, sphereState, sphereShape, sphereInertia);
		btRigidBody* sphereRigidBody = new btRigidBody(sphereRigidBodyCI);
		dynamicsWorld->addRigidBody(sphereRigidBody);

		sphereRigidBody->setFriction(0.7);
		sphereRigidBody->setRestitution(0.0);
		sphereRigidBody->setActivationState(4);					//make the object always alive, so it will not sleep.
		sphereRigidBody->setAngularFactor(btVector3(0,0,0));	//locking the rotation DOFs.


	//[3] convex (in form of box)
		//convexHullShape should be defined on memory stack. So, don't use a pointer for its definition.
		btConvexHullShape convexShape(convexVertex,convexVertexCount,3*sizeof(btScalar));
		convexShape.setMargin(0.0f);	//the collision margin for convex objects is set to 0.04 by default. we set it to zero.
		btDefaultMotionState convexState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-5,0.25)));

		btScalar 	convexMass = 2.0f;
		btVector3 	convexInertia(0,0,0);
		convexShape.calculateLocalInertia(convexMass, convexInertia);

		btRigidBody::btRigidBodyConstructionInfo convexRigidBodyCI(convexMass, &convexState, &convexShape, convexInertia);
		btRigidBody* convexRigidBody = new btRigidBody(convexRigidBodyCI);
		dynamicsWorld->addRigidBody(convexRigidBody);

		convexRigidBody->setFriction(0.7);
		convexRigidBody->setRestitution(0.0);
		convexRigidBody->setActivationState(4);


	//[4] trimesh (in form of box)
		//make the array of Trimesh first:
		btTriangleIndexVertexArray* trimeshVertexArrays = new btTriangleIndexVertexArray(NUM_TRIANGLES, &gIndices[0][0], 3*sizeof(int),NUM_VERTICES,(REAL*) &gVertices[0], sizeof(REAL)*3);

		btGImpactMeshShape* trimeshShape = new btGImpactMeshShape(trimeshVertexArrays);
		trimeshShape->setLocalScaling(btVector3(1.f,1.f,1.f));
		trimeshShape->setMargin(0.0f);
		trimeshShape->updateBound();

		btCollisionShape* compoundTrimeshShape;
		compoundTrimeshShape = btCreateCompoundFromGimpactShape(trimeshShape, 0.0);

		btDefaultMotionState* trimeshState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,10,0.5)));

		btScalar 	trimeshMass = 2.0;
		btVector3 	trimeshInertia(0,0,0);
		trimeshShape->calculateLocalInertia(trimeshMass, trimeshInertia);

		btRigidBody::btRigidBodyConstructionInfo trimeshRigidBodyCI(trimeshMass, trimeshState, compoundTrimeshShape, trimeshInertia);
		btRigidBody* trimeshRigidBody = new btRigidBody(trimeshRigidBodyCI);
		dynamicsWorld->addRigidBody(trimeshRigidBody);

		trimeshRigidBody->setFriction(0.7);
		trimeshRigidBody->setRestitution(0.0);
		trimeshRigidBody->setActivationState(4);



	//Simulation parameters
	btScalar rotAmount = 0; // Radians
	btScalar simTime;
	btScalar deltaTime = 0.01; // Normally this is enough, the safe range is between 1/60 and smaller, depending on the case.

	//output header:
	std::cout << "time(s)" << " " << "planeRot(deg)" << " " << "boxPositionX" << " " << "boxPositionY" << " " << "boxPositionZ" << " " << "spherePositionX" << " " << "spherePositionY" << " " << "spherePositionZ" << " " << "convexPositionX" << " " << "convexPositionY" << " " << "convexPositionZ" << " " << "trimeshPositionX" << " " << "trimeshPositionY" << " " << "trimeshPositionZ" << std::endl;

	//Run
	for (int i=0; i<18000 ; i++){
		dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
		
		simTime = deltaTime*i;
		rotAmount = 0.000024*simTime*simTime;

		//tiltingPlaneRigidBody->setWorldTransform(btTransform(btQuaternion(btVector3(0,1,0),btScalar(rotAmount)),btVector3(0,0,0)));

		//Buffers for positions and rotations defined here:
		btTransform 	planeBuffer = 	tiltingPlaneRigidBody->getWorldTransform();
		btTransform 	boxBuffer 	=	boxRigidBody->getWorldTransform();
		btTransform 	sphereBuffer=	sphereRigidBody->getWorldTransform();
		btTransform 	convexBuffer=	convexRigidBody->getWorldTransform();
		btTransform 	trimeshBuffer=	trimeshRigidBody->getWorldTransform();


		std::cout << simTime << " " << 57.2957795*(planeBuffer.getRotation().getAngle()) << " ";
		std::cout << boxBuffer.getOrigin().getX() << " " << boxBuffer.getOrigin().getY() << " " << boxBuffer.getOrigin().getZ() << " ";
		std::cout << sphereBuffer.getOrigin().getX() << " " << sphereBuffer.getOrigin().getY() << " " << sphereBuffer.getOrigin().getZ() << " ";
		std::cout << convexBuffer.getOrigin().getX() << " " << convexBuffer.getOrigin().getY() << " " << convexBuffer.getOrigin().getZ() << " ";
		std::cout << trimeshBuffer.getOrigin().getX() << " " << trimeshBuffer.getOrigin().getY() << " " << trimeshBuffer.getOrigin().getZ() << std::endl;
	}
	
	
	//cleaning up allocated memories

	delete 		trimeshRigidBody;
	delete 		trimeshState;
	delete 		compoundTrimeshShape;
	delete 		trimeshShape;
	delete 		trimeshVertexArrays;
	delete 		convexRigidBody;
	delete 		sphereRigidBody;
	delete 		sphereState;
	delete 		sphereShape;
	delete 		boxRigidBody;
	delete 		boxState;
	delete 		boxShape;
	delete 		tiltingPlaneRigidBody;
	delete 		tiltingPlaneState;
	delete 		trimeshPlaneShape;
	delete 		trimeshVertexArrays_plane;
	delete 		dynamicsWorld;
	delete 		solver;
	delete 		dispatcher;
	delete 		collisionConfiguration;
	delete 		broadphase;
	
	return 0;
}