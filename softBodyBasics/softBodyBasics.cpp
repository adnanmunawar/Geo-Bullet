#include <iostream>
#include <btBulletDynamicsCommon.h>
#include "softBodyWriter.h"			//  contains the writers functions for visualisations in Paraview
#include "../.internalLibs/STLSoftImporter.h"	// contains the functions needed to import STLmesh as a soft body


/*###### IMPORTANT POINT ######
	The linear stiffness of the softbody depends both on mass of the particles (more
	mass, more stiffness) and the amount of linear stiffness. Note that the springs are
	connected to each other in series.
#############################*/



//Data for Trimesh definition
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

int main() {
	

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
	dynamicsWorld->setGravity(btVector3(0,0,0));


	//adding soft body
	btSoftBody* softBody1	=	btSoftBodyHelpers::CreatePatch(
																dynamicsWorld->getWorldInfo(),
																btVector3(-5,2,-2),
																btVector3(5,2,-2),
																btVector3(-5,2,7),
																btVector3(5,2,7),
																20,
																20,
																1+2,
																true);

	softBody1->m_cfg.collisions		|=	btSoftBody::fCollision::VF_SS; 	// NEEDED FOR SOFT_SOFT COLLISIONS

	dynamicsWorld->addSoftBody(softBody1);

	std::cout << "the default Velocity iterations for soft body is: " << softBody1->m_cfg.viterations << std::endl;
	softBody1->m_cfg.viterations = 20;
	std::cout << "the new Velocity iterations for soft body is: " << softBody1->m_cfg.viterations << std::endl;
	std::cout << "the default Position iterations for soft body is: " << softBody1->m_cfg.piterations << std::endl;
	softBody1->m_cfg.piterations = 20;
	softBody1->getWorldInfo()->m_gravity = btVector3(0,0,0);

	std::cout << "the total mass of the soft body: " << softBody1->getTotalMass() << std::endl;
	softBody1->setTotalMass(5.0);
	std::cout << "the new total mass of the soft body: " << softBody1->getTotalMass() << std::endl;

	//To restrain a specifi node of the softbody you can adjust a partial mass of zero to that, that node will behave as static node
	//softBody1->setMass(100,0); //the node number 100 is set to be static


	btSoftBody* softBody2	=	btSoftBodyHelpers::CreateFromTriMesh(dynamicsWorld->getWorldInfo(),
																	gVertices,
																	&gIndices[0][0],
																	NUM_TRIANGLES);

		
	softBody2->m_cfg.collisions		|=	btSoftBody::fCollision::VF_SS;	// NEEDED FOR SOFT_SOFT COLLISIONS

	dynamicsWorld->addSoftBody(softBody2);

	softBody2->setTotalMass(10);
	softBody2->setMass(0,0);
	softBody2->setMass(1,0);

	softBody2->m_cfg.viterations = 20;
	softBody2->m_cfg.piterations = 20;
	softBody2->getWorldInfo()->m_gravity = btVector3(0,0,0);


	std::cout << "###########################" << std::endl;


	//adding a static object to interact with softbodies
		btCollisionShape* groundShape	= new btBoxShape(btVector3(5,5,5));
		btDefaultMotionState* groundState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-10,0)));
		btRigidBody* groundRB = new btRigidBody(0, groundState, groundShape, btVector3(0,0,0));
		dynamicsWorld->addRigidBody(groundRB);

	//adding rope
	btSoftBody* 	rope	=	btSoftBodyHelpers::CreateRope(	dynamicsWorld->getWorldInfo(),
																btVector3(20,20,20),
																btVector3(20,10,20),
																10,
																1);

	rope->m_cfg.viterations = 40;
	rope->m_cfg.piterations = 40;
	rope->m_materials[0]->m_kLST = 1.5;
	rope->getWorldInfo()->m_gravity = btVector3(0,0,0);

	
	std::cout << "The total mass is: " << rope->getTotalMass() << std::endl;

	rope->setTotalMass(1000);
	//rope->setMass(rope->m_nodes.size()-1,1);

	std::cout << "The new total mass is: " << rope->getTotalMass() << std::endl;
	
	std::cout << "The number of links is: " << rope->m_links.size() << std::endl;
	std::cout << "The initial length is: " << rope->m_links[0].m_rl << std::endl;

		// this block proves that the resting length is the length of the links at the initial condition, and it is static.
		// btScalar sumLength=0;
		// for(int i=0 ; i<rope->m_links.size(); i++){
		// 	sumLength	+=	rope->m_links[i].m_rl;
		// }
		// std::cout << "TOTAL INITIAL LENGTH IS:  " << sumLength << std::endl;

	dynamicsWorld->addSoftBody(rope);

	btSoftBody* STLsoftBody;

	//setting the necessary variables
	STLImporterToSoftBody(dynamicsWorld, STLsoftBody, "cylindricalShape.stl");
	dynamicsWorld->addSoftBody(STLsoftBody);
	STLsoftBody->getWorldInfo()->m_gravity = btVector3(0,0,-9.81);
	STLsoftBody->setMass(0,0);


	btScalar deltaTime = 0.02;

	btScalar endTime = 20;


	//Run
	for (int i=0; i<endTime/deltaTime ; i++){
		dynamicsWorld->stepSimulation(deltaTime,1,deltaTime);
		// softBody1->addForce(btVector3(0,-10,0));

		// std::cout << "the soft body m_faces is: " << softBody1->m_faces.size() << std::endl;	// number of faces
		// std::cout << "the soft body m_nodes is: " << softBody1->m_nodes.size() << std::endl;	// number of nodes
		// std::cout << "the soft body m_nodes is: " << softBody1->m_nodes[1].m_x.getX() << std::endl;

		// for (int i=0; i<softBody1->m_nodes.size() ; i++){

		// 	std::cout << i*deltaTime << " " << softBody1->m_nodes[i].m_x.getX() << " " << softBody1->m_nodes[i].m_x.getY() << " " << softBody1->m_nodes[i].m_x.getZ() << std::endl;
		// }

		// for (int i=0; i<softBody1->m_nodes.size() ; i++){

		// 	std::cout << i*deltaTime << " " << softBody1->m_nodes[i].m_f.getX() << " " << softBody1->m_nodes[i].m_f.getY() << " " << softBody1->m_nodes[i].m_f.getZ() << std::endl;
		// }

		//Adding force to a specific point of the softbody
		rope->addForce(btVector3(0,-100,0),rope->m_nodes.size()-1);

		softBodyPositionWriter(softBody1, softBody1->m_nodes.size(), i*deltaTime, 1);
		softBodyPositionWriter(softBody2, softBody2->m_nodes.size(), i*deltaTime, 2);
		softBodyPositionWriter(rope, rope->m_nodes.size(), i*deltaTime, 3);
		softBodyPositionWriter(STLsoftBody, STLsoftBody->m_nodes.size(), i*deltaTime, 4);


		softBodyDeformationWriter(softBody1, softBody1->m_links.size(), i*deltaTime, 1);

		//std::cout << "DirectChangeLength " << btDistance(rope->m_nodes[rope->m_nodes.size()-1].m_x, rope->m_nodes[0].m_x) - sumLength << std::endl;
		//std::cout << "LERP " << lerp(rope->m_nodes[rope->m_nodes.size()-1].m_x, rope->m_nodes[0].m_x, 0.5).getY() << std::endl;


	}
	
	delete 	STLsoftBody;
	delete 	rope;
	delete 	groundRB;
	delete 	groundState;
	delete 	groundShape;
	delete 	softBody2;
	delete 	softBody1;
	delete 	dynamicsWorld;
	delete 	softBodySolver;
	delete 	solver;
	delete 	dispatcher;
	delete 	collisionConfiguration;
	delete 	broadphase;


	return 0;
}