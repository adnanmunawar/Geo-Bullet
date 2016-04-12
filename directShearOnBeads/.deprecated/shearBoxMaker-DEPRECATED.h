#ifndef SHEARBOXMAKER_H
#define SHEARBOXMAKER_H
#include <btBulletDynamicsCommon.h>
#include "gimpactShearBox.h"

void compoundShearBoxMaker(btDiscreteDynamicsWorld* dynamicsWorld) {
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

	btAlignedObjectArray<btTransform> shearBoxOffset;
	shearBoxOffset.resize(10);
	// fill all the arrays with zero magnitudes:
	for (int i=0; i<9; i++){
		shearBoxOffset[i].setIdentity();
	}
	//btTransform* 		shearBoxOffset[9];		// and offset array for setting the location of boxes with respect to the origin of lowerbox and upperbox.
	//lowerBox offsets
	shearBoxOffset[0].setOrigin(btVector3(0, 0, -((boxThickness/2.f) + (boxHeight/4.f)) ));
	shearBoxOffset[1].setOrigin(btVector3(boxSize/2.f + boxThickness/2.f, 0 , 0));
	shearBoxOffset[2].setOrigin(btVector3(-(boxSize/2.f + boxThickness/2.f), 0, 0));
	shearBoxOffset[3].setOrigin(btVector3(0, boxSize/2.f + boxThickness/2.f, 0));
	shearBoxOffset[4].setOrigin(btVector3(0, -(boxSize/2.f + boxThickness/2.f), 0));
	//upperBox offsets
	shearBoxOffset[5].setOrigin(btVector3(boxSize/2.f + boxThickness/2.f, 0, 0));
	shearBoxOffset[6].setOrigin(btVector3(-(boxSize/2.f + boxThickness/2.f), 0, 0));
	shearBoxOffset[7].setOrigin(btVector3(0, boxSize/2.f + boxThickness/2.f, 0));
	shearBoxOffset[8].setOrigin(btVector3(0, -(boxSize/2.f + boxThickness/2.f), 0));

	// assigning the child shapes of lowerBox and upperBox
	lowerBoxShape->addChildShape(shearBoxOffset[0], shearBoxShape[0]);
	lowerBoxShape->addChildShape(shearBoxOffset[1], shearBoxShape[1]);
	lowerBoxShape->addChildShape(shearBoxOffset[2], shearBoxShape[2]);
	lowerBoxShape->addChildShape(shearBoxOffset[3], shearBoxShape[3]);
	lowerBoxShape->addChildShape(shearBoxOffset[4], shearBoxShape[4]);
	upperBoxShape->addChildShape(shearBoxOffset[5], shearBoxShape[5]);
	upperBoxShape->addChildShape(shearBoxOffset[6], shearBoxShape[6]);
	upperBoxShape->addChildShape(shearBoxOffset[7], shearBoxShape[7]);
	upperBoxShape->addChildShape(shearBoxOffset[8], shearBoxShape[8]);


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

	lowerBoxRB->setFriction(btSqrt(internalFrict));	// in Bullet friction ratio of a contact is a multiplication of both frictions
	upperBoxRB->setFriction(btSqrt(internalFrict));

	lowerBoxRB->setActivationState(4);
	upperBoxRB->setActivationState(4);

	//restrains:
	upperBoxRB->setLinearFactor(btVector3(1,0,0));		//upper box only can move in linear x direction 
	upperBoxRB->setAngularFactor(btVector3(0,0,0));

	dynamicsWorld->updateSingleAabb(lowerBoxRB);		// update the compound shape 
	dynamicsWorld->updateSingleAabb(upperBoxRB);		// update the compound shape 
}


void gimpactShearBoxMaker(btDiscreteDynamicsWorld* dynamicsWorld) {
	// lowerBox	trimesh shape
		btTriangleIndexVertexArray* lowerBoxVertexArrays = new btTriangleIndexVertexArray(NUM_TRIANGLES_LOWERBOX, &gIndices_LOWERBOX[0][0], 3*sizeof(int),NUM_VERTICES_LOWERBOX,(REAL*) &gVertices_LOWERBOX[0], sizeof(REAL)*3);
		btGImpactMeshShape* lowerBoxShape = new btGImpactMeshShape(lowerBoxVertexArrays);
		lowerBoxShape->setLocalScaling(btVector3(1.f,1.f,1.f));
		lowerBoxShape->setMargin(0.0f);
		lowerBoxShape->updateBound();

		// btCollisionShape* compoundLowerShape;
		// compoundLowerShape = btCreateCompoundFromGimpactShape(lowerBoxShape, 0.0f);

		// upperBox trimesh shape
		btTriangleIndexVertexArray* upperBoxVertexArrays = new btTriangleIndexVertexArray(NUM_TRIANGLES_UPPERBOX, &gIndices_UPPERBOX[0][0], 3*sizeof(int),NUM_VERTICES_UPPERBOX,(REAL*) &gVertices_UPPERBOX[0], sizeof(REAL)*3);
		btGImpactMeshShape* upperBoxShape = new btGImpactMeshShape(upperBoxVertexArrays);
		upperBoxShape->setLocalScaling(btVector3(1.f,1.f,1.f));
		upperBoxShape->setMargin(0.0f);
		upperBoxShape->updateBound();

		// btCollisionShape* compoundUpperShape;
		// compoundUpperShape = btCreateCompoundFromGimpactShape(upperBoxShape, 0.0f);

		//motionStates of lower and upper boxes
		btDefaultMotionState* lowerBoxState	=	new btDefaultMotionState(btTransform(upright, btVector3(0, 0, 0)));
		btDefaultMotionState* upperBoxState	=	new btDefaultMotionState(btTransform(upright, btVector3(0, 0, boxHeight/2.f + boxGap)));

		// rigid body construction infos, definitions, adding them to the world, and the settings
		btRigidBody::btRigidBodyConstructionInfo lowerBoxCI(0, lowerBoxState, lowerBoxShape, btVector3(0,0,0));		// lowerbox is static object (animated moved)
		upperBoxShape->calculateLocalInertia(upperBoxMass, upperBoxInertia);
		btRigidBody::btRigidBodyConstructionInfo upperBoxCI(upperBoxMass, upperBoxState, upperBoxShape, upperBoxInertia);

		btRigidBody* lowerBoxRB = new btRigidBody(lowerBoxCI);
		btRigidBody* upperBoxRB = new btRigidBody(upperBoxCI);
		dynamicsWorld->addRigidBody(lowerBoxRB);
		dynamicsWorld->addRigidBody(upperBoxRB);

		lowerBoxRB->setRestitution(restitutionCoef);
		upperBoxRB->setRestitution(restitutionCoef);

		lowerBoxRB->setFriction(btSqrt(internalFrict));	// in Bullet friction ratio of a contact is a multiplication of both frictions
		upperBoxRB->setFriction(btSqrt(internalFrict));

		lowerBoxRB->setActivationState(4);
		upperBoxRB->setActivationState(4);

		//restrains:
		lowerBoxRB->setLinearFactor(btVector3(1,0,0));		//lower box only can move in linear x direction 
		lowerBoxRB->setAngularFactor(btVector3(0,0,0));

		upperBoxRB->setLinearFactor(btVector3(1,0,0));		//upper box only can move in linear x direction 
		upperBoxRB->setAngularFactor(btVector3(0,0,0)); 
}


#endif