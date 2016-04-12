///The btBvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
///If you required moving concave triangle meshes, it is recommended to perform convex decomposition
///using HACD, see Bullet/Demos/ConvexDecompositionDemo. 
///Alternatively, you can use btGimpactMeshShape for moving concave triangle meshes.
///btBvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and 
///cache friendly traversal for PlayStation 3 Cell SPU. 
///It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
#ifndef GIMPACTSHEARBOX_H
#define GIMPACTSHEARBOX_H
#include <btBulletDynamicsCommon.h>


//Data for Trimesh object of lower and upper part of shear box.
	#define REAL btScalar
	//###	LOWERBOX 	### the origin is at the bottom of inside plane
	const int NUM_TRIANGLES_LOWERBOX	=	28;
	const int NUM_VERTICES_LOWERBOX		= 	16;
	const int NUM_INDICES_LOWERBOX		= 	NUM_TRIANGLES_LOWERBOX * 3;


	REAL gVertices_LOWERBOX[NUM_VERTICES_LOWERBOX * 3] = {
		REAL(boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(-boxThickness), 		// #0
		REAL(boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(-boxThickness),		// #1
		REAL(-boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(-boxThickness),		// #2
		REAL(-boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(-boxThickness),		// #3

		REAL(boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(boxHeight/2.f),		// #4
		REAL(boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(boxHeight/2.f),		// #5
		REAL(-boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(boxHeight/2.f),		// #6
		REAL(-boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(boxHeight/2.f),		// #7

		REAL(boxSize/2.f), REAL(boxSize/2.f), REAL(boxHeight/2.f),		// #8
		REAL(boxSize/2.f), REAL(-boxSize/2.f), REAL(boxHeight/2.f),		// #9
		REAL(-boxSize/2.f), REAL(-boxSize/2.f), REAL(boxHeight/2.f),	// #10
		REAL(-boxSize/2.f), REAL(boxSize/2.f), REAL(boxHeight/2.f),		// #11

		REAL(boxSize/2.f), REAL(boxSize/2.f), REAL(0.0),	// #12
		REAL(boxSize/2.f), REAL(-boxSize/2.f), REAL(0.0),	// #13
		REAL(-boxSize/2.f), REAL(-boxSize/2.f), REAL(0.0),	// #14
		REAL(-boxSize/2.f), REAL(boxSize/2.f), REAL(0.0)	// #15
	};

	int gIndices_LOWERBOX[NUM_TRIANGLES_LOWERBOX][3] = {
		{0,1,2},
		{0,2,3},

		{1,0,4},
		{1,4,5},

		{7,3,2},
		{2,6,7},
		
		{0,7,4},
		{0,3,7},

		{2,1,5},
		{2,5,6},

		{5,4,8},
		{5,8,9},

		{8,4,11},
		{4,7,11},

		{7,6,10},
		{7,10,11},

		{6,9,10},
		{6,5,9},

		{8,11,15},
		{8,15,12},

		{11,10,14},
		{11,14,15},

		{10,9,13},
		{10,13,14},

		{9,8,12},
		{9,12,13},

		{14,13,12},
		{14,12,15}
	};


	//###	UPPERBOX 	### the origin is at the center, the lowest point
	const int NUM_TRIANGLES_UPPERBOX =32;
	const int NUM_VERTICES_UPPERBOX = 16;
	const int NUM_INDICES_UPPERBOX  = NUM_TRIANGLES_UPPERBOX * 3;


	REAL gVertices_UPPERBOX[NUM_VERTICES_UPPERBOX * 3] = {
		REAL(boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(0.0),		// #0
		REAL(boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(0.0),		// #1
		REAL(-boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(0.0),	// #2
		REAL(-boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(0.0),		// #3

		REAL(boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(1.5 * boxHeight/2.f),		// #4
		REAL(boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(1.5 * boxHeight/2.f),		// #5
		REAL(-boxSize/2.f + boxThickness), REAL(-boxSize/2.f + boxThickness), REAL(1.5 * boxHeight/2.f),	// #6
		REAL(-boxSize/2.f + boxThickness), REAL(boxSize/2.f + boxThickness), REAL(1.5 * boxHeight/2.f),		// #7

		REAL(boxSize/2.f), REAL(boxSize/2.f), REAL(1.5 * boxHeight/2.f),		// #8
		REAL(boxSize/2.f), REAL(-boxSize/2.f), REAL(1.5 * boxHeight/2.f),		// #9
		REAL(-boxSize/2.f), REAL(-boxSize/2.f), REAL(1.5 * boxHeight/2.f),		// #10
		REAL(-boxSize/2.f), REAL(boxSize/2.f), REAL(1.5 * boxHeight/2.f),		// #11

		REAL(boxSize/2.f), REAL(boxSize/2.f), REAL(0.0),	// #12
		REAL(boxSize/2.f), REAL(-boxSize/2.f), REAL(0.0),	// #13
		REAL(-boxSize/2.f), REAL(-boxSize/2.f), REAL(0.0),	// #14
		REAL(-boxSize/2.f), REAL(boxSize/2.f), REAL(0.0)	// #15
	};

	int gIndices_UPPERBOX[NUM_TRIANGLES_UPPERBOX][3] = {
		{0,5,1},
		{0,4,5},

		{3,2,6},
		{3,6,7},

		{0,7,4},
		{0,3,7},

		{1,5,6},
		{1,6,2},

		{5,8,9},
		{5,4,8},

		{4,7,11},
		{4,11,8},

		{7,6,10},
		{7,10,11},

		{6,5,9},
		{6,9,10},

		{1,13,12},
		{1,12,0},

		{0,15,3},
		{0,12,15},

		{3,14,2},
		{3,15,14},

		{2,13,1},
		{2,14,13},

		{9,12,13},
		{9,8,12},

		{8,11,15},
		{8,15,12},

		{11,10,14},
		{11,14,15},

		{10,13,14},
		{10,9,13}
	};

#endif