#ifndef STLSOFTIMPORTER_H
#define STLSOFTIMPORTER_H

#include <stdio.h> //fopen
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftBodyHelpers.h"							//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftRigidDynamicsWorld.h"					//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btDefaultSoftBodySolver.h"						//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"	//ADDED
#include "../../../Bullet/bullet3/src/BulletSoftBody/btSoftSoftCollisionAlgorithm.h"		//ADDED
#include "STLRigidImporter.h"


//STLImporter (extra feature) The original Bullet function just import the trimesh shape  as individual trimeshes, which will fall apart if used as softbody. This function "ties" the trimshes together to work as a joined softbody
void STLImporterToSoftBody(btSoftRigidDynamicsWorld* dynamicsWorld, btSoftBody* &sb, const char* relativeFileName){
	std::cout << "=====================================" << std::endl;	
	std::cout << "* Importing a STL mesh ..." << std::endl;
	//filling the "stlMesh" with STL object
	GLInstanceGraphicsShape* stlMesh = LoadMeshFromSTL(relativeFileName);
	std::cout << "* The STL mesh is imported successfully ..." << std::endl;

	// the buffers for deriving the trimesh data from STL mesh
		int 		stlNumTriangles		= 	stlMesh->m_numtriangles;
		int 		stlNumVertices 		= 	stlMesh->m_numvertices;
		int 		stlNumIndices  		= 	stlMesh->m_indices->size();
		int 		stlNumVerticesRefactored = 0;
		std::cout << "* Number of vertices of imported STL mesh: " << stlNumVertices << std::endl;
		std::cout << "* Number of triangles of imported STL mesh: " << stlNumTriangles << std::endl;
		std::cout << "* Number of indices of imported STL mesh: " << stlNumIndices << std::endl;


		btScalar	stlVertices[stlNumVertices*3];
		int 		stlIndices[stlNumTriangles][3];
		verticesBuffer		stlVerticesBuffer[stlNumVertices];
		int mainIndex=0;

	//filling the buffer with the counterparts of imported stlMesh
		for (int i=0; i<stlNumVertices; i++){
			//just filling
			stlVerticesBuffer[i].coordinate = btVector3(stlMesh->m_vertices->at(i).xyzw[0], stlMesh->m_vertices->at(i).xyzw[1],stlMesh->m_vertices->at(i).xyzw[2]);
			
			stlVerticesBuffer[i].indication = 0;
			stlVerticesBuffer[i].correctedIndex = 0;


			
			//indicating the duplicate pair (zero means not duplicate)
			if(i>0){
				for(int k=0; k <i ; k++){
					if ( (stlVerticesBuffer[k].coordinate.getX() ==  stlVerticesBuffer[i].coordinate.getX()) &&
						 (stlVerticesBuffer[k].coordinate.getY() ==  stlVerticesBuffer[i].coordinate.getY()) && 
						 (stlVerticesBuffer[k].coordinate.getZ() ==  stlVerticesBuffer[i].coordinate.getZ()) ) {
						
						stlVerticesBuffer[i].coordinate = btVector3(stlMesh->m_vertices->at(i).xyzw[0], stlMesh->m_vertices->at(i).xyzw[1], stlMesh->m_vertices->at(i).xyzw[2]);

						stlVerticesBuffer[i].indication = k+1 - stlVerticesBuffer[k].correctedIndex;

						mainIndex++;
						stlVerticesBuffer[i].correctedIndex = mainIndex;

						// std::cout << "DUPLICATE FOUND!!" << std::endl;
						break; 
					}
					stlVerticesBuffer[i].correctedIndex = mainIndex;

				}
			}
		}

		mainIndex=0;

		for (int i=0; i<stlNumVertices;i++){
			if (stlVerticesBuffer[i].indication == 0){
				stlVertices[(mainIndex*3)]		=	stlVerticesBuffer[i].coordinate.getX();
				stlVertices[(mainIndex*3)+1]	=	stlVerticesBuffer[i].coordinate.getY();
				stlVertices[(mainIndex*3)+2]	=	stlVerticesBuffer[i].coordinate.getZ();
				stlNumVerticesRefactored++;
				mainIndex++;
			}
		}

		std::cout << "* The number of vertices after modification: " << stlNumVerticesRefactored << std::endl;
		std::cout << "* Total number of " << stlNumVertices-stlNumVerticesRefactored << " vertices are merged." << std::endl;

		for (int i=0; i<stlNumTriangles; i++){
			for (int j=0; j <3; j++){
				stlIndices[i][j] = (stlVerticesBuffer[(i*3)+j].indication == 0)? ((i*3) + j - stlVerticesBuffer[(i*3)+j].correctedIndex) : (stlVerticesBuffer[(i*3)+j].indication - 1);
			}
		}

	//passing the buffers to the softbody data using CreateFromTriMesh
	sb 	= 	btSoftBodyHelpers::CreateFromTriMesh(dynamicsWorld->getWorldInfo(), stlVertices, &stlIndices[0][0],stlNumTriangles);
	std::cout << "* The softbody is created based on the STL mesh file successfully ..." << std::endl;
	std::cout << "=====================================" << std::endl;	

	delete stlMesh;
}


#endif
