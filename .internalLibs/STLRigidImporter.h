#ifndef STLRIGIDIMPORTER_H
#define STLRIGIDIMPORTER_H

#include <stdio.h> //fopen
// 	Trimesh shape libs
#include "../../../Bullet/bullet3/src/BulletCollision/Gimpact/btGImpactShape.h"
#include "../../../Bullet/bullet3/src/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "../../../Bullet/bullet3/src/BulletCollision/Gimpact/btCompoundFromGimpact.h"

struct GLInstanceVertex
{
	float xyzw[4];
	float normal[3];
	float uv[2];
};
struct GLInstanceGraphicsShape
{
	btAlignedObjectArray<GLInstanceVertex>*	m_vertices;
	int				m_numvertices;
	btAlignedObjectArray<int>* 		m_indices;
	int				m_numIndices;
	float			m_scaling[4];
	int 			m_numtriangles;
};

struct MySTLTriangle
{
	float normal[3];
	float vertex0[3];
	float vertex1[3];
	float vertex2[3];
};

struct verticesBuffer
{
	btVector3 	coordinate;
	int 		indication;
	int 		correctedIndex;
};

static GLInstanceGraphicsShape* LoadMeshFromSTL(const char* relativeFileName)
{
	GLInstanceGraphicsShape* shape = 0;

	int triangleNumberBuffer;
	
	FILE* file = fopen(relativeFileName,"rb");
	if (file)
	{
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET))
		{
			printf("Error: Cannot access file to determine size of %s\n", relativeFileName);
		} else
		{
			if (size)
			{
				printf("* Open STL file of %d bytes\n",size);
				char* memoryBuffer = new char[size+1];
				int actualBytesRead = fread(memoryBuffer,1,size,file);
				if (actualBytesRead!=size)
				{
					printf("Error reading from file %s",relativeFileName);
				} else
				{
					int numTriangles = *(int*)&memoryBuffer[80];
					triangleNumberBuffer = numTriangles;
					
					if (numTriangles)
					{
						{
							//perform a sanity check instead of crashing on invalid triangles/STL files
							int expectedBinaryFileSize = numTriangles* 50 + 84;
							if (expectedBinaryFileSize != size)
							{
								return 0;
							}

						}
						shape = new GLInstanceGraphicsShape;
//						btAlignedObjectArray<GLInstanceVertex>*	m_vertices;
//						int				m_numvertices;
//						btAlignedObjectArray<int>* 		m_indices;
//						int				m_numIndices;
//						float			m_scaling[4];
						shape->m_scaling[0] = 1;
						shape->m_scaling[1] = 1;
						shape->m_scaling[2] = 1;
						shape->m_scaling[3] = 1;
						int index = 0;
						shape->m_indices = new btAlignedObjectArray<int>();
						shape->m_vertices = new btAlignedObjectArray<GLInstanceVertex>();
						for (int i=0;i<numTriangles;i++)
						{
							char* curPtr = &memoryBuffer[84+i*50];
							MySTLTriangle* tri = (MySTLTriangle*) curPtr;
							
							GLInstanceVertex v0,v1,v2;
							if (i==numTriangles-2)
							{
								printf("* !\n");
							}
							v0.uv[0] = v1.uv[0] = v2.uv[0] = 0.5;
							v0.uv[1] = v1.uv[1] = v2.uv[1] = 0.5;
							for (int v=0;v<3;v++)
							{
								v0.xyzw[v] = tri->vertex0[v];
								v1.xyzw[v] = tri->vertex1[v];
								v2.xyzw[v] = tri->vertex2[v];
								v0.normal[v] = v1.normal[v] = v2.normal[v] = tri->normal[v];
							}
							v0.xyzw[3] = v1.xyzw[3] = v2.xyzw[3] = 0.f;
							
							shape->m_vertices->push_back(v0);
							shape->m_vertices->push_back(v1);
							shape->m_vertices->push_back(v2);
							
							shape->m_indices->push_back(index++);
							shape->m_indices->push_back(index++);
							shape->m_indices->push_back(index++);
							
						}
					}
				}
				
				delete[] memoryBuffer;
			}
		}
		fclose(file);
	}
	shape->m_numIndices 	= shape->m_indices->size();
	shape->m_numvertices 	= shape->m_vertices->size();
	shape->m_numtriangles 	=  triangleNumberBuffer;

	return shape;
}



//STL importer directly to a rigid body
void STLImporterToStaticRigidBody(btGImpactMeshShape* &STLShape, const char* relativeFileName){
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

	const int finalVertNum	= 	stlNumVerticesRefactored;
	const int finalTriNum	=	stlNumTriangles;

	btTriangleIndexVertexArray* trimeshVertexArrays = new btTriangleIndexVertexArray(finalTriNum, &stlIndices[0][0], 3*sizeof(int),finalVertNum,(btScalar*) &stlVertices[0], sizeof(btScalar)*3);

	STLShape = new btGImpactMeshShape(trimeshVertexArrays);
	STLShape->setLocalScaling(btVector3(1.f,1.f,1.f));
	STLShape->setMargin(0.0f);
	STLShape->updateBound();
	
	std::cout << "* The rigid body is created based on the STL mesh file successfully ..." << std::endl;
	std::cout << "=====================================" << std::endl;	

	// delete trimeshVertexArrays;
	// delete stlMesh;
}

//STL importer directly to a rigid body
void STLImporterToDynamicConvexRigidBody(btGImpactMeshShape* &STLShape, const char* relativeFileName, btAlignedObjectArray<btVector3> &rigidVertexPoints){
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

				rigidVertexPoints.push_back(stlVerticesBuffer[i].coordinate);

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

	const int finalVertNum	= 	stlNumVerticesRefactored;
	const int finalTriNum	=	stlNumTriangles;

	btTriangleIndexVertexArray* trimeshVertexArrays = new btTriangleIndexVertexArray(finalTriNum, &stlIndices[0][0], 3*sizeof(int),finalVertNum,(btScalar*) &stlVertices[0], sizeof(btScalar)*3);

	STLShape = new btGImpactMeshShape(trimeshVertexArrays);
	STLShape->setLocalScaling(btVector3(1.f,1.f,1.f));
	STLShape->setMargin(0.0f);
	STLShape->updateBound();
	
	std::cout << "* The rigid body is created based on the STL mesh file successfully ..." << std::endl;
	std::cout << "=====================================" << std::endl;	

	// delete trimeshVertexArrays;
	// delete stlMesh;
}

#endif
