#ifndef MEMBRANE(DEPRECATED)_H
#define MEMBRANE(DEPRECATED_H
#include <iostream>


//The function needed to build the trimesh format of the membrane (cylindrical softbody). It can be used as a softBodyHelper as well
	void trimeshMembraneGenerator(btScalar gVertices[], int gIndices[][3], btScalar membraneHeight, btScalar membraneRadius, const int verticalResolution, const int radialResolution, btScalar membraneThickness){

		btScalar	zComponent	=	-plateThickness;
		btScalar	alpha		=	0;				// the incremental angle (in radians) that changes through every point
		int 		rowIndex	= 0;

		//inner side of membrane
			// recording the vertexes in gVertices: each loop records the x,y and z of one point in gVertices array
			for (int i=0; i < (3 * ((verticalResolution+1) * radialResolution)); 0){
				for(int k=0; k < radialResolution; k++){
					gVertices[i] 	= membraneRadius * btCos(alpha);		// the x coordinate of the point (horizontal)
					gVertices[i+1]	= membraneRadius * btSin(alpha);		// the y coordinate of the point (horizontal)
					gVertices[i+2]	= zComponent;		// the z coordinate of the point (vertical)

					alpha	+=	2 * SIMD_PI / radialResolution;		//here the alpha angle is incremented
					i += 3;	// i is incremented 3
				}
				zComponent 	+= 	membraneHeight/verticalResolution;	//here the height is incremented
				alpha 	=	0; 					//at the end of one loop of horizontal points, alpha is set to zero again for the next loop
			}

			//filling gIndices array:
			for(int i=0; i < 2 * radialResolution * verticalResolution; 0){

				for (int k=0 ; k < radialResolution; k++){

					gIndices[i][0]	=	rowIndex+k;
					gIndices[i][1]	=	(rowIndex+k+1 == rowIndex + radialResolution) ? (rowIndex) : (rowIndex+k+1);
					gIndices[i][2]	=	rowIndex+k+radialResolution;

					gIndices[i+1][0]	=	(rowIndex+k+1 ==  rowIndex + radialResolution) ? (rowIndex) : (rowIndex+k+1);
					gIndices[i+1][1]	=	(rowIndex+k+1 == rowIndex + radialResolution) ? (rowIndex + radialResolution) : (rowIndex+k+1+radialResolution);
					gIndices[i+1][2]	=	rowIndex+k+radialResolution;
					
					i += 2;
				}
				rowIndex += radialResolution;
			}

		//outer side of the membrane
			// recording the vertexes in gVertices: each loop records the x,y and z of one point in gVertices array
			zComponent	=	-plateThickness;
			alpha		=	0;
			rowIndex	= 0;
			membraneRadius += membraneThickness;
			for (int i=(3 * ((verticalResolution+1) * radialResolution)); i < (3 *2 * ((verticalResolution+1) * radialResolution)); 0){
				for(int k=0; k < radialResolution; k++){
					gVertices[i] 	= membraneRadius * btCos(alpha);		// the x coordinate of the point (horizontal)
					gVertices[i+1]	= membraneRadius * btSin(alpha);		// the y coordinate of the point (horizontal)
					gVertices[i+2]	= zComponent;		// the z coordinate of the point (vertical)

					alpha	+=	2 * SIMD_PI / radialResolution;		//here the alpha angle is incremented
					i += 3;	// i is incremented 3
				}
				zComponent 	+= 	membraneHeight/verticalResolution;	//here the height is incremented
				alpha 	=	0; 					//at the end of one loop of horizontal points, alpha is set to zero again for the next loop
			}

			//filling gIndices array:
			for(int i= 2 * radialResolution * verticalResolution; i < 2 * 2 * radialResolution * verticalResolution; 0){

				for (int k=0 ; k < radialResolution; k++){
					gIndices[i][0]	=	rowIndex+k;
					gIndices[i][1]	=	(rowIndex+k+1 == rowIndex + radialResolution) ? (rowIndex) : (rowIndex+k+1);
					gIndices[i][2]	=	rowIndex+k+radialResolution;

					gIndices[i+1][0]	=	(rowIndex+k+1 ==  rowIndex + radialResolution) ? (rowIndex) : (rowIndex+k+1);
					gIndices[i+1][1]	=	(rowIndex+k+1 == rowIndex + radialResolution) ? (rowIndex + radialResolution) : (rowIndex+k+1+radialResolution);
					gIndices[i+1][2]	=	rowIndex+k+radialResolution;
					
					i += 2;
				}
				rowIndex += radialResolution;
			}

		//bottom cap of the membrane
			for(int i= 2 * 2 * radialResolution * verticalResolution; i < ((2 * 2 * radialResolution * verticalResolution) + (radialResolution * 2) ); 0){

				for (int k=0 ; k < radialResolution; k++){

					gIndices[i][0]	=	k;
					gIndices[i][1]	=	(k+1 == radialResolution) ? (0) : (k+1);
					gIndices[i][2]	=	k+(radialResolution*(verticalResolution+1));

					gIndices[i+1][0]	=	(k+1 == radialResolution) ? (0) : (k+1);
					gIndices[i+1][1]	=	(k+1 == radialResolution) ? (radialResolution*(verticalResolution+1)) : (k+1+(radialResolution*(verticalResolution+1)));
					gIndices[i+1][2]	=	k+(radialResolution*(verticalResolution+1));
					
					i += 2;
				}	
			}

		//top of the membrane
			for(int i= ((2 * 2 * radialResolution * verticalResolution) + (radialResolution * 2) ); i < ((2 * 2 * radialResolution * verticalResolution) + (radialResolution * 4) ); 0){
				
				int triangleIndex = radialResolution * verticalResolution;

				for (int k=0 ; k < radialResolution; k++){

					gIndices[i][0]	=	triangleIndex+k;
					gIndices[i][1]	=	(triangleIndex+k+1 == triangleIndex+radialResolution) ? (triangleIndex) : (triangleIndex+k+1);
					gIndices[i][2]	=	triangleIndex+k+(radialResolution*(verticalResolution+1));

					gIndices[i+1][0]	=	(triangleIndex+k+1 == triangleIndex+radialResolution) ? (triangleIndex) : (triangleIndex+k+1);
					gIndices[i+1][1]	=	(triangleIndex+k+1 == triangleIndex+radialResolution) ? (triangleIndex+(radialResolution*(verticalResolution+1))) : (triangleIndex+k+1+(radialResolution*(verticalResolution+1)));
					gIndices[i+1][2]	=	triangleIndex+k+(radialResolution*(verticalResolution+1));
					
					i += 2;
				}	
			}
	}

	//The function needed to build the convex format of the membrane (cylindrical softbody). It can be used as a softBodyHelper as well
	void convexMembraneGenerator(btVector3 convexVertex[], btScalar membraneHeight, btScalar membraneRadius, const int verticalResolution, const int radialResolution){
		btScalar	zComponent	=	-plateThickness/2;
		btScalar	alpha		=	0;				// the incremental angle (in radians) that changes through every point

		// recording the vertexes in gVertices: each loop records the x,y and z of one point in gVertices array
		for (int i=0; i < (verticalResolution+1) * radialResolution; 0){
			for(int k=0; k < radialResolution; k++){
				convexVertex[i].setX(membraneRadius * btCos(alpha));		// the x coordinate of the point (horizontal)
				convexVertex[i].setY(membraneRadius * btSin(alpha));		// the y coordinate of the point (horizontal)
				convexVertex[i].setZ(zComponent);		// the z coordinate of the point (vertical)

				alpha	+=	2 * SIMD_PI / radialResolution;		//here the alpha angle is incremented
				i++;	//increment i to go to the next point
			}
			zComponent 	+= 	membraneHeight/verticalResolution;	//here the height is incremented
			alpha 	=	0; 					//at the end of one loop of horizontal points, alpha is set to zero again for the next loop
		}
	}

#endif