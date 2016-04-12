#ifndef VARIABLES_H
#define VARIABLES_H
#include <iostream>
#include <fstream>
#include <btBulletDynamicsCommon.h>


// Global variables for direct shear test simulation using Bullet3
//########## Variables ##########
// Geometrical variables
		bool 				strainControlledTest=	false;		//to use strain controlled test or stress controlled (using springs)

		btScalar			scaleFactor			=	1000;						//if 1 in m, if 1000 in mm	
		btScalar			boxSize				=	0.06 * scaleFactor;			//input the parameters in SI units
		btScalar			boxHeight			=	0.03 * scaleFactor;
		btScalar			metalBeadDiameter	=	0.004 * scaleFactor;
		btScalar			topPlateGap			=	0.0015 * scaleFactor;
		btScalar			boxGap				=	0.003 * scaleFactor ;
		btVector3			topPlateLinearFactor=	btVector3(1,1,1);		//topPlate restrains (if 1: free, if 0: restrained)
		btVector3			topPlateAngularFactor=	btVector3(1,1,0);		//topPlate cannot rotate about z axis
		btVector3			upperBoxLinearFactor=	btVector3(1,1,0);	//upperBox can move only in x = shearing direction 
		btVector3			upperBoxAngularFactor=	btVector3(0,0,0);		//upperBox cannot rotate in any direction

// Weights
		btScalar			upperBoxMass		= 	1.0f;			//since upperBox is a dynamic object, a non-zero mass should be assigned to it.
		btScalar			topPlateMass		=	1.0f;			//gravity is zero, so, mass should be at reasonable amount.
		btScalar			metalBeadMass		=	0.261f;			//the mass of each metal bead is 261 grams but a reasonable amount should be considered.

// Simulation parameters
		btVector3			gravity				=	btVector3(0,0,0);	// gravity is zero, because direct shear test is not gravity-dependent.
		btScalar			normalStress		=	150000;				//in Pa: on top plate linear rate in consolidation phase.
		btScalar			springStiffness		=	10000000 /scaleFactor;		// in [N/m]
		btScalar			springDamping		=	1000000 /scaleFactor;		// in [N.s/m]
		btScalar			timeFactor			=	1;							//to minimize the scattering of data.
		btScalar			deltaTime			=	0.01 * timeFactor;
		btScalar			consolidationTime	=	1 * timeFactor;			//the time needed for application of vertical stress
		btScalar			shearingTime		=	120 * timeFactor; 		//the time needed for application of shearing stress after consolidation
		btScalar			numberOfIterations	=	70;				//20 is default value, change it only if its needed. The more iteration numbers, the less fluctuations you will have in your results.
		btScalar			restitutionCoef		=	0.0;			//we don't seek for trouble!
		btScalar			internalFrict		=	0.091;			//friction ratio based on ball-ball friction lab tests on metal beads
		btScalar			boundaryFrict		=	internalFrict;	//just an assumption: equal to metal ball-on-metal ball friction
		btScalar			writeOutputInterval	=	0.1 * timeFactor;		// [in seconds] write the data of the output every 0.1s 


// Dependent variables
		int 				approxNumBeads		=	btPow(boxSize,3)/btPow(metalBeadDiameter,3);
		btScalar			endTime				= 	shearingTime + consolidationTime; 	//total time of simulation
		btScalar			boxThickness		=	boxSize / 6.f;
		btScalar			topPlateThickness	= 	boxSize / 10.f;
		btScalar			consolidationSteps	=	consolidationTime/deltaTime;	//steps needed to complete the consolidation phase
		btScalar			shearingSteps		=	shearingTime/deltaTime;			//steps needed to complete the shearing phase
		btScalar			totalSteps			=	endTime/deltaTime;	//here, number of steps depends on the deltaTime and consolidation and shearing times
		btQuaternion		upright				=	btQuaternion(0,0,0,1);
		btScalar			tunningStress		=	0.0;				// A stress tunning factor, dependent on the laboratory results
		btScalar			normalForce 		= 	(normalStress==0) ? (0) : (-(normalStress+tunningStress)*btPow(boxSize/scaleFactor,2)); //vertical force (N) in -z direction
		btScalar			shearDisplacement	=	boxSize * 0.2;		// amount of total shear displacement for lowerBox in the shearing phase
		btTransform			springToSupport		=	btTransform(upright, btVector3(-boxSize, 0, 3.25 * boxHeight/4.f + boxGap));	//Position of spring with respect to the support
		btTransform			springToUpperBox	=	btTransform(upright, btVector3(boxSize + boxThickness, 0, 3.25 * boxHeight/4.f + boxGap));	//Position of spring with respect to the upperbox

// buffers
		btTransform		topPlateBuffer;
		btTransform		upperBoxRBBuffer;
		btTransform		lowerBoxBuffer;

// Unaccessible variables by the user
		btScalar							rollingFriction= 0.05;	// to avoid beads roating forever a small amount of rolling friction in needed.
		btVector3							upperBoxInertia;
		btVector3							topPlateInertia;
		btVector3							metalBeadInertia;
		bool 								xCoordControl=false;	//to control the flow of bead positioning in horizontal direction (controls the shift)
		bool 								zCoordControl=false;	//to control the flow of bead positioning in vertical direction (controls the shift)
		int 								beadIndex=-1;		//an index for counting the beads
		unsigned int 						exactNumBead;		//exact number of beads= max(beadIndex) +1
		float 								xBeadPos = -boxSize/2.f + metalBeadDiameter/2.f;
		float 								yBeadPos = -boxSize/2.f + metalBeadDiameter/2.f;
		float 								zBeadPos = metalBeadDiameter/2.f;
		const float 						horBoxLimit = boxSize/2.f - metalBeadDiameter/2.f + metalBeadDiameter/100.f;	//limit of beads position in x and y directions
		btScalar							pyramidSlant = metalBeadDiameter * btCos(btRadians(30));	// slant of triangular pyramid of beads
		btScalar							pyramidHeight = btSqrt( btPow(pyramidSlant,2) - btPow((btTan(btRadians(30)) * metalBeadDiameter/2.f),2) );
		const float 						verBoxLimit = boxHeight - metalBeadDiameter/4.f; //limit of beads position in z direction
		btAlignedObjectArray<btTransform> 	shearBoxOffset;			//array for offsets of the child shapes of the lower/upper box.
		btScalar							simTime;				//time of simulation
		std::ofstream						consolidationOutput;	//output file during consolidation
		std::ofstream						shearingOutput;			//output file during shearing
		std::ofstream						contactRVEOutput;		//output file during shearing for number of contacts in RVE
		std::ofstream						forceRatioOutput; 		//output file for calculation of normal to friction force ratio
		std::ofstream						beadRotationOutput;		//output file of bead rotations in total sample domain and in RVE
		btTransform							topPlateBasePos;		//state (position and orientation) of the top plate at the end of consolidation.
		int 								checkpointTime1, checkpointTime2, checkpointTime3, checkpointTime4, checkpointTime5, checkpointTime6, checkpointTime7, checkpointTime8, checkpointTime9, checkpointTime10, checkpointTime11, checkpointTime12, checkpointTime13, checkpointTime14 ;		//controlling integers for the shearing output writer
		std::ofstream						beadsPositions;			//output file for beads positions (paraview visualisations)
		btScalar 							xDispBuffer[7000] = {0};	//buffers for displacement increment in x dir
		btScalar 							yDispBuffer[7000] = {0};	//buffers for displacement increment in y dir
		btScalar 							zDispBuffer[7000] = {0};	//buffers for displacement increment in z dir
		btScalar							lowerBoxXPos1 = 0; 				//lowerBoxPos to minus from the increments  (for paraview output)
		btScalar							lowerBoxXPos2 = 0; 				//lowerBoxPos to minus from the increments  (for paraview output)
		btScalar							deltaLowerBoxPos;
		btScalar							fixedSphereRadius = metalBeadDiameter;
		btScalar							boxFittingYDir;				//amount of the gap between the last bead edge to the edge of the shear box in y-dir


#endif