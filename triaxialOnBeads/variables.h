#ifndef VARIABLES_H
#define VARIABLES_H
#include <iostream>
#include <fstream>
#include <btBulletDynamicsCommon.h>


// Global variables for triaxial test on beads simulation using Bullet3
//########## Variables ##########
// Geometrical variables
		btScalar			scaleFactor				=	1000;						//if 1 in m, if 1000 in mm	
		btScalar			sampleRadius			=	0.025 * scaleFactor;			//input the parameters in SI units
		btScalar			sampleHeight			=	0.1 * scaleFactor;
		btScalar			metalBeadDiameter		=	0.004 * scaleFactor;
		btScalar			plateThickness			= 	sampleRadius / 3.f;
		btVector3			plateLinearFactor		=	btVector3(0,0,1);	//plates can move only in vertical direction 
		btVector3			plateAngularFactor		=	btVector3(0,0,0);		//plates cannot rotate in any direction

// Softbody variables (for membrane)
		bool 	soft_rigid_collision	= true;
		bool 	soft_soft_collision		= false;
		bool 	soft_self_collision		= false;

		btScalar	membraneHeight		=	sampleHeight + 2 * plateThickness;		//membrane is taller than the sample to cover the plates edges too
		btScalar	membraneRadius		=	sampleRadius;	//a small amount added to avoid initial overlapping with metal beads
		btScalar	membraneLinearStiffness		=	0.0019;	// for parameters of softbody, refer to 'membraneCharacterisation' folder in variables header file
		btScalar	membraneAngularStiffness	=	0.05;
		btScalar	membraneVolumetricStiffness	=	0.05;
		btScalar	membraneBendingStiffness	=	0.05;
		btScalar 	membraneDampingFactor		=	0.028;	// refer to 'membraneCharacterisation' folder in variables header file
		btScalar	membraneDynamicFriction		=	1.0;
		

		btScalar	membraneTotalMass				=	1.0;	// refer to 'membraneCharacterisation' folder in variables header file
		int 		membraneVelocityIteration		=	70;
		int 		membranePositionIteration		=	70;
		int 		membraneDriftSolverIteration	=	70;
		int 		membraneClusterSolverIteration	=	70;

// Weights
		btScalar			plateMass			=	1.0f;			//gravity is zero, so, mass should be at reasonable amount.
		btScalar			metalBeadMass		=	0.261f;			//the mass of each metal bead is 261 grams but a reasonable amount should be considered.

// Simulation parameters
		btVector3			gravity					=	btVector3(0,0,0);	// gravity is zero, because direct shear test is not gravity-dependent.
		btScalar			confiningStress			=	50000;				//in Pa: on top plate linear rate in consolidation phase.
		btScalar			timeFactor				=	1;							//to minimize the scattering of data.
		btScalar			deltaTime				=	0.005 * timeFactor;
		btScalar			consolidationTime		=	5 * timeFactor;			//the time needed for application of vertical stress
		btScalar			shearingTime			=	300 * timeFactor; 		//the time needed for application of shearing stress after consolidation
		btScalar			maxShearDisplacement	=	metalBeadDiameter;
		btScalar			numberOfIterations		=	70;				//20 is default value, change it only if its needed. The more iteration numbers, the less fluctuations you will have in your results.
		btScalar			restitutionCoef		=	0.0;			//we don't seek for trouble!
		btScalar			internalFrict		=	0.091;			//friction ratio based on ball-ball friction lab tests on metal beads
		btScalar			boundaryFrict		=	1;	//just an assumption: equal to metal ball-on-metal ball friction
		btScalar			writeOutputInterval	=	0.1 * timeFactor;		// [in seconds] write the data of the output every 0.1s
		btScalar			rollingFriction		= 	0.05;	// to avoid beads rotating forever a small amount of rolling friction in needed.



// Dependent variables
		int 				approxNumBeads		=	(btPow(sampleRadius,2)* SIMD_PI * sampleHeight /btPow(metalBeadDiameter,3))*2;
		btScalar			endTime				= 	shearingTime + consolidationTime; 	//total time of simulation
		btScalar			consolidationSteps	=	consolidationTime/deltaTime;	//steps needed to complete the consolidation phase
		btScalar			shearingSteps		=	shearingTime/deltaTime;			//steps needed to complete the shearing phase
		btScalar			totalSteps			=	endTime/deltaTime;	//here, number of steps depends on the deltaTime and consolidation and shearing times
		btQuaternion		upright				=	btQuaternion(0,0,0,1);
		btScalar			shearDisplacement	=	sampleHeight * 0.2;		// amount of total shear displacement in the shearing phase

// buffers
		

// Unaccessible variables by the user
		btScalar							fixedSphereRadius =  sampleRadius/2;
		btScalar							topmostBeadCenter;	// this will be filled within CylindricalBeadPackaging function with the topmost bead center
		btVector3							plateInertia;
		btVector3							metalBeadInertia;
		int 								beadIndex;		//an index for counting the beads
		unsigned int 						exactNumBead;		//exact number of beads= max(beadIndex) +1		
		btScalar							simTime = 0;				//time of simulation
		std::ofstream						consolidationOutput;	//output file during consolidation
		std::ofstream						shearingOutput;			//output file during shearing
		std::ofstream						contactRVEOutput;		//output file during shearing for number of contacts in RVE
		std::ofstream						forceRatioOutput; 		//output file for calculation of normal to friction force ratio
		std::ofstream						beadRotationOutput;		//output file of bead rotations in total sample domain and in RVE
		int 								checkpointTime1, checkpointTime2, checkpointTime3, checkpointTime4, checkpointTime5, checkpointTime6, checkpointTime7, checkpointTime8;		//controlling integers for the shearing output writer
		std::ofstream						beadsPositions;			//output file for beads positions (paraview visualisations)
		btScalar 							xDispBuffer[7000] = {0};	//buffers for displacement increment in x dir
		btScalar 							yDispBuffer[7000] = {0};	//buffers for displacement increment in y dir
		btScalar 							zDispBuffer[7000] = {0};	//buffers for displacement increment in z dir

#endif