#ifndef CYLINDRICALBEADPACKING_H
#define CYLINDRICALBEADPACKING_H
#include <iostream>
#include <fstream>


// For function for generation of rhombic sphere packing in cylindrical shape of the triaxial sample
void cylindricalSphereClosePacking(btAlignedObjectArray<btVector3*> &beadsCenter, btScalar sampleRadius, btScalar sampleHeight, btScalar metalBeadDiameter, int &beadIndex, btScalar &topmostPositionOfBeads){

	beadIndex	= -1; // initial setting
	btScalar 	xBeadPos	= 0;
	btScalar 	yBeadPos	= 0;
	btScalar 	rBeadPos	= 0;	// radial distance from center in x-y plane (horizon)
	btScalar 	zBeadPos	= metalBeadDiameter/2;
	btScalar	xShift		= metalBeadDiameter;
	btScalar	yShift 		= metalBeadDiameter * btCos(btRadians(30));
	btScalar	zShift		= btSqrt( btPow(yShift,2) - btPow((btTan(btRadians(30)) * metalBeadDiameter/2.f),2) );

	btScalar	zXFit =	-metalBeadDiameter/2;							//for shift of the metal beads in vertical direction
	btScalar	zYFit = (metalBeadDiameter/2) * btTan(btRadians(30));	//for shift of the metal beads in vertical direction

	bool 		shiftedRow = false;	// A controlling integer which controls the shifting of the metal bead positions with respect to vertical rows.

	for(zBeadPos= metalBeadDiameter/2; zBeadPos <= (sampleHeight - metalBeadDiameter/2);){

		//Central bead positioning
		rBeadPos = btSqrt( btPow(xBeadPos,2) + btPow(yBeadPos,2));	//updating the rbeadPos
		if (rBeadPos <= sampleRadius){
			beadIndex++;
			beadsCenter[beadIndex]->setX(xBeadPos);
			beadsCenter[beadIndex]->setY(yBeadPos);
			beadsCenter[beadIndex]->setZ(zBeadPos);
			xBeadPos += xShift;
		}

		//surrounded beads positioning
		for (int circularIndex=1; circularIndex < (ceil(sampleRadius/metalBeadDiameter) + 2) ; circularIndex++){
			for (int i = 0; i < circularIndex*6; i++){
				rBeadPos = btSqrt( btPow(xBeadPos,2) + btPow(yBeadPos,2));	//updating the rbeadPos
				if (rBeadPos <= sampleRadius) {
					beadIndex++;
					beadsCenter[beadIndex]->setX(xBeadPos);
					beadsCenter[beadIndex]->setY(yBeadPos);
					beadsCenter[beadIndex]->setZ(zBeadPos);
					topmostBeadCenter = zBeadPos;
					
				}

				//increments
				if (floor(i/circularIndex) == 0) {
					xBeadPos -= xShift/2;
					yBeadPos += yShift;
				} else if (floor(i/circularIndex) == 1) {
					xBeadPos -= xShift;
				} else if (floor(i/circularIndex) == 2) {
					xBeadPos -= xShift/2;
					yBeadPos -= yShift;
				} else if (floor(i/circularIndex) == 3) {
					xBeadPos += xShift/2;
					yBeadPos -= yShift;
				} else if (floor(i/circularIndex) == 4) {
					xBeadPos += xShift;
				} else if (floor(i/circularIndex) == 5 && i != (circularIndex*6) -1) {
					xBeadPos += xShift/2;
					yBeadPos += yShift;
				} else if (i == (circularIndex * 6) -1) {
					xBeadPos += (3 * xShift / 2);
					yBeadPos += yShift;
				}
			}
		}

		//A loop for changing the shiftedRow index from 0 to 1 and from 0 to 1 in each row.
		if (shiftedRow == false) {
			shiftedRow = true;
		} else if (shiftedRow == true){
			shiftedRow = false;
		}

		zBeadPos	+= zShift;									// Z increment
		xBeadPos	= 0 + (shiftedRow==false)?(0):(zXFit);		// bringing back to zero (or shifted)
		yBeadPos	= 0 + (shiftedRow==false)?(0):(zYFit);		// bringing back to zero (or shifted)
	}
}

#endif