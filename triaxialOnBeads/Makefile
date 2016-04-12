#Compiler setting
CC=				g++
CFLAGS=			-c -Wall
GLFLAGS=		-lGL -lGLU -lglut
SDLFLAGS=		-lSDL -lSL_image
TFLAGS=			-std=c++0x 
#File name setting
CODE=			triaxialOnBeads.cpp
#Bullet3 libraries and headers addresses
B3SOURCE=		~/Bullet/bullet3/src/
B3SB=			~/Bullet/bullet3/bin/libBulletSoftBody_gmake_x64_release.a
B3D=			~/Bullet/bullet3/bin/libBulletDynamics_gmake_x64_release.a ~/Bullet/bullet3/bin/libBullet3Dynamics_gmake_x64_release.a
B3C=			~/Bullet/bullet3/bin/libBulletCollision_gmake_x64_release.a ~/Bullet/bullet3/bin/libBullet3Collision_gmake_x64_release.a
B3LM=			~/Bullet/bullet3/bin/libLinearMath_gmake_x64_release.a
B3FILELOADER=	~/Bullet/bullet3/bin/libBullet2FileLoader_gmake_x64_release.a
B3OPENCL=		~/Bullet/bullet3/bin/libBullet3OpenCL_clew_gmake_x64_release.a
#Bullet2 libraries and headers addresses
B2SOURCE=		~/Bullet/bullet-2.82/src/
B2SB=			~/Bullet/bullet-2.82/lib/libBulletSoftBody.a
B2D=			~/Bullet/bullet-2.82/lib/libBulletDynamics.a
B2C=			~/Bullet/bullet-2.82/lib/libBulletCollision.a
B2LM=			~/Bullet/bullet-2.82/lib/libLinearMath.a
#Output setting
OUTPUT=			output.out
EXECUTE=		./$(OUTPUT)
TEXTOUTPUT=		output.txt
OUTPUTLOG_SOFTBODYPOS=		softbodyPosition
OUTPUTLOG_SOFTBODYDEF=		softbodyDeformation
OUTPUTLOG_BEADPOS=		beadsPositions
OUTPUTLOG_CONTACTDATA=	contactData
OUTPUTLOG_POLARFORCEDATA= polarForceData

python = python
liveGraph = ../.internalLibs/liveGraph_triaxialOnBead.py


bullet3: $(CODE)
	@ mkdir $(OUTPUTLOG_SOFTBODYPOS) $(OUTPUTLOG_SOFTBODYDEF) $(OUTPUTLOG_BEADPOS) $(OUTPUTLOG_CONTACTDATA) $(OUTPUTLOG_POLARFORCEDATA) || true && $(CC) $(CODE) -I  $(B3SOURCE) $(B3SB) $(B3D) $(B3C) $(B3LM) $(TFLAGS) -o $(OUTPUT)

liveRun: $(OUTPUT)
	@  $(python) $(liveGraph) & $(EXECUTE)

run: $(OUTPUT)
	@  $(EXECUTE)

clean: 
	@ rm *.out *.o *.txt || true && rm -r $(OUTPUTLOG_SOFTBODYPOS) $(OUTPUTLOG_SOFTBODYDEF) $(OUTPUTLOG_BEADPOS) $(OUTPUTLOG_CONTACTDATA) $(OUTPUTLOG_POLARFORCEDATA)

write: $(OUTPUT)
	@ $(EXECUTE)>$(TEXTOUTPUT) && libreoffice --calc $(TEXTOUTPUT)

profile: $(OUTPUT)
	@ valgrind $(EXECUTE)