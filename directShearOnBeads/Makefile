#Compiler setting
CC=				g++
CFLAGS=			-c -Wall
GLFLAGS=		-lGL -lGLU -lglut
SDLFLAGS=		-lSDL -lSL_image
TFLAGS=			-std=c++0x 
#File name setting
CODE=			directShearTest.cpp
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
CONSOLIDATION= 	consolidation.txt
SHEARING=		shearing.txt
OUTPUTLOG_BEADPOS=		beadsPositions
OUTPUTLOG_CONTACTDATA=	contactData
OUTPUTLOG_POLARFORCEDATA= polarForceData

python = python
liveGraphStrainControlled = ../.internalLibs/liveGraph_directShearOnBead-strainControlled.py
liveGraphStressControlled = ../.internalLibs/liveGraph_directShearOnBead-stressControlled.py



bullet3: $(CODE) spring.o 
	@ mkdir $(OUTPUTLOG_BEADPOS) $(OUTPUTLOG_CONTACTDATA) $(OUTPUTLOG_POLARFORCEDATA) || true && $(CC) $(CODE) spring.o $(TFLAGS) -I $(B3SOURCE) $(B3D) $(B3C) $(B3LM) -o $(OUTPUT)

# bullet2: $(CODE)
# 	@ $(CC) $(CODE) $(GLFLAGS) -I $(B2SOURCE) $(B2D) $(B2C) $(B2LM) -o $(OUTPUT)

liveRun-strainControlled: $(OUTPUT)
	@  $(python) $(liveGraphStrainControlled) & $(EXECUTE)

liveRun-stressControlled: $(OUTPUT)
	@  $(python) $(liveGraphStressControlled) & $(EXECUTE)

run: $(OUTPUT)
	@  $(EXECUTE)

spring.o: ../.internalLibs/btGeneric6DofSpring2Constraint.cpp
	@ $(CC) $(CFLAGS) ../.internalLibs/btGeneric6DofSpring2Constraint.cpp -I $(B3SOURCE) -o spring.o

clean: 
	@ rm *.out *.o *.txt || true && rm -r $(OUTPUTLOG_BEADPOS)/ $(OUTPUTLOG_CONTACTDATA)/ $(OUTPUTLOG_POLARFORCEDATA)/

consoilidation: $(OUTPUT) $(CONSOLIDATION)
	@ libreoffice --calc $(CONSOLIDATION)

shearing: $(OUTPUT) $(SHEARING)
	@ libreoffice --calc $(SHEARING) 

profile: $(OUTPUT)
	@ valgrind $(EXECUTE)