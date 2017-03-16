UNAME := $(shell uname -s)
ifeq ($(UNAME), Darwin)
	MAC_OMP := $(shell clang-omp++ --version 2>/dev/null)
ifdef MAC_OMP
	CPP_COMPILER = clang-omp++
	C_COMPILER = clang-omp++
	PREPROCESSOR = -D __APPLE_OMP__ -fopenmp
else
	CPP_COMPILER = clang++
	C_COMPILER = clang++
endif
endif

ifeq ($(UNAME), Linux)
	CPP_COMPILER = g++
	C_COMPILER = gcc
	PREPROCESSOR = -fopenmp
endif

# Compiler flags...
#for Linux
#for MAC

# Include paths...
Release_Include_Path=-I"../usr/include" 

# Library paths...
Release_Library_Path=

# Additional libraries...
Release_Libraries=

# Preprocessor definitions...
Release_Preprocessor_Definitions=-D GCC_BUILD -D NDEBUG -D _LIB $(PREPROCESSOR)

# Implictly linked object files...
Release_Implicitly_Linked_Objects=

# Compiler flags...
Release_Compiler_Flags=-O2 -fPIC

# Builds all configurations for this project...
.PHONY: build_all_configurations
build_all_configurations: Release 

# Builds the Release configuration...
.PHONY: Release
Release: create_folders tmp/Win32/gccRelease/LieGroup.o tmp/Win32/gccRelease/PrimColDet.o tmp/Win32/gccRelease/rmatrix3.o tmp/Win32/gccRelease/vp1DOFJoint.o tmp/Win32/gccRelease/vpBJoint.o tmp/Win32/gccRelease/vpBody.o tmp/Win32/gccRelease/vpCollisionDetector.o tmp/Win32/gccRelease/vpCollisionGraphBuilder.o tmp/Win32/gccRelease/vpCollisionResolver.o tmp/Win32/gccRelease/vpContactResolver.o tmp/Win32/gccRelease/vpDynamics.o tmp/Win32/gccRelease/vpGeom.o tmp/Win32/gccRelease/vpJoint.o tmp/Win32/gccRelease/vpKinematics.o tmp/Win32/gccRelease/vpMaterial.o tmp/Win32/gccRelease/vpNDOFJoint.o tmp/Win32/gccRelease/vpPenetrationRelaxer.o tmp/Win32/gccRelease/vpPJoint.o tmp/Win32/gccRelease/vpPrimitiveCollisionDetector.o tmp/Win32/gccRelease/vpRJoint.o tmp/Win32/gccRelease/vpSingleSystem.o tmp/Win32/gccRelease/vpSJoint.o tmp/Win32/gccRelease/vpSpring.o tmp/Win32/gccRelease/vpSystem.o tmp/Win32/gccRelease/vpTimer.o tmp/Win32/gccRelease/vpUJoint.o tmp/Win32/gccRelease/vpWJoint.o tmp/Win32/gccRelease/vpWorld.o tmp/Win32/gccRelease/vpWorld_IO.o 
	ar rcs ../usr/lib/Win32/gccRelease/libvpLib.a tmp/Win32/gccRelease/LieGroup.o tmp/Win32/gccRelease/PrimColDet.o tmp/Win32/gccRelease/rmatrix3.o tmp/Win32/gccRelease/vp1DOFJoint.o tmp/Win32/gccRelease/vpBJoint.o tmp/Win32/gccRelease/vpBody.o tmp/Win32/gccRelease/vpCollisionDetector.o tmp/Win32/gccRelease/vpCollisionGraphBuilder.o tmp/Win32/gccRelease/vpCollisionResolver.o tmp/Win32/gccRelease/vpContactResolver.o tmp/Win32/gccRelease/vpDynamics.o tmp/Win32/gccRelease/vpGeom.o tmp/Win32/gccRelease/vpJoint.o tmp/Win32/gccRelease/vpKinematics.o tmp/Win32/gccRelease/vpMaterial.o tmp/Win32/gccRelease/vpNDOFJoint.o tmp/Win32/gccRelease/vpPenetrationRelaxer.o tmp/Win32/gccRelease/vpPJoint.o tmp/Win32/gccRelease/vpPrimitiveCollisionDetector.o tmp/Win32/gccRelease/vpRJoint.o tmp/Win32/gccRelease/vpSingleSystem.o tmp/Win32/gccRelease/vpSJoint.o tmp/Win32/gccRelease/vpSpring.o tmp/Win32/gccRelease/vpSystem.o tmp/Win32/gccRelease/vpTimer.o tmp/Win32/gccRelease/vpUJoint.o tmp/Win32/gccRelease/vpWJoint.o tmp/Win32/gccRelease/vpWorld.o tmp/Win32/gccRelease/vpWorld_IO.o  $(Release_Implicitly_Linked_Objects)

# Compiles file LieGroup.cpp for the Release configuration...
-include tmp/Win32/gccRelease/LieGroup.d
tmp/Win32/gccRelease/LieGroup.o: LieGroup.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c LieGroup.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/LieGroup.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM LieGroup.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/LieGroup.d

# Compiles file PrimColDet.cpp for the Release configuration...
-include tmp/Win32/gccRelease/PrimColDet.d
tmp/Win32/gccRelease/PrimColDet.o: PrimColDet.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c PrimColDet.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/PrimColDet.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM PrimColDet.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/PrimColDet.d

# Compiles file rmatrix3.cpp for the Release configuration...
-include tmp/Win32/gccRelease/rmatrix3.d
tmp/Win32/gccRelease/rmatrix3.o: rmatrix3.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c rmatrix3.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/rmatrix3.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM rmatrix3.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/rmatrix3.d

# Compiles file vp1DOFJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vp1DOFJoint.d
tmp/Win32/gccRelease/vp1DOFJoint.o: vp1DOFJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vp1DOFJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vp1DOFJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vp1DOFJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vp1DOFJoint.d

# Compiles file vpBJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpBJoint.d
tmp/Win32/gccRelease/vpBJoint.o: vpBJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpBJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpBJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpBJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpBJoint.d

# Compiles file vpBody.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpBody.d
tmp/Win32/gccRelease/vpBody.o: vpBody.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpBody.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpBody.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpBody.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpBody.d

# Compiles file vpCollisionDetector.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpCollisionDetector.d
tmp/Win32/gccRelease/vpCollisionDetector.o: vpCollisionDetector.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpCollisionDetector.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpCollisionDetector.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpCollisionDetector.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpCollisionDetector.d

# Compiles file vpCollisionGraphBuilder.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpCollisionGraphBuilder.d
tmp/Win32/gccRelease/vpCollisionGraphBuilder.o: vpCollisionGraphBuilder.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpCollisionGraphBuilder.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpCollisionGraphBuilder.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpCollisionGraphBuilder.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpCollisionGraphBuilder.d

# Compiles file vpCollisionResolver.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpCollisionResolver.d
tmp/Win32/gccRelease/vpCollisionResolver.o: vpCollisionResolver.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpCollisionResolver.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpCollisionResolver.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpCollisionResolver.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpCollisionResolver.d

# Compiles file vpContactResolver.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpContactResolver.d
tmp/Win32/gccRelease/vpContactResolver.o: vpContactResolver.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpContactResolver.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpContactResolver.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpContactResolver.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpContactResolver.d

# Compiles file vpDynamics.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpDynamics.d
tmp/Win32/gccRelease/vpDynamics.o: vpDynamics.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpDynamics.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpDynamics.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpDynamics.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpDynamics.d

# Compiles file vpGeom.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpGeom.d
tmp/Win32/gccRelease/vpGeom.o: vpGeom.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpGeom.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpGeom.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpGeom.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpGeom.d

# Compiles file vpJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpJoint.d
tmp/Win32/gccRelease/vpJoint.o: vpJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpJoint.d

# Compiles file vpKinematics.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpKinematics.d
tmp/Win32/gccRelease/vpKinematics.o: vpKinematics.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpKinematics.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpKinematics.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpKinematics.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpKinematics.d

# Compiles file vpMaterial.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpMaterial.d
tmp/Win32/gccRelease/vpMaterial.o: vpMaterial.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpMaterial.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpMaterial.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpMaterial.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpMaterial.d

# Compiles file vpNDOFJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpNDOFJoint.d
tmp/Win32/gccRelease/vpNDOFJoint.o: vpNDOFJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpNDOFJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpNDOFJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpNDOFJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpNDOFJoint.d

# Compiles file vpPenetrationRelaxer.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpPenetrationRelaxer.d
tmp/Win32/gccRelease/vpPenetrationRelaxer.o: vpPenetrationRelaxer.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpPenetrationRelaxer.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpPenetrationRelaxer.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpPenetrationRelaxer.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpPenetrationRelaxer.d

# Compiles file vpPJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpPJoint.d
tmp/Win32/gccRelease/vpPJoint.o: vpPJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpPJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpPJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpPJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpPJoint.d

# Compiles file vpPrimitiveCollisionDetector.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpPrimitiveCollisionDetector.d
tmp/Win32/gccRelease/vpPrimitiveCollisionDetector.o: vpPrimitiveCollisionDetector.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpPrimitiveCollisionDetector.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpPrimitiveCollisionDetector.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpPrimitiveCollisionDetector.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpPrimitiveCollisionDetector.d

# Compiles file vpRJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpRJoint.d
tmp/Win32/gccRelease/vpRJoint.o: vpRJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpRJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpRJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpRJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpRJoint.d

# Compiles file vpSingleSystem.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpSingleSystem.d
tmp/Win32/gccRelease/vpSingleSystem.o: vpSingleSystem.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpSingleSystem.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpSingleSystem.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpSingleSystem.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpSingleSystem.d

# Compiles file vpSJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpSJoint.d
tmp/Win32/gccRelease/vpSJoint.o: vpSJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpSJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpSJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpSJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpSJoint.d

# Compiles file vpSpring.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpSpring.d
tmp/Win32/gccRelease/vpSpring.o: vpSpring.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpSpring.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpSpring.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpSpring.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpSpring.d

# Compiles file vpSystem.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpSystem.d
tmp/Win32/gccRelease/vpSystem.o: vpSystem.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpSystem.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpSystem.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpSystem.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpSystem.d

# Compiles file vpTimer.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpTimer.d
tmp/Win32/gccRelease/vpTimer.o: vpTimer.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpTimer.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpTimer.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpTimer.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpTimer.d

# Compiles file vpUJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpUJoint.d
tmp/Win32/gccRelease/vpUJoint.o: vpUJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpUJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpUJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpUJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpUJoint.d

# Compiles file vpWJoint.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpWJoint.d
tmp/Win32/gccRelease/vpWJoint.o: vpWJoint.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpWJoint.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpWJoint.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpWJoint.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpWJoint.d

# Compiles file vpWorld.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpWorld.d
tmp/Win32/gccRelease/vpWorld.o: vpWorld.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpWorld.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpWorld.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpWorld.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpWorld.d

# Compiles file vpWorld_IO.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpWorld_IO.d
tmp/Win32/gccRelease/vpWorld_IO.o: vpWorld_IO.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpWorld_IO.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpWorld_IO.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpWorld_IO.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpWorld_IO.d

# Creates the intermediate and output folders for each configuration...
.PHONY: create_folders
create_folders:
	mkdir -p tmp/Win32/gccRelease/source
	mkdir -p ../usr/lib/Win32/gccRelease

# Cleans intermediate and output files (objects, libraries, executables)...
.PHONY: clean
clean:
	rm -f tmp/Win32/gccRelease/*.o
	rm -f tmp/Win32/gccRelease/*.d
	rm -f ../usr/lib/Win32/gccRelease/*.a
	rm -f ../usr/lib/Win32/gccRelease/*.so
	rm -f ../usr/lib/Win32/gccRelease/*.dll
	rm -f ../usr/lib/Win32/gccRelease/*.exe
