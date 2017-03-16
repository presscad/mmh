# Compiler flags...
CPP_COMPILER = g++
C_COMPILER = gcc

# Include paths...
Release_Include_Path=-I"../usr/include" 

# Library paths...
Release_Library_Path=

# Additional libraries...
Release_Libraries=

# Preprocessor definitions...
Release_Preprocessor_Definitions=-D GCC_BUILD -D NDEBUG -D _LIB 

# Implictly linked object files...
Release_Implicitly_Linked_Objects=

# Compiler flags...
Release_Compiler_Flags=-O2 

# Builds all configurations for this project...
.PHONY: build_all_configurations
build_all_configurations: Release 

# Builds the Release configuration...
.PHONY: Release
Release: create_folders tmp/Win32/gccRelease/GLee.o tmp/Win32/gccRelease/GLFramework.o tmp/Win32/gccRelease/vpRenderer.o 
	ar rcs ../usr/lib/Win32/gccRelease/libvpRenderer.a tmp/Win32/gccRelease/GLee.o tmp/Win32/gccRelease/GLFramework.o tmp/Win32/gccRelease/vpRenderer.o  $(Release_Implicitly_Linked_Objects)

# Compiles file GLee.c for the Release configuration...
-include tmp/Win32/gccRelease/GLee.d
tmp/Win32/gccRelease/GLee.o: GLee.c
	$(C_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c GLee.c $(Release_Include_Path) -o tmp/Win32/gccRelease/GLee.o
	$(C_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM GLee.c $(Release_Include_Path) > tmp/Win32/gccRelease/GLee.d

# Compiles file GLFramework.cpp for the Release configuration...
-include tmp/Win32/gccRelease/GLFramework.d
tmp/Win32/gccRelease/GLFramework.o: GLFramework.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c GLFramework.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/GLFramework.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM GLFramework.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/GLFramework.d

# Compiles file vpRenderer.cpp for the Release configuration...
-include tmp/Win32/gccRelease/vpRenderer.d
tmp/Win32/gccRelease/vpRenderer.o: vpRenderer.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c vpRenderer.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/vpRenderer.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM vpRenderer.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/vpRenderer.d

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
