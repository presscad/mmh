# Compiler flags...
CPP_COMPILER = g++
C_COMPILER = gcc

# Include paths...
Release_Include_Path=-I"../../usr/include" 

# Library paths...
Release_Library_Path=

# Additional libraries...
Release_Libraries=-Wl,--start-group -lkernel32 -luser32 -lgdi32 -lwinspool -lcomdlg32 -ladvapi32 -lshell32 -lole32 -loleaut32 -luuid -lodbc32 -lodbccp32 -l -lvpLib -lvpRenderer -lfreeglut  -Wl,--end-group

# Preprocessor definitions...
Release_Preprocessor_Definitions=-D GCC_BUILD -D NDEBUG -D _CONSOLE 

# Implictly linked object files...
Release_Implicitly_Linked_Objects=

# Compiler flags...
Release_Compiler_Flags=-O2 

# Builds all configurations for this project...
.PHONY: build_all_configurations
build_all_configurations: Release 

# Builds the Release configuration...
.PHONY: Release
Release: create_folders tmp/Win32/gccRelease/main.o tmp/Win32/gccRelease/test.o 
	g++ tmp/Win32/gccRelease/main.o tmp/Win32/gccRelease/test.o  $(Release_Library_Path) $(Release_Libraries) -Wl,-rpath,./ -o ../exe/Win32/gccRelease/tutorial.exe

# Compiles file main.cpp for the Release configuration...
-include tmp/Win32/gccRelease/main.d
tmp/Win32/gccRelease/main.o: main.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c main.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/main.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM main.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/main.d

# Compiles file test.cpp for the Release configuration...
-include tmp/Win32/gccRelease/test.d
tmp/Win32/gccRelease/test.o: test.cpp
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -c test.cpp $(Release_Include_Path) -o tmp/Win32/gccRelease/test.o
	$(CPP_COMPILER) $(Release_Preprocessor_Definitions) $(Release_Compiler_Flags) -MM test.cpp $(Release_Include_Path) > tmp/Win32/gccRelease/test.d

# Creates the intermediate and output folders for each configuration...
.PHONY: create_folders
create_folders:
	mkdir -p tmp/Win32/gccRelease/source
	mkdir -p ../exe/Win32/gccRelease

# Cleans intermediate and output files (objects, libraries, executables)...
.PHONY: clean
clean:
	rm -f tmp/Win32/gccRelease/*.o
	rm -f tmp/Win32/gccRelease/*.d
	rm -f ../exe/Win32/gccRelease/*.a
	rm -f ../exe/Win32/gccRelease/*.so
	rm -f ../exe/Win32/gccRelease/*.dll
	rm -f ../exe/Win32/gccRelease/*.exe
