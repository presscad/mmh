# ys2010

How to run
==================


in linux,
-------------------

1. sudo apt-get install python-dev python-pip python-numpy libgle3 freeglut3-dev libfreetype6-dev libpng12-dev libboost-dev libboost-python-dev libfltk1.3-dev python-fltk libode-dev 
2. pip install pyopengl pyopengl-accelerate pyode matplotlib
3. build implicitMassSpringSolver2 and VirtualPhysics2010 in PyCommon/external_libraries( just make )
4. copy .a files to PyCommon/modules/usr/lib (if path doesn't exist, make dir)
5. "python setup.py build"  in PyCommon/modules
6. run main_PreprocessBvh.py, main_PreprocessMcfg.py, and main_PreprocessSeg.py (adjust the files on your case)
7. python main_TrackingExamples2.py
