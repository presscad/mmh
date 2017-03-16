import psyco; psyco.full()
from fltk import *
import time

import sys
sys.path.append('../PyCommon/modules')
import Motion.ysMotion as ym
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv
import Mesh.ysMeshUtil as ysu
import Resource.ysOgreDataLoader as yol
import Util.ysGlHelper as ygh
import Math.mmMath as mmMath

import csSpringModel as csm
import csSpringModelRenderer as csr


def main(): 
#    mesh = yol.readOgreMeshFileAsMesh('Data/box_rotate.mesh.xml', .1)
#    mesh = yol.readOgreMeshFileAsMesh('Data/box.mesh.xml', .1)
#    mesh = yol.readOgreMeshFileAsMesh('Data/foot_box.mesh.xml', .1)
#    mesh = yol.readOgreMeshFileAsMesh('Data/woody2_4.mesh.xml', .01)

    mesh, motions= yol.readOgreDataFiles('Data/woody2_7.mesh.xml', .01, None)
    
    ysu.mergePoints(mesh)
    print 'MESH'
    print mesh
    
    motion, frameTime = yf.readBvhFileAsJointMotion('Data/wd2_WalkSameSame00.bvh', .01)
    motion = motion[:100]
    mesh.update(motion[0])

    config = {}
    config['meshKs'] = 10000.
    config['meshKd'] = 500.
    config['muscleKs'] = 10000.
    config['muscleKd'] = 500.
    config['footKs'] = 10000.
    config['footKd'] = 500.
#    config['2D'] = True
    config['vertexMasses'] = [1.]*len(mesh.vertices)
    config['mu'] = 1.
    model = csm.SpringModel3D(mesh, config)
    print 'MODEL'
    print model
    print
    
    viewer = ysv.SimpleViewer()

    viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))
    viewer.doc.addObject('mesh', mesh)

    viewer.doc.addRenderer('model', csr.SpringModel3DRenderer(model, (255,255,255), True, True))
    viewer.setRecSimulObjs([model])

    viewer.setMaxFrame(len(motion)-1)
    
    frameTime = 1./30.
    stepsPerFrame = 10
    timeStep = frameTime / stepsPerFrame
    print 'SIMULATION'
    print 'timeStep', timeStep
    print


    # with rendering
    pt = [0]
    def preFrameCallback(frame):
        if frame == 0:
            pt[0] = time.time()
            
        mesh.update(motion[frame])
        
    def simulateCallback(frame):
        print frame
        model.updateSprings(mesh)
        for i in range(int(stepsPerFrame)):
            model.step(timeStep)
            
        print frame, model.getUpperCOMPos()[1]
        
        if frame == len(motion)-1:
            print 'elapsed time:', time.time()-pt[0]
            
    viewer.setPreFrameCallback(preFrameCallback)
    viewer.setSimulateCallback(simulateCallback)
    
    
#    # without rendering
#    pt = time.time()
#    for frame in range(len(motion)):
#        motion.frame = frame
#        mesh.update(motion[frame])
#        model.updateSprings(mesh)
#        for step in range(int(stepsPerFrame)):
#            model.step(timeStep)
#    print 'elapsed time:', time.time()-pt
    
    
    viewer.startTimer(frameTime)
    viewer.show()
    
    Fl.run()
    