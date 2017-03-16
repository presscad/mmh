import psyco; psyco.full()
from fltk import *
import copy, time, math
import numpy as np

import sys
sys.path.append('..')
import Math.mmMath as mmMath
import GUI.ysSimpleViewer as ysv
import Util.ysGlHelper as ygh
import Resource.ysOgreDataLoader as yol
import Mesh.ysMeshUtil as ysu
import Resource.ysMotionLoader as yf
import Motion.ysMotionUtil as ymu
import Renderer.ysRenderer as yr

import csMetric as cmt
#import csSpringModel as csm
#import csSpringModelRenderer as csr

def test_calcPointCloudMetric():
    pointsA = [(1,0,0), (0,0,0), (0,0,1)]
    R = mmMath.exp(mmMath.v3(0,1,0), math.pi/4)
    pointsB = []
    for p in pointsA:
        pointsB.append(np.dot(R, p)+(0,1,0))
    
    T = mmMath.I_SE3()
    distance = cmt.calcPointCloudMetric(pointsA, pointsB, T)
    print distance
    print T
    
    pointsC = []
    for p in pointsB:
        pointsC.append(mmMath.T2p(np.dot(T, mmMath.p2T(p))))
    
    viewer = ysv.SimpleViewer()
    def extraDrawCallback():
        for p in pointsA:
            ygh.drawPoint(p, (255,0,0))
        for p in pointsB:
            ygh.drawPoint(p, (0,255,0))
        for p in pointsC:
            ygh.drawPoint(p, (0,255,255))
    viewer.setExtraDrawCallback(extraDrawCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()

#def test_calcPointCloudMetric_simul():    
#
#    massMap = {}
#    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
#    
#    # torso : 10
#    massMap['Hips'] += 2.
#    massMap['Spine'] += 6.
#    massMap['Spine1'] += 2.
#    
#    # head : 3
#    massMap['Spine1'] += 1.
#    massMap['Head'] += 2.
#    
#    # right upper arm : 2
#    massMap['Spine1'] += .5
#    massMap['RightShoulder'] += .5
#    massMap['RightArm'] += 1.
#    
#    # left upper arm : 2
#    massMap['Spine1'] += .5
#    massMap['LeftShoulder1'] += .5
#    massMap['LeftArm'] += 1.
#    
#    # right lower arm : 1
#    massMap['RightForeArm'] = .8
#    massMap['RightHand'] = .2 
#    
#    # left lower arm : 1
#    massMap['LeftForeArm'] = .8
#    massMap['LeftHand'] = .2 
#    
#    # right thigh : 7
#    massMap['Hips'] += 2.
#    massMap['RightUpLeg'] += 5.
#    
#    # left thigh : 7
#    massMap['Hips'] += 2.
#    massMap['LeftUpLeg'] += 5.
#    
#    # right shin : 5
#    massMap['RightLeg'] += 5.
#    
#    # left shin : 5
#    massMap['LeftLeg'] += 5.
#    
#    # right foot : 4
#    massMap['RightFoot'] += 2.
#    massMap['RightToes'] += 2.
#    
#    # left foot : 4
#    massMap['LeftFoot'] += 2.
#    massMap['LeftToes'] += 2.
#    
#    mesh, temp_motions= yol.readOgreDataFiles('../samples/woody2_15.mesh.xml', .01, None)
#    motion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)
#    frameTime = 1/30.
#    stepsPerFrame = 10
#    timeStep = frameTime / stepsPerFrame
#    print 'timeStep', timeStep
#    
#    for p in motion:
#        p.updateGlobalT()
##    motion = motion[:100]
#    mesh.update(motion[0])
#    
#    transformedMotion = copy.deepcopy(motion)    
#
#    config = {}
#    config['meshKs'] = 20000.
#    config['meshKd'] = 500.
#    config['muscleKs'] = 20000.
#    config['muscleKd'] = 500.
#    config['footKs'] = 20000.
#    config['footKd'] = 500.
#    vertexMasses = ysu.getDistributedVertexMasses(mesh, massMap)
#    config['vertexMasses'] = [1.]*len(mesh.vertices)
#    config['mu'] = 200.
#    model = csm.SpringModel3D(mesh, config)
#    
#    viewer = ysv.SimpleViewer()
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (102, 153, 255), yr.LINK_BONE))
#    viewer.doc.addObject('motion', motion)
#    viewer.doc.addRenderer('transformedMotion', yr.JointMotionRenderer(transformedMotion, (0,255,0), yr.LINK_BONE))
#    viewer.doc.addObject('transformedMotion', transformedMotion)
#    viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))
#    viewer.doc.addObject('mesh', mesh)
#    viewer.doc.addRenderer('model', csr.SpringModel3DRenderer(model, (200,200,200), False, False))
##    viewer.setRecSimulObjs([model])
#    viewer.setMaxFrame(len(motion)-1)
#    
#    def preFrameCallback(frame):
#        mesh.update(motion[frame])
##        T = mmMath.I_SE3()
##        distance = cmt.calcPointCloudMetric(model.getPositions(), [v.pos for v in mesh.vertices], T)
##        transformedMotion[frame].mulT(T)
#        
#    def postFrameCallback(frame):
#        transfT = mmMath.I_SE3()
#        distance = cmt.calcPointCloudMetric(model.getPositions(), [v.pos for v in mesh.vertices], transfT)
#        transformedMotion[frame] = ymu.getTransformedJointPosture(transfT, transformedMotion[frame])
#        
#    def simulateCallback(frame):
#        model.updateSprings(mesh)
#        for i in range(int(stepsPerFrame)):
#            model.step(timeStep)
#            
#    viewer.setPreFrameCallback(preFrameCallback)
#    viewer.setSimulateCallback(simulateCallback)
#    viewer.setPostFrameCallback(postFrameCallback)
#    
#    viewer.startTimer(frameTime)
#    viewer.show()
#    
#    Fl.run()
        
    
    
if __name__=='__main__':
    test_calcPointCloudMetric()
#    test_calcPointCloudMetric_simul()