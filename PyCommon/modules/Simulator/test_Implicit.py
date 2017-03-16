import psyco; psyco.full()
from fltk import *
import copy, time

import sys
sys.path.append('..')
import GUI.ysSimpleViewer as ysv
import Resource.ysOgreDataLoader as yol
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import Util.ysPythonEx as ype

import Implicit.csIMSModel as cmm
import Renderer.csIMSRenderer as cr
import ysIMSUtil as yiu

massMap = {}
def makeMassMap():
    global massMap
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
    
    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 6.
    massMap['Spine1'] += 2.
    
    # head : 3
    massMap['Spine1'] += 1.
    massMap['Head'] += 2.
    
    # right upper arm : 2
    massMap['Spine1'] += .5
    massMap['RightShoulder'] += .5
    massMap['RightArm'] += 1.
    
    # left upper arm : 2
    massMap['Spine1'] += .5
    massMap['LeftShoulder1'] += .5
    massMap['LeftArm'] += 1.
    
    # right lower arm : 1
    massMap['RightForeArm'] = .8
    massMap['RightHand'] = .2 
    
    # left lower arm : 1
    massMap['LeftForeArm'] = .8
    massMap['LeftHand'] = .2 
    
    # right thigh : 7
    massMap['Hips'] += 2.
    massMap['RightUpLeg'] += 5.
    
    # left thigh : 7
    massMap['Hips'] += 2.
    massMap['LeftUpLeg'] += 5.
    
    # right shin : 5
    massMap['RightLeg'] += 5.
    
    # left shin : 5
    massMap['LeftLeg'] += 5.
    
    # right foot : 4
    massMap['RightFoot'] += 2.
    massMap['RightToes'] += 2.
    
    # left foot : 4
    massMap['LeftFoot'] += 2.
    massMap['LeftToes'] += 2.
makeMassMap()

def test_IMSModel_mesh():
    frameTime = 1/30.
    stepsPerFrame = 10
    timeStep = frameTime / stepsPerFrame
    print 'time step', timeStep
    
    mesh, t= yol.readOgreDataFiles('../samples/woody2_15.mesh.xml', .01, None)
    motion, frameTime = yf.readBvhFileAsJointMotion('../samples/wd2_WalkSameSame00.bvh', .01)
    mesh.update(motion[0])
    
    dynamicMu = .5
    staticMu = 1.
    particleConfigs = yiu.getParticleConfigsFromMesh(mesh, massMap, motion[:2], dynamicMu, staticMu)
    
    Ks = 10000.
    Kd = 100.
    springConfigs = yiu.getSpringConfigsFromMesh(mesh, Ks, Kd)
    
    systemConfig = cmm.SystemConfig()
    model = cmm.IMSModel(particleConfigs, springConfigs, systemConfig)

    viewer = ysv.SimpleViewer()
#    viewer.record(False)
    
    viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh, (50,100,150)))
    viewer.doc.addObject('mesh', mesh)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('model', cr.IMSModelRenderer(model))
#    viewer.setRecSimulObjs([model])

    def preFrameCallback(frame):
        mesh.update(motion[frame])
    
    def simulateCallback(frame):
        model.updateSprings(yiu.getSpringLengthsFromMesh(mesh, springConfigs))
        for i in range(int(stepsPerFrame)):
            model.step(timeStep)
    
    viewer.setPreFrameCallback(preFrameCallback)
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
        
def test_IMSModel_freefall():
    frameTime = 1/30.
    stepsPerFrame = 10
    timeStep = frameTime / stepsPerFrame
    print 'time step', timeStep
    
    numParticles = 5
    particleConfigs = [cmm.ParticleConfig((0,.5 + i*.5,0), 1., (0,0,0), 1., 1.) for i in range(numParticles)]
    springConfigs = [cmm.SpringConfig(i, i+1, 1000., 100.) for i in range(numParticles-1)]
    systemConfig = cmm.SystemConfig()
    model = cmm.IMSModel(particleConfigs, springConfigs, systemConfig)

    springLengths = [.5]*(numParticles-1)

    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('model', cr.IMSModelRenderer(model, (255,255,255), True))
    viewer.doc.addObject('model', model)
    viewer.setMaxFrame(50)
    
    def simulateCallback(frame):
        print frame
        model.updateSprings(springLengths)
        for i in range(int(stepsPerFrame)):
            model.step(timeStep)
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
    
def test_IMSModel_friction():
    frameTime = 1/30.
    stepsPerFrame = 10
    timeStep = frameTime / stepsPerFrame
    print 'time step', timeStep
    
    numParticles = 2
    
    particleConfigs = [cmm.ParticleConfig((0,.5 + i*.5,0), 1., (1,0,0), 1., 1.) for i in range(numParticles)]
    springConfigs = [cmm.SpringConfig(i, i+1, 1000., 10.) for i in range(numParticles-1)]
    systemConfig = cmm.SystemConfig()
    model = cmm.IMSModel(particleConfigs, springConfigs, systemConfig)
    
#    model.setMu([.5]*numParticles, [1.]*numParticles)
    model.setMu(.5, 1., range(numParticles))
    
#    springLengths = [.5]*(numParticles-1)

    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('model', cr.IMSModelRenderer(model, (255,255,255), True))
    viewer.doc.addObject('model', model)
    viewer.setMaxFrame(50)
    
    def simulateCallback(frame):
        print frame
#        model.updateSprings(springLengths)
        for i in range(int(stepsPerFrame)):
            model.step(timeStep)
            
    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
        
def main():
#    test_IMSModel_mesh()
#    test_IMSModel_freefall()
    test_IMSModel_friction()

main()