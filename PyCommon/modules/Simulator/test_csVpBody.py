import psyco; psyco.full()
from fltk import *
import copy, time, math, cPickle, random
import numpy as np

import sys
sys.path.append('..')
import GUI.ysSimpleViewer as ysv
import Resource.ysOgreDataLoader as yol
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import Util.ysPythonEx as ype
import Motion.ysSkeletonEdit as yme
import Math.mmMath as mm
import Simulator.csVpModel as cvm
import Simulator.csVpWorld as cvw
import Simulator.csVpBody as cvb
import Simulator.ysPhysConfig as ypc
import Renderer.csVpRenderer as cvr
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysControl as yct

def test_default_contact():
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    vpWorld = cvw.VpWorld(wcfg)

    body = cvb.VpBody(vpWorld)
    body.addBoxGeom((1,1,1), 1)
    
    vpWorld.initialize()
    
    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('box', yr.VpBodyRenderer(body, (255,0,0)))
    viewer.setMaxFrame(100)

    def simulateCallback(frame):
        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer((1/30.)*.4)
    viewer.show()
    Fl.run()
    
def test_penalty_contact():
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    wcfg.useDefaultContactModel = False
    vpWorld = cvw.VpWorld(wcfg)

    body = cvb.VpBody(vpWorld)
    body.addBoxGeom((1,.1,1), 1)
    body.setFrame(mm.p2T((0,.1,0)))
    
    vpWorld.initialize()
    
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1.]*len(bodyIDsToCheck)
    Ks = 1000; Ds = 2*(Ks**.5)
    
    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('box', yr.VpBodyRenderer(body, (255,0,0)))
    viewer.setMaxFrame(100)

    def simulateCallback(frame):
        for i in range(stepsPerFrame):
            bodyIDs, positions, positionLocals, forces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, positionLocals, forces)
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer((1/30.)*.4)
    viewer.show()
    Fl.run()

def test_stack():
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = -1.
    stepsPerFrame = 30
    wcfg.timeStep = (1/30.)/stepsPerFrame
    vpWorld = cvw.VpWorld(wcfg)

    boxNum = 10
    bodies = [None]*boxNum
    for i in range(boxNum):
        bodies[i] = cvb.VpBody(vpWorld)
        bodies[i].addBoxGeom((1,1,1), 1)
        bodies[i].setPosition((10,i,11))
        bodies[i].setOrientation(mm.rotY(10*mm.RAD*random.random()))
#        bodies[i].setFrame(mm.Rp2T( mm.rotY(10*mm.RAD*random.random()), (10,i,11)))
    
    bodies2 = [None]*(boxNum*boxNum)
    count = 0
    for i in range(boxNum):
        for j in range(boxNum):
            bodies2[count] = cvb.VpBody(vpWorld)
            bodies2[count].addBoxGeom((1,1,1), 1)
            bodies2[count].setPosition((.1*random.random(),i,j))
#            bodies2[count].setOrientation(mm.rotY(10*mm.RAD*random.random()))
            count += 1
    
    vpWorld.initialize()
    
    viewer = ysv.SimpleViewer()
    viewer.doc.addRenderer('box', yr.VpBodiesRenderer(bodies, (255,0,0)))
    viewer.doc.addRenderer('box2', yr.VpBodiesRenderer(bodies2, (0,0,255)))
    viewer.setMaxFrame(100)

    def simulateCallback(frame):
        for i in range(stepsPerFrame):
            vpWorld.step()
    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer((1/30.)*.4)
    viewer.show()
    Fl.run()
    
pass
#test_default_contact()
#test_penalty_contact()
test_stack()