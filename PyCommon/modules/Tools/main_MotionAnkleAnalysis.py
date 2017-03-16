import psyco; psyco.full()
from fltk import *
import numpy as np
import operator as op

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Motion.ysMotion as ym
import Resource.ysMotionLoader as yf
import Motion.ysMotionAnalysis as yma
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv
import Util.ysGlHelper as ygh
import Util.ysMatplotEx as ymp

if __name__ == "__main__":
    bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
#    bvhFilePath = 'samples/wd2_2foot_walk_turn2.bvh'
#    bvhFilePath = 'samples/wd2_WalkSukiko00.bvh'
    motion = yf.readBvhFile(bvhFilePath, .01)

    LFOOT = motion[0].skeleton.getElementIndex('LeftFoot')
    RFOOT = motion[0].skeleton.getElementIndex('RightFoot')
    LTOE = motion[0].skeleton.getElementIndex('LeftToes')
    RTOE = motion[0].skeleton.getElementIndex('RightToes')
    
    hRef = .1; vRef = .5
    clHeel = yma.getElementContactStates(motion, LFOOT, hRef, vRef)
    crHeel = yma.getElementContactStates(motion, RFOOT, hRef, vRef)
    clToe = yma.getElementContactStates(motion, LTOE, hRef, vRef)
    crToe = yma.getElementContactStates(motion, RTOE, hRef, vRef)
    clFoot = map(op.or_, clHeel, clToe)
    crFoot = map(op.or_, crHeel, crToe)
    
    lFootStates = yma.getFootStates(clHeel, clToe)
    rFootStates = yma.getFootStates(crHeel, crToe)
    lIntervals, lStates = yma.states2intervals(lFootStates)
    rIntervals, rStates = yma.states2intervals(rFootStates)
    
    footNames = [LFOOT, RFOOT]
    horizontalDirection = mm.v3(-1,0,0)
    horizontalRight = np.cross(horizontalDirection, (0,1,0))
    horizontalLeft = -horizontalRight
    
    sagittalAxis = horizontalRight
    lateralAxisMap = {LFOOT:horizontalDirection, RFOOT:-horizontalDirection}
    
    size_sagittals_map = {LFOOT:[None]*len(motion), RFOOT:[None]*len(motion)}
    size_laterals_map = {LFOOT:[None]*len(motion), RFOOT:[None]*len(motion)}
    for i in range(len(motion)):
        for footName in footNames:
            Rg = motion[i].getGlobalRFromParent(footName)
            logRg = mm.logSO3(Rg)
            logRg_sagittal = mm.projectionOnVector(logRg, sagittalAxis)
            logRg_lateral = mm.projectionOnVector(logRg, lateralAxisMap[footName])
            size_sagittals_map[footName][i] = mm.componentOnVector(logRg, sagittalAxis)
            size_laterals_map[footName][i] = mm.componentOnVector(logRg, lateralAxisMap[footName])

    lPeakFrames, lPeakTypes = yma.getFootPeakFramesAndTypes(size_sagittals_map[LFOOT], lFootStates, 5)
    rPeakFrames, rPeakTypes = yma.getFootPeakFramesAndTypes(size_sagittals_map[RFOOT], rFootStates, 5)
            

    showL = True
    showR = True
    plot = ymp.SmartPlot()
    plot.setXdata('frame', range(len(motion)))
#    plot.addYdata('lFootStates', lFootStates, False)
#    plot.addYdata('rFootStates', rFootStates, False)

    if showL:
        plot.addXspans('lSTANCE', [lIntervals[i] for i in range(len(lStates)) if lStates[i]==yma.FootState.STANCE], False)
        plot.addXspans('lTAKING', [lIntervals[i] for i in range(len(lStates)) if lStates[i]==yma.FootState.TAKING], False)
        plot.addXspans('lSWING', [lIntervals[i] for i in range(len(lStates)) if lStates[i]==yma.FootState.SWING], False)
        plot.addXspans('lLANDING', [lIntervals[i] for i in range(len(lStates)) if lStates[i]==yma.FootState.LANDING], False)
        plot.addYdata('lFoot_sag', size_sagittals_map[LFOOT], False)
        plot.addYdata('lFoot_lat', size_laterals_map[LFOOT], False)
        plot.addXlines('lPeakFrames', lPeakFrames, False)
        
    if showR:
        plot.addXspans('rSTANCE', [rIntervals[i] for i in range(len(rStates)) if rStates[i]==yma.FootState.STANCE], False)
        plot.addXspans('rTAKING', [rIntervals[i] for i in range(len(rStates)) if rStates[i]==yma.FootState.TAKING], False)
        plot.addXspans('rSWING', [rIntervals[i] for i in range(len(rStates)) if rStates[i]==yma.FootState.SWING], False)
        plot.addXspans('rLANDING', [rIntervals[i] for i in range(len(rStates)) if rStates[i]==yma.FootState.LANDING], False)
        plot.addYdata('rFoot_sag', size_sagittals_map[RFOOT], False)
        plot.addYdata('rFoot_lat', size_laterals_map[RFOOT], False)
        plot.addXlines('rPeakFrames', rPeakFrames, False)

    plot.showModeless()
        
    contactStatesMap = yma.getMotionContactStates(motion, hRef, vRef)
    
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (100,100,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('tpose', yr.JointMotionRenderer(ym.Motion([motion[0].getTPose()]), (255,100,100), yr.LINK_BONE), False)
    viewer.doc.addObject('tpose', motion)
    

    def viewer_onClose(data):
        plot.close()
        viewer.onClose(data)
    viewer.callback(viewer_onClose)

    def extraDraw():
        frame = viewer.getCurrentFrame()
        
        for footName in footNames:
#            if footName == RFOOT:
            if footName == LFOOT:
#            if True:
                footPos = motion[frame].getPosition(footName)
                Rg = motion[frame].getGlobalRFromParent(footName)
    #            ygh.drawSO3(Rg, footPos, (0,255,0))
                
                logRg = mm.logSO3(Rg)
                logRg_sagittal = mm.projectionOnVector(logRg, sagittalAxis)
                logRg_lateral = mm.projectionOnVector(logRg, lateralAxisMap[footName])
                ygh.drawVector(logRg_sagittal, footPos, (255,255,0))
                ygh.drawVector(logRg_lateral, footPos, (0,255,255))
        
        for index, contactStates in contactStatesMap.items():
            if contactStates[frame]:
                ygh.drawPoint(motion.getPosition(index, frame), (255,255,255), 5.)
                
    viewer.setExtraDrawCallback(extraDraw)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()