import math
from fltk import *

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import Motion.ysMotionExtend as yme
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.ysMotionBlend as ymb
import Motion.dcMotionSeg as dms
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv

class MotionGenerator:

    def __init__(self,motion):
        #self.motion = motion.copy()
        # motion -> motion segmentations
        self.motionSegs, self.segInfos = dms.segmentMotion(motion)
        self.nextSegIdx = [i+1 for i in range(len(self.motionSegs)-1)]

        for i in range(len(self.motionSegs)):
            print self.motionSegs[i]
            print self.segInfos[i]['interval']
        print self.nextSegIdx

        # motion segmentations --> result motion
        self.clearResult()

    def clearResult(self):
        self.resultMotion = []
        self.resultMotionSegIdx = []
        self.resultSegInfos = []
        self.validIdx = -1
        self.curIdx = -1
        # 0, 1, 2, ..., validIdx
        # 0, curIdx, 2, 3, ..., validIdx

    # =====================================
    # source motion segmentation
    # =====================================
    def addMotionSeg(self, motionSeg, segInfo = None):
        if segInfo == None:
            motionSegs, segInfos = dms.segmentMotion(motionSeg)
            motionSeg = motionSegs[0]
            segInfo = segInfos[0]

        self.motionSegs.append(motionSeg.copy())
        self.segInfos.append(segInfo.copy())

    def setMotionSeg(self, segIdx, motionSeg, segInfo = None):
        if segInfo == None:
            motionSegs, segInfos = dms.segmentMotion(motionSeg)
            motionSeg = motionSegs[0]
            segInfo = segInfos[0]

        self.motionSegs[segIdx] = motionSeg.copy()
        self.segInfos[segIdx] = segInfo.copy()

    def setNextSegIdx(self, curSegIdx, nextSegIdx):
        self.nextSegIdx[curSegIdx] = nextSegIdx


    # =======================================
    # result of motion generation
    # =======================================
    def generateNext(self):
        if self.validIdx < 0:
            # initialize
            self.resultMotionSegIdx.append(0)
            self.resultMotion.append(self.motionSegs[0])
            self.resultSegInfos.append(self.segInfos[0])
            self.validIdx = 0
        else:
            motionSegIdx = self.nextSegIdx[self.resultMotionSegIdx[self.validIdx]]
            motionSeg = self.motionSegs[motionSegIdx].copy()
            prevPosture = self.resultMotion[self.validIdx][-1]
            alignedMotionSeg = ymb.getAttachedNextMotion(motionSeg, prevPosture, True, True) 
            segInfo = self.segInfos[motionSegIdx].copy()

            self.resultMotionSegIdx.append(motionSegIdx)
            self.resultMotion.append(alignedMotionSeg)
            self.resultSegInfos.append(segInfo)
            self.validIdx = self.validIdx + 1
        #return self.resultMotion[-1], self.resultSegInfos[-1], self.resultMotionSegIdx[-1]

    def generateResultMotion(self, maxIdx):
        while self.validIdx < maxIdx:
            self.generateNext()
        #return self.getResultMotion()

    def getNext(self):
        if self.curIdx < 0:
            self.curIdx = 0
        else:
            self.curIdx = self.curIdx + 1

        while self.validIdx < self.curIdx:
            self.generateNext()

        return self.resultMotion[self.curIdx], self.resultSegInfos[self.curIdx], self.resultMotionSegIdx[self.curIdx]

    def getResultMotion(self):
        # self.resultMotion = [resultMotionSeg0, resultMotionSeg1, ...]
        # attatchedResultMotion = [resultMotionSeg0;resultMotionSeg1;...]
        attatchedResultMotion = []
        for resultMotionSeg in self.resultMotion:
            attatchedResultMotion.extend(resultMotionSeg)
        
        return attatchedResultMotion, self.resultMotion, self.resultSegInfos, self.resultMotionSegIdx



    def resetCurIdx(self):
        self.curIdx = -1

    def setCurIdx(self, idx):
        self.curIdx = idx

    def getCurIdx(self, idx):
        return self.curIdx

    def resetValidIdx(self):
        self.clearResult()

    def setValidIdx(self, idx):
        if idx < 0:
            self.clearResult()
        else:
            self.resultMotionSegIdx = self.resultMotionSegIdx[:idx+1]
            self.resultMotion = self.resultMotion[:idx+1]
            self.resultSegInfos = self.resultSegInfos[:idx+1]
            self.validIdx = idx

    def getValididx(self):
        return self.validIdx

if __name__=='__main__':

    # test
    motionDir = './../../../Tracking/wd2/motion/'
    motionFile = 'wd2_WalkForwardNormal00.bvh'
    motionFilePath = motionDir + motionFile

    motion = yf.readBvhFile(motionFilePath)
    motionGen = MotionGenerator(motion)

    resultMotion = []
    motionGen.generateResultMotion(4)
    resultMotion = motionGen.getResultMotion()
    print resultMotion

    viewer = ysv.SimpleViewer()
    viewer.initialize()
    viewer.record(False)

    viewer.doc.addRenderer('origin_motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_BONE))
    viewer.doc.addObject('origin_motion', motion)
    viewer.doc.addRenderer('result', yr.JointMotionRenderer(resultMotion, (0,255,200), yr.LINK_BONE))
    viewer.doc.addObject('result', resultMotion)
    '''
    def simulateCallback(frame):
        if not frame < len(resultMotion):
            print frame
            print "hoihoihooih"
            resultMotion.extend(motionGen.getNext())
            print "aaaaaaaaaaaaa"
            print resultMotion

    viewer.setSimulateCallback(simulateCallback)
    '''
    
    viewer.startTimer(1.0/motion.fps)
    viewer.show()

    Fl.run()

