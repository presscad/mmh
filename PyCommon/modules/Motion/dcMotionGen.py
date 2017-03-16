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

    def __init__(self,motion,segInfos=None):
        #self.motion = motion.copy()
        # motion -> motion segmentations
        self.motionSegs, self.segInfos = dms.segmentMotion(motion)
        if segInfos != None:
            self.segInfos = segInfos
        
        self.nextSegIdxs = [i+1 for i in range(len(self.motionSegs)-1)]
        self.nextSegIdxs.append(None)
        self.curIdx = 0

    def resetCurIdx(self):
        self.curIdx = 0

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
        self.nextSegIdxs.append(None)
        return len(self.motionSegs)-1        

    def setMotionSeg(self, segIdx, motionSeg, segInfo = None):
        if segInfo == None:
            motionSegs, segInfos = dms.segmentMotion(motionSeg)
            motionSeg = motionSegs[0]
            segInfo = segInfos[0]

        self.motionSegs[segIdx] = motionSeg.copy()
        self.segInfos[segIdx] = segInfo.copy()
        self.nextSegIdxs[segIdx] = None
        return segIdx

    def blendAndAdd(self, motionSegIdx0, motionSegIdx1):
        motionSeg0 = self.motionSegs[motionSegIdx0]
        motionSeg1 = self.motionSegs[motionSegIdx1]

        newMotionSeg = ymb.blendSegmentSmooth(motionSeg0, motionSeg1, True, True)
        newMotionSegInfo = self.segInfos[motionSegIdx0]

        return self.addMotionSeg(newMotionSeg, newMotionSegInfo)

    def setNextSegIdx(self, curSegIdx, nextSegIdx):
        self.nextSegIdxs[curSegIdx] = nextSegIdx
    
    def setNextSegIdxs(self, nextSegIdxs):
        self.nextSegIdxs = nextSegIdxs

    # =======================================
    # result of motion generation
    # =======================================
    def getMotionSeg(self, idx):
        return self.motionSegs[idx].copy()

    def getSegInfo(self, idx):
        return self.segInfos[idx].copy()
    
    def getCurMotionSeg(self):
        return self.motionSegs[self.curIdx].copy()

    def getCurSegInfo(self):
        return self.segInfos[self.curIdx].copy()

    def goToNext(self):
        self.curIdx = self.nextSegIdxs[self.curIdx]
        return self.curIdx

    def goTo(self,idx):
        if idx < len(self.motionSegs):
            self.curIdx = idx
        else:
            self.curIdx = None

    def getNthNextSegIdx(self, n):
        idx = self.curIdx
        while n>0 and idx != None:
            idx = self.nextSegIdxs[idx]
            n = n - 1

        return idx

if __name__=='__main__':

    # test
    motionDir = './../../../Tracking/wd2/motion/'
    motionFile = 'wd2_WalkForwardNormal00.bvh'
    motionFilePath = motionDir + motionFile

    motion = yf.readBvhFile(motionFilePath)
    motionGen = MotionGenerator(motion)
    bIdx = motionGen.blendAndAdd(4,2)
    print "bIdx", bIdx
    motionGen.setNextSegIdx(3,bIdx)
    motionGen.setNextSegIdx(bIdx,3)

#    motionGen23 = MotionGenerator(motion)
#    motionGen23.setNextSegIdx(3,2)
#    motionGen23.setNextSegIdx(2,3)

    resultMotion = motionGen.getCurMotionSeg()
#    motion23 = motionGen23.getCurMotionSeg()

    viewer = ysv.SimpleViewer()
    viewer.initialize()
    viewer.setMaxFrame(1000)
#    viewer.record(False)

    viewer.doc.addRenderer('origin_motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_BONE))
#    viewer.doc.addObject('origin_motion', motion)
    viewer.doc.addRenderer('result', yr.JointMotionRenderer(resultMotion, (0,255,200), yr.LINK_BONE))
#    viewer.doc.addObject('result', resultMotion)

#    viewer.doc.addRenderer('motion23', yr.JointMotionRenderer(motion23, (255,255,200), yr.LINK_BONE))


#    def postFrameCallback_Always(frame):
#        print frame

#    viewer.setPostFrameCallback_Always(postFrameCallback_Always)

    def simulateCallback(frame):
        print frame
        if not frame < len(resultMotion):
            print frame
            print "hoihoihooih"
            motionGen.goToNext()
            resultMotion.extend(ymb.getAttachedNextMotion(motionGen.getCurMotionSeg(), resultMotion[-1], True, True))
            #resultMotion.extend(ymb.getAttachedNextMotion(motionGen.getCurMotionSeg(), resultMotion[-1], False, False))
            
 #           motionGen23.goToNext()
 #           motion23.extend(ymb.getAttachedNextMotion(motionGen23.getCurMotionSeg(), motion23[-1],True,True))
    
            print "aaaaaaaaaaaaa"

        motion.goToFrame(frame)
        resultMotion.goToFrame(frame)
 #       motion23.goToFrame(frame)

    viewer.setSimulateCallback(simulateCallback)
    viewer.startTimer(1.0/motion.fps)
    viewer.show()

    Fl.run()

