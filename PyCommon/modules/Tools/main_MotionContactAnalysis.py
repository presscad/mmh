import psyco; psyco.full()
from fltk import *
import operator as op

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mmMath
import Resource.ysMotionLoader as yf
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv
import Util.ysGlHelper as ygh
import Util.ysPythonEx as ype
import Util.ysMatplotEx as ymp

if __name__ == "__main__":
    bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
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
#    gaitStates = yba.getBipedGaitStates(clFoot, crFoot)
#    gaitStates0 = yba.getBipedGaitStates(clFoot, crFoot, 20, 0., 20 ,0.)
#    gaitStates1 = yba.getBipedGaitStates(clFoot, crFoot, 20, 1., 20 ,1.)
    
    lFootStates = yma.getFootStates(clHeel, clToe)
    rFootStates = yma.getFootStates(crHeel, crToe)
    
    lt, ll = yma.getTakingLandingFrames(clFoot)
    rt, rl = yma.getTakingLandingFrames(crFoot)
    
    plot = ymp.SmartPlot()
    plot.setXdata('frame', range(len(motion)))
    plot.addYdata('clHeel', clHeel, False)
    plot.addYdata('crHeel', crHeel, False)
    plot.addYdata('clToe', clToe, False)
    plot.addYdata('crToe', crToe, False)
    plot.addYdata('clFoot', clFoot, False)
    plot.addYdata('crFoot', crFoot, False)
#    plot.addYdata('gaitStates', gaitStates, False)
#    plot.addYdata('gaitStates1', gaitStates1, False)
#    plot.addYdata('gaitStates0', gaitStates0, False)
    plot.addYdata('lFootStates', lFootStates, False)
    plot.addYdata('rFootStates', rFootStates, False)
    plot.addYdata('lPos.y', [motion.getPosition(LFOOT, i)[1] for i in range(len(motion))], False)
    plot.addYdata('rPos.y', [motion.getPosition(RFOOT, i)[1] for i in range(len(motion))], False)
    plot.addYlines('hRef', [hRef], False)
    plot.addYdata('lVel.length', [mmMath.length(motion.getVelocity(LFOOT, i)) for i in range(len(motion))], False)
    plot.addYdata('rVel.length', [mmMath.length(motion.getVelocity(RFOOT, i)) for i in range(len(motion))], False)
    plot.addYlines('vRef', [vRef], False)
    plot.showModeless()
        
    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (100,255,100), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    
#    lSwingRenderer = yr.JointMotionRenderer(motion, (255,153,0), yr.LINK_BONE)
#    lSwingRenderer.renderFrames = [i for i in range(len(motion)) if lFootStates[i]==False]
#    viewer.doc.addRenderer('lSwing', lSwingRenderer)
#    
#    rSwingRenderer = yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE)
#    rSwingRenderer.renderFrames = [i for i in range(len(motion)) if rFootStates[i]==False]
#    viewer.doc.addRenderer('rSwing', rSwingRenderer)
    
    llr = yr.JointMotionRenderer(motion, (255,0,0), yr.LINK_BONE)
    llr.renderFrames = ll
    viewer.doc.addRenderer('lLanding', llr)

    ltr = yr.JointMotionRenderer(motion, (255,153,0), yr.LINK_BONE)
    ltr.renderFrames = lt
    viewer.doc.addRenderer('lTaking', ltr)

    rlr = yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE)
    rlr.renderFrames = rl
    viewer.doc.addRenderer('rLanding', rlr)
    rtr = yr.JointMotionRenderer(motion, (204,153,255), yr.LINK_BONE)
    rtr.renderFrames = rt
    viewer.doc.addRenderer('rTaking', rtr)

    contactStatesMap = yma.getMotionContactStates(motion, hRef, vRef)
    
    def extraDrawCallback():
        frame = viewer.getCurrentFrame()
        for index, contactStates in contactStatesMap.items():
            if contactStates[frame]:
                ygh.drawPoint(motion.getPosition(index, frame), (255,255,255), 5.)
                
    viewer.setExtraDrawCallback(extraDrawCallback)
    
    def viewer_onClose(data):
        plot.close()
        viewer.onClose(data)
    viewer.callback(viewer_onClose)
        
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()