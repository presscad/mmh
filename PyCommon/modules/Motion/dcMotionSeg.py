import math

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba

def segmentMotion(motion, hRef = 0.1, vRef = 0.4, partsNames=["LeftFoot", "RightFoot", "LeftLeg", "RightLeg", "LeftUpLeg", "RightUpLeg"], jumpThreshold = 15, jumpBias = 1.0, stopThreshold = 15, stopBias = 0.0):
    skeleton = motion[0].skeleton
    
    lFoot = skeleton.getJointIndex(partsNames[0])
    rFoot = skeleton.getJointIndex(partsNames[1])
    lKnee = skeleton.getJointIndex(partsNames[2])
    rKnee = skeleton.getJointIndex(partsNames[3])
    lHip = skeleton.getJointIndex(partsNames[4])
    rHip = skeleton.getJointIndex(partsNames[5])

    #lc = yma.getElementContactStates(motion, lFoot, hRef, vRef)
    #rc = yma.getElementContactStates(motion, rFoot, hRef, vRef)
    lc = yma.getElementContactStates(motion, "LeftFoot", hRef, vRef)
    rc = yma.getElementContactStates(motion, "RightFoot", hRef, vRef)
    intervals, states = yba.getBipedGaitIntervals2(lc, rc, jumpThreshold, jumpBias, stopThreshold, stopBias)
    
    motions = yma.splitMotionIntoSegments(motion.copy(), intervals)

    seginfos = [{} for i in range(len(intervals))]
    for i in range(len(intervals)):
        start = intervals[i][0]
        end = intervals[i][1]

        seginfos[i]['interval'] = intervals[i]
        seginfos[i]['state'] = states[i]

        stanceHips = []; swingHips = []; stanceFoots = []; swingFoots = []; swingKnees = []

        if states[i] == yba.GaitState.LSWING:
            stanceHips = [rHip]; stanceFoots = [rFoot]; swingHips = [lHip]; swingFoots = [lFoot]; swingKnees = [lKnee]
        elif states[i] == yba.GaitState.RSWING:
            stanceHips = [lHip]; stanceFoots = [lFoot]; swingHips = [rHip]; swingFoots = [rFoot]; swingKnees = [rKnee]
        elif states[i] == yba.GaitState.STOP:
            stanceHips = [rHip, lHip]; stanceFoots = [rFoot, lFoot]
        elif states[i] == yba.GaitState.JUMP:
            swingHips = [rHip, lHip]; swingFoots = [rFoot, lFoot]

        seginfos[i]['stanceHips'] = stanceHips
        seginfos[i]['swingHips'] = swingHips
        seginfos[i]['stanceFoots'] = stanceFoots
        seginfos[i]['swingFoots'] = swingFoots
        seginfos[i]['swingKnees'] = swingKnees

        if start < end:
            seginfos[i]['ground_height'] = min([posture_seg.getJointPositionGlobal(foot)[1] for foot in [lFoot, rFoot] for posture_seg in motion[start:end]])

    return motions, seginfos
