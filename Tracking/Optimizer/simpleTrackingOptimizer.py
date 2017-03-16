from fltk import *
import numpy as np
import cPickle, copy

import sys
if not '../../PyCommon/modules/' in sys.path:
    sys.path.append('../../PyCommon/modules/')
if not '../Simulator/' in sys.path:
    sys.path.append('../Simulator/')

import Math.mmMath as mm
import Math.csMath as cm
import Math.ysFunctionGraph as yfg
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Resource.ysMotionLoader as yf
import trackingSimulator as ts

SIGNIFICANT_DIGIT = 6

def energyFuncSag(simulator):

    if simulator.falldown:
        return 100

    motion_simulated = simulator.motion_simulated
    motion_control = simulator.motion_control

    diff_simulated = motion_simulated[simulator.curFrame].getJointPositionGlobal(0) - motion_simulated[0].getJointPositionGlobal(0)
    diff_control = motion_control[simulator.curFrame].getJointPositionGlobal(0) - motion_control[0].getJointPositionGlobal(0)

    diff_simulated_sag, diff_simulated_cor = mm.projectionOnVector2(diff_simulated, diff_control)
    
    energy = mm.length(diff_simulated_sag)
    print energy

    return energy

def cutoffSag(simulator):

    return None


def energyFuncCorVel(simulator):

    if simulator.falldown:
        return 100

    motion_simulated = simulator.motion_simulated
    motion_control = simulator.motion_control

    total_diff = 0.0

    interval_simulated = simulator.interval_simulated[:-1]
    for interval in interval_simulated:
        posture_control = motion_control[interval[1]]
        posture_simulated = motion_simulated[interval[1]]

        diff = mm.length(mm.logSO3(mm.T2R(np.dot(posture_control.getJointOrientationGlobal(0).T, posture_simulated.getJointOrientationGlobal(0)))))
        total_diff += diff
        
        print "diff ", diff
        print "total_diff   ", total_diff
        
    return total_diff

class simpleTrackingOptimizer:

    def __init__(self, motion, mcfg, segInfos=None):

        self.refMotion = motion
        self.mcfg = mcfg
        self.refSegInfos = segInfos
        #self.simulator = ts.TrackingSimulator(self.refMotion, self.mcfg, self.refSegInfos)

        # optimized control parameter names and values
        self.optParamValues = {}
       
    def optimizeByFunc(self, numSegs, energyFunc, controlParamNames, initParamValue = 0.0, minValue = 0.0, maxValue = 1.0, interval = 0.1, threshold = 0.01 ):
        
        paramValue = initParamValue
        print "init paramValue  ", paramValue
        cpns = controlParamNames
        for cpn in cpns:
            if cpn in self.optParamValues:
                paramValue = self.optParamValues[cpn]
                break

        while maxValue - minValue < interval * 2.0:
            interval = interval / 2.0

        if paramValue - interval < minValue:
            paramValue = minValue + interval
        elif paramValue + interval > maxValue:
            paramValue = maxValue - interval
        paramValue = round(paramValue, SIGNIFICANT_DIGIT)        

        energyResults = {}

        def simulate(pv):
          
            if pv < minValue:
                pv = minValue
            if pv > maxValue:
                pv = maxValue

            pv = round(pv, SIGNIFICANT_DIGIT)

            if pv in energyResults:
                return energyResults[pv]
            
            simulator = ts.TrackingSimulator(self.refMotion, self.mcfg, self.refSegInfos)

            for cpn in self.optParamValues:
                setattr(simulator, cpn, self.optParamValues[cpn])

            for cpn in cpns:
                setattr(simulator, cpn, pv)

            for i in range(numSegs):
                if simulator.simulateOneSeg() == False:
                    print "the model falls down at seg ",i," with ",cpns[0]," value ",pv
                    break

            energyResults[pv] = energyFunc(simulator)
            
            #energyResults[pv] = (pv-1.0)**2
            return energyResults[pv]

        def recSearch(pv):
            
            if pv - interval < minValue:
                return pv
            elif pv + interval > maxValue:
                return pv

            a = simulate(pv - interval)
            b = simulate(pv)
            c = simulate(pv + interval)
            
            if b <= a and b <= c:
                return pv
            elif a <= b and a <= c:
                return recSearch(pv - interval)
            else:
                return recSearch(pv + interval)

        paramValue = recSearch(paramValue)
        print paramValue

        while interval > threshold:
        
            apv = paramValue - interval
            bpv = paramValue
            cpv = paramValue + interval

#            if apv < minValue:
#                apv = minValue
#            if cpv > maxValue:
#                cpv = maxValue

            a = simulate(apv)
            b = simulate(bpv)
            c = simulate(cpv)

            print "apv  ", apv, "   bpv ", bpv, "   cpv ", cpv
            print "a    ", a, " b   ", b, " c   ", c

            if b <= a and b <= c:
                paramValue = bpv
            elif a <= b and a <= c:
                paramValue = apv
            else:
                paramValue = cpv

            interval = interval / 2.0

#        print energyResults
        
        paramValue = round(paramValue, SIGNIFICANT_DIGIT)

        for cpn in cpns:
            self.optParamValues[cpn] = paramValue

        return paramValue


if __name__=='__main__':

    motionDir = '../Model/wd2/Motion/'
    motionFile = 'wd2_WalkForwardNormal00.bvh'
    motionFile = 'wd2_WalkForwardVFast00.bvh'
#    motionFile = 'wd2_WalkAzuma01.bvh'
    motionFilePath = motionDir + motionFile

    motion = yf.readBvhFile(motionFilePath)

    mcfgFilePath = motionDir + 'mcfg'
    mcfgFile = open(mcfgFilePath)
    mcfg = cPickle.load(mcfgFile)
    mcfgFile.close()

    sto = simpleTrackingOptimizer(motion, mcfg)

    result = sto.optimizeByFunc(5, energyFuncSag, ["K_swp_vel_sag", "K_swp_pos_sag"])
    print "result", result
    print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    print ""
    
    result = sto.optimizeByFunc(5, energyFuncCorVel, ["K_swp_pos_cor"])
    print "result   ", result

    

