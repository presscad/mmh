from fltk import *
import numpy as np
import cPickle, copy

import sys
if not '../../PyCommon/modules/' in sys.path:
    sys.path.append('../../PyCommon/modules/')

import Math.mmMath as mm
import Math.csMath as cm
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import GUI.ysSimpleViewer as ysv
import GUI.ysMultiViewer as ymv
import Resource.ysMotionLoader as yf
import Motion.dcMotionGen as dmg
import trackingSimulator as ts

if __name__=='__main__':

    motionDir = '../Model/wd2/Motion/'
    motionName = 'wd2_WalkForwardNormal00'
    #motionFile = 'wd2_WalkForwardNormal00.bvh'
    #motionFile = 'wd2_WalkAzuma01.bvh'
    #motionFile = 'wd2_WalkForwardVFast00.bvh'
    motionFilePath = motionDir + motionName + '.bvh'

    motion = yf.readBvhFile(motionFilePath)

    mcfgFilePath = motionDir + 'mcfg'
    mcfgFile = open(mcfgFilePath)
    mcfg = cPickle.load(mcfgFile)
    mcfgFile.close()

    #segFile = open(motionDir + 'wd2_WalkForwardNormal00.seg')
    #segFile = open(motionDir + 'wd2_WalkAzuma01.seg')
    #segInfos = cPickle.load(segFile)
    #segFile.close()

    for swp_vel_sag in range(10):
        for swp_pos_sag in range(10):
            for swp_vel_cor in range(10):
                for swp_pos_cor in range(10):
                    for stb_vel_pos in range(3):
                        simulator = ts.TrackingSimulator(motion, mcfg)
                        
                        simulator.K_swp_vel_sag = 0.1 * swp_vel_sag
                        simulator.K_swp_pos_sag = 0.1 * swp_pos_sag
                        simulator.K_swp_pos_sag_faster = simulator.K_swp_pos_sag
                        simulator.K_swp_vel_cor = 0.05 * swp_vel_cor
                        simulator.K_swp_pos_cor = 0.1 * swp_pos_cor
                        simulator.K_stb_vel = 0.1 * stb_vel_pos
                        simulator.K_stb_pos = 0.1 * stb_vel_pos
                        
                        segLen = len(simulator.motionGen.motionSegs)
                        falldown_seg_index = None
                        for i in range(segLen):
                            if simulator.simulateOneSeg() == False:
                                falldown_seg_index = i
                                break

                        result = {}
                        result['total_extended_frames'] = simulator.total_extended_frames
                        result['max_precontacted_frames'] = simulator.max_precontacted_frames
                        result['interval_simulated'] = simulator.interval_simulated
                        result['falldown_seg_index'] = falldown_seg_index
                        #result['motion_simulated'] = simulator.motion_simulated
                        #result['motion_control'] = simulator.motion_control

                        simulationName = str(swp_vel_sag) + str(swp_pos_sag) + str(swp_vel_cor) + str(swp_pos_cor) + str(stb_vel_pos)
                        resultFilePath = './' + motionName + '/' + simulationName + '.res'
                        resultFile = open(resultFilePath, 'wb')
                        cPickle.dump(result, resultFile)
                        resultFile.close()

                        yf.writeBvhFile('./' + motionName + '/' + simulationName + '_simulated.bvh', simulator.motion_simulated)
                        yf.writeBvhFile('./' + motionName + '/' + simulationName + '_control.bvh', simulator.motion_control)



                        print str(swp_vel_sag) + str(swp_pos_sag) + str(swp_vel_cor) + str(swp_pos_cor) + str(stb_vel_pos)

    print "done"

