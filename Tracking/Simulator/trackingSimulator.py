from fltk import *
import numpy as np
import cPickle, copy

import sys
if not '../../PyCommon/modules/' in sys.path:
    sys.path.append('../../PyCommon/modules/')

import Math.mmMath as mm
import Math.csMath as cm
import Math.ysFunctionGraph as yfg
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.ysVpUtil as yvu
import GUI.ysSimpleViewer as ysv
import GUI.ysMultiViewer as ymv
import ArticulatedBody.ysControl as yct
import ArticulatedBody.ysReferencePoints as yrp
import Motion.ysMotionAnalysis as yma
import Motion.ysBipedAnalysis as yba
import Motion.ysMotion as ym
import Motion.ysMotionBlend as ymb
import Motion.ysMotionExtend as ymt
import Motion.ysSkeletonEdit as yhe
import Motion.mmAnalyticIK as aik
import Motion.dcMotionGen as dmg
import Util.ysMatplotEx as ymp
import Resource.ysMotionLoader as yf
import Simulator.ysPhysConfig as ypc

#MOTION_COLOR = (128,128,128)
#CHARACTER_COLOR = (102,102,153)
MOTION_COLOR = (213,111,162)
CHARACTER_COLOR = (20,166,188)

MULTI_VIEWER = False
CAMERA_TRACKING = False
TORQUE_PLOT = False
NO_FOOT_SLIDING = True

class TrackingSimulator:

    class ForceInfo:
        def __init__(self, startFrame, duration, force):
            self.startFrame = startFrame    # frame
            self.duration = duration        # sec
            self.force = force              # N
            self.targetBody = None

    def __init__(self, motion, mcfg, segInfos=None):

        self.initGlobalParam()
        self.initControlParam()
        self.initFunctionGraph()

        self.refMotion = motion
        self.refSegInfos = segInfos
        self.frameTime = 1.0/self.refMotion.fps
        self.skeleton = self.refMotion[0].skeleton

        self.initMotionGen()

        self.initVpWorld()
        self.initVpModel(mcfg)
        self.initModelInformation()

        self.initExternalForceInfos()

        self.initControlMotion()
        self.initSimulationVariable()
        self.initSimulationData()



    #==================================================

    def initGlobalParam(self):

        # global parameters
        # PD control
        self.Kt = 20.;      self.Dt = 2*(self.Kt**.5)
        # penalty force control
        self.Ks = 2000.;    self.Ds = 2*(self.Ks**.5)
        self.mu = 1.

        # constants
        self.c_min_contact_vel = 100.
        #c_min_contact_vel = 2.
        self.c_min_contact_time = .7
        self.c_landing_duration = .2
        self.c_taking_duration = .3
        self.c_swf_mid_offset = .0
        self.c_locking_vel = .05
        #c_swf_offset = .0
        self.c_swf_offset = .01
       
        # thresholds for fall down check
        self.falldown_root_y_threshold = 0.5
        self.falldown_extended_frame_threshold = 10

    def initControlParam(self):
        
        self.SEGMENT_EDITING = True

        self.STANCE_FOOT_STABILIZE = True
        self.MATCH_STANCE_LEG = True
        
        self.SWING_FOOT_PLACEMENT = True
        self.SWING_FOOT_HEIGHT = True
        self.OLD_SWING_HEIGHT = False
        self.HIGHER_OFFSET = True
        self.SWING_FOOT_ORIENTATION = False#True

        self.STANCE_FOOT_PUSH = False#True
        self.STANCE_FOOT_BALANCING = True

        self.NO_FOOT_SLIDING = True

        # SWING_FOOT_PLACEMENT
        self.K_swp_vel_sag = 0.0
        self.K_swp_vel_cor = 0.0
        self.K_swp_pos_sag = 0.0
        self.K_swp_pos_sag_faster = 0.0
        self.K_swp_pos_cor = 0.0

        # SWING_FOOT_HEIGHT
        self.c5 = 0.5
        self.c6 = 0.02

        # STANCE_FOOT_BALANCING
        self.K_stb_vel = 0.1
        self.K_stb_pos = 0.1

    def initFunctionGraph(self):
        
        self.stitch_func = lambda x : 1. - yfg.hermite2nd(x)
        
        self.stf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [self.c_landing_duration])
        self.match_stl_func = yfg.hermite2nd
    
        self.swf_placement_func = yfg.hermite2nd
        self.swf_height_func = yfg.hermite2nd
        self.swf_height_sine_func = yfg.sine
    
        self.stf_balancing_func = yfg.hermite2nd
    
    #==================================================

    def initMotionGen(self):

        '''
        # debug
        print "preprocessed"
        for i in range(len(self.refSegInfos)):
            print "i    ", i
            print "interval ", self.refSegInfos[i]["interval"]
            print "state    ", self.refSegInfos[i]["state"]

        self.motionGen = dmg.MotionGenerator(self.refMotion)
        print "runtime generation"
        for i in range(len(self.motionGen.segInfos)):
            print "i    ", i
            print "interval ", self.motionGen.segInfos[i]["interval"]
            print "state    ", self.motionGen.segInfos[i]["state"]
        '''

        #self.motionGen = dmg.MotionGenerator(self.refMotion)
        self.motionGen = dmg.MotionGenerator(self.refMotion, self.refSegInfos)
        if self.refSegInfos == None:
            self.refSegInfos = copy.copy(self.motionGen.segInfos)


    def initVpWorld(self):

        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = 0.0
        wcfg.useDefaultContactModel = False
        wcfg.lockingVel = self.c_locking_vel
        self.stepsPerFrame = 30
        wcfg.timeStep = (self.frameTime) / self.stepsPerFrame

        self.vpWorld = cvw.VpWorld(wcfg)
        #self.vpWorld.initialize()

    def initVpModel(self, mcfg):
        self.motionModel = cvm.VpMotionModel(self.vpWorld, self.refMotion[0], mcfg)
        self.controlModel = cvm.VpControlModel(self.vpWorld, self.refMotion[0], mcfg)
        self.vpWorld.initialize()

        self.controlModel.initializeHybridDynamics()

    
    def initModelInformation(self):
    
        self.bodyIDsToCheck = range(self.vpWorld.getBodyNum())
        self.mus = [self.mu]*len(self.bodyIDsToCheck)

        controlModel = self.controlModel
        self.lID = controlModel.name2id('LeftFoot')
        self.rID = controlModel.name2id('RightFoot')
        
        skeleton = self.skeleton
        self.lUpLeg = skeleton.getJointIndex('LeftUpLeg')
        self.rUpLeg = skeleton.getJointIndex('RightUpLeg')
        self.lKnee = skeleton.getJointIndex('LeftLeg')
        self.rKnee = skeleton.getJointIndex('RightLeg')
        self.lFoot = skeleton.getJointIndex('LeftFoot')
        self.rFoot = skeleton.getJointIndex('RightFoot')
        self.spine = skeleton.getJointIndex('Spine')
    
        self.uppers = [skeleton.getJointIndex(name) for name in ['Hips', 'Spine', 'Spine1', 'LeftArm', 'LeftForeArm', 'RightArm', 'RightForeArm']]
        self.lLegs = [skeleton.getJointIndex(name) for name in ['LeftUpLeg', 'LeftLeg', 'LeftFoot']]
        self.rLegs = [skeleton.getJointIndex(name) for name in ['RightUpLeg', 'RightLeg', 'RightFoot']]
    
        #self.allJoints = set(range(skeleton.getJointNum()))
        
        #self.bodyMasses = controlModel.getBodyMasses()
        #self.totalMass = controlModel.getTotalMass()
        #self.upperMass = sum([bodyMasses[i] for i in self.uppers])

        self.halfFootHeight = controlModel.getBodyShape(self.lFoot)[1]/2.


    #==================================================

    def initExternalForceInfos(self):
        self.forceInfos = []
        # self.forceInfos = [ForceInfo(70, .4, (100,0,0))]

    def initControlMotion(self):
        
        motionSeg0 = self.motionGen.getMotionSeg(0)
        self.motion_seg_orig = ym.JointMotion()
        self.motion_seg_orig += motionSeg0
        self.motion_seg = ym.JointMotion()
        self.motion_seg += motionSeg0
        self.motion_stitch = ym.JointMotion()
        self.motion_stitch += motionSeg0

        self.motion_stf_stabilize = ym.JointMotion()
        self.motion_match_stl = ym.JointMotion()

        self.motion_swf_placement = ym.JointMotion()
        self.motion_swf_height = ym.JointMotion()
        self.motion_swf_orientation = ym.JointMotion()
        
        self.motion_stf_balancing = ym.JointMotion()
        self.motion_stf_push = ym.JointMotion()
        
        self.motion_control = ym.JointMotion()

        self.motion_simulated = ym.JointMotion()

        # the posture of frame 0
        self.motion_stf_stabilize.extend([motionSeg0[0]])
        self.motion_match_stl.extend([motionSeg0[0]])

        self.motion_swf_placement.extend([motionSeg0[0]])
        self.motion_swf_height.extend([motionSeg0[0]])
        self.motion_swf_orientation.extend([motionSeg0[0]])

        self.motion_stf_balancing.extend([motionSeg0[0]])
        self.motion_stf_push.extend([motionSeg0[0]])

        self.motion_control.extend([motionSeg0[0]])

        self.motion_simulated.extend([motionSeg0[0]])


    
    def initSimulationVariable(self):
      
        self.falldown = False
        self.simulationEnd = False

        # loop variable
        self.curFrame = 0
        self.segIndex = 0
        #self.seg_index = [0]

        self.acc_offset = [0]
        self.extended = [False]
        self.extended_frame = 0
        self.prev_R_swp = [None]
    
        self.stl_y_limit_num = [0]
        self.stl_xz_limit_num = [0]
       
        self.direction = mm.O_Vec3()
        self.directionAxis = mm.O_Vec3()

        self.avg_dCM = [mm.O_Vec3()]
        #self.avg_stf_v = [mm.O_Vec3()]
        #self.avg_stf_av = [mm.O_Vec3()]
  
        # penalty force
        self.bodyIDs = None
        self.contactPositions = None
        self.contactPositionLocals = None
        self.contactForces = None

        # for ys's stance foot push
        self.step_length_cur = [0.]
        self.step_length_tar = [0.]
        self.step_axis = [mm.O_Vec3()]

    def initSimulationData(self):
        
        self.rhip_torques = []
        self.rknee_torques = []
        self.rankle_torques = []
    
        self.rd_CM = [None]
        self.rd_CP = [None]
        self.rd_CMP = [None]
        self.rd_forces = [None]
        self.rd_force_points = [None]
        self.rd_torques = []
        self.rd_joint_positions = []

        # data for optimization
        # for the swing_foot_placement of the sagital plane
        self.total_extended_frames = 0
        self.max_precontacted_frames = 0

        self.interval_simulated = [[0,None]]

    #==================================================

    def modulateControlMotion(self):
        # modulate a current frame of control motion by control rules with control params
        segInfo = self.motionGen.getCurSegInfo()
        
        curState = segInfo['state']
        curInterval = yma.offsetInterval(self.acc_offset[0], segInfo['interval'])
        stanceLegs = segInfo['stanceHips']
        swingLegs = segInfo['swingHips']
        stanceFoots = segInfo['stanceFoots']
        swingFoots = segInfo['swingFoots']
        swingKnees = segInfo['swingKnees']
        groundHeight = segInfo['ground_height']
        #maxStfPushFrame = segInfo['max_stf_push_frame']
       
        frame = self.curFrame
        prev_frame = frame-1 if frame>0 else 0
       
        # models
        motionModel = self.motionModel
        controlModel = self.controlModel

        # motions
        motion_seg_orig = self.motion_seg_orig
        motion_seg = self.motion_seg
        motion_stitch = self.motion_stitch

        motion_stf_stabilize = self.motion_stf_stabilize
        motion_match_stl = self.motion_match_stl

        motion_swf_placement = self.motion_swf_placement
        motion_swf_height = self.motion_swf_height
        motion_swf_orientation = self.motion_swf_orientation
        
        motion_stf_balancing = self.motion_stf_balancing
        motion_stf_push = self.motion_stf_push
        
        motion_control = self.motion_control
        
        # information
        dCM_tar = motion_seg.getJointVelocityGlobal(0, prev_frame)
        CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], prev_frame)
        CMr_tar = CM_tar - stf_tar
            
        dCM = self.avg_dCM[0]
        CM = controlModel.getJointPositionGlobal(0)
        #CMreal = yrp.getCM(controlModel.getJointPositionsGlobal(), controlModel.getBodyMasses(), controlModel.getTotalMass())
        stf = controlModel.getJointPositionGlobal(stanceFoots[0])
        CMr = CM - stf
        
        diff_dCM = mm.projectionOnPlane(dCM-dCM_tar, (1,0,0), (0,0,1))
        diff_dCM_axis = np.cross((0,1,0), diff_dCM)
        #rd_vec1[0] = diff_dCM; rd_vecori1[0] = CM_tar
        
        diff_CMr = mm.projectionOnPlane(CMr-CMr_tar, (1,0,0), (0,0,1))
#        rd_vec1[0] = diff_CMr; rd_vecori1[0] = stf_tar
        diff_CMr_axis = np.cross((0,1,0), diff_CMr)
        
        self.direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
#        direction = mm.normalize2(mm.projectionOnPlane(dCM, (1,0,0), (0,0,1)))
        self.directionAxis = np.cross((0,1,0), self.direction)
        
        diff_dCM_sag, diff_dCM_cor = mm.projectionOnVector2(diff_dCM, self.direction)
#        rd_vec1[0] = diff_dCM_sag; rd_vecori1[0] = CM_tar
        diff_dCM_sag_axis = np.cross((0,1,0), diff_dCM_sag)
        diff_dCM_cor_axis = np.cross((0,1,0), diff_dCM_cor)
            
        diff_CMr_sag, diff_CMr_cor = mm.projectionOnVector2(diff_CMr, self.direction)
        diff_CMr_sag_axis = np.cross((0,1,0), diff_CMr_sag)
        diff_CMr_cor_axis = np.cross((0,1,0), diff_CMr_cor)
        
        t = (frame-curInterval[0])/float(curInterval[1]-curInterval[0])
        if t>1.: t=1.
        
        p_root = motion_stitch[frame].getJointPositionGlobal(0)
        R_root = motion_stitch[frame].getJointOrientationGlobal(0)

        motion_seg_orig.goToFrame(frame)
        motion_seg.goToFrame(frame)
        motion_stitch.goToFrame(frame)
        

        # stance foot stabilize
        motion_stf_stabilize.append(motion_stitch[frame].copy())
        motion_stf_stabilize.goToFrame(frame)
        if self.STANCE_FOOT_STABILIZE:
            for stanceFoot in stanceFoots:
                R_target_foot = motion_seg[frame].getJointOrientationGlobal(stanceFoot)
                R_current_foot = motion_stf_stabilize[frame].getJointOrientationGlobal(stanceFoot)
                motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , self.stf_stabilize_func(t)))
#                R_target_foot = motion_seg[frame].getJointOrientationLocal(stanceFoot)
#                R_current_foot = motion_stf_stabilize[frame].getJointOrientationLocal(stanceFoot)
#                motion_stf_stabilize[frame].setJointOrientationLocal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))

        # match stance leg 
        motion_match_stl.append(motion_stf_stabilize[frame].copy())
        motion_match_stl.goToFrame(frame)
        if self.MATCH_STANCE_LEG:
            if curState!=yba.GaitState.STOP:
                for i in range(len(stanceLegs)):
                    stanceLeg = stanceLegs[i]
                    stanceFoot = stanceFoots[i]
                    
#                    # motion stance leg -> character stance leg as time goes
                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
                    motion_match_stl[frame].setJointOrientationGlobal(stanceLeg, cm.slerp(R_motion, R_character, self.match_stl_func(t)))

#                    t_y = match_stl_func_y(t)
#                    t_xz = match_stl_func(t)
#                    
#                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
#                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
#                    R = np.dot(R_character, R_motion.T)
#                    R_y, R_xz = mm.projectRotation((0,1,0), R)
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_xz, t_xz))
#                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_y, t_y))

        # swing foot placement
        motion_swf_placement.append(motion_match_stl[frame].copy())
        motion_swf_placement.goToFrame(frame)
        if self.SWING_FOOT_PLACEMENT:
            t_swing_foot_placement = self.swf_placement_func(t);
           
            # ys origin
            if self.extended[0]:
                R_swp_sag = self.prev_R_swp[0][0]
                R_swp_cor = self.prev_R_swp[0][1]
            else:
                R_swp_sag = mm.I_SO3(); R_swp_cor = mm.I_SO3()
                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * self.K_swp_vel_sag * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_dCM_cor_axis * self.K_swp_vel_cor * -t_swing_foot_placement))
                if np.dot(self.direction, diff_CMr_sag) < 0:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * self.K_swp_pos_sag * -t_swing_foot_placement))
                else:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * self.K_swp_pos_sag_faster * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_CMr_cor_axis * self.K_swp_pos_cor * -t_swing_foot_placement))

            '''
            # dcjo test
            if extended[0]:
                R_swp_sag = prev_R_swp[0][0]
                R_swp_cor = prev_R_swp[0][1]
            else:
                R_swp_sag = mm.I_SO3(); R_swp_cor = mm.I_SO3()

                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_axis * K_swp_vel_sag * -t_swing_foot_placement))
                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_axis * K_swp_pos_sag * -t_swing_foot_placement))
            '''

            for i in range(len(swingLegs)):
                swingLeg = swingLegs[i]
                swingFoot = swingFoots[i] 
                
                # save swing foot global orientation
#                R_swf = motion_swf_placement[frame].getJointOrientationGlobal(swingFoot)
                
                # rotate swing leg
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_sag)
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_cor)
                
                # restore swing foot global orientation
#                motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, R_swf)
                self.prev_R_swp[0] = (R_swp_sag, R_swp_cor)

        # swing foot height
        motion_swf_height.append(motion_swf_placement[frame].copy())
        motion_swf_height.goToFrame(frame)
        if self.SWING_FOOT_HEIGHT:
            for swingFoot in swingFoots:
                stanceFoot = stanceFoots[0]

                # save foot global orientation
                R_foot = motion_swf_height[frame].getJointOrientationGlobal(swingFoot)
                R_stance_foot = motion_swf_height[frame].getJointOrientationGlobal(stanceFoot)

                if self.OLD_SWING_HEIGHT:
                    height_tar = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
                else:
                    height_tar = motion_swf_height[prev_frame].getJointPositionGlobal(swingFoot)[1] - groundHeight
                    d_height_tar = motion_swf_height.getJointVelocityGlobal(swingFoot, prev_frame)[1]
#                motion_debug1[frame] = motion_swf_height[frame].copy()

                # rotate
                motion_swf_height[frame].rotateByTarget(controlModel.getJointOrientationGlobal(0))
#                motion_debug2[frame] = motion_swf_height[frame].copy()
#                motion_debug2[frame].translateByTarget(controlModel.getJointPositionGlobal(0))

                if self.OLD_SWING_HEIGHT:
                    height_cur = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
                else:
                    height_cur = controlModel.getJointPositionGlobal(swingFoot)[1] - self.halfFootHeight - self.c_swf_offset
                    d_height_cur = controlModel.getJointVelocityGlobal(swingFoot)[1]

                if self.OLD_SWING_HEIGHT:
                    offset_height = (height_tar - height_cur) * self.swf_height_func(t) * c5
                else:
                    offset_height = ((height_tar - height_cur) * self.c5
                                     + (d_height_tar - d_height_cur) * self.c6) * self.swf_height_func(t)

                offset_sine = self.c_swf_mid_offset * self.swf_height_sine_func(t)
                
                offset = 0.
                offset += offset_height
                offset += offset_sine

                if offset > 0.:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(swingFoot)
                    newPosition[1] += offset
                    aik.ik_analytic(motion_swf_height[frame], swingFoot, newPosition)
                else:
                    if self.HIGHER_OFFSET:
                        newPosition =  motion_swf_height[frame].getJointPositionGlobal(stanceFoot)
                        newPosition[1] -= offset
                        aik.ik_analytic(motion_swf_height[frame], stanceFoot, newPosition)

                # return
#                motion_debug3[frame] = motion_swf_height[frame].copy()
#                motion_debug3[frame].translateByTarget(controlModel.getJointPositionGlobal(0))
                motion_swf_height[frame].rotateByTarget(R_root)
                
                # restore foot global orientation
                motion_swf_height[frame].setJointOrientationGlobal(swingFoot, R_foot)
                motion_swf_height[frame].setJointOrientationGlobal(stanceFoot, R_stance_foot)

#                if plot!=None:
#                    plot.addDataPoint('debug1', frame, offset_height)
#                    plot.addDataPoint('debug2', frame, height_cur)
#                    plot.addDataPoint('diff', frame, diff)

        # stance foot push                
        motion_stf_push.append(motion_swf_height[frame].copy())
        motion_stf_push.goToFrame(frame)
        if self.STANCE_FOOT_PUSH:
            for swingFoot in swingFoots:
                
                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [c_taking_duration*2])
                R_swp_sag = mm.I_SO3()
                R_swp_sag = np.dot(R_swp_sag, mm.exp((self.step_length_tar[0] - self.step_length_cur[0])*self.step_axis[0] * self.K_stp_pos * -self.stf_push_func(t)))
                    
                motion_stf_push[frame].mulJointOrientationGlobal(swingFoot, R_swp_sag)
        
        # stance foot balancing 
        motion_stf_balancing.append(motion_stf_push[frame].copy())
        motion_stf_balancing.goToFrame(frame)
        if self.STANCE_FOOT_BALANCING:
            R_stb = mm.exp(diff_dCM_axis * self.K_stb_vel * self.stf_balancing_func(t))
            R_stb = np.dot(R_stb, mm.exp(diff_CMr_axis * self.K_stb_pos * self.stf_balancing_func(t)))
            for stanceFoot in stanceFoots:
                if frame < 5: continue
                motion_stf_balancing[frame].mulJointOrientationGlobal(stanceFoot, R_stb)
        
        # control trajectory
        motion_control.append(motion_stf_balancing[frame].copy())
        motion_control.goToFrame(frame)


    def editSegment(self):
        
        segInfo = self.motionGen.getCurSegInfo()
        
        curState = segInfo['state']
        curInterval = yma.offsetInterval(self.acc_offset[0], segInfo['interval'])
        stanceLegs = segInfo['stanceHips']
        swingLegs = segInfo['swingHips']
        stanceFoots = segInfo['stanceFoots']
        swingFoots = segInfo['swingFoots']
        swingKnees = segInfo['swingKnees']
        groundHeight = segInfo['ground_height']
        #maxStfPushFrame = segInfo['max_stf_push_frame']
      
        motion_seg_orig = self.motion_seg_orig
        motion_seg = self.motion_seg
        motion_stitch = self.motion_stitch
        motion_control = self.motion_control

        controlModel = self.controlModel

        frame = self.curFrame
        t = (frame-curInterval[0])/float(curInterval[1]-curInterval[0])
        if t>1.: t=1.
        
        lastFrame = False
        
        if self.SEGMENT_EDITING:
            if curState==yba.GaitState.STOP:
                if frame == len(motion_seg)-1:
                    lastFrame = True
                    
            elif (curState==yba.GaitState.LSWING or curState==yba.GaitState.RSWING) and t>self.c_min_contact_time:
                swingID = self.lID if curState==yba.GaitState.LSWING else self.rID

                contact = False
                if swingID in self.bodyIDs:
                    minContactVel = 1000.
                    for i in range(len(self.bodyIDs)):
                        if self.bodyIDs[i]==swingID:
                            vel = controlModel.getBodyVelocityGlobal(swingID, self.contactPositionLocals[i])
                            vel[1] = 0
                            contactVel = mm.length(vel)
                            if contactVel < minContactVel: minContactVel = contactVel
                    if minContactVel < self.c_min_contact_vel: contact = True
                
                self.extended[0] = False
                
                if contact:
                    #print minContactVel
                    #print frame, 'foot touch'
                    lastFrame = True
                    self.acc_offset[0] += frame - curInterval[1]
                    precontacted_frame = curInterval[1] - frame
                    if precontacted_frame > self.max_precontacted_frames:
                        self.max_precontacted_frames = precontacted_frame

                elif frame == len(motion_seg)-1:
                    print frame, 'extend frame', frame+1
                    self.extended_frame += 1
                    self.total_extended_frames += 1
                    if self.extended_frame >= self.falldown_extended_frame_threshold:
                        self.falldown = True
                        self.simulationEnd = True
                        print "fall down"
                        return False

                    preserveJoints = []
#                    preserveJoints = [lFoot, rFoot]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee]
#                    preserveJoints = [lFoot, rFoot, lKnee, rKnee, lUpLeg, rUpLeg]
                    stanceKnees = [self.rKnee] if curState==yba.GaitState.LSWING else [self.lKnee]
                    preserveJoints = [stanceFoots[0], stanceKnees[0], stanceLegs[0]]
   
                    diff = 3
                    motion_seg_orig.extend([motion_seg_orig[-1]])
                    motion_seg.extend(ymt.extendByIntegration_root(motion_seg, 1, diff))
                    
                    motion_stitch.extend(ymt.extendByIntegration_constant(motion_stitch, 1, preserveJoints, diff))
                    
                    self.extended[0] = True
        else:
            if frame == len(motion_seg)-1: lastFrame = True
                    
        if lastFrame:
            self.extended_frame = 0
            self.interval_simulated[self.segIndex][1] = frame
            self.interval_simulated.append([frame,None])
            if self.motionGen.getNthNextSegIdx(1) != None:
                print '%d (%d): end of %dth seg (%s, %s)'%(frame, frame-curInterval[1],self.segIndex, yba.GaitState.text[curState], curInterval)
                #if plot!=None: plot.addDataPoint('diff', frame, (frame-curInterval[1])*.01)
                
                if len(stanceFoots)>0 and len(swingFoots)>0:
#                    step_cur = controlModel.getJointPositionGlobal(swingFoots[0]) - controlModel.getJointPositionGlobal(stanceFoots[0])
#                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(swingFoots[0]) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    step_cur = controlModel.getJointPositionGlobal(0) - controlModel.getJointPositionGlobal(stanceFoots[0])
                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(0) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    
                    step_cur = mm.projectionOnPlane(step_cur, (1,0,0), (0,0,1))
                    step_tar = mm.projectionOnPlane(step_tar, (1,0,0), (0,0,1))
                    
                    step_cur_sag, step_cur_cor = mm.projectionOnVector2(step_cur, self.direction)
                    step_tar_sag, step_tar_cor = mm.projectionOnVector2(step_tar, self.direction)
                    
                    self.step_length_tar[0] = mm.length(step_tar_sag)
                    if np.inner(step_tar_sag, step_cur_sag) > 0:
                        self.step_length_cur[0] = mm.length(step_cur_sag)
                    else:
                        self.step_length_cur[0] = -mm.length(step_cur_sag)
                    
                    self.step_axis[0] = self.directionAxis
                    
#                    rd_vec1[0] = step_tar_sag
#                    rd_vecori1[0] = motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
#                    rd_vec2[0] = step_cur_sag
#                    rd_vecori2[0] = controlModel.getJointPositionGlobal(stanceFoots[0])

                self.segIndex += 1
                self.motionGen.goToNext()
                curSeg = self.motionGen.getCurMotionSeg()
                #self.stl_y_limit_num[0] = 0
                #self.stl_xz_limit_num[0] = 0
                
                del motion_seg_orig[frame+1:]
                motion_seg_orig.extend(ymb.getAttachedNextMotion(curSeg, motion_seg_orig[-1], False, False))
                
                del motion_seg[frame+1:]
                del motion_stitch[frame+1:]
                transitionLength = len(curSeg)-1

#                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, False))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, False))

                d = motion_seg[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, d, True, True))
                
                if self.NO_FOOT_SLIDING:
                    if self.motionGen.getNthNextSegIdx(2) == None:
                        Rl = motion_control[-1].getJointOrientationLocal(self.lUpLeg)
                        Rr = motion_control[-1].getJointOrientationLocal(self.rUpLeg)
                        Rlk = motion_control[-1].getJointOrientationLocal(self.lKnee)
                        Rrk = motion_control[-1].getJointOrientationLocal(self.rKnee)
                        Rlf = motion_control[-1].getJointOrientationLocal(self.lFoot)
                        Rrf = motion_control[-1].getJointOrientationLocal(self.rFoot)
                        for p in curSeg:
                            p.setJointOrientationLocal(self.lUpLeg, Rl, False)
                            p.setJointOrientationLocal(self.rUpLeg, Rr, False)
                            p.setJointOrientationLocal(self.lKnee, Rlk, False)
                            p.setJointOrientationLocal(self.rKnee, Rrk, False)
                            p.setJointOrientationLocal(self.lFoot, Rlf, False)
                            p.setJointOrientationLocal(self.rFoot, Rrf, False)
                            p.updateGlobalT()
                
                d = motion_control[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, d, transitionLength, self.stitch_func, True, True))
                
#                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, True))
#                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, True))
            else:
                print '%d (%d): end of %dth seg (%s, %s)'%(frame, frame-curInterval[1],self.segIndex, yba.GaitState.text[curState], curInterval)
                self.segIndex += 1
                self.simulationEnd = True
                motion_seg_orig.append(motion_seg_orig[-1])
                motion_seg.append(motion_seg[-1])
                motion_stitch.append(motion_control[-1])
        
        return True
        

    def simulateOnestep(self):
        
        if self.simulationEnd:
            return False

        self.curFrame = self.curFrame + 1

        self.modulateControlMotion()

        #=================================
        # simulation
        #=================================
        controlModel = self.controlModel
        vpWorld = self.vpWorld
        motion_simulated = self.motion_simulated
        frame = self.curFrame
        th_r = self.motion_control.getDOFPositions(frame)
        th = self.controlModel.getDOFPositions()
        dth_r = self.motion_control.getDOFVelocities(frame)
        dth = self.controlModel.getDOFVelocities()
        ddth_r = self.motion_control.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, self.Kt, self.Dt)

        CP = mm.v3(0.,0.,0.)
        F = mm.v3(0.,0.,0.)
        self.avg_dCM[0] = mm.v3(0.,0.,0.)
        
        # external force rendering info
        del self.rd_forces[:]; del self.rd_force_points[:]
        for fi in self.forceInfos:
            if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/self.frameTime):
                self.rd_forces.append(fi.force)
                self.rd_force_points.append(controlModel.getBodyPositionGlobal(fi.targetBody))
        for i in range(self.stepsPerFrame):
            self.bodyIDs, self.contactPositions, self.contactPositionLocals, self.contactForces = vpWorld.calcPenaltyForce(self.bodyIDsToCheck, self.mus, self.Ks, self.Ds)
            vpWorld.applyPenaltyForce(self.bodyIDs, self.contactPositionLocals, self.contactForces)
            
            # apply external force
            for fi in self.forceInfos:
                if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                    controlModel.applyBodyForceGlobal(fi.targetBody, fi.force)
            
            controlModel.setDOFAccelerations(ddth_des)
            controlModel.solveHybridDynamics()
#            if TORQUE_PLOT:
#                self.rhip_torques[frame] += mm.length(controlModel.getJointTorqueLocal(self.rUpLeg))
#                self.rknee_torques[frame] += mm.length(controlModel.getJointTorqueLocal(self.rKnee))
#                self.rankle_torques[frame] += mm.length(controlModel.getJointTorqueLocal(self.rFoot))
            
            self.rd_torques[:] = [controlModel.getJointTorqueLocal(i)/100. for i in range(self.skeleton.getJointNum())]
            self.rd_joint_positions[:] = controlModel.getJointPositionsGlobal()
            vpWorld.step()
#            yvu.align2D(controlModel)

            if len(self.contactForces) > 0:
                CP += yrp.getCP(self.contactPositions, self.contactForces)
                F += sum(self.contactForces)
            self.avg_dCM[0] += controlModel.getJointVelocityGlobal(0)
#            self.avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), self.bodyMasses, self.upperMass, self.uppers)
#            self.avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), self.bodyMasses, self.totalMass)

#            stanceFoots = self.motionGen.getCurSegInfo()['stanceFoots']
#            if len(stanceFoots)>0:
#                avg_stf_v[0] += controlModel.getJointVelocityGlobal(stanceFoots[0])
#                avg_stf_av[0] += controlModel.getJointAngVelocityGlobal(stanceFoots[0])
        
        CP /= self.stepsPerFrame
        F /= self.stepsPerFrame
        self.avg_dCM[0] /= self.stepsPerFrame
        
#        stanceFoots = self.motionGen.getCurSegInfo()['stanceFoots']
#        if len(stanceFoots)>0:
#            avg_stf_v[0] /= stepsPerFrame
#            avg_stf_av[0] /= stepsPerFrame
#            rd_vec1[0] = avg_stf_av[0]; rd_vec1[0][0] = 0.; rd_vec1[0][2] = 0.
#            rd_vecori1[0]= controlModel.getJointPositionGlobal(stanceFoots[0])

        #==================================
        # record simulation result
        motion_simulated.append(self.motion_control[frame].copy())
        motion_simulated[frame].setRootPos(controlModel.getJointPositionGlobal(0))
        motion_simulated[frame].setJointOrientationsLocal(controlModel.getJointOrientationsLocal())
        motion_simulated.goToFrame(frame)
        
        # rd_CP, rd_CMP
        # which one is better, prev CMreal or cur CMreal?
        CMreal = yrp.getCM(controlModel.getJointPositionsGlobal(), controlModel.getBodyMasses(), controlModel.getTotalMass())
        self.rd_CP[0] = CP
        self.rd_CMP[0] = (CMreal[0] - (F[0]/F[1])*CMreal[1], 0, CMreal[2] - (F[2]/F[1])*CMreal[1])

        #================================
        # fall down test
        if controlModel.getJointPositionGlobal(0)[1] < self.motion_seg[frame].getJointPositionGlobal(0)[1] * self.falldown_root_y_threshold:
            print "fall down    : the root position of control model is too low"
            self.falldown = True
            self.simulationEnd = True
            return False

        if controlModel.getJointPositionGlobal(0)[1] < controlModel.getJointPositionGlobal(self.lFoot)[1] and controlModel.getJointPositionGlobal(0)[1] < controlModel.getJointPositionGlobal(self.rFoot)[1]:
            print "fall down    : the root position of control model is lower than those of feet"
            self.falldown = True
            self.simulationEnd = True
            return False

        #=================================

        return self.editSegment()


    def simulateOneSeg(self):

        # for f in oneSeg:
        #   simulateOneStep

        # until self.segIndex increase
        curSegIndex = self.segIndex 
        while curSegIndex == self.segIndex:
            if self.simulateOnestep() == False:
                return False

        # rendering
        self.motionModel.update(self.motion_control[self.curFrame])
        return True

if __name__=='__main__':

    motionDir = '../Model/wd2/Motion/'
    motionFile = 'wd2_WalkForwardNormal00.bvh'
    #motionFile = 'wd2_WalkAzuma01.bvh'
    motionFile = 'wd2_WalkForwardVFast00.bvh'
    motionFilePath = motionDir + motionFile

    motion = yf.readBvhFile(motionFilePath)

    mcfgFilePath = motionDir + 'mcfg'
    mcfgFile = open(mcfgFilePath)
    mcfg = cPickle.load(mcfgFile)
    mcfgFile.close()

    #segFile = open(motionDir + 'wd2_WalkForwardNormal00.seg')
    #segFile = open(motionDir + 'wd2_WalkAzuma01.seg')
    #segInfos = cPickle.load(segFile)
    #segFile.close()

    simulator = TrackingSimulator(motion, mcfg)
    #simulator = TrackingSimulator(motion, mcfg, segInfos)

    viewer = ysv.SimpleViewer()
    viewer.initialize()
    viewer.setMaxFrame(1000)

    for i in range(8):
        if simulator.simulateOneSeg() == False:
            print "Fail: the model falls down"
            print simulator.interval_simulated
            print simulator.total_extended_frames
            print simulator.max_precontacted_frames
            break
        print simulator.interval_simulated
        print simulator.total_extended_frames
        print simulator.max_precontacted_frames

    viewer.doc.addRenderer('motion_simulated', yr.JointMotionRenderer(simulator.motion_simulated, (200, 0, 255), yr.LINK_BONE))
    viewer.doc.addRenderer('motion_control', yr.JointMotionRenderer(simulator.motion_control, (200,200,200), yr.LINK_BONE))
    #viewer.doc.addRenderer('motion_seg', yr.JointMotionRenderer(simulator.motion_seg, (200,200,200), yr.LINK_BONE))
    #viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(simulator.motion_stitch, (0,200,200), yr.LINK_BONE))

    
    def simulateCallback(frame):
        print frame
        simulator.motion_control.goToFrame(frame)
        simulator.motion_simulated.goToFrame(frame)
#        simulator.motion_seg.goToFrame(frame)
#        simulator.motion_stitch.goToFrame(frame)
    
    viewer.setSimulateCallback(simulateCallback)
    viewer.startTimer(simulator.frameTime)
    viewer.show()

    Fl.run()

