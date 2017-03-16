import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Motion.ysMotionUtil as ymu
import Util.ysPythonEx as ype

#===============================================================================
# Notice : 
# In this module, each joint DOF is assumed to be rotational DOF. 
# 1~3DOF rotational joint and 6DOF floating joint are supported.
#
# jacobian J
# rowNum : (number of effector dofs) * (effector number)
# colNum : number of joint dofs
#===============================================================================

# len(jointDOFs) : number of real joints
#    ex. jointDOFs = [3,3] : 2 ball joints 
# sum of all elements of jointDOFs : total DOF 
def makeEmptyJacobian(jointDOFs, effectorNum, applyOrientation = True):
    dof_per_effector = 3 if applyOrientation==False else 6
    rowNum = dof_per_effector * effectorNum
    colNum = 0
    for i in range(len(jointDOFs)):
        colNum += jointDOFs[i]
    return np.zeros((rowNum, colNum), float)

#===============================================================================
# # Treat one 3DOF joint as one 3DOF joint
#===============================================================================
# compute J that dx = J*dq
# Treat one 3DOF joint as one 3DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# linearFirst=True : generalized velocity = [ v^t w^t ]^t, False : [ w^t v^t ]^t
# effectorJointMasks : 
# ex. len(jointPositions) == 3, len(effectorPositions) == 2
#     effectorJointMasks == [[1,1,1], [1,1,1]] : every effector affected by every joint
#     effectorJointMasks == [[1,0,0], [1,1,0]] : 1st effector affected by only 1st joint, 2nd effector affected by 1st & 2nd joints
# 
# (jointDOFs[0]==6) means joint[0] is 6DOF floating joint, jointAxeses[0] should be [(1,0,0), (0,1,0), (0,0,1), R^t[0], R^t[1], R^t[2]]
def computeJacobian2(J, jointDOFs, jointPositions, jointAxeses, effectorPositions, effectorJointMasks=None, linearFirst=True):
    rowNum, colNum = J.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6

    for e in range(len(effectorPositions)):
        col = 0
        
        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            for d in range(jointDOF_jth_joint):
                if effectorJointMasks==None or effectorJointMasks[e][j]:
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        instanteneousAngVelocity_colth_dof = axis_colth_dof 
                        instanteneousVelocity_colth_dof = np.cross(axis_colth_dof, effectorPositions[e] - jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngVelocity_colth_dof = (0.,0.,0.) 
                        instanteneousVelocity_colth_dof = axis_colth_dof
                else:
                    instanteneousAngVelocity_colth_dof = (0.,0.,0.) 
                    instanteneousVelocity_colth_dof = (0.,0.,0.)
        
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousAngVelocity_colth_dof[i]
                        else:
                            J[e*dof_per_effector + i, col] = instanteneousAngVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousVelocity_colth_dof[i]
                    else:
                        J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                        
                col += 1

    
# compute dJ that ddx = dJ*dq + J*ddq
# Treat one 3DOF joint as one 3DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
#
# jointDOFs[0] = 6 means joint[0] is 6DOF floating joint, jointAxeses[0] should be [(1,0,0), (0,1,0), (0,0,1), R^t[0], R^t[1], R^t[2]]
def computeJacobianDerivative2(dJ, jointDOFs, jointPositions, jointAxeses, linkAngVels, effectorPositions, effectorJointMasks, internalJointsOnly=False, linearFirst=True):
    rowNum, colNum = dJ.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for e in range(len(effectorPositions)):
        col = 0

        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            if internalJointsOnly:
                parentLinkAngVel_jth_joint = linkAngVels[j]
            else:
                parentLinkAngVel_jth_joint = linkAngVels[j-1] if j>0 else (0,0,0)
            
            for d in range(jointDOF_jth_joint):
                if effectorJointMasks==None or effectorJointMasks[e][j]:
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        # dZ(i) = w(i)<cross>Z(i)
                        instanteneousAngAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                        
                        # Z(i)<cross>(<sum j=i to n>w(j+1)<cross>P(j+1, j)) + (w(i)<cross>Z(i))<cross>P(n+1, i)
                        instanteneousAcceleration_colth_dof = np.cross(axis_colth_dof, get_dP_effector_from_joint2(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
                                                              + np.cross(np.cross(parentLinkAngVel_jth_joint, axis_colth_dof), effectorPositions[e]-jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngAcceleration_colth_dof = (0.,0.,0.) 
                        instanteneousAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                else:
                    instanteneousAngAcceleration_colth_dof = (0.,0.,0.) 
                    instanteneousAcceleration_colth_dof = (0.,0.,0.)
                    
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAngAcceleration_colth_dof[i]
                        else:
                            dJ[e*dof_per_effector + i, col] = instanteneousAngAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAcceleration_colth_dof[i]
                    else:
                        dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                        
                col += 1    

# P_effector_from_joint = effectorPositions[e]-jointPositions[j]
# dP_effector_from_joint = time derivative of P_effector_from_joint
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
#
# <sum j=i to n>w(j+1)<cross>P(j+1, j)
def get_dP_effector_from_joint2(jointIndex, jointPositions, linkAngVels, effectorJointMask, effectorPosition, internalJointsOnly=False):
    jointIndexes = jointMask_2_jointIndexesDownward(effectorJointMask, jointIndex)

    dP = mm.v3(0,0,0)
    for i in range(len(jointIndexes)):
        index = jointIndexes[i]
        jointPosition = jointPositions[index]
        childPosition = jointPositions[jointIndexes[i+1]] if i < len(jointIndexes)-1 else effectorPosition
        linkAngVel = linkAngVels[index] if internalJointsOnly==False else linkAngVels[index+1]
        dP += np.cross(linkAngVel, childPosition - jointPosition)
    return dP 


#===============================================================================
# # Treat each DOF of 3DOF joint as one 1DOF joint    
#===============================================================================
# compute J that dx = J*dq
# Treat each DOF of 3DOF joint as one 1DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# linearFirst=True : generalized velocity = [ v^t w^t ]^t, False : [ w^t v^t ]^t
# effectorJointMasks : 
# ex. len(jointPositions) == 3
#     len(effectorPositions) == 2
#     effectorJointMasks == [[1,1,1], [1,1,1]] : every effector affected by every joint
#     effectorJointMasks == [[1,0,0], [1,1,0]] : 1st effector affected by only 1st joint, 2nd effector affected by 1st & 2nd joints 
def computeJacobian(J, jointPositions, jointAxes, effectorPositions, effectorJointMasks=None, linearFirst=True):
    rowNum, colNum = J.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for j in range(colNum):
        instanteneousAngVelocity_for_j_th_joint = jointAxes[j]
        
        for e in range(len(effectorPositions)):
            if effectorJointMasks==None or effectorJointMasks[e][j]:
                instanteneousVelocity_for_j_th_joint = np.cross(jointAxes[j], effectorPositions[e]-jointPositions[j])
                
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            J[e*dof_per_effector + i, j] = instanteneousVelocity_for_j_th_joint[i]
                            J[e*dof_per_effector + 3 + i, j] = instanteneousAngVelocity_for_j_th_joint[i]
                            
                            # don't affect to angular velocity
#                            J[e*dof_per_effector + 3 + i, j] = 0.
                        else:
                            # don't affect to angular velocity
#                            J[e*dof_per_effector + i, j] = jointAxes[j][i]

                            J[e*dof_per_effector + i, j] = instanteneousAngVelocity_for_j_th_joint[i]
                            J[e*dof_per_effector + 3 + i, j] = instanteneousVelocity_for_j_th_joint[i]
                    else:
                        J[e*dof_per_effector + i, j] = instanteneousVelocity_for_j_th_joint[i]
            else:
                for i in range(dof_per_effector):
                    J[e*dof_per_effector + i, j] = 0.
                        
# compute dJ that ddx = dJ*dq + J*ddq
# Treat each DOF of 3DOF joint as one 1DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
def computeJacobianDerivative(dJ, jointPositions, jointAxes, linkAngVels, effectorPositions, effectorJointMasks, internalJointsOnly=False, linearFirst=True):
    rowNum, colNum = dJ.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for j in range(colNum):
        if not internalJointsOnly:
            if j>=0 and j<=2:
                parentLinkAngVel = mm.v3(0,0,0)  
            else:
                parentLinkAngVel = linkAngVels[j-3]
        else:
            parentLinkAngVel = linkAngVels[j]

        # dZ(i) = w(i)<cross>Z(i)
        instanteneousAngAcceleration_for_j_th_joint = np.cross(parentLinkAngVel, jointAxes[j])
        
        for e in range(len(effectorPositions)):
            if effectorJointMasks[e][j]:
                instanteneousAcceleration_for_j_th_joint = np.cross(jointAxes[j], get_dP_effector_from_joint(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
                                                        + np.cross(np.cross(linkAngVels[j], jointAxes[j]), effectorPositions[e]-jointPositions[j])
                 
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            dJ[e*dof_per_effector + i, j] = instanteneousAcceleration_for_j_th_joint[i]
                            dJ[e*dof_per_effector + 3 + i, j] = instanteneousAngAcceleration_for_j_th_joint[i]
                        else:
                            dJ[e*dof_per_effector + i, j] = instanteneousAngAcceleration_for_j_th_joint[i]
                            dJ[e*dof_per_effector + 3 + i, j] = instanteneousAcceleration_for_j_th_joint[i]
                    else:
                        dJ[e*dof_per_effector + i, j] = instanteneousAcceleration_for_j_th_joint[i]
            else:
                for i in range(dof_per_effector):
                    dJ[e*dof_per_effector + i, j] = 0.

# P_effector_from_joint = effectorPositions[e]-jointPositions[j]
# dP_effector_from_joint = time derivative of P_effector_from_joint
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
#
# <sum j=i to n>w(j+1)<cross>P(j+1, j)
def get_dP_effector_from_joint(jointIndex, jointPositions, linkAngVels, effectorJointMask, effectorPosition, internalJointsOnly=False):
    jointIndexes = jointMask_2_jointIndexesDownward(effectorJointMask, jointIndex)

    dP = mm.v3(0,0,0)
    for i in range(len(jointIndexes)):
        index = jointIndexes[i]
        jointPosition = jointPositions[index]
        childPosition = jointPositions[jointIndexes[i+1]] if i < len(jointIndexes)-1 else effectorPosition
        linkAngVel = linkAngVels[index] if internalJointsOnly==False else linkAngVels[index+3]
        dP += np.cross(linkAngVel, childPosition - jointPosition)
    return dP 


#===============================================================================
# utility function
#===============================================================================
def jointMask_2_jointIndexesDownward(effectorJointMask, startJointIndex=None):
    if startJointIndex==None:
        return [i for i in range(len(effectorJointMask)) if effectorJointMask[i]==1]
    else:
        return [i for i in range(startJointIndex, len(effectorJointMask)) if effectorJointMask[i]==1]

def getRepeatedEffectorJointMasks(jointDOFs, effectorJointMasks):
    totalDOF = 0
    for i in range(len(jointDOFs)):
        totalDOF += jointDOFs[i]
    
    repeatedMasks = [None]*len(effectorJointMasks)
    for i in range(len(effectorJointMasks)):
        repeated_per_joint = [None]*totalDOF
        ype.repeatListElements(effectorJointMasks[i], repeated_per_joint, jointDOFs)
        repeatedMasks[i] = repeated_per_joint
    
    return repeatedMasks

# linkNum == 3, jointNum == 3
# return mask == [[1,1,1], [1,1,1]] : every link affected by every joint
# return mask == [[1,0,0], [1,1,0]] : link0 affected by only joint0, link1 affected by joint0, joint1 
def getLinkJointMask(skeleton, linkIndex):
    jointNum = skeleton.getJointNum()
    mask = [0]*jointNum
    
    # get parent joints of joint at linkIndex
    affectingJointIndexes = ymu.getParentJointIndexes(skeleton, linkIndex)
    
    # append body own index
    affectingJointIndexes.append(linkIndex)
    
    # fill masks
    for jointIndex in affectingJointIndexes:
        mask[jointIndex] = 1
        
    return mask
    
# linkNum == 3, jointNum == 3
# return mask == [[1,1], [1,1]] : every link affected by every joint
# return mask == [[1,0], [1,1]] : link0 affected by only joint1(position 0 in internal joint indexes), link1 affected by joint1, joint2 
def getLinkInternalJointMask(skeleton, linkIndex):
    jointNum = skeleton.getInternalJointNum()
    mask = [0]*jointNum
    
    # get parent joints of joint at linkIndex
    affectingJointIndexes = ymu.getParentJointIndexes(skeleton, linkIndex)
    
    # append body own index
    affectingJointIndexes.append(linkIndex)
    
    # fill masks
    for jointIndex in affectingJointIndexes:
        if jointIndex!=0:   # don't mark at root joint
            mask[jointIndex-1] = 1
        
    return mask

def getAllLinkJointMasks(skeleton):
    linkNum = skeleton.getLinkNum()
    masks = [None]*linkNum
    for i in range(linkNum):
        masks[i] = getLinkJointMask(skeleton, i)
    return masks
    
def getAllLinkInternalJointMasks(skeleton):
    linkNum = skeleton.getLinkNum()
    masks = [None]*linkNum
    for i in range(linkNum):
        masks[i] = getLinkInternalJointMask(skeleton, i)
    return masks
    

if __name__=='__main__':
    import psyco; psyco.full()
    import copy, math
    from fltk import *
    import Resource.ysMotionLoader as yf
    import Motion.ysMotion as ym
    import Motion.ysMotionUtil as ymu
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Util.ysPythonEx as ype
    import Simulator.csVpModel as cvm
    import Simulator.csVpWorld as cvw
    import Simulator.ysPhysConfig as ypc
    import Renderer.csVpRenderer as cvr
    

    JOINT_DOF = 3
#    JOINT_DOF = 1
    
    ORIENTATION = True
#    ORIENTATION = False
    
    LINEAR_FIRST = True
#    LINEAR_FIRST = False

    COMPUTE_JACOBIAN2 = True
#    COMPUTE_JACOBIAN2 = False
    

    def solve(J, jointDOFs, jointPositions, jointAxes, targetPositions, inoutPosture, effectorNames):
        iterationLimit = 200
        effectorIndex = inoutPosture.skeleton.getElementIndex(effectorNames[0])
        positionThreshold = .1
        beta = .01
        
        iterNum = 0
        positionDiff = 0.
        
        while True:
#            for i in range(inoutPosture.skeleton.getElementNum()):
#                R = inoutPosture.getGlobalR(i)
#                jointAxes[3*i]   = np.transpose(np.take(R, (0,), 1))[0]
#                jointAxes[3*i+1] = np.transpose(np.take(R, (1,), 1))[0]
#                jointAxes[3*i+2] = np.transpose(np.take(R, (2,), 1))[0]
#            print 'j', jointAxes[0:3]
            
            currentPositions = [inoutPosture.getPosition(effectorIndex)]
            
            if ORIENTATION == False:
                forceVec = targetPositions[0] - currentPositions[0]
            else:
                a = targetPositions[0] - currentPositions[0]
                if LINEAR_FIRST:
                    forceVec = np.array([a[0],a[1],a[2],0,0,0],float)
                else:
                    forceVec = np.array([0,0,0,a[0],a[1],a[2]],float)

            if COMPUTE_JACOBIAN2:
                computeJacobian2(J, jointDOFs, jointPositions, jointAxes, targetPositions, None, LINEAR_FIRST)
            else:
                computeJacobian(J, jointPositions, jointAxes, targetPositions, None, LINEAR_FIRST)
            
            deltaThetas = np.dot(np.transpose(J), forceVec)*beta
            applyThetas(inoutPosture, jointAxes, deltaThetas)
            
            iterNum += 1
            if iterNum > iterationLimit:
                print 'iter'
                break;
            
            positionDiff = 0.
            for i in range(len(targetPositions)):
                positionDiff += mm.length(targetPositions[i] - inoutPosture.getPosition( effectorIndex) )
            if positionDiff < positionThreshold:
                print 'converge'
                break;
            
    def applyThetas(inoutPosture, axes, deltaThetas):
        if COMPUTE_JACOBIAN2:
            if JOINT_DOF == 1:
                for i in range(inoutPosture.skeleton.getElementNum()):
                    inoutPosture.mulGlobalR(i, mm.exp(axes[i][0], deltaThetas[i]))
                inoutPosture.updateGlobalT()
            elif JOINT_DOF == 3:
                cnt = 0
                for i in range(len(axes)):
                    for j in range(len(axes[i])):
                        inoutPosture.mulGlobalR(i, mm.exp(axes[i][j], deltaThetas[cnt]))
                        inoutPosture.updateGlobalT(i)
                        cnt += 1
        else:
            if JOINT_DOF == 1:
                for i in range(inoutPosture.skeleton.getElementNum()):
                    inoutPosture.mulGlobalR(i, mm.exp(axes[i], deltaThetas[i]))
                inoutPosture.updateGlobalT()
            elif JOINT_DOF == 3:
                for i in range(len(axes)):
                    inoutPosture.mulGlobalR(i/3, mm.exp(axes[i], deltaThetas[i]))
                    inoutPosture.updateGlobalT(i/3)
        

    def test_IK():
        np.set_printoptions(precision=4, linewidth=200)
                
#        bvhFilePath = '../samples/block_3_rotate.bvh'
#        bvhFilePath = '../samples/chain_10.bvh'
        bvhFilePath = '../samples/chain_6.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        print motion[0].skeleton
        
        origPosture = ym.Motion([motion[0]])
        skeleton = motion[0].skeleton
        newPosture = copy.deepcopy(origPosture)
        
#        effectorNames = ['body3_Effector']
#        effectorNames = ['body1_Effector']
#        effectorNames = ['link10_Effector']
#        effectorNames = ['link4']
        effectorNames = ['link6_Effector']
#        targetPositions = [(.5,.5,.2)]
        targetPositions = [(0, 0.5, 2.5)]
        
        jointNum = skeleton.getElementNum()

        if JOINT_DOF == 1:
            jointDOFs = [1]*motion[0].skeleton.getElementNum()
            totalDOF = motion[0].skeleton.getElementNum()*1
        elif JOINT_DOF == 3:
            jointDOFs = [3]*motion[0].skeleton.getElementNum()
            totalDOF = motion[0].skeleton.getElementNum()*3

        if ORIENTATION:    
            J = makeEmptyJacobian(jointDOFs, len(targetPositions))
        else:
            J = makeEmptyJacobian(jointDOFs, len(targetPositions), False)
        
        if COMPUTE_JACOBIAN2:
            jointPositions = origPosture[0].getPositions()
            jointAxes = [[mm.v3(0,1,0)]]*jointNum if JOINT_DOF==1 else [[mm.v3(1,0,0),mm.v3(0,1,0),mm.v3(0,0,1)]]*jointNum
        else:
            jointPositions = [None]*totalDOF
            ype.repeatListElements(origPosture[0].getPositions(), jointPositions, jointDOFs)
            jointAxes = [mm.v3(0,1,0)]*totalDOF if JOINT_DOF==1 else [mm.v3(1,0,0),mm.v3(0,1,0),mm.v3(0,0,1)]*(totalDOF/3)
        
        solve(J, jointDOFs, jointPositions, jointAxes, targetPositions, newPosture[0], effectorNames)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('targetPosition', yr.PointsRenderer(targetPositions))
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
    
    def test_computeJacobian():
        np.set_printoptions(precision=4, linewidth=200)

#        bvhFilePath = '../samples/chain_2_rotate_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_freely_expt_root.bvh'
        bvhFilePath = '../samples/chain_3_rotate_freely.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_freely_move.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        skeleton = motion[0].skeleton
        print skeleton
        
        internalJointsOnly = False
#        internalJointsOnly = True

        if internalJointsOnly:
            jointDOFs = [3]*skeleton.getInternalJointNum()
            totalDOF = skeleton.getInternalJointNum()*3
        else:
            jointDOFs = [3]*(skeleton.getJointNum())
            totalDOF = (skeleton.getJointNum())*3
            
        jointPositions_ext = [None]*totalDOF
        
        J = makeEmptyJacobian(jointDOFs, skeleton.getLinkNum())
        dJ = J.copy()
        
        if internalJointsOnly:
            linkJointMasks = getAllLinkInternalJointMasks(skeleton)
        else:
            linkJointMasks = getAllLinkJointMasks(skeleton)
        linkJointMasks_ext = getRepeatedEffectorJointMasks(jointDOFs, linkJointMasks)
        
        dth_flat = ype.makeFlatList(totalDOF)
        ddth_flat = ype.makeFlatList(totalDOF)
        v_sol = ype.makeNestedList([6]*skeleton.getLinkNum())
        a_sol = ype.makeNestedList([6]*skeleton.getLinkNum())
        
        p = []

        v_original = []
        av_original = []
        a_original = []
        aa_original = []

        v_jacobian = []
        av_jacobian = []
        a_jacobian = []
        aa_jacobian = []
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)

        viewer.doc.addRenderer('v_original', yr.VectorsRenderer(v_original, p, (255,0,0)))
        viewer.doc.addRenderer('v_jacobian', yr.VectorsRenderer(v_jacobian, p, (0,255,0)))

        viewer.doc.addRenderer('av_original', yr.VectorsRenderer(av_original, p, (255,0,0)))
        viewer.doc.addRenderer('av_jacobian', yr.VectorsRenderer(av_jacobian, p, (0,255,0)))
        
#        viewer.doc.addRenderer('a_original', yr.VectorsRenderer(a_original, p, (255,0,0)))
#        viewer.doc.addRenderer('a_jacobian', yr.VectorsRenderer(a_jacobian, p, (0,255,0)))
#
#        viewer.doc.addRenderer('aa_original', yr.VectorsRenderer(aa_original, p, (255,0,0)))
#        viewer.doc.addRenderer('aa_jacobian', yr.VectorsRenderer(aa_jacobian, p, (0,255,0)))

        
        def preFrameCallback(frame):
            if internalJointsOnly:
                jointPositions = motion.getInternalJointPositionsGlobal(frame)
            else:
                jointPositions = motion.getJointPositionsGlobal(frame)
            ype.repeatListElements(jointPositions, jointPositions_ext, jointDOFs)
            
            if internalJointsOnly:
                Rs = motion.getInternalJointOrientationsGlobal(frame)
            else:
                Rs = motion.getJointOrientationsGlobal(frame)
            
            jointAxes = np.concatenate(Rs, 1).transpose()

            linkPositions = motion.getJointPositionsGlobal(frame)
            linkVelocities = motion.getJointVelocitiesGlobal(frame)
            linkAccelerations = motion.getJointAccelerationsGlobal(frame)
            linkAngVelocities = motion.getJointAngVelocitiesGlobal(frame)
            linkAngAccelerations = motion.getJointAngAccelerationsGlobal(frame)
            
            linkAngVels_ext = [None]*(3*skeleton.getLinkNum())
            ype.repeatListElements(linkAngVelocities, linkAngVels_ext, [3]*skeleton.getLinkNum())
            
            computeJacobian(J, jointPositions_ext, jointAxes, linkPositions, linkJointMasks_ext)
            computeJacobianDerivative(dJ, jointPositions_ext, jointAxes, linkAngVels_ext, linkPositions, linkJointMasks_ext, internalJointsOnly)
            
            if internalJointsOnly:
                dth = motion.getInternalJointAngVelocitiesLocal(frame)
                ddth = motion.getInternalJointAngAccelerationsLocal(frame)
            else:
                dth = motion.getJointAngVelocitiesLocal(frame)
                ddth = motion.getJointAngAccelerationsLocal(frame)
            ype.flatten(dth, dth_flat)
            ype.flatten(ddth, ddth_flat)
            
            # dx = J*dq
            v_flat = np.dot(J, dth_flat)
            ype.nested(v_flat, v_sol)
            
            # ddx = dJ*dq + J*ddq
            a_flat = np.dot(dJ, dth_flat)
            
            a_flat2 = copy.copy(a_flat)
            a_flat2 = np.dot(J, ddth_flat)
            a_flat = [a_flat[i] + a_flat2[i] for i in range(len(a_flat))]

            ype.nested(a_flat, a_sol)
            
                        
            p[:] = linkPositions
            v_original[:] = linkVelocities
            av_original[:] = linkAngVelocities
            a_original[:] = linkAccelerations
            aa_original[:] = linkAngAccelerations
            
            v_jacobian[:] = [v_sol[i][0:3] for i in range(len(v_sol))]
            av_jacobian[:] = [v_sol[i][3:6] for i in range(len(v_sol))]
            a_jacobian[:] = [a_sol[i][0:3] for i in range(len(a_sol))]
            aa_jacobian[:] = [a_sol[i][3:6] for i in range(len(a_sol))]

        viewer.setPreFrameCallback(preFrameCallback)

        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()

    def test_computeJacobian2():
        np.set_printoptions(precision=4, linewidth=200)

#        bvhFilePath = '../samples/chain_2_rotate_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_freely_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_freely.bvh'
        bvhFilePath = '../samples/chain_3_rotate_freely_move.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        skeleton = motion[0].skeleton
        print skeleton
        
        DOFs = skeleton.getDOFs()
        totalDOF = skeleton.getTotalDOF()
        
        J = makeEmptyJacobian(DOFs, skeleton.getLinkNum())
        dJ = J.copy()
        
        linkJointMasks = getAllLinkJointMasks(skeleton)
        
        dth_flat = ype.makeFlatList(totalDOF)
        ddth_flat = ype.makeFlatList(totalDOF)
        v_sol = ype.makeNestedList([6]*skeleton.getLinkNum())
        a_sol = ype.makeNestedList([6]*skeleton.getLinkNum())
        
        p = []

        v_original = []
        av_original = []
        a_original = []
        aa_original = []

        v_jacobian = []
        av_jacobian = []
        a_jacobian = []
        aa_jacobian = []
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)

#        viewer.doc.addRenderer('v_original', yr.VectorsRenderer(v_original, p, (255,0,0)))
#        viewer.doc.addRenderer('v_jacobian', yr.VectorsRenderer(v_jacobian, p, (0,255,0)))

#        viewer.doc.addRenderer('av_original', yr.VectorsRenderer(av_original, p, (255,0,0)))
#        viewer.doc.addRenderer('av_jacobian', yr.VectorsRenderer(av_jacobian, p, (0,255,0)))
        
#        viewer.doc.addRenderer('a_original', yr.VectorsRenderer(a_original, p, (255,0,0)))
#        viewer.doc.addRenderer('a_jacobian', yr.VectorsRenderer(a_jacobian, p, (0,255,0)))
#
        viewer.doc.addRenderer('aa_original', yr.VectorsRenderer(aa_original, p, (255,0,0)))
        viewer.doc.addRenderer('aa_jacobian', yr.VectorsRenderer(aa_jacobian, p, (0,255,0)))

        
        def preFrameCallback(frame):
            jointPositions = motion.getJointPositionsGlobal(frame)
            jointAxeses = motion.getDOFAxeses(frame)

            linkPositions = motion.getJointPositionsGlobal(frame)
            linkVelocities = motion.getJointVelocitiesGlobal(frame)
            linkAccelerations = motion.getJointAccelerationsGlobal(frame)
            linkAngVelocities = motion.getJointAngVelocitiesGlobal(frame)
            linkAngAccelerations = motion.getJointAngAccelerationsGlobal(frame)
            
            computeJacobian2(J, DOFs, jointPositions, jointAxeses, linkPositions, linkJointMasks)
            computeJacobianDerivative2(dJ, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, linkJointMasks)

            dth = motion.getDOFVelocities(frame) 
            ddth = motion.getDOFAccelerations(frame)
            ype.flatten(dth, dth_flat)
            ype.flatten(ddth, ddth_flat)
            
            # dx = J*dq
            v_flat = np.dot(J, dth_flat)
            ype.nested(v_flat, v_sol)
            
            # ddx = dJ*dq + J*ddq
            a_flat = np.dot(dJ, dth_flat)
            
            a_flat2 = copy.copy(a_flat)
            a_flat2 = np.dot(J, ddth_flat)
            a_flat = [a_flat[i] + a_flat2[i] for i in range(len(a_flat))]

            ype.nested(a_flat, a_sol)
            
                        
            p[:] = linkPositions
            v_original[:] = linkVelocities
            av_original[:] = linkAngVelocities
            a_original[:] = linkAccelerations
            aa_original[:] = linkAngAccelerations
            
            v_jacobian[:] = [v_sol[i][0:3] for i in range(len(v_sol))]
            av_jacobian[:] = [v_sol[i][3:6] for i in range(len(v_sol))]
            a_jacobian[:] = [a_sol[i][0:3] for i in range(len(a_sol))]
            aa_jacobian[:] = [a_sol[i][3:6] for i in range(len(a_sol))]

        viewer.setPreFrameCallback(preFrameCallback)

        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()

    def test_velocity_relation_simple():
        #===============================================================================
        # setting
        #===============================================================================
#        frame = mm.I_SE3()
        frame = mm.Rp2T(mm.exp(mm.v3(1,1,0), math.pi/8.), (0,0,0))
        R, o = mm.T2Rp(frame)

#        arm_local = mm.v3(1,.5,.1)
        arm_local = mm.v3(1,0,0)
        av_local = mm.v3(1,1,1)
        
        #===============================================================================
        # global linear velocity computed by global angular velocity
        #===============================================================================
        arm_global = np.dot(R, arm_local)
        av_global = np.dot(R, av_local)
        v_global = np.cross(av_global, arm_global)
        
        #===============================================================================
        # global linear velocity computed by jacobian and local angular velocity
        #===============================================================================
        axes = R.transpose()
#        J = makeEmptyJacobian([3], 1, False)
        J = makeEmptyJacobian([3], 1)
        
#        computeJacobian(J, [o,o,o], axes, [arm_global], None)
        computeJacobian2(J, [3], [o], [axes], [arm_global], None)
        
        gen_v = np.dot(J, av_local)
        av_global_j = gen_v[3:6]
        v_global_j = gen_v[0:3]
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('frame', yr.FramesRenderer([frame], (0,0,255)))
        viewer.doc.addRenderer('av_global', yr.VectorsRenderer([av_global], [o], (255,100,100)))
        viewer.doc.addRenderer('v_global', yr.VectorsRenderer([v_global], [arm_global+o], (255,0,0)))
        viewer.doc.addRenderer('av_global_j', yr.VectorsRenderer([av_global_j], [o], (100,255,100)))
        viewer.doc.addRenderer('v_global_j', yr.VectorsRenderer([v_global_j], [arm_global+o], (0,255,0)))
        
        viewer.show()
        Fl.run()
    
    def test_effector_joint_masks_funcs():
        jointDOFs = [3]
        effectorJointMasks = [[0],[0],[1]]
        repeatedEffectorJointMasks = getRepeatedEffectorJointMasks(jointDOFs, effectorJointMasks)
        print repeatedEffectorJointMasks
        print
        
#        bvhFilePath = '../samples/chain_2_rotate.bvh'
#        bvhFilePath = '../samples/chain_3_rotate.bvh'
#        bvhFilePath = '../samples/block_3_rotate.bvh'
#        bvhFilePath = '../samples/block_tree_rotate.bvh'
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        skeleton = motion[0].skeleton
        print skeleton
        print
        
        print 'getAllParentIndexes'
        indexes = ymu.getAllParentIndexes(skeleton)
        print indexes
        
        for i in range(len(indexes)):
            print 'parents of', skeleton.getElementName(i), ':',
            for parent in indexes[i]:
                print skeleton.getElementName(parent),
            print
        print
            
        print 'getAllLinkJointMasks'
        masks = getAllLinkJointMasks(skeleton)
        print masks
        for i in range(len(masks)):
            print 'affecting joint of', skeleton.getJointName(i), ':',
            for j in range(len(masks[i])):
                if masks[i][j]:
                    print skeleton.getJointName(j),
            print
        print
        
        print 'getAllLinkInternalJointMasks'
        masks = getAllLinkInternalJointMasks(skeleton)
        print masks
        for i in range(len(masks)):
            print 'affecting joint of', skeleton.getJointName(i), ':',
            for j in range(len(masks[i])):
                if masks[i][j]:
                    print skeleton.getInternalJointName(j),
            print
        print


        jointIndex = 10
        
        print 'getLinkJointMask'
        masks2 = getLinkJointMask(skeleton, jointIndex)
        print masks2
        print 'affecting joint of', skeleton.getJointName(jointIndex), ':',
        for j in range(len(masks2)):
            if masks2[j]:
                print skeleton.getJointName(j),
        print
        print 'jointMask_2_jointIndexesDownward'
        indexes = jointMask_2_jointIndexesDownward(masks2, None) 
        print indexes
        for i in indexes:
            print skeleton.getJointName(i),
        print
        print

        print 'getLinkInternalJointMask'
        masks2 = getLinkInternalJointMask(skeleton, jointIndex)
        print masks2
        print 'affecting joint of', skeleton.getJointName(jointIndex), ':',
        for j in range(len(masks2)):
            if masks2[j]:
                print skeleton.getInternalJointName(j),
        print
        print 'jointMask_2_jointIndexesDownward'
        indexes = jointMask_2_jointIndexesDownward(masks2, None) 
        print indexes
        for i in indexes:
            print skeleton.getInternalJointName(i),
        print

        
    def test_get_dP_effector_from_joint():
#        bvhFilePath = '../samples/chain_2_rotate_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
        bvhFilePath = '../samples/chain_3_rotate_freely.bvh'
#        bvhFilePath = '../samples/chain_3_rotate_freely_expt_root.bvh'
#        bvhFilePath = '../samples/chain_6_rotate_expt_root.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        skeleton = motion[0].skeleton
        print skeleton
        
#        internalJointsOnly = True
        internalJointsOnly = False
        
#        MODE = 0    # treat one 3DOF joint as one 3DOF joint 
        MODE = 1    # split 3DOF joint into 3 1DOF joints
        
        if internalJointsOnly:
            linkJointMasks = getAllLinkInternalJointMasks(skeleton)
        else:
            linkJointMasks = getAllLinkJointMasks(skeleton)
            
        if MODE==1:
            linkJointMasks = getRepeatedEffectorJointMasks([3]*skeleton.getJointNum(), linkJointMasks)
                    
        p = []

        P = []
        dP = []
        dP_ref = []
        
        viewer = ysv.SimpleViewer()
#        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)

        viewer.doc.addRenderer('P', yr.VectorsRenderer(P, p, (0,255,0)))
        viewer.doc.addRenderer('dP', yr.VectorsRenderer(dP, p, (0,255,0)))
        viewer.doc.addRenderer('dP_ref', yr.VectorsRenderer(dP_ref, p, (255,0,0)))
        
        
        def preFrameCallback(frame):
            if internalJointsOnly:
                jointPositions = motion.getInternalJointPositionsGlobal(frame)
            else:
                jointPositions = motion.getJointPositionsGlobal(frame)
            
            linkPositions = motion.getJointPositionsGlobal(frame)
            linkAngVelocities = motion.getJointAngVelocitiesGlobal(frame)
            
            joint = 0
            effector = 2

            if MODE==0:
                dp = get_dP_effector_from_joint2(joint, jointPositions, linkAngVelocities, linkJointMasks[effector], linkPositions[effector], internalJointsOnly)
                print joint, dp
            elif MODE==1:
                joint_ext = joint*3
                
                jointPositions_ext = [None]*(3*skeleton.getJointNum())
                ype.repeatListElements(jointPositions, jointPositions_ext, [3]*skeleton.getJointNum())
                linkAngVelocities_ext = [0]*(3*skeleton.getLinkNum())
                ype.repeatListElements(linkAngVelocities, linkAngVelocities_ext, [3]*skeleton.getLinkNum())

                dp = get_dP_effector_from_joint(joint_ext, jointPositions_ext, linkAngVelocities_ext, linkJointMasks[effector], linkPositions[effector], internalJointsOnly)
                print joint_ext, dp
            
            
            p[:] = [jointPositions[joint]]
            if len(P)>0: dP_ref[:] = [(linkPositions[effector] - jointPositions[joint] - P[0])*30.]
            P[:] = [linkPositions[effector] - jointPositions[joint]]
            dP[:] = [dp]

        viewer.setPreFrameCallback(preFrameCallback)

        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()        

    def test_computeJacobian2_VpModel():
        np.set_printoptions(precision=4, linewidth=200)
    
#        bvhFilePath = '../samples/chain_1.bvh'
    #        bvhFilePath = '../samples/chain_2_rotate_expt_root.bvh'
    #        bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
        bvhFilePath = '../samples/chain_3_rotate.bvh'
    #        bvhFilePath = '../samples/chain_3_rotate_freely_expt_root.bvh'
    #        bvhFilePath = '../samples/chain_3_rotate_freely.bvh'
    #    bvhFilePath = '../samples/chain_3_rotate_freely_move.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .8
        for i in range(motion[0].skeleton.getElementNum()):
            mcfg.addNode(motion[0].skeleton.getElementName(i))
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = -1.
        wcfg.gravity = (0,0,0)
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        vpWorld = cvw.VpWorld(wcfg)
        cchar = cvm.VpControlModel(vpWorld, motion[0], mcfg)
        vpWorld.initialize()
        print cchar
    
        DOFs = cchar.getDOFs()
        totalDOF = cchar.getTotalDOF()
        
        J = makeEmptyJacobian(DOFs, cchar.getBodyNum())
        dJ = J.copy()
        
        linkJointMasks = getAllLinkJointMasks(motion[0].skeleton)
        
        dth_flat = ype.makeFlatList(totalDOF)
        ddth_flat = ype.makeFlatList(totalDOF)
        v_sol = ype.makeNestedList([6]*cchar.getBodyNum())
        a_sol = ype.makeNestedList([6]*cchar.getBodyNum())
        
        p = []
    
        v_original = []
        av_original = []
        a_original = []
        aa_original = []
    
        v_jacobian = []
        av_jacobian = []
        a_jacobian = []
        aa_jacobian = []
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
    #    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('cchar', cvr.VpModelRenderer(cchar, (255,240,255), yr.POLYGON_LINE))
    
        viewer.doc.addRenderer('v_original', yr.VectorsRenderer(v_original, p, (255,0,0)))
        viewer.doc.addRenderer('v_jacobian', yr.VectorsRenderer(v_jacobian, p, (0,255,0)))
    
        viewer.doc.addRenderer('av_original', yr.VectorsRenderer(av_original, p, (255,0,0)))
        viewer.doc.addRenderer('av_jacobian', yr.VectorsRenderer(av_jacobian, p, (0,255,0)))
        
#        viewer.doc.addRenderer('a_original', yr.VectorsRenderer(a_original, p, (255,0,0)))
#        viewer.doc.addRenderer('a_jacobian', yr.VectorsRenderer(a_jacobian, p, (0,255,0)))
#
#        viewer.doc.addRenderer('aa_original', yr.VectorsRenderer(aa_original, p, (255,0,0)))
#        viewer.doc.addRenderer('aa_jacobian', yr.VectorsRenderer(aa_jacobian, p, (0,255,0)))
    
        cchar.applyBodyTorqueGlobal(0, (0,100,0))
        cchar.applyBodyTorqueGlobal(1, (0,0,100))
        
        def simulateCallback(frame):
            
            for i in range(stepsPerFrame):
                vpWorld.step()
                
            jointPositions = cchar.getJointPositionsGlobal()
            
#            Rs = cchar.getJointOrientationsGlobal()
#            jointAxeses = [np.concatenate((mm.I_SO3(), mm.I_SO3()))]+[Rs[i].transpose() for i in range(1,len(Rs))]
            jointAxeses = cchar.getDOFAxeses()
    
            linkPositions = cchar.getBodyPositionsGlobal()
            linkVelocities = cchar.getBodyVelocitiesGlobal()
            linkAccelerations = cchar.getBodyAccelerationsGlobal()
            linkAngVelocities = cchar.getBodyAngVelocitiesGlobal()
            linkAngAccelerations = cchar.getBodyAngAccelerationsGlobal()
            
            computeJacobian2(J, DOFs, jointPositions, jointAxeses, linkPositions, linkJointMasks)
            computeJacobianDerivative2(dJ, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, linkJointMasks)
            
#            dth = [cchar.getJointVelocityGlobal(0)] + cchar.getJointAngVelocitiesLocal()
#            ddth = [cchar.getJointAccelerationGlobal(0)] + cchar.getJointAngAccelerationsLocal()
            dth = cchar.getDOFVelocities() 
            ddth = cchar.getDOFAccelerations()
            ype.flatten(dth, dth_flat)
            ype.flatten(ddth, ddth_flat)
            
            # dx = J*dq
            v_flat = np.dot(J, dth_flat)
            ype.nested(v_flat, v_sol)
            
            # ddx = dJ*dq + J*ddq
            a_flat = np.dot(dJ, dth_flat)
            
            a_flat2 = copy.copy(a_flat)
            a_flat2 = np.dot(J, ddth_flat)
            a_flat = [a_flat[i] + a_flat2[i] for i in range(len(a_flat))]
    
            ype.nested(a_flat, a_sol)
            
                        
            p[:] = linkPositions
            v_original[:] = linkVelocities
            av_original[:] = linkAngVelocities
            a_original[:] = linkAccelerations
            aa_original[:] = linkAngAccelerations
            
            v_jacobian[:] = [v_sol[i][0:3] for i in range(len(v_sol))]
            av_jacobian[:] = [v_sol[i][3:6] for i in range(len(v_sol))]
            a_jacobian[:] = [a_sol[i][0:3] for i in range(len(a_sol))]
            aa_jacobian[:] = [a_sol[i][3:6] for i in range(len(a_sol))]
                
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
    
    def test_force_torque():
        np.set_printoptions(precision=4, linewidth=200)
    
#        bvhFilePath = '../samples/chain_1.bvh'
#        bvhFilePath = '../samples/chain_2.bvh'
#            bvhFilePath = '../samples/chain_2_rotate_expt_root.bvh'
        bvhFilePath = '../samples/chain_3_rotate_expt_root.bvh'
#        bvhFilePath = '../samples/chain_3_rotate.bvh'
    #        bvhFilePath = '../samples/chain_3_rotate_freely_expt_root.bvh'
    #        bvhFilePath = '../samples/chain_3_rotate_freely.bvh'
    #    bvhFilePath = '../samples/chain_3_rotate_freely_move.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .8
        for i in range(motion[0].skeleton.getElementNum()):
            mcfg.addNode(motion[0].skeleton.getElementName(i))
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = -1.
        wcfg.gravity = (0,0,0)
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        vpWorld = cvw.VpWorld(wcfg)
        cchar = cvm.VpControlModel(vpWorld, motion[0], mcfg)
        vpWorld.initialize()
        print cchar
    
        DOFs = cchar.getDOFs()
        totalDOF = cchar.getTotalDOF()
        
        J = makeEmptyJacobian(DOFs, 1)
        
        endIndex = 2
        endJointMask = getLinkJointMask(motion[0].skeleton, endIndex)
        
        # target force
        force = (0,1,0, 0,0,0)
        torques = ype.makeNestedList(cchar.getDOFs())
        
        p_end = []
        f_end = []

        p_joint = []
        t_joint = []
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
    #    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('cchar', cvr.VpModelRenderer(cchar, (255,240,255), yr.POLYGON_LINE))
    
        viewer.doc.addRenderer('f_end', yr.VectorsRenderer(f_end, p_end, (255,0,0)))
        viewer.doc.addRenderer('t_joint', yr.VectorsRenderer(t_joint, p_joint, (0,255,0)))
    
#        cchar.applyBodyTorqueGlobal(0, (0,100,0))
#        cchar.applyBodyTorqueGlobal(1, (0,0,100))
        
        def simulateCallback(frame):
            cchar.applyBodyGenForceGlobal(endIndex, force[3:6], force[0:3])
            
            for i in range(stepsPerFrame):
                vpWorld.step()
                
            jointPositions = cchar.getJointPositionsGlobal()
            jointAxeses = cchar.getDOFAxeses()
            endPosition = cchar.getBodyPositionGlobal(endIndex)
            computeJacobian2(J, DOFs, jointPositions, jointAxeses, [endPosition], [endJointMask])
            
            # torque = J^t * force
            torques_flat = np.dot(J.transpose(), force)
            ype.nested(torques_flat, torques)
            
            p_end[:] = [endPosition]
            f_end[:] = [force[0:3]]
            
            p_joint[:] = jointPositions
            t_joint[:] = torques
            
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
    
    pass
#    test_IK()
#    test_computeJacobian()
#    test_computeJacobian2()
#    test_velocity_relation_simple()
#    test_effector_joint_masks_funcs()
#    test_get_dP_effector_from_joint()
#    test_computeJacobian2_VpModel()
    test_force_torque()