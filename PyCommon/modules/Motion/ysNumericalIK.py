import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import Resource.ysMotionLoader as yf
import GUI.ysSimpleViewer as ysv
import Renderer.ysRenderer as yr
import Math.mmMath as mm

def jointSkeleton2IKTree(jointSkeleton):
    tree = IKTree()
    nodeMap = {}
    tree.root = _joint2Node(jointSkeleton.root, None, nodeMap)
    tree.nodes = [None]*jointSkeleton.getElementNum()
    for i in range(jointSkeleton.getElementNum()):
        tree.nodes[i] = nodeMap[jointSkeleton.getElementName(i)]
    tree.nodeNames = jointSkeleton.elementNames
    tree.reverseNodeNames = jointSkeleton.reverseElementNames
    return tree
    
def _joint2Node(joint, parentNode, nodeMap):
    node = IKNode()
    node.name = joint.name
    node.parent = parentNode
    if parentNode!=None:
        parentNode.children.append(node)
    for childJoint in joint.children:
        _joint2Node(childJoint, node, nodeMap)
    
    nodeMap[node.name] = node
    return node

class IKTree:
    def __init__(self):
        self.root = None
        self.nodes = []
        self.nodeNames = []
        self.reverseNodeNames = {}
    def __str__(self):
        s = ''
        s += '<NODES>\n'
        for i in range(len(self.nodes)):
            s += '[%d]:\'%s\', '%(i, self.nodes[i].name)
        s += '\n'
        s += '<HIERARCHY>\n'
        s += self.root.__strHierarchy__()
        return s
    def getNode(self, index):
        return self.nodes[i]
    def getNodeName(self, index):
        return self.nodeNames[i]
    def getNodeIndex(self, name):
        return self.reverseNodeNames[name]
    def getNodeNum(self):
        return len(self.nodes)
    def changeRootNode(self, root_name_or_index):
        if isinstance(root_name_or_index, int):
            self.root = self.nodes[root_name_or_index]
        else:
            self.root = self.nodes[self.getNodeIndex(root_name_or_index)]
        self._rebuildHierarchy(self.root, None)
            
    def _rebuildHierarchy(self, node, parentNode):
        linkedNodes = node.children + [node.parent]
        
        node.parent = parentNode
        if parentNode!=None:
            parentNode.children.append(node)

        del node.children[:]
        for childCandidate in linkedNodes:
            if childCandidate != parentNode and childCandidate != None:
                self._rebuildHierarchy(childCandidate, node)
                
    def getCOMPos(positions, masses, totalMass):
        comPos = mmMath.Vec3(0.,0.,0.)
        for vi in vertexIndices:
            weightedPos = positions[vi] * masses[vi]
            comPos += weightedPos
        comPos /= totalMass
        return comPos
        

class IKNode:
    def __init__(self):
        self.name = None
        self.parent = None
        self.children = []
        self.mass = 0.
    def __str__(self):
        s = ''
        s += self.name
        return s
    def __strHierarchy__(self, depth = 0):
        s = ''
        tab = '  '*depth
        s += '%s%s\n'%(tab, self.__str__())
        depth += 1
        for child in self.children:
            s += child.__strHierarchy__(depth)
        return s
    
class Solver:
    def __init__(self, skeleton, effectorNames):
        pass
    def solve(self, inoutPosture, targetPositions):
        pass
    
    
class JTSolver_IKTree(Solver):
    def __init__(self, skeleton, effectorNames):
        self.positionThreshold = .1
        self.iterationLimit = 1000
        self.effectorIndices = [skeleton.getNodeIndex(name) for name in effectorNames]
        self.skeleton = skeleton
        self.jacobian = np.zeros((3, skeleton.getNodeNum()-1))
        
    def solve(self, inoutPosture, targetPositions):
        iterNum = 0
        positionDiff = 0.
        
        while True:
            currentPositions = [inoutPosture.getPosition(self.effectorIndices[0])]
            axes = self.calcJacobian(inoutPosture, currentPositions, targetPositions)
            forceVec = targetPositions[0] - currentPositions[0]
            deltaThetas = np.dot(np.transpose(self.jacobian), forceVec)*.1
            self.applyThetas(deltaThetas, axes, inoutPosture)
            
            iterNum += 1
            if iterNum > self.iterationLimit:
                print 'iter'
                break;
            
            positionDiff = 0.
            for i in range(len(targetPositions)):
                positionDiff += mm.length(targetPositions[i] - inoutPosture.getPosition( self.effectorIndices[i]) )
            if positionDiff < self.positionThreshold:
                print 'converge'
                break;
    
    def calcJacobian(self, posture, effectorPositions, targetPositions):
        axes = [None]*(self.skeleton.getNodeNum()-1)
        for i in range(self.skeleton.getNodeNum()-1):
            jointPos = posture.getPosition(i)
            toEffector = effectorPositions[0] - jointPos
            toTarget = targetPositions[0] - jointPos
            axes[i] = np.cross(toTarget, toEffector)
            instanteneousVelocity_for_i = -np.cross(toEffector, axes[i])
            for j in range(3):
                self.jacobian[j,i] = instanteneousVelocity_for_i[j]
        return axes
            
    def applyThetas(self, deltaThetas, axes, inoutPosture):
        for i in range(self.skeleton.getNodeNum()-1):
            inoutPosture.mulGlobalR(i, mm.exp(axes[i], deltaThetas[i]))
        inoutPosture.updateGlobalT()
       

# don't use
class JTSolver_IKTree_COM(Solver):
    def __init__(self, skeleton):
        self.positionThreshold = .1
        self.iterationLimit = 1000
        self.skeleton = skeleton
        self.jacobian = np.zeros((3, skeleton.getNodeNum()-1))
        self.totalMass = 0.
        for i in range(len(skeleton.getNodeNum())):
            self.totalMass += skeleton.getNode(i).mass 
        
    def solve(self, inoutPosture, targetPositions):
        iterNum = 0
        positionDiff = 0.
        
        while True:
#            currentPositions = [inoutPosture.getPosition(self.effectorIndices[0])]
            currentPositions = [getCOMPos(inoutPosture.getPositions, self.masses, self.totalMass)]
            axes = self.calcJacobian(inoutPosture, currentPositions, targetPositions)
            forceVec = targetPositions[0] - currentPositions[0]
            deltaThetas = np.dot(np.transpose(self.jacobian), forceVec)*.1
            self.applyThetas(deltaThetas, axes, inoutPosture)
            
            iterNum += 1
            if iterNum > self.iterationLimit:
                print 'iter'
                break;
            
            positionDiff = 0.
            for i in range(len(targetPositions)):
                positionDiff += mm.length(targetPositions[i] - inoutPosture.getPosition( self.effectorIndices[i]) )
            if positionDiff < self.positionThreshold:
                print 'converge'
                break;
    
    def calcJacobian(self, posture, effectorPositions, targetPositions):
        axes = [None]*(self.skeleton.getNodeNum()-1)
        for i in range(self.skeleton.getNodeNum()-1):
            jointPos = posture.getPosition(i)
            toEffector = effectorPositions[0] - jointPos
            toTarget = targetPositions[0] - jointPos
            axes[i] = np.cross(toTarget, toEffector)
#            instanteneousVelocity_for_i = -np.cross(toEffector, axes[i])
#            instanteneousVelocity_for_i = -np.cross(toEffector, axes[i])*(/self.totalMass)
            for j in range(3):
                self.jacobian[j,i] = instanteneousVelocity_for_i[j]
        return axes
            
    def applyThetas(self, deltaThetas, axes, inoutPosture):
        for i in range(self.skeleton.getNodeNum()-1):
            inoutPosture.mulGlobalR(i, mm.exp(axes[i], deltaThetas[i]))
        inoutPosture.updateGlobalT()
        
class JTSolver_Skeleton(Solver):
    def __init__(self, skeleton, effectorNames):
        self.positionThreshold = .01
        self.iterationLimit = 100000
        self.effectorIndices = [skeleton.getElementIndex(name) for name in effectorNames]
        self.skeleton = skeleton
        self.jacobian = np.zeros((3, skeleton.getElementNum()-1))
        
    def solve(self, inoutPosture, targetPositions):
        iterNum = 0
        positionDiff = 0.
        
        while True:
            currentPositions = [inoutPosture.getPosition(self.effectorIndices[0])]
            axes = self.calcJacobian(inoutPosture, currentPositions, targetPositions)
            forceVec = targetPositions[0] - currentPositions[0]
            deltaThetas = np.dot(np.transpose(self.jacobian), forceVec)*.1
            self.applyThetas(deltaThetas, axes, inoutPosture)
           
            iterNum += 1
            print iterNum
            if iterNum > self.iterationLimit:
                print 'iter'
                break;
            
            positionDiff = 0.
            for i in range(len(targetPositions)):
                positionDiff += mm.length(targetPositions[i] - inoutPosture.getPosition( self.effectorIndices[i]) )
            if positionDiff < self.positionThreshold:
                print 'converge', iterNum
                break;
    
    def calcJacobian(self, posture, effectorPositions, targetPositions):
        axes = [None]*(self.skeleton.getElementNum()-1)
        for i in range(self.skeleton.getElementNum()-1):
            jointPos = posture.getPosition(i)
            toEffector = effectorPositions[0] - jointPos
            toTarget = targetPositions[0] - jointPos
            axes[i] = np.cross(toTarget, toEffector)
            instanteneousVelocity_for_i = -np.cross(toEffector, axes[i])
            for j in range(3):
                self.jacobian[j,i] = instanteneousVelocity_for_i[j]
        return axes
            
    def applyThetas(self, deltaThetas, axes, inoutPosture):
        for i in range(self.skeleton.getElementNum()-1):
            inoutPosture.mulGlobalR(i, mm.exp(axes[i], deltaThetas[i]))
        inoutPosture.updateGlobalT()
    
# jacobian pseudo inverse
class JPISolver_Skeleton(Solver):
    def __init__(self, skeleton, effectorNames):
        self.positionThreshold = .01
        self.iterationLimit = 100000
        self.effectorIndices = [skeleton.getElementIndex(name) for name in effectorNames]
        self.skeleton = skeleton
        self.jacobian = np.zeros((3, skeleton.getElementNum()-1))
        
    def solve(self, inoutPosture, targetPositions):
        iterNum = 0
        positionDiff = 0.
        
        while True:
            currentPositions = [inoutPosture.getPosition(self.effectorIndices[0])]
            axes = self.calcJacobian(inoutPosture, currentPositions, targetPositions)
            forceVec = targetPositions[0] - currentPositions[0]
            # dt = J * dtheta
            # Jt * dt = Jt * J * dtheta
            # (Jt * J)^-1 * Jt * dt = dtheta
            #deltaThetas = np.dot(np.dot(np.linalg.inv(np.dot(np.transpose(self.jacobian),self.jacobian)),np.transpose(self.jacobian)), forceVec)*.1
            # moore-penrose pseudo inverse
            deltaThetas = np.dot(np.linalg.pinv(self.jacobian), forceVec)*.1
            self.applyThetas(deltaThetas, axes, inoutPosture)
           
            iterNum += 1
            print iterNum
            if iterNum > self.iterationLimit:
                print 'iter'
                break;
            
            positionDiff = 0.
            for i in range(len(targetPositions)):
                positionDiff += mm.length(targetPositions[i] - inoutPosture.getPosition( self.effectorIndices[i]) )
            if positionDiff < self.positionThreshold:
                print 'converge', iterNum
                break;
    
    def calcJacobian(self, posture, effectorPositions, targetPositions):
        axes = [None]*(self.skeleton.getElementNum()-1)
        for i in range(self.skeleton.getElementNum()-1):
            jointPos = posture.getPosition(i)
            toEffector = effectorPositions[0] - jointPos
            toTarget = targetPositions[0] - jointPos
            axes[i] = np.cross(toTarget, toEffector)
            instanteneousVelocity_for_i = -np.cross(toEffector, axes[i])
            for j in range(3):
                self.jacobian[j,i] = instanteneousVelocity_for_i[j]
        return axes
            
    def applyThetas(self, deltaThetas, axes, inoutPosture):
        for i in range(self.skeleton.getElementNum()-1):
            inoutPosture.mulGlobalR(i, mm.exp(axes[i], deltaThetas[i]))
        inoutPosture.updateGlobalT()

if __name__=='__main__':
#    import psyco; psyco.full()
    import copy
    from fltk import *
    
    def test_JTSolver_Skeleton():
        bvhFilePath = '../samples/block_3_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        skeleton = motion[0].skeleton
        origPosture = ym.Motion([motion[0]])
        newPosture = copy.deepcopy(origPosture)
        
        effectorNames = ['body3_Effector']
#        effectorNames = ['body1']
        targetPositions = [(1.,0,0)]
        
        solver = JTSolver_Skeleton(skeleton, effectorNames)
        solver.solve(newPosture[0], targetPositions)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_LINE))
        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_LINE))
#        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_SOLIDBOX))
#        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_SOLIDBOX))
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_JPISolver_Skeleton():
        bvhFilePath = '../samples/block_3_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        skeleton = motion[0].skeleton
        origPosture = ym.Motion([motion[0]])
        newPosture = copy.deepcopy(origPosture)
        
        effectorNames = ['body3_Effector']
#        effectorNames = ['body1']
        targetPositions = [(1.,0,0)]
        
        solver = JPISolver_Skeleton(skeleton, effectorNames)
        solver.solve(newPosture[0], targetPositions)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_LINE))
        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_LINE))
#        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_SOLIDBOX))
#        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_SOLIDBOX))
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_JTSolver_IKTree():
        bvhFilePath = '../samples/block_3_rotate.bvh'
#        bvhFilePath = '../../../Walking/adun/adun.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        skeleton = motion[0].skeleton
        origPosture = ym.Motion([motion[0]])
        newPosture = copy.deepcopy(origPosture)
       
        print skeleton
        print motion[0]
        print origPosture[0]

        effectorNames = ['body3_Effector']
#        effectorNames = ['mtp_l_Effector']
#        effectorNames = ['body1']
        targetPositions = [(1.,0,0)]
        tree = jointSkeleton2IKTree(skeleton)
#        print tree
#        tree.changeRootNode('body3_Effector')
#        print tree
        solver = JTSolver_IKTree(tree, effectorNames)
        solver.solve(newPosture[0], targetPositions)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_LINE))
#        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_LINE))
        viewer.doc.addRenderer('targetPosition', yr.PointsRenderer(targetPositions))
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
    def test_JTSolver_IKTree():
        bvhFilePath = '../samples/block_3_rotate.bvh'
#        bvhFilePath = '../../../Walking/adun/adun.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        skeleton = motion[0].skeleton
        origPosture = ym.Motion([motion[0]])
        newPosture = copy.deepcopy(origPosture)
       
        print skeleton
        print motion[0]
        print origPosture[0]

        effectorNames = ['body3_Effector']
#        effectorNames = ['mtp_l_Effector']
#        effectorNames = ['body1']
        targetPositions = [(1.,0,0)]
        tree = jointSkeleton2IKTree(skeleton)
#        print tree
#        tree.changeRootNode('body3_Effector')
#        print tree
        solver = JTSolver_IKTree(tree, effectorNames)
        solver.solve(newPosture[0], targetPositions)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_LINE))
#        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_LINE))
        viewer.doc.addRenderer('targetPosition', yr.PointsRenderer(targetPositions))
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_JTSolver_IKTree_COM():
        bvhFilePath = '../samples/block_3_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        skeleton = motion[0].skeleton
        origPosture = ym.Motion([motion[0]])
        newPosture = copy.deepcopy(origPosture)
        
#        effectorNames = ['body3_Effector']
        targetPositions = [(.5,.5,0)]
#        tree = jointSkeleton2IKTree(skeleton)
#        solver = JTSolver_IKTree_COM(tree, effectorNames)
#        solver.solveCOM(newPosture[0], targetPositions)
#        COM = getCOMPos()
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
#        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('origPosture', yr.JointMotionRenderer(origPosture, (0,0,255), yr.LINK_LINE))
#        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_WIREBOX))
        viewer.doc.addRenderer('newPosture', yr.JointMotionRenderer(newPosture, (0,255,0), yr.LINK_LINE))
        viewer.doc.addRenderer('targetPosition', yr.PointsRenderer(targetPositions))
#        viewer.doc.addRenderer('COM', yr.PointsRenderer([COM]))
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_IKTree():
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        tree = jointSkeleton2IKTree(motion[0].skeleton)
        print tree
        
        tree.changeRootNode('LeftToes_Effector')
        print tree

        
    test_JTSolver_Skeleton()
#    test_JPISolver_Skeleton()

#    test_JTSolver_IKTree()
#    test_JTSolver_IKTree_COM()
#    test_IKTree()
