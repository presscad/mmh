import numpy
import copy

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mmMath
import Motion.ysMotion as ym

class Vertex:
    def __init__(self):
        self.pos = numpy.array([0.,0.,0.])
    def __str__(self):
        s = self.pos.__str__()
        return s

class Face:
    def __init__(self):
        self.vertexIndex = [None]*3 
    def __str__(self):
        s = self.vertexIndex.__str__()
        return s

class Mesh():
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.submeshMap = {}
        self.resourceName = 'unnamed'
    def getState(self):
        return [v.pos for v in self.vertices]
    def setState(self, state):
        for i in range(len(state)):
            self.vertices[i].pos = state[i]
    def __str__(self):
        s = ''
        s += 'vertices %d\n'%len(self.vertices)
        for i in range(len(self.vertices)):
            s += 'vertex %d '%i + self.vertices[i].__str__() + '\n' 
        s += '\n'
        s += 'faces %d\n'%len(self.faces)
        for i in range(len(self.faces)):
            s += 'face %d '%i + self.faces[i].__str__() + '\n'
        s += '\n'
        s += 'submeshes %d\n'%len(self.submeshMap)
        i = 0
        for submeshName, faceIndices in self.submeshMap.items():
            s += 'submesh %d name:%s, faceIndices:%s'%(i, submeshName, str(faceIndices)) + '\n'
            i += 1
        return s
    def getSubmeshFaceIndices(self, submeshName):
        return self.submeshMap[submeshName]
    def getSubmeshVertexIndices(self, submeshName):
        vertexIndexSet = set()
        faceIndices = self.getSubmeshFaceIndices(submeshName)
        for fi in faceIndices:
            for vi in self.faces[fi].vertexIndex:
                vertexIndexSet.add(vi)
        return list(vertexIndexSet)
    def getSubmeshVertexPositions(self, submeshName):
        vertexIndices = self.getSubmeshVertexIndices(submeshName)
        return [self.vertices[vi].pos for vi in vertexIndices]
    def getVertexPosition(self, vertexIndex):
        return self.vertices[vertexIndex].pos
    def getVertexPositions(self, vertexIndices):
        return [self.vertices[vi].pos for vi in vertexIndices]
    def updateFromPositions(self, positions):
        for i in range(len(self.vertices)):
            self.vertices[i].pos = positions[i]
    def getVertexNum(self):
        return len(self.vertices)
    def getFaceNum(self):
        return len(self.faces)
        
class SkinMesh(Mesh):
    def __init__(self):
        Mesh.__init__(self)
        self.vertexBoneWeights = None
        self.skeleton = None 
        self.initialVertices = None
        self.initialPosture = None
    def initialize(self, initialPosture):
        initialPosture.updateGlobalT()
        self.initialPosture = initialPosture
        self.initialVertices = copy.deepcopy(self.vertices)
        self.skeleton = initialPosture.skeleton
#        self.invMrefMap = {}
#        for boneName in self.skeleton.joints: 
#            self.invMrefMap[boneName] = mmMath.invertSE3(initialPosture.getGlobalT(boneName))
        self.invMrefs = [None]*self.skeleton.getElementNum()
        for i in range(self.skeleton.getElementNum()):
            self.invMrefs[i] = mmMath.invertSE3(initialPosture.getGlobalT(i))
    def update(self, targetPosture):
#        M_dot_invMref_Map = {}
#        for boneName in self.skeleton.joints: 
#            M = targetPosture.getGlobalT(boneName)
#            invMref = self.invMrefMap[boneName]            
#            M_dot_invMref_Map[boneName] = numpy.dot(M, invMref)
        M_dot_invMrefs = [None]*self.skeleton.getElementNum()
        for i in range(self.skeleton.getElementNum()):
            M = targetPosture.getGlobalT(i)
            invMref = self.invMrefs[i]            
            M_dot_invMrefs[i] = numpy.dot(M, invMref)
            
        for i in range(len(self.vertices)):
            self.vertices[i].pos = mmMath.O_Vec3()
            v = mmMath.p2T(self.initialVertices[i].pos)
            
#            for boneIndex, weight in self.vertexBoneWeights[i]:
#                boneName = self.skeleton.jointsAr[boneIndex].name
#                w = weight
#                
##                v = mmMath.p2T(self.initialVertices[i].pos)
##                invMref = mmMath.invertSE3(self.initialPosture.calcGlobalT(boneName))
##                M = targetPosture.calcGlobalT(boneName)
##                MMv = numpy.dot(M, numpy.dot(invMref, v))
#                MMv = numpy.dot(M_dot_invMref_Map[boneName], v)

            for boneIndex, weight in self.vertexBoneWeights[i]:
                w = weight
                
#                v = mmMath.p2T(self.initialVertices[i].pos)
#                invMref = mmMath.invertSE3(self.initialPosture.calcGlobalT(boneName))
#                M = targetPosture.calcGlobalT(boneName)
#                MMv = numpy.dot(M, numpy.dot(invMref, v))
                MMv = numpy.dot(M_dot_invMrefs[boneIndex], v)
                
                self.vertices[i].pos += w * mmMath.T2p(MMv)
        
        
if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    from datetime import datetime
    import cProfile, os
    import Resource.ysOgreDataLoader as yol
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Resource.ysMotionLoader as yf
    import Util.ysGlHelper as ygh

    def test_invMref():
        skeletonFilePath = '../samples/physics2_woody_binding1.skeleton.xml'
        bindJointName = 'LeftHand'
        jointMotions = yol.readOgreSkeletonFile_SkeletonAnimations(skeletonFilePath, .01)
        jointMotion = jointMotions[0]
        frameTime = 1./30.
        
#        print jointMotion[0].skeleton
        
        initialPosture = jointMotion[0].getTPose()
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addMotion(jointMotion)
        viewer.doc.addRenderer('initialPosture', yr.JointMotionRenderer(ym.Motion([initialPosture]), (255, 0, 0), yr.LINK_LINE))
        viewer.doc.addRenderer('targetPosture', yr.JointMotionRenderer(jointMotion, (0, 0, 255), yr.LINK_LINE))
        
#        v = numpy.array([0,2,2])
        v = initialPosture.getGlobalPos(bindJointName) + (1,0,0)
#        v = mmMath.p2T(numpy.array([0,0,2]))
#        print 'v', v

        v_new = [numpy.zeros(3)]
        def preFrameCallback(frame):
            targetPosture = jointMotion[frame]
            
            invMref = mmMath.invertSE3(initialPosture.calcGlobalT(bindJointName))
            v_local = mmMath.T2p(numpy.dot(invMref, mmMath.p2T(v)))
    #        v_local = numpy.dot(invMref, v)
#            print 'v_local', v_local
            M = targetPosture.calcGlobalT(bindJointName)
            v_new[0] = mmMath.T2p(numpy.dot(M, mmMath.p2T(v_local)))
    #        v_new = numpy.dot(M, v_local)
#            print 'v_new', v_new

        def extraDrawCallback():
            ygh.drawLine(initialPosture.getGlobalPos(bindJointName), v, (255, 178, 178))
            ygh.drawLine(jointMotion[jointMotion.frame].getGlobalPos(bindJointName), v_new[0], (178, 178, 255))
            
        viewer.setPreFrameCallback(preFrameCallback)
        viewer.setExtraDrawCallback(extraDrawCallback)
        
        viewer.startTimer(frameTime)
        viewer.show()
        Fl.run()

    def test_Mesh():
        meshFilePath = '../samples/physics2_woody_binding1.mesh.xml'
        mesh = yol.readOgreMeshFileAsMesh(meshFilePath)
        print mesh
        
    def test_submesh():
        meshFilePath = '../samples/woody2_4.mesh.xml'
        mesh = yol.readOgreMeshFileAsMesh(meshFilePath, .01)
        print mesh
        
        print mesh.getSubmeshFaceIndices('woody2_14_FOOT1')
        print mesh.getSubmeshVertexIndices('woody2_14_FOOT1')
        print mesh.getSubmeshVertexPositions('woody2_14_FOOT1')
        positions = mesh.getSubmeshVertexPositions('woody2_14_FOOT1')
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh, (127,127,127)))
        viewer.doc.addObject('mesh', mesh)
        
        def extraDrawCallback():
            for p in positions:
                ygh.drawPoint(p)
        viewer.setExtraDrawCallback(extraDrawCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    def profile_skinning():
        meshFilePath = '../samples/woody2_15.mesh.xml'
        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        mesh, ms = yol.readOgreDataFiles(meshFilePath, .01, skeletonFilePath)
        motion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)        
        
        profileDataFile = '../samples/cProfile_%s.profile'%datetime.today().strftime('%y%m%d_%H%M%S')
        cProfile.runctx('mesh.update(motion[50])', globals(), locals(), profileDataFile)
        os.system('python ../../Tools/pprofui.py %s'%profileDataFile)
            
        
    pass
#    test_submesh()
#    test_invMref()
#    test_Mesh()
    profile_skinning()