import xml.sax
import xml.sax.handler
import xml.dom.minidom
import numpy, os

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Mesh.ysMesh as yms
import Motion.ysMotion as ym
import Math.mmMath as mmMath

'''
Maya(8.5) Ogre Exporter(1.2.6) Setting (supported in this module)
Mesh
  v Export mesh
  v Use shared geometry
  v Include vertex bone assignements
    Include diffuse vertex colours
    Include texture coordinates
    Build edges list(for shadows)
    Build tanget vectors(for normal maps)
Materials
    Export materials to Ogre .material file
Skeleton
  v Export skeleton
Skeleton Animations
  v Export animations (requires export of skeleton)
Blend Shapes
    Export blend shapes (to mesh file)
Blend Shape Animations
    Export animations (to mesh file)
Vertex Animations
    Export animations (to mesh file)
Cameras
    Export cameras to Ogre .camera file
Particles
    Export particles to Ogre .particles file
'''

def readOgreDataFiles(meshFilePath, scale=1.0, skeletonFilePath=None):
    parser = xml.sax.make_parser()   
    handler = OgreSkinMeshSaxHandler(scale)
    parser.setContentHandler(handler)
    parser.parse(open(meshFilePath))
    
    if skeletonFilePath==None:
        if handler.skeletonlink!=None:
            dirname = os.path.dirname(meshFilePath)
            skeletonFilePath = dirname+'/'+handler.skeletonlink+'.xml'

    jointMotions = []
    if skeletonFilePath!=None:
        jointSkeleton, initialRs, jointMotions = readOgreSkeletonFile(skeletonFilePath, scale)
        initialPosture = ym.JointPosture(jointSkeleton)
#        initialPosture.initLocalRMap(initialRMap)
        initialPosture.initLocalRs(initialRs)
        handler.mesh.initialize(initialPosture)
    
    return handler.mesh, jointMotions

def readOgreMeshFileAsMesh(meshFilePath, scale=1.0):
    parser = xml.sax.make_parser()   
    handler = OgreMeshSaxHandler(scale)
    parser.setContentHandler(handler)
    parser.parse(open(meshFilePath))
    return handler.mesh

class OgreMeshSaxHandler(xml.sax.handler.ContentHandler):
    def __init__(self, scale):
        self.mesh = None
        self.scale = scale
        self.currentSubmeshName = None
    def startDocument(self):
        self.mesh = yms.Mesh()
    def startElement(self, name, attrs):
        if name == 'submesh':
            self.currentSubmeshName = attrs.get('material').encode()
            self.mesh.submeshMap[self.currentSubmeshName] = []
        elif name == 'face':
            face = yms.Face()
            face.vertexIndex[0] = int(attrs.get('v1',""))
            face.vertexIndex[1] = int(attrs.get('v2',""))
            face.vertexIndex[2] = int(attrs.get('v3',""))
            self.mesh.faces.append(face)
            self.mesh.submeshMap[self.currentSubmeshName].append(len(self.mesh.faces)-1)
        elif name == 'position':
            vertex = yms.Vertex()
            vertex.pos[0] = float(attrs.get('x', "")) * self.scale
            vertex.pos[1] = float(attrs.get('y', "")) * self.scale
            vertex.pos[2] = float(attrs.get('z', "")) * self.scale
            self.mesh.vertices.append(vertex)
    
def readOgreMeshFileAsSkinMesh(meshFilePath, jointSkeleton, initialRs, scale=1.0):
    parser = xml.sax.make_parser()   
    handler = OgreSkinMeshSaxHandler(scale)
    parser.setContentHandler(handler)
    parser.parse(open(meshFilePath))
    
    initialPosture = ym.JointPosture(jointSkeleton)
    initialPosture.initLocalRs(initialRs)
    handler.mesh.initialize(initialPosture)
    
    return handler.mesh

class OgreSkinMeshSaxHandler(OgreMeshSaxHandler):
    def __init__(self, scale):
        OgreMeshSaxHandler.__init__(self, scale)
        self.skeletonlink = None
    def startDocument(self):
        self.mesh = yms.SkinMesh()
    def startElement(self, name, attrs):
        OgreMeshSaxHandler.startElement(self, name, attrs)
        if name == 'sharedgeometry':
            self.mesh.vertexBoneWeights = [None]*int(attrs.get('vertexcount'))
            for i in range(int(attrs.get('vertexcount'))):
                self.mesh.vertexBoneWeights[i] = []
        if name == 'skeletonlink':
            self.skeletonlink = attrs.get('name')
        if name == 'vertexboneassignment':
            vertexIndex = int(attrs.get('vertexindex'))
            boneIndex = int(attrs.get('boneindex'))
            weight = float(attrs.get('weight'))
            self.mesh.vertexBoneWeights[vertexIndex].append((boneIndex, weight))
                
def readOgreSkeletonFile(skeletonFilePath, scale=1.0):
    dom = xml.dom.minidom.parse(skeletonFilePath)
    jointSkeleton, initialRs = _readOgreSkeleton(dom, scale)
    jointMotions = _readOgreSkeletonAnimations(dom, jointSkeleton, initialRs, scale)
    return jointSkeleton, initialRs, jointMotions
    
def readOgreSkeletonFile_Skeleton(skeletonFilePath, scale=1.0):
    dom = xml.dom.minidom.parse(skeletonFilePath)
    jointSkeleton, initialRs = _readOgreSkeleton(dom, scale)
    return jointSkeleton, initialRs

def readOgreSkeletonFile_SkeletonAnimations(skeletonFilePath, scale=1.0):
    dom = xml.dom.minidom.parse(skeletonFilePath)
    jointSkeleton, initialRs = _readOgreSkeleton(dom, scale)
    jointMotions = _readOgreSkeletonAnimations(dom, jointSkeleton, initialRs, scale)
    return jointMotions
    
def _readOgreSkeleton(dom, scale=1.0):
    bones = dom.getElementsByTagName('bone')
    jointMap = {}
#    initialRMap = {}
    initialRs = []
    for bone in bones:
        joint = ym.Joint(bone.getAttribute('name').encode(), None)
        jointMap[joint.name] = joint
        
        posEle = bone.getElementsByTagName('position')[0]
        joint.offset[0] = float(posEle.getAttribute('x')) * scale
        joint.offset[1] = float(posEle.getAttribute('y')) * scale
        joint.offset[2] = float(posEle.getAttribute('z')) * scale
        
        rotEle = bone.getElementsByTagName('rotation')[0]
        angle = float(rotEle.getAttribute('angle'))
        axisEle = rotEle.getElementsByTagName('axis')[0]
        axis = mmMath.s2v((float(axisEle.getAttribute('x')), float(axisEle.getAttribute('y')), float(axisEle.getAttribute('z'))))
        R = mmMath.exp(axis, angle)
#        initialRMap[joint.name] = R
        initialRs.append(R)

    rootJoint = jointMap[bones[0].getAttribute('name').encode()]

    boneParents = dom.getElementsByTagName('boneparent')
    for bp in boneParents:
        joint = jointMap[bp.getAttribute('bone').encode()]
        parentJoint = jointMap[bp.getAttribute('parent').encode()]
        joint.parent = parentJoint
        parentJoint.addChild(joint)
        
    jointSkeleton = ym.JointSkeleton(rootJoint)
    for bone in bones:
#        jointSkeleton.jointsAr.append(jointMap[bone.getAttribute('name')])
        boneName = bone.getAttribute('name').encode()
        jointSkeleton.addElement(jointMap[boneName], boneName)
#    return jointSkeleton, initialRMap
    return jointSkeleton, initialRs

def _readOgreSkeletonAnimations(dom, jointSkeleton, initialRs, scale=1.0):
    jointMotions = []
    animationEles = dom.getElementsByTagName('animation')
    
    for animationEle in animationEles:
        jointMotion = ym.Motion()
#        jointMotion.resourceName = animationEle.getAttribute('name').encode()
        trackEles = animationEle.getElementsByTagName('track')
        first_keyframes = trackEles[0].getElementsByTagName('keyframe')
        len_keyframes = len(first_keyframes)
        time2frameMap = {}
        for i in range(len_keyframes):
            jointPosture = ym.JointPosture(jointSkeleton)
#            jointPosture.initLocalRMap(initialRMap)
            jointPosture.initLocalRs(initialRs)
            jointMotion.append(jointPosture)
            
            # because each bone may not have same number of keyframes
            time2frameMap[first_keyframes[i].getAttribute('time')] = i  
        
        for trackEle in trackEles:
#            print i, trackEle.getAttribute('bone'), len(trackEle.getElementsByTagName('keyframe'))
            keyframeEles = trackEle.getElementsByTagName('keyframe')
            
            for keyframeEle in keyframeEles:
                keyframeTime = keyframeEle.getAttribute('time')
                
                # because each bone may not have same number of keyframes
                frame = time2frameMap[keyframeTime]
                jointPosture = jointMotion[frame]
                
                boneName = trackEle.getAttribute('bone').encode()
                if boneName == jointSkeleton.root.name:
                    transEle = keyframeEle.getElementsByTagName('translate')[0]
                    jointPosture.rootPos[0] = float(transEle.getAttribute('x')) * scale
                    jointPosture.rootPos[1] = float(transEle.getAttribute('y')) * scale
                    jointPosture.rootPos[2] = float(transEle.getAttribute('z')) * scale
                    
                rotEle = keyframeEle.getElementsByTagName('rotate')[0]
                angle = float(rotEle.getAttribute('angle'))
                axisEle = rotEle.getElementsByTagName('axis')[0]
                axis = mmMath.v3(float(axisEle.getAttribute('x')), float(axisEle.getAttribute('y')), float(axisEle.getAttribute('z')))
                R = mmMath.exp(axis, angle)
                
#                jointPosture.mulLocalR(boneName, R)
                jointPosture.mulLocalR(jointSkeleton.getElementIndex(boneName), R)
                jointPosture.updateGlobalT()
                    
        jointMotions.append(jointMotion)
    return jointMotions

if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import profile
    import copy
    from datetime import datetime
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Mesh.ysMeshUtil as ysu
    import Util.ysGlHelper as ygh
    import Resource.ysMotionLoader as yf

    def test_dom():
        nodeTypes = ['ELEMENT_NODE', 'ATTRIBUTE_NODE', 'TEXT_NODE', 'CDATA_SECTION_NODE', 'ENTITY_NODE', 'PROCESSING_INSTRUCTION_NODE', 'COMMENT_NODE', 'DOCUMENT_NODE', 'DOCUMENT_TYPE_NODE', 'NOTATION_NODE']
        
        def _print(ele, depth=0):
            s = ' '*depth
            s += 'name: ' + ele.nodeName + ', ' 
            s += 'type: ' + nodeTypes[ele.nodeType] + ', '
            s += 'value: ' + repr(ele.nodeValue) + ', '
            s += 'attrs: ['
            if ele.attributes:
                for i in range(ele.attributes.length):
                    s += ele.attributes.item(i).name + ', '
            s += ']'
            print s 
                    
            for child in ele.childNodes:
                _print(child, depth+2)
                
        xmlFilePath = '../samples/test.xml'
        dom = xml.dom.minidom.parse(xmlFilePath)
        _print(dom)
            
    def test_sax():
        class TestSaxHandler(xml.sax.handler.ContentHandler):
            def startDocument(self):
                print 'startDocument: '
            def startElement(self, name, attrs):
                print 'startElement: ', name
            def endElement(self, name):
                print 'endElement: ', name
                
        xmlFilePath = '../samples/test.xml'
        parser = xml.sax.make_parser()   
        handler = TestSaxHandler()
        parser.setContentHandler(handler)
        parser.parse(open(xmlFilePath))
        
    def test_readOgreMeshFileAsMesh():
        meshFilePath = '../samples/woody2_15.mesh.xml'
        mesh = readOgreMeshFileAsMesh(meshFilePath, .01)
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))
        viewer.doc.addObject('mesh', mesh)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_readOgreSkeletonFile_Skeleton():
#        meshFilePath = '../samples/woody2_15.mesh.xml'
        meshFilePath = '../samples/physics2_woody_binding1.mesh.xml'
        mesh = readOgreMeshFileAsMesh(meshFilePath, .01)
        
#        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        skeletonFilePath = '../samples/physics2_woody_binding1.skeleton.xml'
        jointSkeleton, initialRs = readOgreSkeletonFile_Skeleton(skeletonFilePath, .01)
        
        skeletonPosture = ym.JointPosture(jointSkeleton)
        skeletonPosture.initLocalRs(initialRs)
#        skeletonPosture.initLocalRs()
        skeletonMotion = ym.Motion([skeletonPosture])
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addMotion(skeletonMotion)
        viewer.doc.addRenderer('skeleton', yr.JointMotionRenderer(skeletonMotion, (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    def test_readOgreSkeletonFile_SkeletonAnimation():
        meshFilePath = '../samples/physics2_woody_binding1.mesh.xml'
        mesh = readOgreMeshFileAsMesh(meshFilePath, .01)
        
#        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        skeletonFilePath = '../samples/physics2_woody_binding1.skeleton.xml'
        jointMotions = readOgreSkeletonFile_SkeletonAnimations(skeletonFilePath, .01)
        
        viewer = ysv.SimpleViewer()
        for i in range(len(jointMotions)):
            viewer.doc.addMotion(jointMotions[i])
            viewer.doc.addRenderer(jointMotions[i].resourceName, yr.JointMotionRenderer(jointMotions[i], (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))

        def extraDrawCallback():
            frame = viewer.getCurrentFrame()
            for i in range(jointMotions[0][0].skeleton.getElementNum()):
                ygh.drawPoint(jointMotions[0][frame].getPosition(i))
        
        viewer.setExtraDrawCallback(extraDrawCallback)

        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    def test_readOgreMeshFileAsSkinMesh():
        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        
#        jointSkeleton, initialRMap = readOgreSkeletonFile_Skeleton(skeletonFilePath, .1)
#        jointMotions = readOgreSkeletonFile_SkeletonAnimations(skeletonFilePath, .1)
        jointSkeleton, initialRs, jointMotions = readOgreSkeletonFile(skeletonFilePath, .01)
        
        meshFilePath = '../samples/woody2_15.mesh.xml'
        skinMesh = readOgreMeshFileAsSkinMesh(meshFilePath, jointSkeleton, initialRs, .01)

        jointMotion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)        
        
        viewer = ysv.SimpleViewer()
#        for i in range(len(jointMotions)):
#            viewer.doc.addMotion(jointMotions[i])
#            viewer.doc.addRenderer(jointMotions[i].resourceName, yr.JointMotionRenderer(jointMotions[i], (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addRenderer('jointMotion', yr.JointMotionRenderer(jointMotion, (0,0,255), yr.LINK_BONE))
        viewer.doc.addObject('jointMotion', jointMotion)
        viewer.doc.addRenderer('skinMesh', yr.MeshRenderer(skinMesh))
        
        def preFrameCallback(frame):
            skinMesh.update(jointMotion[frame])
        
        viewer.setPreFrameCallback(preFrameCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_readOgreDataFiles():
        meshFilePath = '../samples/physics2_woody_binding1.mesh.xml'
        skeletonFilePath = '../samples/physics2_woody_binding1.skeleton.xml'
        skinMesh, jointMotions = readOgreDataFiles(meshFilePath, .01, skeletonFilePath)
        
        for jointPosture in jointMotions[0]:
            jointPosture.updateGlobalT()
        
        viewer = ysv.SimpleViewer()
        for i in range(len(jointMotions)):
            viewer.doc.addObject('motion%d'%i, jointMotions[i])
            viewer.doc.addRenderer('motion%d'%i, yr.JointMotionRenderer(jointMotions[i], (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addRenderer('skinMesh', yr.MeshRenderer(skinMesh))
        
        def preFrameCallback(frame):
            skinMesh.update(jointMotions[0][frame])
        
        viewer.setPreFrameCallback(preFrameCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_compare_skeletonanimation_vs_bvhmotion():
        meshFilePath = '../samples/physics2_woody_binding1.mesh.xml'
        skeletonFilePath = '../samples/physics2_woody_binding1.skeleton.xml'
        skeletonMesh, skeletonMotions = readOgreDataFiles(meshFilePath, .01, skeletonFilePath)
        skeletonMotion = skeletonMotions[0]
        ysu.mergePoints(skeletonMesh)
        
        bvhMotion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)
        bvhMesh = copy.deepcopy(skeletonMesh)
#        bvhMesh.initialize(bvhMotion[0])
        
        viewer = ysv.SimpleViewer()
        viewer.doc.addRenderer('skeletonMotion', yr.JointMotionRenderer(skeletonMotion, (0, 0, 255), yr.LINK_LINE))
        viewer.doc.addMotion2('skeletonMotion', skeletonMotion)
        viewer.doc.addRenderer('skeletonMesh', yr.MeshRenderer(skeletonMesh, (255*.5,255*.5,255)))
        
        viewer.doc.addRenderer('bvhMotion', yr.JointMotionRenderer(bvhMotion, (0, 255, 0), yr.LINK_LINE))
        viewer.doc.addMotion2('bvhMotion', bvhMotion)
        viewer.doc.addRenderer('bvhMesh', yr.MeshRenderer(bvhMesh, (255*.5,255,255*.5)))
    
        def preFrameCallback(frame):
            if frame < len(skeletonMotion):
                skeletonMesh.update(skeletonMotion[frame])
            if frame < len(bvhMotion):
                bvhMesh.update(bvhMotion[frame])
            
        viewer.setPreFrameCallback(preFrameCallback)
            
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
            
#    test_sax()
#    test_dom()

#    test_readOgreMeshFileAsMesh()
#    test_readOgreSkeletonFile_Skeleton()
#    test_readOgreSkeletonFile_SkeletonAnimation()                    
    test_readOgreMeshFileAsSkinMesh()
#    test_readOgreDataFiles()
#    test_compare_skeletonanimation_vs_bvhmotion()
