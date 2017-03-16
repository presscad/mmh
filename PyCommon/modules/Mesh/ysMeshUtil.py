import copy

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mmMath
import Mesh.ysMesh as yms
import Motion.ysMotion as ym

def mergePoints(mesh, distance = 0.0):
    len_vertices = len(mesh.vertices)
    
    mergeMap = {}
    for fromIndex in range(len_vertices):
        for toIndex in range(fromIndex+1, len_vertices):
            length = mmMath.length(mesh.vertices[fromIndex].pos - mesh.vertices[toIndex].pos)
            if length <= distance:
                mergeMap[fromIndex] = toIndex
#    print mergeMap

    transitions = [i for i in range(len_vertices)]
#    print transitions

    for i in range(len_vertices-1,-1,-1):
#        print i
        if i in mergeMap:
            toBeMerged = i
#            print 'toBeMerged', toBeMerged
            for j in range(toBeMerged+1, len_vertices):
                transitions[j] -= 1
            for key, targetIndex in mergeMap.items():
                if targetIndex >= toBeMerged:
#                    print key, ':', mergeMap[key], '->', mergeMap[key]-1  
                    mergeMap[key] -= 1
                    
#    print transitions
#    print mergeMap

    for toBeMerged in mergeMap:
        transitions[toBeMerged] = mergeMap[toBeMerged]
#    print transitions
    
    # reorder vertex list
    old_vertices = mesh.vertices
    mesh.vertices = [None]*(len_vertices - len(mergeMap))
    for i in range(len(old_vertices)):
        mesh.vertices[transitions[i]] = old_vertices[i]

    # update face
    for f in mesh.faces:
        for j in range(len(f.vertexIndex)):
            f.vertexIndex[j] = transitions[f.vertexIndex[j]]
    for f in mesh.faces[:]:
        for i in range(len(f.vertexIndex)):
            for j in range(i+1, len(f.vertexIndex)):
                if f.vertexIndex[i] == f.vertexIndex[j]:
                    mesh.faces.remove(f)
            
    if(isinstance(mesh, yms.SkinMesh)):
        # reorder bone weight list
        old_vertexBoneWeights = mesh.vertexBoneWeights
        mesh.vertexBoneWeights = [None]*(len_vertices - len(mergeMap))
        for i in range(len(old_vertexBoneWeights)):
            mesh.vertexBoneWeights[transitions[i]] = old_vertexBoneWeights[i]
        mesh.initialize(mesh.initialPosture)

def meshAnimation2PointMotion(skinMesh, jointMotion):
    sm = copy.deepcopy(skinMesh)
    
    pointSkeleton = ym.PointSkeleton()
    for i in range(sm.getVertexNum()):
        pointSkeleton.addElement(None, None)
    
    pointMotion = ym.Motion()
    for jointPosture in jointMotion:
        sm.update(jointPosture)
        pointMotion.append(mesh2PointPosture(sm, pointSkeleton))
    return pointMotion

def mesh2PointPosture(mesh, pointSkeleton=None):
    if pointSkeleton==None:
        pointSkeleton = ym.PointSkeleton()
        for i in range(mesh.getVertexNum()):
            pointSkeleton.addElement(None, None)
         
    pointPosture = ym.PointPosture(pointSkeleton)
#    for v in mesh.vertices:
#        pointPosture.addPoint2(v.pos)
    for i in range(mesh.getVertexNum()):
        pointPosture.setPosition(i, mesh.vertices[i].pos)
    return pointPosture

def vertexBoneWeight2BoneVertexWeights(vertexBoneWeights, jointSkeleton):
#    boneVertexWeights = [None]*len(jointSkeleton.joints)
    boneVertexWeights = [None]*jointSkeleton.getElementNum()
#    for i in range(len(jointSkeleton.joints)):
    for i in range(jointSkeleton.getElementNum()):
        boneVertexWeights[i] = []
    
    for vertexIndex in range(len(vertexBoneWeights)):
        for boneIndex, weight in vertexBoneWeights[vertexIndex]:
            boneVertexWeights[boneIndex].append((vertexIndex, weight))

#    # test
#    vertexIndex = 33
#    boneIndex = skinMesh.vertexBoneWeights[vertexIndex][0][0]
#    weight = skinMesh.vertexBoneWeights[vertexIndex][0][1]
#    print 'vertexIndex:', vertexIndex, 'boneIndex:', boneIndex, 'weight:', weight
#    print 'boneIndex:', boneIndex, boneVertexWeights[boneIndex]
    return boneVertexWeights

def getDistributedVertexMasses(skinMesh, boneMassMap):
    boneVertexWeights = vertexBoneWeight2BoneVertexWeights(skinMesh.vertexBoneWeights, skinMesh.skeleton)
    
#    boneWeights = [0]*len(skinMesh.skeleton.joints)
    boneWeights = [0]*skinMesh.skeleton.getElementNum()
    for boneIndex in range(len(boneVertexWeights)):
        for vertexIndex, weight in boneVertexWeights[boneIndex]:
            boneWeights[boneIndex] += weight
#            vertexMasses[vertexIndex] += weight

    vertexMasses = [0]*len(skinMesh.vertices)
    for boneIndex in range(len(boneVertexWeights)):
        for vertexIndex, weight in boneVertexWeights[boneIndex]:
#            vertexMasses[vertexIndex] += weight/boneWeights[boneIndex] * boneMassMap[skinMesh.skeleton.jointsAr[boneIndex].name] 
            vertexMasses[vertexIndex] += weight/boneWeights[boneIndex] * boneMassMap[skinMesh.skeleton.getElementName(boneIndex)] 

    return vertexMasses
    
#def getDistributedVertexMasses2(skinMesh, boneMassMap):
#    vertexMasses = [0]*len(skinMesh.vertexBoneWeights)
#    for vertexIndex in range(len(skinMesh.vertexBoneWeights)):
#        totalBoneMass = 0
#        for boneIndex, weight in skinMesh.vertexBoneWeights[vertexIndex]:
#            boneName = skinMesh.skeleton.jointsAr[boneIndex].name
#            totalBoneMass += boneMassMap[boneName]
#        totalBoneMass /= len(skinMesh.vertexBoneWeights[vertexIndex])
#        for boneIndex, weight in skinMesh.vertexBoneWeights[vertexIndex]:
#            vertexMasses[vertexIndex] += weight * totalBoneMass
#    return vertexMasses

def getTotalMass(vertexMasses, vertexIndices=None):
    if vertexIndices==None:
        vertexIndices = range(len(vertexMasses))

    totalMass = 0.
    for vi in vertexIndices:
        totalMass += vertexMasses[vi] 
    return totalMass

def getSubmeshVertexIndicesEx(mesh, partialSubmeshName):
    for submeshName in mesh.submeshMap:
        if partialSubmeshName in submeshName:
            return mesh.getSubmeshVertexIndices(submeshName)
    return []
    
def getCOMPos(positions, vertexMasses, totalMass, vertexIndices=None):
    if vertexIndices==None:
        vertexIndices = range(len(vertexMasses))

    comPos = mmMath.Vec3(0.,0.,0.)
    for vi in vertexIndices:
        weightedPos = positions[vi] * vertexMasses[vi]
        comPos += weightedPos
    comPos /= totalMass
    return comPos
    
def getCOMVel(velocities, vertexMasses, totalMass, vertexIndices=None):
    if vertexIndices==None:
        vertexIndices = range(len(vertexMasses))

    comPos = mmMath.Vec3(0.,0.,0.)
    for vi in vertexIndices:
        weightedPos = velocities[vi] * vertexMasses[vi]
        comPos += weightedPos
    comPos /= totalMass
    return comPos

if __name__=='__main__':
    import psyco; psyco.full()
    from fltk import *
    import copy, numpy
    import Resource.ysOgreDataLoader as yol
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Resource.ysMotionLoader as yf
    import Util.ysGlHelper as ygh
    import Math.mmMath as mmMath
    
    def test_massMap():
        massMap = {}
        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
        
        # torso : 10
        massMap['Hips'] += 2.
        massMap['Spine'] += 6.
        massMap['Spine1'] += 2.
    
        # head : 3
        massMap['Spine1'] += 1.
        massMap['Head'] += 2.
        
        # right upper arm : 2
        massMap['Spine1'] += .5
        massMap['RightShoulder'] += .5
        massMap['RightArm'] += 1.
        
        # left upper arm : 2
        massMap['Spine1'] += .5
        massMap['LeftShoulder1'] += .5
        massMap['LeftArm'] += 1.
        
        # right lower arm : 1
        massMap['RightForeArm'] = .8
        massMap['RightHand'] = .2 
        
        # left lower arm : 1
        massMap['LeftForeArm'] = .8
        massMap['LeftHand'] = .2 
        
        # right thigh : 7
        massMap['Hips'] += 2.
        massMap['RightUpLeg'] += 5.
        
        # left thigh : 7
        massMap['Hips'] += 2.
        massMap['LeftUpLeg'] += 5.
        
        # right shin : 5
        massMap['RightLeg'] += 5.
        
        # left shin : 5
        massMap['LeftLeg'] += 5.
        
        # right foot : 4
        massMap['RightFoot'] += 2.
        massMap['RightToes'] += 2.
        
        # left foot : 4
        massMap['LeftFoot'] += 2.
        massMap['LeftToes'] += 2.
        
        return massMap
    massMap = test_massMap()
        
    def test_mergePoints():
        meshFilePath = '../samples/physics2_woody_binding1.mesh.xml'
        skinMesh, jointMotions = yol.readOgreDataFiles(meshFilePath, .1)

        mergedSkinMesh = copy.deepcopy(skinMesh)
        mergePoints(mergedSkinMesh)
        
#        print skinMesh
#        print mergedSkinMesh

        viewer = ysv.SimpleViewer()
        viewer.record(False)
        for i in range(len(jointMotions)):
            viewer.doc.addRenderer(jointMotions[i].resourceName, yr.JointMotionRenderer(jointMotions[i], (0, 0, 255), yr.LINK_LINE))
            viewer.doc.addMotion2(jointMotions[i].resourceName, jointMotions[i])
        viewer.doc.addRenderer('skinMesh', yr.MeshRenderer(skinMesh))
        viewer.doc.addObject('skinMesh', skinMesh)
        viewer.doc.addRenderer('mergedSkinMesh', yr.MeshRenderer(mergedSkinMesh, (0,255,0)))
        viewer.doc.addObject('mergedSkinMesh', mergedSkinMesh)
        
        def preFrameCallback(frame):
            skinMesh.update(jointMotions[0][frame])
            mergedSkinMesh.update(jointMotions[0][frame])
        
        viewer.setPreFrameCallback(preFrameCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
                
    def test_mergePoints2():
        mesh = yms.Mesh()
        mesh.vertices = [yms.Vertex() for i in range(5)]
        
        mesh.vertices[0].pos = numpy.array([1.,0.,0.],float)
        mesh.vertices[1].pos = numpy.array([0.,0.,0.],float)
        mesh.vertices[2].pos = numpy.array([2.,0.,0.],float)
        mesh.vertices[3].pos = numpy.array([0.,0.,0.],float)
        mesh.vertices[4].pos = numpy.array([0.,0.,0.],float)
        print '<before merge>'
        print mesh
        
        mergePoints(mesh)
        print '<after merge>'
        print mesh
        
    def test_mesh2PointPosture():
        meshFilePath = '../samples/woody2_15.mesh.xml'
        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        skinMesh, js = yol.readOgreDataFiles(meshFilePath, .01, skeletonFilePath)
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
        
        meshPointMotion = meshAnimation2PointMotion(skinMesh, jointMotion)

        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('skinMesh', yr.MeshRenderer(skinMesh))
        viewer.doc.addObject('skinMesh', skinMesh)
        viewer.doc.addRenderer('meshPointMotion', yr.PointMotionRenderer(meshPointMotion))
        viewer.doc.addObject('meshPointMotion', meshPointMotion)
        
        def preFrameCallback(frame):
            skinMesh.update(jointMotion[frame])
        viewer.setPreFrameCallback(preFrameCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()

    def test_getDistributedMasses():

        totalMass = 0
        for key, value in massMap.items():
            print key, value
            totalMass += value
        print totalMass
        
        
        meshFilePath = '../samples/woody2_15.mesh.xml'
        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        skinMesh, js = yol.readOgreDataFiles(meshFilePath, .01, skeletonFilePath)
        
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        jointMotion = yf.readBvhFile(bvhFilePath, .01)
        
        vertexMasses = getDistributedVertexMasses(skinMesh, massMap)
        totalMass = 0
        for i in range(len(vertexMasses)):
            print i, vertexMasses[i]
            totalMass += vertexMasses[i]
        print totalMass
        print
    
#        vertexMasses = getDistributedVertexMasses2(skinMesh, massMap)
#        totalMass = 0
#        for i in range(len(vertexMasses)):
#            print i, vertexMasses[i]
#            totalMass += vertexMasses[i]
#        print totalMass
    
    def test_getCOMPos_Vel():
        meshFilePath = '../samples/woody2_15.mesh.xml'
        skeletonFilePath = '../samples/woody2_15.skeleton.xml'
        mesh, js = yol.readOgreDataFiles(meshFilePath, .01, skeletonFilePath)
        
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        
        meshMotion = meshAnimation2PointMotion(mesh, motion)
        
        vertexMasses = getDistributedVertexMasses(mesh, massMap)
        upperIndices = getSubmeshVertexIndicesEx(mesh, 'UPPER')
        totalMass = getTotalMass(vertexMasses)
        upperMass = getTotalMass(vertexMasses, upperIndices)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))
        viewer.doc.addObject('mesh', mesh)
        viewer.doc.addRenderer('meshMotion', yr.PointMotionRenderer(meshMotion))
        viewer.doc.addObject('meshMotion', meshMotion)
    
        def extraDrawCallback():
            frame = viewer.getCurrentFrame()
            positions = meshMotion[frame].getPositions()
            COM = getCOMPos(positions, vertexMasses, totalMass)
            upperCOM = getCOMPos(positions, vertexMasses, upperMass, upperIndices)

            if frame > 0:
                velocities = meshMotion.getVelocities(frame)
                ygh.drawVector(getCOMVel(velocities, vertexMasses, totalMass), COM, (255,255,0))
                ygh.drawVector(getCOMVel(velocities, vertexMasses, upperMass, upperIndices), upperCOM, (0,255,0))
    
        viewer.setExtraDrawCallback(extraDrawCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        Fl.run()
        
    
    pass
#    test_mergePoints()
#    test_mergePoints2()
#    test_mesh2PointPosture()
#    test_getDistributedMasses()
    test_getCOMPos_Vel()