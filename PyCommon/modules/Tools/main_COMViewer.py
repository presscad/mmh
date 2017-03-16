import psyco; psyco.full()
from fltk import *

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv
import Resource.ysOgreDataLoader as yol
import Mesh.ysMeshUtil as ysu
import Util.ysGlHelper as ygh

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

if __name__ == "__main__":
#    mesh, js = yol.readOgreDataFiles('Data/woody2_7.mesh.xml', .01)
    mesh, js = yol.readOgreDataFiles('../samples/woody2_15.mesh.xml', .01)
    
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkSameSame00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkForwardSlow01.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkForwardFast00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkForwardVFast00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkBackward00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkAzuma00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkSoldier00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkLean00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_WalkSukiko00.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_2foot_walk_turn2.bvh', .01)
#    motion, f = yf.readBvhFileAsJointMotion('Data/wd2_2foot_walk_turn.bvh', .01)
    motion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)
    meshMotion = ysu.meshAnimation2PointMotion(mesh, motion)

    vertexMasses = ysu.getDistributedVertexMasses(mesh, massMap)
    upperVertexIndices, upperMass = ysu.getUpperInfo(mesh, vertexMasses)
    totalMass = ysu.getTotalMass(vertexMasses)

    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh))
    viewer.doc.addObject('mesh', mesh)
    viewer.doc.addRenderer('meshMotion', yr.PointMotionRenderer(meshMotion))
    viewer.doc.addObject('meshMotion', meshMotion)

    def preFrameCallback(frame):
        mesh.update(motion[frame])

    def extraDrawCallback():
        frame = viewer.getCurrentFrame()
        meshPositions = meshMotion[frame].getPointPositions()
        ygh.drawPoint(ysu.getUpperCOMPos(meshPositions, vertexMasses, upperVertexIndices, upperMass))
        ygh.drawPoint(ysu.getCOMPos(meshPositions, vertexMasses, totalMass), (255,255,0))
        if frame > 0:
            meshVelocities = meshMotion.getPointVelocities(frame, frame - 1)
            ygh.drawVector(ysu.getUpperCOMVel(meshVelocities, vertexMasses, upperVertexIndices, upperMass), ysu.getUpperCOMPos(meshPositions, vertexMasses, upperVertexIndices, upperMass))
            ygh.drawVector(ysu.getCOMVel(meshVelocities, vertexMasses, totalMass), ysu.getCOMPos(meshPositions, vertexMasses, totalMass), (255,255,0))

    viewer.setPreFrameCallback(preFrameCallback)
    viewer.setExtraDrawCallback(extraDrawCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    Fl.run()
    