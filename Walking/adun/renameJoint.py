# rename joints of adun
#
# shoulder2 -> shoulder_r
# elbow -> elbow_r
# radioulnar -> radioulnar_r
# shoulder2_l -> shoulder_l

import cPickle

import sys
if '../../PyCommon/modules' not in sys.path:
    sys.path.append('../../PyCommon/modules')
import Resource.ysMotionLoader as yml
import Resource.dcVRMLLoader as dvl
import numpy.core.multiarray

def buildJointMap():
    jointMap = {}
    jointMap["shoulder2"] = "shoulder_r"
    jointMap["elbow"] = "elbow_r"
    jointMap["radioulnar"] = "radioulnar_r"
    jointMap["shoulder2_l"] = "shoulder_l"
    return jointMap

def renameBvhJoint(bvh):
    jointMap = buildJointMap()
    for joint in bvh.joints:
        for k in jointMap.keys():
            if joint.name == k:
                joint.name = jointMap[k]
   
    return bvh

def renameWrlJoint(wrl):
    jointMap = buildJointMap()
    for joint in wrl.joints:
        for k in jointMap.keys():
            if joint.name == k:
                joint.name = jointMap[k]

    return wrl

def renameMuscleJoint(msclDict):
    # rename joint names of path points of muscles
    jointMap = buildJointMap()
    
    for mscl in msclDict.values():
        pathPointSet = mscl["GeometryPath"]["PathPointSet"]
        for pathPoint in pathPointSet.values():
            for k in jointMap.keys():
                if pathPoint["joint"] == k:
                    pathPoint["joint"] = jointMap[k]

    return msclDict

if __name__=='__main__':

    bvhFilePath = './ad.bvh'
    msclFilePath = './mscl'

    bvh = yml.readBvhFileAsBvh(bvhFilePath)
    bvh = renameBvhJoint(bvh)
    bvh.writeBvhFile(bvhFilePath)
    
    msclFile = open(msclFilePath, 'rb')
    msclDict = cPickle.load(msclFile)
    msclFile.close()
    msclDict = renameMuscleJoint(msclDict)
    msclFile = open(msclFilePath, 'wb')
    cPickle.dump(msclDict, msclFile)
    msclFile.close()
