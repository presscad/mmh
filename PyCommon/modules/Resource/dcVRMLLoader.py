import numpy, math
import cPickle

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import Resource.ysMotionLoader as yml

class Wrl:

    childTypes = ['JOINT', 'SEGMENT', 'TRANSFORM', 'SHAPE']

    class Child:
        def __init__(self):
            self.childType = None
        def getType(self):
            return self.childType

    class Joint(Child):
        def __init__(self, name):
            self.childType = 'JOINT'
            self.name = name
            self.jointType = None
            self.translation = None
            self.jointAxes = []
            self.children = []
        def __strHierarchy__(self, depth=0):
            s = ''
            tab1 = '  '*depth
            tab2 = '  '*(depth+1)
            s += '%sJOINT %s\n'%(tab1, self.name)
            s += '%s{\n'%tab1
            s += '%sTRANSLATION %s\n'%(tab2, self.translation)

            jointAxesString = ''
            for axis in self.jointAxes:
                jointAxesString += axis +' '
            s += '%sJOINTAXES %s\n'%(tab2, jointAxesString)

            for child in self.children:
                s += child.__strHierarchy__(depth+1)
            s += '%s}\n'%tab1

            return s
        def getChildSegment(self):
            childSegment = None
            for child in self.children:
                if child.getType() == 'SEGMENT':
                    if childSegment == None:
                        childSegment = child
                    else:
                        print "two childsegment exist"

            if childSegment == None:
                print "no childsegment exists"

            return childSegment                

    class Segment(Child):
        def __init__(self):
            self.childType = 'SEGMENT'
            self.centerOfMass = None
            self.mass = None
            self.momentsOfInertia = None
            self.children = []
        def __strHierarchy__(self, depth=0):
            s = ''
            tab1 = '  '*depth
            tab2 = '  '*(depth+1)
            s += '%sSEGMENT\n'%tab1
            s += '%s{\n'%tab1
            s += '%sCENTEROFMASS %s\n'%(tab2, self.centerOfMass)

            s += '%s{\n'%tab1
            s += '%sMASS %s\n'%(tab2, self.mass)

            s += '%s{\n'%tab1
            s += '%sMOMENTSOFINETIA %s\n'%(tab2, self.momentsOfInertia)

            for child in self.children:
                s += child.__strHierarchy__(depth+1)
            s += '%s}\n'%tab1
            
            return s

    class Shape(Child):
        def __init__(self):
            self.childType = 'SHAPE'
            self.objPath = None
        def __strHierarchy__(self, depth=0):
            s = ''
            tab1 = '  '*depth
            tab2 = '  '*(depth+1)
            s = '%sSHAPE\n'%tab1
            s += '%s{\n'%tab1
            s += '%sGEOMETRY OBJ %s\n'%(tab2, self.objPath)

            return s

    class Transform(Child):
        def __init__(self):
            self.childType = 'TRANSFORM'
            self.rotation = None
            self.translation = None
            self.scale = None
            self.children = []
        def __strHierarchy__(self, depth=0):
            s = ''
            tab1 = '  '*depth
            tab2 = '  '*(depth+1)
            
            for child in self.children:
                s += child.__strHierarchy__(depth+1)
            s += '%s}\n'%tab1

            return s

    def __init__(self):
        self.name = None
        self.joints = []

    def __str__(self):
        s = 'NAME\n'
        s += self.name
        s += 'HIERARCHY\n'
        s += self.joints[0].__strHierarchy__()
        
        return s

#===========================================================================
# read
#===========================================================================
    def parseWrl(self, filepath_or_fileobject):
        if isinstance(filepath_or_fileobject, str): 
            file = open(filepath_or_fileobject)
        else:
            file = filepath_or_fileobject

        tokens = file.read().split()
        if not isinstance(filepath_or_fileobject, str): file.close()        
        
        tokens.reverse()
#        print tokens
        
        if tokens.pop().upper() != "DEF":
            print "'DEF' is missing"
            return None
        tokens.pop()
        if tokens.pop().upper() != "HUMANOID":
            print "'Humanoid' is missing"
            return None
        self.parseWrlHumanoid(tokens)

    def parseWrlHumanoid(self, tokens):
        if tokens.pop() != "{":
            print "'{' missing"
            return None
        t = tokens.pop().upper()
        if t == "NAME":
            self.name = tokens.pop()
            t = tokens.pop().upper()
        if t != "HUMANOIDBODY":
            print "'humanoidbody' is missing"
            return None
        if tokens.pop() != "[":
            print "'[' is missing"
            return None
        if tokens.pop().upper() != "DEF":
            print "'DEF' is missing"
            return None
        rootJointName = tokens.pop()
        if tokens.pop().upper() != "JOINT":
            print "'Joint' is missing"
            return None
        self.parseWrlJoint(rootJointName, tokens)

    def parseWrlJoint(self, name, tokens):
        wrlJoint = Wrl.Joint(name)
        self.joints.append(wrlJoint)

        if tokens.pop() != "{":
            print "'{' is missing"
            return None

        endDetected = False
        while not endDetected:
            t = tokens.pop().upper()
            if t == '}':
                endDetected = True
            elif t == 'JOINTTYPE':
                jt = tokens.pop()
                if jt[0] == '"' and jt[-1] == '"':
                    wrlJoint.jointType = jt[1:-1]
                else:
                    wrlJoint.jointType = jt
            elif t == 'TRANSLATION':
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                wrlJoint.translation = numpy.array([x, y, z], float)
            elif t == 'JOINTAXIS':
                jt = tokens.pop().upper()
                for c in jt:
                    if c.upper() == 'Z':
                        wrlJoint.jointAxes.append('ZROTATION')
                    elif c.upper() == 'X':
                        wrlJoint.jointAxes.append('XROTATION')
                    elif c.upper() == 'Y':
                        wrlJoint.jointAxes.append('YROTATION')
            elif t == 'CHILDREN':
                wrlJoint.children = wrlJoint.children + self.parseWrlChildren(tokens)

        return wrlJoint
    
    def parseWrlChildren(self, tokens):
        children = []

        if tokens.pop() != '[':
            print "'[' is missing"
            return None

        endDetected = False
        while not endDetected:
            t = tokens.pop().upper()
            if t == ']':
                endDetected = True
            elif t == 'DEF':
                name = tokens.pop()
                t = tokens.pop().upper()
                if t == 'JOINT':
                    children.append(self.parseWrlJoint(name, tokens))
                else:
                    print "'DEF' is incorrect"
                    return None
            elif t == 'SEGMENT':
                children.append(self.parseWrlSegment(tokens))
            elif t == 'TRANSFORM':
                children.append(self.parseWrlTransform(tokens))
            elif t == 'SHAPE':
                children.append(self.parseWrlShape(tokens))

        return children

    def parseWrlSegment(self, tokens):
        wrlSegment = Wrl.Segment()
        
        if tokens.pop() != '{':
            print "'{' is missing"
            return None

        endDetected = False
        while not endDetected:
            t = tokens.pop().upper()
            if t == '}':
                endDetected = True
            elif t == 'CENTEROFMASS':
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                wrlSegment.centerOfMass = numpy.array([x,y,z],float)
            elif t == 'MASS':
                wrlSegment.mass = float(tokens.pop())
            elif t == 'MOMENTSOFINERTIA':
                mat = []
                for i in range(3):
                    row = []
                    for j in range(3):
                        t = tokens.pop()
                        if t[0] == '[':
                            t = t[1:]
                        elif t[-1] == ']':
                            t = t[:-1]
                        row.append(float(t))
                    mat.append(row)
                wrlSegment.momentsOfInertia = numpy.matrix(mat)
            elif t == 'CHILDREN':
                wrlSegment.children = wrlSegment.children + self.parseWrlChildren(tokens)

        return wrlSegment

    def parseWrlShape(self, tokens):
        wrlShape = Wrl.Shape()
        if tokens.pop() != '{':
            print "'{' is missing"
            return None
        if tokens.pop().upper() != "GEOMETRY":
            print "'GEOMETRY' is missing"
            return None
        if tokens.pop().upper() != "OBJ":
            print "'OBJ' is missing"
            return None
        t = tokens.pop()
        if t[-1] == '}':
            t = t[:-1]
        wrlShape.objPath = t
        return wrlShape

    def parseWrlTransform(self,tokens):
        wrlTransform = Wrl.Transform()

        if tokens.pop() != '{':
            print "'{' is missing"
            return None

        endDetected = False
        while not endDetected:
            t = tokens.pop().upper()
            if t == '}':
                endDetected = True
            elif t == 'ROTATION':
                w = float(tokens.pop())
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                wrlTransform.rotation = numpy.array([w,x,y,z],float)
            elif t == 'TRANSLATION':
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                wrlTransform.translation = numpy.array([x,y,z],float)
            elif t == 'SCALE':
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                wrlTransform.scale = numpy.array([x,y,z],float)
            elif t == 'CHILDREN':
                wrlTransform.children = wrlTransform.children + self.parseWrlChildren(tokens)

        return wrlTransform

#=============================================
#=============================================

    def toBvh(self):
        bvh = yml.Bvh()
        self.wrlJointTobvhJoint(bvh, self.joints[0])
        bvh.frameNum = 1
        bvh.frameTime = 1./30.

        #=============================
        # make default bvh motion
        #=============================
        bvh.motionList = []
        bvh.motionList.append([None]*bvh.totalChannelCount)
        for i in range(bvh.totalChannelCount):
            bvh.motionList[0][i] = 0.

        return bvh

    def wrlJointTobvhJoint(self, bvh, wrlJoint):
        bvhJoint = yml.Bvh.Joint(wrlJoint.name)
        bvh.joints.append(bvhJoint)
        bvhJoint.offset = wrlJoint.translation
        if wrlJoint.jointType.upper() == 'FREE':
            for idof in range(len(yml.Bvh.channelTypes6dof)):
                bvhJoint.channels.append(yml.Bvh.Channel(yml.Bvh.channelTypes6dof[idof], bvh.totalChannelCount + idof))
        else:
            for idof in range(len(wrlJoint.jointAxes)):
                bvhJoint.channels.append(yml.Bvh.Channel(wrlJoint.jointAxes[idof], bvh.totalChannelCount + idof))
        bvh.totalChannelCount += len(bvhJoint.channels)
        hasJointChild = False
        for child in wrlJoint.children:
            if child.getType() == 'JOINT':
                bvhJoint.children.append(self.wrlJointTobvhJoint(bvh,child))
                hasJointChild = True

        if not hasJointChild:
            endSite = yml.Bvh.Joint("%s_Effector"%bvhJoint.name)
            endSite.offset = None
            for child in wrlJoint.children:
                if child.getType() == 'SEGMENT':
                    if endSite.offset == None:
                        endSite.offset = child.centerOfMass.copy()*2
                    else:
                        print "an end site joint has segments more than two"
            bvh.joints.append(endSite)
            bvhJoint.children.append(endSite)
        
        return bvhJoint

#============================================================
# member function
#============================================================
    def getWrlJoint(self, name):
        for joint in self.joints:
            if joint.name == name:
                return joint
        return None

#===========================================================================
# write
#===========================================================================


#=========================================
# function
#=========================================
def readWrlFileasWrl(wrlFilePath_or_wrlFile):
    wrl = Wrl()
    wrl.parseWrl(wrlFilePath_or_wrlFile)
    return wrl

#=========================================
# test
#=========================================

def testParse():
    wrl = Wrl()
    wrl.parseWrl("/home/jo/Research/yslee/Resource/motion/opensim/FullBody2_lee.wrl")
    print wrl

def testToBvh():
    wrl = Wrl()
    wrl.parseWrl("/home/jo/Research/yslee/Resource/motion/opensim/FullBody2_lee.wrl")
    bvh = wrl.toBvh()
    print bvh
    print "Total Channel Count: %d\n"%bvh.totalChannelCount
    #bvh = yml.readBvhFileAsBvh("/home/jo/ys2010/Walking/adun/adun.bvh")
    #aa = bvh.toJointMotion(100, False)
    #bvh.fromJointMotion(bvh.toJointMotion(100, False))
    bvh.writeBvhFile("/home/jo/ys2010/Walking/adun/adun.bvh")

#======
# test
#======
#if __name__=='__main__':
    #testParse()
    #testToBvh()
