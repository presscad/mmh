import sys, numpy, ode, math, copy

if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

class OdeModel():
    class Node:
        def __init__(self, name):
            self.name = name
            self.body = None
            self.joint = None
            self.motor = None
            self.geom = None
            self.Kp = None
            self.Kd = None
        def __str__(self):
            return str(self.__dict__)

    def __init__(self, world, space, createPosture, config):
        self.world = world
        self.space = space
        self.nodes = {}
        self.boneTs = {}
        self.boneRs = {}
        self.newTs = {}
        self.config = config
        self.rootNode = None
        self.createBodies(createPosture)
    def __str__(self):
        s = ''
        s += '<NODES>\n'
        for key in self.nodes:
            s += '[%s]:\'%s\', '%(key, self.nodes[key].name)
        s += '<BODY MASSES>\n'
        for key in self.nodes:
            s += '[%s]:\'%s\', '%(key, self.nodes[key].body.getMass().mass)
        s += '<BODY INERTIAS>\n'
        for key in self.nodes:
            s += '[%s]:\'%s\', '%(key, self.nodes[key].body.getMass()) + '\n'
        return s
    def getBodyNum(self):
        return len(self.nodes)
    def getBody(self, name):
        return self.nodes[name].body
    def getBodyPositionsGlobal(self):
        return [node.body.getPosition() for node in self.nodes.values()]
    def getBodyVelocitiesGlobal(self):
        return [node.body.getLinearVel() for node in self.nodes.values()]
    def getBodyAngVelocitiesGlobal(self):
        return [node.body.getAngularVel() for node in self.nodes.values()]
    def getBodies(self):
        bodies = []
        for node in self.nodes.values():
            bodies.append(node.body)
        return bodies
    def setState(self, state):
        for body in self.getBodies():
            body.setPosition(state[body][0])
            body.setQuaternion(state[body][1])
            body.setLinearVel(state[body][2])
            body.setAngularVel(state[body][3])
    def getState(self):
        state = {}
        for body in self.getBodies():
            p = body.getPosition()
            r = body.getQuaternion()
            lv = body.getLinearVel()
            av = body.getAngularVel()
            state[body] = (p, r, lv, av)
        return state
      
    def createBodies(self, posture):
        joint = posture.skeleton.root
        T = mm.TransVToSE3(posture.rootPos)

#        self._createBody(joint, T, posture)
        tpose = posture.getTPose() 
        self._createBody(joint, T, tpose)
        
        self.rootNode = self.nodes[joint.name]

    def _createBody(self, joint, parentT, posture):
        T = parentT
        
        P = mm.TransVToSE3(joint.offset)
        T = numpy.dot(T, P)
 
#        R = mm.SO3ToSE3(posture.localRMap[joint.name])
        R = mm.SO3ToSE3(posture.localRs[posture.skeleton.getElementIndex(joint.name)])
        T = numpy.dot(T, R)
        
        if len(joint.children)>0 and joint.name in self.config.nodes:
            offset = numpy.array([0.,0.,0.])
            for childJoint in joint.children:
                offset += childJoint.offset
            offset = offset/len(joint.children)
            
            boneT = mm.TransVToSE3(offset/2.)
            
            defaultBoneV = numpy.array([0,0,1])
            boneR = mm.getSO3FromVectors(defaultBoneV, offset)
            self.boneRs[joint.name] = boneR
            boneT = numpy.dot(boneT, mm.SO3ToSE3(boneR))
            
            node = OdeModel.Node(joint.name)
            self.nodes[joint.name] = node
            
            node.body = ode.Body(self.world)
            mass = ode.Mass()
            
            
            cfgNode = self.config.getNode(joint.name)
            if cfgNode.length:
                length = cfgNode.length * cfgNode.boneRatio
            else:
                length = mm.length(offset) * cfgNode.boneRatio

            if cfgNode.width:
                width = cfgNode.width
                if cfgNode.mass:
                    height = (cfgNode.mass/(cfgNode.density*length))/width
                else:
                    height = .1
            else:
                if cfgNode.mass:
                    width = (cfgNode.mass/(cfgNode.density*length))**.5
                else:
                    width = .1
                height = width
                    
            node.geom = ode.GeomBox(self.space, (width, height, length))
            node.geom.name = joint.name
            mass.setBox(cfgNode.density, width, height, length)
            
            boneT = numpy.dot(boneT, mm.TransVToSE3(cfgNode.offset))
            self.boneTs[joint.name] = boneT
            newT = numpy.dot(T, boneT)

            p = mm.SE3ToTransV(newT)
            r = mm.SE3ToSO3(newT)
            node.geom.setBody(node.body)
            node.body.setMass(mass)
            node.body.setPosition(p)
            node.body.setRotation(mm.SO3ToOdeSO3(r))
    
        for childJoint in joint.children:
            self._createBody(childJoint, T, posture)

    def update(self, posture):
        joint = posture.skeleton.root
        T = mm.TransVToSE3(posture.rootPos)
        self._updateBody(joint, T, posture)
        
    def _updateBody(self, joint, parentT, posture):
        P = mm.TransVToSE3(joint.offset)
        T = numpy.dot(parentT, P)
 
#        R = mm.SO3ToSE3(posture.localRMap[joint.name])
        R = mm.SO3ToSE3(posture.localRs[posture.skeleton.getElementIndex(joint.name)])
        T = numpy.dot(T, R)
        
        if len(joint.children)>0 and joint.name in self.config.nodes:
            boneT = self.boneTs[joint.name]
            newT = numpy.dot(T, boneT)
            
            node = self.nodes[joint.name] 
            cfgNode = self.config.getNode(joint.name)
#                newT = numpy.dot(newT, mm.TransVToSE3(cfgNode.offset))
            
            p = mm.SE3ToTransV(newT)
            r = mm.SE3ToSO3(newT)

            node.body.setPosition(p)
            node.body.setRotation(mm.SO3ToOdeSO3(r))
    
        for childJoint in joint.children:
            self._updateBody(childJoint, T, posture)        

class OdeControlModel(OdeModel):
    def __init__(self, world, space, createPosture, config = None):
        OdeModel.__init__(self, world, space, createPosture, config)
        
#        self.createJoints(createPosture)
        self.createJoints(createPosture.getTPose())
        
        self.update(createPosture)
        
        for node in self.nodes.values():
            if isinstance(node.joint, ode.FixedJoint):
                node.joint.setFixed()
                
#        self.Rpds = {}
#        self.Rpcs = {}
        
    def createJoints(self, posture):
        joint = posture.skeleton.root
        T = mm.TransVToSE3(posture.rootPos)
        self._createJoint(joint, T, posture)
        
    def _createJoint(self, joint, parentT, posture):
        P = mm.TransVToSE3(joint.offset)
        T = numpy.dot(parentT, P)
 
#        R = mm.SO3ToSE3(posture.localRMap[joint.name])
        R = mm.SO3ToSE3(posture.localRs[posture.skeleton.getElementIndex(joint.name)])
        T = numpy.dot(T, R)
        
        temp_joint = joint
        nodeExistParentJoint = None
        while True:
            if temp_joint.parent == None:
                nodeExistParentJoint = None
                break
            elif temp_joint.parent.name in self.nodes: 
                nodeExistParentJoint = temp_joint.parent
                break
            else:
                temp_joint = temp_joint.parent
        
#        if joint.parent and len(joint.children)>0:
        if nodeExistParentJoint and len(joint.children)>0:
            if joint.name in self.config.nodes:
                p = mm.SE3ToTransV(T)
                
                node = self.nodes[joint.name] 
                cfgNode = self.config.getNode(joint.name)
                if cfgNode.jointAxes != None:
                    if len(cfgNode.jointAxes)==0:
                        node.joint = ode.BallJoint(self.world)
                        node.motor = ode.AMotor(self.world)
                        
#                            node.joint.attach(self.nodes[joint.parent.name].body, node.body)
                        node.joint.attach(self.nodes[nodeExistParentJoint.name].body, node.body)
                        node.joint.setAnchor(p)
#                            node.motor.attach(self.nodes[joint.parent.name].body, node.body)
                        node.motor.attach(self.nodes[nodeExistParentJoint.name].body, node.body)
                        node.motor.setNumAxes(3)
                        
                        node.motor.setAxis(0, 0, (1,0,0))
                        node.motor.setAxis(1, 0, (0,1,0))
                        node.motor.setAxis(2, 0, (0,0,1))
                        
                        node.motor.setParam(ode.ParamLoStop, cfgNode.jointLoStop)
                        node.motor.setParam(ode.ParamHiStop, cfgNode.jointHiStop)
                        node.motor.setParam(ode.ParamLoStop2, cfgNode.jointLoStop)
                        node.motor.setParam(ode.ParamHiStop2, cfgNode.jointHiStop)
                        node.motor.setParam(ode.ParamLoStop3, cfgNode.jointLoStop)
                        node.motor.setParam(ode.ParamHiStop3, cfgNode.jointHiStop)
                        
                    elif len(cfgNode.jointAxes)==1:
                        node.joint = ode.HingeJoint(self.world)
                        
#                            node.joint.attach(self.nodes[joint.parent.name].body, node.body)
                        node.joint.attach(self.nodes[nodeExistParentJoint.name].body, node.body)
                        node.joint.setAnchor(p)
                        
                        newR = mm.SE3ToSO3(self.newTs[joint.name])
                        node.joint.setAxis(numpy.dot(newR, cfgNode.jointAxes[0]))
                        
                        node.joint.setParam(ode.ParamLoStop, cfgNode.jointLoStop)
                        node.joint.setParam(ode.ParamHiStop, cfgNode.jointHiStop)
                        
                    elif len(cfgNode.jointAxes)==2:
                        pass
                        node.joint = ode.UniversalJoint(self.world)
#                        
#                            node.joint.attach(self.nodes[joint.parent.name].body, node.body)
                        node.joint.attach(self.nodes[nodeExistParentJoint.name].body, node.body)
                        node.joint.setAnchor(p)
                        
                        newR = mm.SE3ToSO3(self.newTs[joint.name])
                        node.joint.setAxis1(numpy.dot(newR, cfgNode.jointAxes[0]))
                        node.joint.setAxis2(numpy.dot(newR, cfgNode.jointAxes[1]))
                        
                        node.joint.setParam(ode.ParamLoStop, cfgNode.jointLoStop)
                        node.joint.setParam(ode.ParamHiStop, cfgNode.jointHiStop)
                        node.joint.setParam(ode.ParamLoStop2, cfgNode.jointLoStop)
                        node.joint.setParam(ode.ParamHiStop2, cfgNode.jointHiStop)
                else:
                    node.joint = ode.FixedJoint(self.world)
#                        node.joint.attach(self.nodes[joint.parent.name].body, node.body)
                    node.joint.attach(self.nodes[nodeExistParentJoint.name].body, node.body)
#                    node.joint.setFixed()
                    
                node.Kp = cfgNode.Kp
                node.Kd = cfgNode.Kd
                
            else:
                pass
            
        for childJoint in joint.children:
            self._createJoint(childJoint, T, posture)        
                    
    def fixRootBody(self):
        self.fixedJoint = ode.FixedJoint(self.world)
        self.fixedJoint.attach(self.rootNode.body, ode.environment)
        self.fixedJoint.setFixed()

    def calcPDTorque(self, posture):
        torques = {}
        joint = posture.skeleton.root
        R = mm.I_SO3()
        self._calcPDTorqueJoint(joint, R, posture, torques)
        return torques 
                
    def _calcPDTorqueJoint(self, joint, parentR, posture, torques):
#        R = numpy.dot(parentR, posture.localRMap[joint.name])
        R = numpy.dot(parentR, posture.localRs[posture.skeleton.getElementIndex(joint.name)])
        
#        if joint.name in self.nodes and joint.parent:
        temp_joint = joint
        nodeExistParentJoint = None
        while True:
            if temp_joint.parent == None:
                nodeExistParentJoint = None
                break
            elif temp_joint.parent.name in self.nodes: 
                nodeExistParentJoint = temp_joint.parent
                break
            else:
                # Gp' * Lc' = Gc' (= Gp)
                # Gp' = Gp * inv(Lc')
                parentR = numpy.dot(parentR, numpy.transpose(posture.localRs[posture.skeleton.getElementIndex(temp_joint.parent.name)]))
                temp_joint = temp_joint.parent

        if joint.name in self.nodes and nodeExistParentJoint: 
            node = self.nodes[joint.name]
            ode_joint = node.joint
            
            if isinstance(ode_joint, ode.FixedJoint) == False:
#                Rpd = numpy.dot(parentR, self.boneRs[joint.parent.name])    # parent_desired_SO3
                Rpd = numpy.dot(parentR, self.boneRs[nodeExistParentJoint.name])    # parent_desired_SO3
                Rcd = numpy.dot(R, self.boneRs[joint.name])   # child_desired_SO3
#                Rpd = mm.I_SO3()
#                Rcd = mm.I_SO3()

#                if posture not in self.Rpds:
#                    self.Rpds[posture] = {}
#                    if ode_joint not in self.Rpds[posture]:
#                        Rpd = numpy.dot(parentR, self.boneRs[joint.parent.name])    # parent_desired_SO3
#                        self.Rpds[posture][ode_joint] = Rpd
#                else:
#                    Rpd = self.Rpds[posture][ode_joint]
#                    
#                if posture not in self.Rpcs:
#                    self.Rpcs[posture] = {}
#                    if ode_joint not in self.Rpcs[posture]:
#                        Rcd = numpy.dot(R, self.boneRs[joint.name])   # child_desired_SO3
#                        self.Rpcs[posture][ode_joint] = Rcd
#                else:
#                    Rcd = self.Rpcs[posture][ode_joint]
            
            
                parent = ode_joint.getBody(0)
                child = ode_joint.getBody(1)
                
                Rpc = mm.odeSO3ToSO3(parent.getRotation())   # parent_current_SO3
                Rcc = mm.odeSO3ToSO3(child.getRotation())    # child_current_SO3

                
            if isinstance(ode_joint, ode.BallJoint):
                Ra = numpy.dot(Rpc, Rpd.transpose())   # align_SO3
                Rcd2 = numpy.dot(Ra, Rcd)
                
#                dR = -mm.logSO3_tuple(numpy.dot(Rcd2, Rcc.transpose())) # diff_rot
                dR = mm.logSO3(numpy.dot(Rcd2, Rcc.transpose())) # diff_rot
                
                Wpc = parent.getAngularVel()
                Wcc = child.getAngularVel()
                rW = (-Wpc[0]+Wcc[0], -Wpc[1]+Wcc[1], -Wpc[2]+Wcc[2])
#                rW = numpy.array([-Wpc[0]+Wcc[0], -Wpc[1]+Wcc[1], -Wpc[2]+Wcc[2]])
                
                ode_motor = node.motor
                ode_motor.setAxis(0, 0, -dR)
                ode_motor.setAxis(1, 0, rW)
#                ode_motor.setAxis(0, 0, dR+dW)
                
                torques[joint.name] = (node.Kp*(mm.length(dR)), node.Kd*(mm.length(rW)))
#                torques[joint.name] = node.Kp*(mm.length(dR)) + node.Kd*(mm.length(dW))
                
            elif isinstance(ode_joint, ode.HingeJoint):
                desiredAngle = mm.logSO3(numpy.dot(Rcd, Rpd.transpose()))
                currentAngle = mm.logSO3(numpy.dot(Rcc, Rpc.transpose()))
#                desiredAngle = mm.logSO3_old(numpy.dot(Rcd, Rpd.transpose()))
#                currentAngle = mm.logSO3_old(numpy.dot(Rcc, Rpc.transpose()))
                
                jointAxis = ode_joint.getAxis()
                desiredScala = numpy.inner(jointAxis, desiredAngle)
                currentScala = numpy.inner(jointAxis, currentAngle)
                
#                diffAngle = currentScala - desiredScala
                diffAngle = mm.diffAngle(currentScala, desiredScala)
                
#                print 'c:', mm.rad2Deg(currentScala), 'd:', mm.rad2Deg(desiredScala) 
#                print 'diff:', mm.rad2Deg(diffAngle)
#                if currentScala * desiredScala < 0:
#                    diffAngle2 = currentScala - (math.pi*2 - desiredScala)
##                    diffAngle2 = 2*math.pi - diffAngle 
##                    diffAngle2 = diffAngle - 2*math.pi 
##                    print 'chdiff:', mm.rad2Deg(diffAngle), '->', mm.rad2Deg(diffAngle2)
#                    diffAngle = diffAngle2
#                else:
#                    diffAngle = currentScala - desiredScala
#                    print 'diff:', mm.rad2Deg(diffAngle)
                
                angleRate = ode_joint.getAngleRate()
                
                torques[joint.name] = (node.Kp*(diffAngle), node.Kd*(angleRate))
                
            elif isinstance(ode_joint, ode.UniversalJoint):                
                desiredAngle = mm.logSO3(numpy.dot(Rcd, Rpd.transpose()))
                currentAngle = mm.logSO3(numpy.dot(Rcc, Rpc.transpose()))
                
                jointAxis1 = ode_joint.getAxis1()
                desiredScala1 = numpy.inner(jointAxis1, desiredAngle)
                currentScala1 = numpy.inner(jointAxis1, currentAngle)
                
                jointAxis2 = ode_joint.getAxis2()
                desiredScala2 = numpy.inner(jointAxis2, desiredAngle)
                currentScala2 = numpy.inner(jointAxis2, currentAngle)
                
                diffAngle1 = mm.diffAngle(currentScala1, desiredScala1)
                diffAngle2 = mm.diffAngle(currentScala2, desiredScala2)
                
                Wpc = parent.getAngularVel()
                Wcc = child.getAngularVel()
                rW = (-Wpc[0]+Wcc[0], -Wpc[1]+Wcc[1], -Wpc[2]+Wcc[2])
                
                angleRate1 = -numpy.inner(jointAxis1, rW)
                angleRate2 = -numpy.inner(jointAxis2, rW)
                
                torques[joint.name] = ((node.Kp*(diffAngle1), node.Kd*(angleRate1)), (node.Kp*(diffAngle2), node.Kd*(angleRate2))) 
                                
            elif isinstance(ode_joint, ode.FixedJoint):
                pass
            
#            print joint.name
#            print self.boneRs
#            print Rpd
#            print Rcd
#            print Rpc
#            print Rcc
#            print dR
#            print dW
                        
        for childJoint in joint.children:
            self._calcPDTorqueJoint(childJoint, R, posture, torques)        
        
    def applyTorque(self, torques):
        for name, node in self.nodes.items():
            if isinstance(node.joint, ode.BallJoint):
                node.motor.addTorques(torques[name][0], 0, 0)
                node.motor.addTorques(0, torques[name][1], 0)
#                node.motor.addTorques(torques[name], 0, 0)
            elif isinstance(node.joint, ode.HingeJoint):
                node.joint.addTorque(torques[name][0] - torques[name][1])
            elif isinstance(node.joint, ode.UniversalJoint):
                node.joint.addTorques(torques[name][0][0] - torques[name][0][1], torques[name][1][0] - torques[name][1][1])
    
class OdeMotionModel(OdeModel):
    def __init__(self, world, space, createPosture, config = None):
        OdeModel.__init__(self, world, space, createPosture, config)
        for node in self.nodes.values():
            node.body.disable()
        self.rootFixed = False
        
        self.update(createPosture)
        
    def fixRootBody(self):
        self.rootFixed = True

    def update(self, posture):
        if self.rootFixed:
            T = mm.I_SE3()
            joint = posture.skeleton.root
            self._updateBody(joint, T, posture)
        else:
            OdeModel.update(self, posture)


if __name__=='__main__':
    import psyco; psyco.full()
    import copy
    from fltk import *
    import Resource.ysMotionLoader as yf
    import GUI.ysSimpleViewer as ysv
    import Renderer.ysRenderer as yr
    import Simulator.ysOdeWorld as yow
    import Simulator.ysPhysConfig as ypc
    
    def test_ode():
        M_PI     =    3.14159265358979323846
        M_TWO_PI   =  6.28318530717958647693        #// = 2 * pi
        M_PI_SQRT2 =    2.22144146907918312351       # // = pi / sqrt(2)
        M_PI_SQR   =  9.86960440108935861883        #// = pi^2
        M_RADIAN   =  0.0174532925199432957692    #// = pi / 180
        M_DEGREE    = 57.2957795130823208768       # // = pi / 180
        
    ##    vpWorld        world;
#        world = ode.World()
#        space = ode.Space()
        odeWorld = yow.OdeWorld()
        world = odeWorld.world
        space = odeWorld.space
         
    ##    vpBody        ground, pendulum;
    ##    vpBody        base, body1, body2;
        base = ode.Body(world)
        body1 = ode.Body(world)
        body2 = ode.Body(world)
        
        
    ##    vpBJoint    J1;
    ##    vpBJoint    J2;
        J1 = ode.BallJoint(world)
        J2 = ode.BallJoint(world)
        M1 = ode.AMotor(world)
        M2 = ode.AMotor(world)
        M1.setNumAxes(2)
        M2.setNumAxes(2)
    
    ##    Vec3 axis1(1,1,1);
    ##    Vec3 axis2(1,0,0);
        axis1 = numpy.array([1,1,0])
        axis2 = numpy.array([1,1,1])
        axisX = numpy.array([1,0,0])
        axisY = numpy.array([0,1,0])
        axisZ = numpy.array([0,0,1])
        
    ##    scalar timeStep = .01;
    ##    int deg1 = 0;
    ##    int deg2 = 0;
        timeStep = .01
        deg1 = [0.]
        deg2 = 0.
        
        odeWorld.timeStep = timeStep
        
    
    ##    ground.AddGeometry(new vpBox(Vec3(10, 10, 0)));
    ##    ground.SetFrame(Vec3(0,0,-.5));
    ##    ground.SetGround();
    
    #    // bodies
    ##    base.AddGeometry(new vpBox(Vec3(3, 3, .5)));
    ##    base.SetGround();
        geom_base = ode.GeomBox(space, (3,.5,3))
        geom_base.setBody(base)
        geom_base.setPosition((0,.5,0))
    #    geom_base.setRotation(mm.SO3ToOdeSO3(mm.exp((1,0,0),math.pi/2)))
    
        fj = ode.FixedJoint(world)
        fj.attach(base, ode.environment)
        fj.setFixed()
        
        mass_base = ode.Mass()
        mass_base.setBox(100, 3,.5,3)
        base.setMass(mass_base)
    
    ##    body1.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
    ##    body2.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
    ##    body1.SetCollidable(false);
    ##    body2.SetCollidable(false);
    #    geom_body1 = ode.GeomCapsule(space, .2, 4)
        geom_body1 = ode.GeomBox(space, (.4, 4, .4))
        geom_body1.setBody(body1)
        geom_body1.setPosition((0,.5+.25+2,0))
        mass_body1= ode.Mass()
        mass_body1.setBox(100, .4, 4, .4)
        body1.setMass(mass_body1)
        
    #    init_ori = mm.exp((1,0,0),math.pi/2)
    #    geom_body1.setRotation(mm.SO3ToOdeSO3(init_ori))
    #    geom_body2 = ode.GeomCapsule(space, .2, 4)
        geom_body2 = ode.GeomBox(space, (.4, 4, .4))
        geom_body2.setBody(body2)
        geom_body2.setPosition((0,.5+.25+2+4.2,0))
    #    geom_body2.setRotation(mm.SO3ToOdeSO3(mm.exp((1,0,0),math.pi/2)))
        mass_body2= ode.Mass()
        mass_body2.setBox(100, .4, 4, .4)
        body2.setMass(mass_body2)
    
    ##    axis1.Normalize();
    ##    axis2.Normalize();
        axis1 = mm.normalize(axis1);
        axis2 = mm.normalize(axis2);
    
    ##    //J1.SetAxis(axis1);
    ##    base.SetJoint(&J1, Vec3(0, 0, .1));
    ##    body1.SetJoint(&J1, Vec3(0, 0, -.1));
        J1.attach(base, body1)
        J1.setAnchor((0,.5+.25,0))
        M1.attach(base, body1)
    
    ##    //J2.SetAxis(axis2);
    ##    //body1.SetJoint(&J2, Vec3(0, 0, 4.1));
    ##    //body2.SetJoint(&J2, Vec3(0, 0, -.1));
        J2.attach(body1, body2)
        J2.setAnchor((0,.5+.25+4.1,0))
        M2.attach(body1, body2)
    
    
    ##    world.AddBody(&ground);
    ##    world.AddBody(&base);
    
    ##    world.SetGravity(Vec3(0.0, 0.0, -10.0));
        world.setGravity((0,0,0))
    
        def PDControl(frame):
#            global deg1[0], J1, J2, M1, M2
            
    #        scalar Kp = 1000.;
    #        scalar Kd = 100.;
            Kp = 100.
            Kd = 2.
    
    #        SE3 desiredOri1 = Exp(Axis(axis1), scalar(deg1[0] * M_RADIAN));
    #        SE3 desiredOri2 = Exp(Axis(axis2), scalar(deg2 * M_RADIAN));
            desiredOri1 = mm.exp(axis1, deg1[0] * M_RADIAN)
    #        desiredOri2 = mm.exp(axis2, deg2 * M_RADIAN)
        
    #        se3 log1= Log(J1.GetOrientation() % desiredOri1);
    #        se3 log2= Log(J2.GetOrientation() % desiredOri2);
            parent1 = J1.getBody(0)
            child1 = J1.getBody(1)
            
            parent1_desired_SO3 = mm.exp((0,0,0), 0)
            child1_desired_SO3 = desiredOri1
    #        child1_desired_SO3 = parent1_desired_SO3
            
            parent1_body_SO3 = mm.odeSO3ToSO3(parent1.getRotation())
            child1_body_SO3 = mm.odeSO3ToSO3(child1.getRotation())
    
    #        init_ori = (mm.exp((1,0,0),math.pi/2))
    #        child1_body_SO3 = numpy.dot(mm.odeSO3ToSO3(child1.getRotation()), init_ori.transpose())
            
            align_SO3 = numpy.dot(parent1_body_SO3, parent1_desired_SO3.transpose())
            child1_desired_SO3 = numpy.dot(align_SO3, child1_desired_SO3)
    
            diff_rot = mm.logSO3(numpy.dot(child1_desired_SO3,child1_body_SO3.transpose()))
    #        print diff_rot
    
            parent_angleRate = parent1.getAngularVel()
            child_angleRate = child1.getAngularVel()
    #        print child_angleRate
            angleRate = numpy.array([-parent_angleRate[0] + child_angleRate[0],-parent_angleRate[1] + child_angleRate[1],-parent_angleRate[2] + child_angleRate[2]])
    
    #        torque1 = Kp*diff_rot - Kd*angleRate
    #        print torque1
            
    #        J1_ori = 
    #        log1 = mm.logSO3_tuple(numpy.dot(desiredOri1 ,J1.GetOrientation().transpose()))
    #        log2 = mm.logSO3_tuple(numpy.dot(desiredOri1 ,J1.GetOrientation().transpose()))
        
    #        Vec3 torque1 = Kp*(Vec3(log1[0],log1[1],log1[2])) - Kd*J1.GetVelocity();
    #        Vec3 torque2 = Kp*(Vec3(log2[0],log2[1],log2[2])) - Kd*J2.GetVelocity();
            M1.setAxis(0,0,diff_rot)
            M1.setAxis(1,0,angleRate)
    #        M2.setAxis(0,0,torque1)
            
    ##        J1.SetTorque(torque1);
    ##        J2.SetTorque(torque2);
            M1.addTorques(-Kp*mm.length(diff_rot),0,0)
            M1.addTorques(0,Kd*mm.length(angleRate),0)
    #        print diff_rot
    #        print angleRate
    #        M2.addTorques(torque2, 0, 0)
        
    #        cout << "SimulationTime " << world.GetSimulationTime() << endl;
    #        cout << "deg1[0] " << deg1[0] << endl;
    #        cout << "J1.vel " << J1.GetVelocity();
    #        cout << "J1.torq " << torque1;
    #        cout << endl;
    
        def calcPDTorque(Rpd, Rcd, Rpc, Rcc, Wpc, Wcc, Kp, Kd, joint):
            Rpc = mm.odeSO3ToSO3(Rpc);
            Rcc = mm.odeSO3ToSO3(Rcc);
        
            Ra = numpy.dot(Rpc, Rpd.transpose())
            Rcd = numpy.dot(Ra, Rcd)
    
            dR = mm.logSO3(numpy.dot(Rcd,Rcc.transpose()))
            dW = numpy.array([-Wpc[0] + Wcc[0],-Wpc[1] + Wcc[1],-Wpc[2] + Wcc[2]])
    
    #        joint.setAxis(0,0,dR)
    #        joint.setAxis(1,0,dW)
            joint.setAxis(0,0,dR-dW)
    
    #        joint.addTorques(-Kp*mm.length(dR),0,0)
    #        joint.addTorques(0,Kd*mm.length(dW),0)
            joint.addTorques(-Kp*mm.length(dR)-Kd*mm.length(dW),0,0)
    
        def PDControl3(frame):
    #        Kp = 100.*70;
    #        Kd = 10.*3;
            Kp = 1000.;
            Kd = 10.;
        
            desiredOri1 = mm.exp(axis1, deg1[0] * M_RADIAN)
            desiredOriX = mm.exp(axisX, deg1[0] * M_RADIAN)
            desiredOriY = mm.exp(axisY, deg1[0] * M_RADIAN)
            desiredOriZ = mm.exp(axisZ, deg1[0] * M_RADIAN)
    
            calcPDTorque(mm.I_SO3(), mm.exp(axisX, -90*M_RADIAN), base.getRotation(), body1.getRotation(), base.getAngularVel(), body1.getAngularVel(), Kp, Kd, M1);
    #        calcPDTorque(mm.I_SO3, desiredOriY, base.getRotation(), body1.getRotation(), base.getAngularVel(), body1.getAngularVel(), Kp, Kd, M1);
    #        calcPDTorque(mm.I_SO3, desiredOriX, body1.getRotation(), body2.getRotation(), body1.getAngularVel(), body2.getAngularVel(), Kp, Kd, M2);
            
        
    
    
   #    ground = ode.GeomPlane(space, (0,1,0), 0)
        
    
        viewer = ysv.SimpleViewer()
        viewer.setMaxFrame(100)
#        viewer.record(False)
        viewer.doc.addRenderer('object', yr.OdeRenderer(space, (255,255,255)))
        
        def preFrameCallback(frame):
#            global deg1[0]
            print 'deg1[0]', deg1[0]
            deg1[0] += 1
            
    #        PDControl(frame)
            PDControl3(frame)
        viewer.setPreFrameCallback(preFrameCallback)
        
        def simulateCallback(frame):
            odeWorld.step()
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()        
        
        
    def test_OdeMotionModel():
        bvhFilePath = '../samples/block_3_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .8
        for i in range(motion[0].skeleton.getElementNum()):
            mcfg.addNode(motion[0].skeleton.getElementName(i))
        node = mcfg.getNode('body1')
        node.mass = 1.
        node = mcfg.getNode('body2')
        node.mass = 1.
        node.boneRatio = .5
        node.offset = (0,0,.1)
        node = mcfg.getNode('body3')
        node.mass = 1.
        node.length = .1
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = None
        odeWorld = yow.OdeWorld(wcfg, mcfg)
        
        motionModel = OdeMotionModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('model', yr.OdeModelRenderer(motionModel, (255,255,255), yr.POLYGON_LINE))
        
        def preFrameCallback(frame):
            motionModel.update(motion[frame])
        viewer.setPreFrameCallback(preFrameCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()        
    
    def test_OdeControlModel():
#        mcfg.defaultContactMode = ode.ContactBounce | ode.ContactSoftERP | ode.ContactSoftCFM
#        mcfg.defaultJointAxes = []
#        mcfg.defaultJointLoStop = mm.deg2Rad(-5)
#        mcfg.defaultJointHiStop = mm.deg2Rad(5)
        
        bvhFilePath = '../samples/block_3_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        motion = motion[30:]
        
        AX = numpy.array([1,0,0])
        AY = numpy.array([0,1,0])
        AZ = numpy.array([0,0,1])
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .8
        mcfg.defaultKp = 100.
        mcfg.defaultKd = 1.
        for i in range(motion[0].skeleton.getElementNum()):
            mcfg.addNode(motion[0].skeleton.getElementName(i))
        node = mcfg.getNode('body2')
        node.length = 1
        node.offset = (0,0,.2)
#        node.jointAxes = [AY]
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = None
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        odeWorld = yow.OdeWorld(wcfg, mcfg)
        
        controlModel = OdeControlModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        controlModel.fixRootBody()
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('model', yr.OdeModelRenderer(controlModel, (255,255,255), yr.POLYGON_FILL))

        def simulateCallback(frame):
            for i in range(stepsPerFrame):
                torques = controlModel.calcPDTorque(motion[frame])
                controlModel.applyTorque(torques)
                odeWorld.step()
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()        
        
    def test_delNode():
        bvhFilePath = '../samples/block_3_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultBoneRatio = .8
        node = mcfg.addNode('body1')
#        node = mcfg.addNode('body2')
        node = mcfg.addNode('body3')
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = None
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        odeWorld = yow.OdeWorld(wcfg, mcfg)
        
        controlModel = OdeControlModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        controlModel.fixRootBody()
        
        motionModel = OdeMotionModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('controlModel', yr.OdeModelRenderer(controlModel, (255,255,255), yr.POLYGON_FILL))
        viewer.doc.addRenderer('motionModel', yr.OdeModelRenderer(motionModel, (255,255,255), yr.POLYGON_LINE))
        
        def preFrameCallback(frame):
            motionModel.update(motion[frame])
        viewer.setPreFrameCallback(preFrameCallback)
        
        def simulateCallback(frame):
            for i in range(stepsPerFrame):
                torques = controlModel.calcPDTorque(motion[frame])
                controlModel.applyTorque(torques)
                odeWorld.step()
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()     
        
    def test_biped_delNode():
        def buildMassMap():
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
        massMap = buildMassMap()
        
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion = yf.readBvhFile(bvhFilePath, .01)
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9

        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            
        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25
        
        node = mcfg.getNode('Head')
        node.length = .2
        
        node = mcfg.getNode('Spine')
        node.width = .22
        
        node = mcfg.getNode('RightFoot')
        node.length = .25
        node = mcfg.getNode('LeftFoot')
        node.length = .25
        
        mcfg.delNode('Spine1')
        mcfg.delNode('RightShoulder')
        mcfg.delNode('LeftShoulder1')
        mcfg.delNode('RightHand')
        mcfg.delNode('LeftHand')
        mcfg.delNode('RightToes')
        mcfg.delNode('LeftToes')
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = None
        stepsPerFrame = 10
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        odeWorld = yow.OdeWorld(wcfg, mcfg)
        
        controlModel = OdeControlModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
#        controlModel.fixRootBody()
        print controlModel
        
        motionModel = OdeMotionModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        
        viewer = ysv.SimpleViewer()
        viewer.record(False)
        viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('controlModel', yr.OdeModelRenderer(controlModel, (255,255,255), yr.POLYGON_FILL))
        viewer.doc.addRenderer('motionModel', yr.OdeModelRenderer(motionModel, (255,255,255), yr.POLYGON_LINE))
        
        def preFrameCallback(frame):
            motionModel.update(motion[frame])
        viewer.setPreFrameCallback(preFrameCallback)
        
        def simulateCallback(frame):
            for i in range(stepsPerFrame):
#                torques = controlModel.calcPDTorque(motion[frame])
#                controlModel.applyTorque(torques)
                odeWorld.step()
        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
        
    def test_body_pos_vel_acc_functions():
    #    bvhFilePath = '../samples/chain_6.bvh'
    #    bvhFilePath = '../samples/block_3_rotate.bvh'
        bvhFilePath = '../samples/block_tree_rotate.bvh'
        motion = yf.readBvhFile(bvhFilePath)
        
        bvhFilePath = '../samples/chain_1.bvh'
        motion2 = yf.readBvhFile(bvhFilePath)
        
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .8
        for i in range(motion[0].skeleton.getElementNum()):
            mcfg.addNode(motion[0].skeleton.getElementName(i))
    
        mcfg2 = ypc.ModelConfig()
        for i in range(motion2[0].skeleton.getElementNum()):
            mcfg2.addNode(motion2[0].skeleton.getElementName(i))
        
        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = -1.
        wcfg.gravity = (0,0,0)
        stepsPerFrame = 30
        wcfg.timeStep = (1/30.)/stepsPerFrame
        
        odeWorld = yow.OdeWorld(wcfg, mcfg)
        motionModel = OdeMotionModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        controlModel = OdeControlModel(odeWorld.world, odeWorld.space, motion[0], mcfg)
        controlModel2 = OdeControlModel(odeWorld.world, odeWorld.space, motion2[0], mcfg2)

        controlModel.fixRootBody()
#        controlModel2.setBodyPositionGlobal(0, (0,1,0))
        controlModel2.getBody('link0').setPosition((0,1,0))
        
        print controlModel
        print controlModel2
        
        cm_p = [mm.O_Vec3()]*controlModel.getBodyNum()
        cm_v = [mm.O_Vec3()]*controlModel.getBodyNum()
        cm_a = [mm.O_Vec3()]*controlModel.getBodyNum()
        cm_av = [mm.O_Vec3()]*controlModel.getBodyNum()
        cm_aa = [mm.O_Vec3()]*controlModel.getBodyNum()
        
    
        viewer = ysv.SimpleViewer()
        viewer.record(False)
    #    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,0,255), yr.LINK_WIREBOX))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('controlModel', yr.OdeModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
#        viewer.doc.addRenderer('motionModel', yr.OdeModelRenderer(motionModel, (100,100,100), yr.POLYGON_LINE))
        viewer.doc.addRenderer('controlModel2', yr.OdeModelRenderer(controlModel2, (255,240,255), yr.POLYGON_LINE))
        
        viewer.doc.addRenderer('cm_p', yr.PointsRenderer(cm_p, (255,0,0)))
        viewer.doc.addRenderer('cm_v', yr.VectorsRenderer(cm_v, cm_p, (0,0,255)))
#        viewer.doc.addRenderer('cm_a', yr.VectorsRenderer(cm_a, cm_p, (0,255,0)))
        viewer.doc.addRenderer('cm_av', yr.VectorsRenderer(cm_av, cm_p, (255,255,0)))
#        viewer.doc.addRenderer('cm_aa', yr.VectorsRenderer(cm_aa, cm_p, (0,255,255)))
    
        def simulateCallback(frame):
#    
            for i in range(stepsPerFrame):
                odeWorld.step()

#            motionModel.update(motion[frame])
            controlModel.getBody('left1').setTorque((0,0,10))
            controlModel2.getBody('link0').setTorque((0,0,10))
#            
            cm_p[:] = controlModel.getBodyPositionsGlobal() + controlModel2.getBodyPositionsGlobal()
            cm_v[:] = controlModel.getBodyVelocitiesGlobal() + controlModel2.getBodyVelocitiesGlobal()
            
            # there is no acceleration function in ode
#            cm_a[:] = controlModel.getBodyAccelerationsGlobal() + controlModel2.getBodyAccelerationsGlobal()

            cm_av[:] = controlModel.getBodyAngVelocitiesGlobal() + controlModel2.getBodyAngVelocitiesGlobal()
            
            # there is no acceleration function in ode
#            cm_aa[:] = controlModel.getBodyAngAccelerationsGlobal() + controlModel2.getBodyAngAccelerationsGlobal()

        viewer.setSimulateCallback(simulateCallback)
        
        viewer.startTimer(1/30.)
        viewer.show()
        
        Fl.run()
                     
    pass
#    test_ode()         
#    test_OdeMotionModel()
#    test_OdeControlModel()
#    test_delNode()
#    test_biped_delNode()
    test_body_pos_vel_acc_functions()