import ode
import ysOdeModel as ypm
import ysPhysConfig as ypc

defaultMcfgNode = ypc.ModelConfig.Node('')

class OdeWorld:
    def __init__(self, config = None, modelConfig = None):
#        self.timeStep = 0.001
        self.createWorld(config)
        self.modelConfig = modelConfig
        
    def createWorld(self, config):
        self.world = ode.World()
        self.space = ode.Space()
        self.contactgroup = ode.JointGroup()

        if config == None:
            config = ypc.WorldConfig()

        self.timeStep = config.timeStep

        self.world.setGravity(config.gravity)
        if config.planeHeight != None:
            self.plane = ode.GeomPlane(self.space, (0,1,0), config.planeHeight)
        self.world.setContactSurfaceLayer(config.ContactSurfaceLayer)
        self.world.setContactMaxCorrectingVel(config.ContactMaxCorrectingVel)
        self.world.setERP(config.ERP)
        self.world.setCFM(config.CFM)
        
    def step(self):
        self.space.collide((self.world,self.contactgroup), self.near_callback)        
        self.world.step(self.timeStep)
        self.contactgroup.empty()
        
    # Collision callback
    def near_callback(self, args, geom1, geom2):
        if (type(geom1) != ode.GeomPlane) and (type(geom2) != ode.GeomPlane):
            return
        
        if self.modelConfig != None and geom1.name in self.modelConfig.nodes:
            mcfgNode = self.modelConfig.nodes[geom1.name] 
        else:
            mcfgNode = defaultMcfgNode
  
        # Check if the objects do collide
        contacts = ode.collide(geom1, geom2)
        
        # Create contact joints
        self.world, self.contactgroup = args
        for c in contacts:
            c.setMode(mcfgNode.contactMode)
            c.setMu(mcfgNode.contactMu)
            c.setBounce(mcfgNode.contactBounce)
            c.setSoftERP(mcfgNode.contactSoftERP)
            c.setSoftCFM(mcfgNode.contactSoftCFM)
            
            j = ode.ContactJoint(self.world, self.contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())

    def setState(self, state):
        for i in range(self.space.getNumGeoms()):
            geom = self.space.getGeom(i)
            if type(geom) != ode.GeomPlane:
                body = geom.getBody()
                body.setPosition(state[body][0])
                body.setQuaternion(state[body][1])
                body.setLinearVel(state[body][2])
                body.setAngularVel(state[body][3])
    def getState(self):
        state = {}
        for i in range(self.space.getNumGeoms()):
            geom = self.space.getGeom(i)
            if type(geom) != ode.GeomPlane:
                body = geom.getBody()
                p = body.getPosition()
                r = body.getQuaternion()
                lv = body.getLinearVel()
                av = body.getAngularVel()
                state[body] = (p, r, lv, av)
        return state
