from datetime import datetime
import logging
from scipy.optimize import fmin

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.ysProbability as yp
import Util.ysPythonEx as pe

class ParamVector:
    def __init__(self):
        self.paramNames = []
        self.paramValues = {}
    def addParamValue(self, paramName, paramValue):
        self.paramNames.append(paramName)
        self.paramValues[paramName] = paramValue
    def getParamValue(self, paramName):
        return self.paramValues[paramName]
    def getParamName(self, index):
        return self.paramNames[index]
    def getLength(self):
        return len(self.paramNames)
    def __str__(self):
        str = ''
        str += '{'
        for i in range(len(self.paramNames)):
            str += '\''+self.paramNames[i]+'\'' + ': ' + self.paramValues[self.paramNames[i]].__str__()
            if i != len(self.paramNames)-1:
                str += ', '
        str += '}'
        return str
    def fromString(self, str):
        self.paramNames = []
        self.paramValues = {}
         
        tokens = str.split('\'')
        for i in range(len(tokens)):
            if i%2 == 1:
                self.paramNames.append(tokens[i])
        self.paramValues = eval(str)
        
        if len(self.paramNames) != len(self.paramValues):
            print 'fromString error!'
    def toList(self):
        ls = [None]*len(self.paramNames)
        for i in range(len(self.paramNames)):
            ls[i] = self.paramValues[self.paramNames[i]]
        return ls
    def fromList(self, ls):
        for i in range(len(ls)):
             self.paramValues[self.paramNames[i]] = ls[i]
    def copyParamNames(self, srcParamVector):
        for name in srcParamVector.paramNames:
            self.addParamValue(name, None)
        
        

class GridConfig:
    def __init__(self):
        self.paramNames = []
        self.paramValueGroups = []
    def __str__(self):
        str = ''
        for i in range(len(self.paramNames)):
            str += self.paramNames[i] + ' ' + self.paramValueGroups[i].__str__() + '\n'
        return str
    def addParamRangeStepNum(self, paramName, start, stop, numSteps):
        self.paramNames.append(paramName)
        self.paramValueGroups.append(pe.frange(start, stop, float(stop-start)/numSteps))
    def addParamRangeStepSize(self, paramName, start, stop, step):
        self.paramNames.append(paramName)
        self.paramValueGroups.append(pe.frange(start, stop, step))
    def addParamValueGroup(self, paramName, paramValues):
        self.paramNames.append(paramName)
        self.paramValueGroups.append(paramValues)
    def makeParamVectors(self):
        ParamVectors = []
        valueSets = yp.makeNumberOfCasesSetFromCasesSets(self.paramValueGroups)
        for valueSet in valueSets:
            paramVec = ParamVector()
            for i in range(len(self.paramNames)):
                paramVec.addParamValue(self.paramNames[i], valueSet[i])
            yield paramVec
    def countParamVectors(self):
        return yp.countNumberOfCasesSetFromCasesSets(self.paramValueGroups)
            

def gridOptimize(gridCfg, objectiveFunc, extraArgs):
    paramVecs = gridCfg.makeParamVectors()
    
    logging.info('gridOptimize()')
    logging.info('len(paramVecs) %s', gridCfg.countParamVectors())
    logging.info('')
    
    count = 0
    minObjectiveFuncValue = 1E+38
    minParamVector = None
    first = True
    for paramVec in paramVecs:
        if first:
            pt = datetime.today()
            
        logging.info('paramVecs[%d] %s', count, paramVec)
        objectiveFuncValue = objectiveFunc(paramVec, extraArgs)
        logging.info('objectiveFuncValue %s', objectiveFuncValue)
        count += 1
        
        if objectiveFuncValue < minObjectiveFuncValue:
            minObjectiveFuncValue = objectiveFuncValue
            minParamVector = paramVec
            logging.info('minObjectiveFuncValue updated! : %s', minObjectiveFuncValue)
        
        if first:
            logging.info('')
            et = datetime.today()-pt
            logging.info('first elapsed time %s', et)
            
            eet = et * gridCfg.countParamVectors()
            logging.info('total expected time %s', eet)
            logging.info('')
            first = False
        
    logging.info('gridOptimize() minParamVector %s', minParamVector)
    logging.info('gridOptimize() minObjectiveFuncValue %s', minObjectiveFuncValue)
    
    return minParamVector, minObjectiveFuncValue 

class FminConfig:
    def __init__(self):
        self.initialParamVector = None
        self.firstLeapSize = 1.
        self.firstLeapSizes = {}
        self.xtol = .001
        self.ftol = .01
        self.maxiter = None
        self.maxfun = 10

# downhill simplex method
def fminOptimize(fminCfg, objectiveFunc, extraArgs):
    initialParamVector = fminCfg.initialParamVector
    leapCriteriaSizes = [None]*initialParamVector.getLength()
    for i in range(initialParamVector.getLength()):
        paramName = initialParamVector.getParamName(i)
        if paramName in fminCfg.firstLeapSizes:
            leapCriteriaSizes[i] = fminCfg.firstLeapSizes[paramName] * 20.  
        else:
            leapCriteriaSizes[i] = fminCfg.firstLeapSize * 20.
    
#    leapCriteriaSizes = [fminCfg.firstLeapSize * 20.]*initialParamVector.getLength()
    
    count = [0]
    minObjectiveFuncValue = [1E+38]
    minParamVector = [None]
    first = [True]
    
    initialParamList = initialParamVector.toList() 
    paramVec = ParamVector()
    paramVec.copyParamNames(initialParamVector)
    
    def _innerObjectiveFunc(xn, *args):
        ls = [None]*len(xn)
        for i in range(len(xn)):
            ls[i] = initialParamList[i] - (leapCriteriaSizes[i] - xn[i])
        
        paramVec.fromList(ls)
        objectiveFunc = args[0]
        extraArgs = args[1]
#        
        if first[0]:
            pt = datetime.today()
#            
        logging.info('%dth evaluation %s', count[0]+1, paramVec)
        objectiveFuncValue = objectiveFunc(paramVec, extraArgs)
        logging.info('objectiveFuncValue %s', objectiveFuncValue)
        count[0] += 1
        
        if objectiveFuncValue < minObjectiveFuncValue[0]:
            minObjectiveFuncValue[0] = objectiveFuncValue
            minParamVector[0] = paramVec
            logging.info('minObjectiveFuncValue updated! : %s', minObjectiveFuncValue[0])
        
        if first[0]:
            logging.info('')
            et = datetime.today()-pt
            logging.info('first elapsed time %s', et)
            
            eet = et * fminCfg.maxfun
            logging.info('total approximately expected time %s', eet)
            logging.info('')
            first[0] = False
        
        return objectiveFuncValue
    
    logging.info('fminOptimize()')
    logging.info('maxfun %s', fminCfg.maxfun)
    logging.info('')
    
    if fminCfg.maxfun > 0:
        opt = fmin(_innerObjectiveFunc, leapCriteriaSizes, (objectiveFunc, extraArgs), fminCfg.xtol, fminCfg.ftol, fminCfg.maxiter, fminCfg.maxfun, 1, 1, 0, None)
#        print opt 
        
    logging.info('fminOptimize() minParamVector %s', minParamVector[0])
    logging.info('fminOptimize() minObjectiveFuncValue %s', minObjectiveFuncValue[0])
    
    return minParamVector[0], minObjectiveFuncValue[0] 


def fillObjectUsingParamVector(instance, paramVec):
    for name, value in paramVec.paramValues.items():
        if name in instance.__dict__:
            instance.__dict__[name] = value
        

if __name__=='__main__':
    def test_GridConfig():
        gc = GridConfig()
#        gc.addParamValueGroup('param1', [-1,0,1])
#        gc.addParamValueGroup('param2', [-2,2])
        gc.addParamRangeStepNum('param1', -10, 10, 100)
        gc.addParamRangeStepNum('param2', -1, 1, 10)
        print gc
        
        sets = gc.makeParamVectors()
        
        print gc.countParamVectors()
        for set in sets:
            print set
            
    def test_fillObjectUsingParamVector():
        
        class WorldConfig:
            def __init__(self):
        #        self.timeStep = 0.001
                
                self.gravity = (0, -9.8, 0)
                self.planeHeight = 0.0
                
                # below values are ODE default values
                self.ContactSurfaceLayer = 0.0
                self.ContactMaxCorrectingVel = 100000
                self.ERP = 0.2
                self.CFM = 1E-5
                
                self.contactMode = 10
                self.contactMu = 100000
                self.contactBounce = 0.1
                self.contactSoftERP = 0.0
                self.contactSoftCFM = 0.0        
                
        instance = WorldConfig()
        print instance.__dict__
        
        paramVec = ParamVector()
        paramVec.addParamValue('gravity', (0,0,0))
        fillObjectUsingParamVector(instance, paramVec)
        
        print instance.__dict__ 
        
    def test_ParamVector():
        str = '''{'param1': -10.0, 'param2': -1.0}'''
        print 'string', str
        
        paramVec = ParamVector()
        paramVec.fromString(str)
        print 'fromString', paramVec

        ls = paramVec.toList()
        print 'toList', ls
        
        paramVec2 = ParamVector()
        paramVec2.copyParamNames(paramVec)
        print 'copyParamNames', paramVec2
        paramVec.fromList(ls)
        print 'fromList', paramVec
        
    def test_objective(paramVec, extraArgs):
        return paramVec.getParamValue('a')*paramVec.getParamValue('b')
        
    def test_gridOptimize():
        logging.basicConfig(level=logging.DEBUG,
                            format='%(message)s')
        
        gc = GridConfig()
        gc.addParamValueGroup('a', [0,1])
        gc.addParamValueGroup('b', [0,2])
        
        gridOptimize(gc, test_objective, None)
        
    def test_fminOptimize():
        logging.basicConfig(level=logging.DEBUG,
                            format='%(message)s')
        
        paramVec = ParamVector()
#        paramVec.addParamValue('a', 1)
#        paramVec.addParamValue('b', 2)
        paramVec.addParamValue('a', -1)
        paramVec.addParamValue('b', -2)
        
        fc = FminConfig()
        fc.initialParamVector = paramVec
        fc.firstLeapSize = 10
        fc.firstLeapSizes['b'] = 1
         
        fminOptimize(fc, test_objective, None)
    
#    test_GridConfig()
#    test_ParamVector()
#    test_fillObjectUsingParamVector()
#    test_gridOptimize()
    test_fminOptimize()