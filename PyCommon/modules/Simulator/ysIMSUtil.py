import sys
if '..' not in sys.path:
    sys.path.append('..')
import Mesh.ysMeshUtil as ysu
import Math.mmMath as mm

import Implicit.csIMSModel as cmm

def getSpringLengthsFromMesh(mesh, springConfigs):
    springLengths = [None]*len(springConfigs)
    for i in range(len(springConfigs)):
        springLengths[i] = mm.length(mesh.getVertexPosition(springConfigs[i].particleIndex0) - mesh.getVertexPosition(springConfigs[i].particleIndex1))
    return springLengths 

def getParticleConfigsFromMesh(mesh, massMap, initialMotion, dynamicMu, staticMu):
    particleConfigs = [None]*len(mesh.vertices)
    vertexMasses = ysu.getDistributedVertexMasses(mesh, massMap)
#    initialVelocities = ysu.meshAnimation2PointMotion(mesh, initialMotion).getPointVelocities(0, 1)
    initialVelocities = ysu.meshAnimation2PointMotion(mesh, initialMotion).getVelocities(0, 1)
    for i in range(len(mesh.vertices)):
        particleConfigs[i] = cmm.ParticleConfig(mesh.vertices[i].pos, vertexMasses[i], \
                                                initialVelocities[i], dynamicMu, staticMu)
    return particleConfigs

def getSpringConfigsFromMesh(mesh, Ks, Kd):
    springSet = set()
    
    for f in mesh.faces:
        for j in range(3):
            i0 = f.vertexIndex[j]
            oj = j+1
            if oj == 3: oj = 0 
            i1 = f.vertexIndex[oj]
            
            if i0 < i1:
                s = (i0, i1)
            else:
                s = (i1, i0)
                
            if s not in springSet:
                springSet.add(s)
    
    muscleSpringSet = set()
    for i in range(len(mesh.vertices)):
        neighbors = []
        for s in springSet:
            if i == s[0]:
                neighbors.append(s[1])
            elif i == s[1]:
                neighbors.append(s[0])
        for k in range(len(neighbors)):
            for m in range(len(neighbors)): 
                if k != m:
                    if neighbors[k] < neighbors[m]:
                        s = (neighbors[k], neighbors[m])
                    else:
                        s = (neighbors[m], neighbors[k])
                    
                    if s not in springSet and s not in muscleSpringSet:
                        muscleSpringSet.add(s)
                        
    subspringsNames = ['']*len(springSet) + ['__MUSCLE__']*len(muscleSpringSet)
    springs = list(springSet) + list(muscleSpringSet)
    springConfigs = [None]*len(springs)
    for i in range(len(springs)):
        sc = cmm.SpringConfig(springs[i][0], springs[i][1], Ks, Kd)
        sc.subspringsName = subspringsNames[i] 
        springConfigs[i] = sc
    
    return springConfigs