import psyco; psyco.full()
import glob, os, sys

def usuage(scriptName):
    print 'Usage: %s [src_amc_dir] [dest_bvh_dir]'%scriptName

def makeListFiles(asfFilePaths, amcFilePaths, bvhFilePaths):
    list_asf = open('list_asf.txt', 'w')
    list_amc = open('list_amc.txt', 'w')
    list_bvh = open('list_bvh.txt', 'w')
    
    for path in asfFilePaths:
        list_asf.write(path+'\n')
    for path in amcFilePaths:
        list_amc.write(path+'\n')
    for path in bvhFilePaths:
        list_bvh.write(path+'\n')
    
    list_asf.close()
    list_amc.close()
    list_bvh.close()
    
def removeListFiles():
    os.remove('list_asf.txt')
    os.remove('list_amc.txt')
    os.remove('list_bvh.txt')
    
def readBvhJointNames(bvhFilePath):
    jointNames = []
    f = open(bvhFilePath)
    for line in f:
        tokens = line.split()
        if tokens[0] == 'ROOT' or tokens[0] == 'JOINT':
            jointNames.append(tokens[1])
        if tokens[0] == 'MOTION':
            break
    return jointNames
            
def readAsfBoneNames(asfFilePath):
    boneNames = []
    f = open(asfFilePath)
    for line in f:
        tokens = line.split()
        if tokens[0] == 'name':
            boneNames.append(tokens[1])
    return boneNames        
    
def getDefaultChangeMap(asfBoneNames):
    changeMap = {}
    for boneNames in asfBoneNames:
        changeMap[boneNames.lower()] = boneNames
    return changeMap

# changeMap[oldJointName] = newJointName
def changeBvhJointNamesInFile(bvhFilePath, changeMap):
    f = open(bvhFilePath, 'r')
    oldLines = f.readlines()
    f.close()
    
    newLines = [None]*len(oldLines)
    for i in range(len(oldLines)):
        oldLine = oldLines[i]
        tokens = oldLine.split()
        if tokens[0] == 'ROOT' or tokens[0] == 'JOINT':
            oldJointName = tokens[1]
            if oldJointName in changeMap:
                newJointName = changeMap[oldJointName]
                newLine = oldLine.replace(oldJointName, newJointName)
                newLines[i] = newLine
            else:
                newLines[i] = oldLine
        else:
            newLines[i] = oldLine

    f = open(bvhFilePath, 'w')
    f.writelines(newLines)
    f.close()

def convertAmc2Bvh(amcDir, bvhDir):
    asfFilePath = glob.glob(amcDir+'/*.asf')[0]
    print 'asf:', asfFilePath
    print
    
    amcFilePaths = glob.glob(amcDir+'/*.amc')
    bvhFilePaths = []
    asfFilePaths = [] 
    
    print 'amc:'
    for i in range(len(amcFilePaths)):
        amcPath = amcFilePaths[i]
        print '[%d] %s'%(i, amcPath)
        
        amcFileName = os.path.basename(amcPath)
        fileName = os.path.splitext(amcFileName)[0]
        
        bvhFileName = fileName + '.bvh'
        bvhFilePath = bvhDir + '/' + bvhFileName
        bvhFilePaths.append(bvhFilePath)
        
        asfFilePaths.append(asfFilePath)
    print
    
    makeListFiles(asfFilePaths, amcFilePaths, bvhFilePaths)
    os.system('Amc2Bvh.exe')
    removeListFiles()
    
    changeMap = getDefaultChangeMap(readAsfBoneNames(asfFilePath))
    changeMap['hip'] = 'Hips'
    changeMap['head'] = 'Head'
    
    print 'bvh:'
    for i in range(len(amcFilePaths)):
        bvhPath = bvhFilePaths[i] 
        changeBvhJointNamesInFile(bvhPath, changeMap)
        print '[%d] %s'%(i, bvhPath)
    print
    
    print 'Done'    


if __name__=='__main__':
    if len(sys.argv) > 3:
        usuage(os.path.basename(sys.argv[0]))
    else:
        if len(sys.argv) == 1:
            srcDir = '.'
            destDir = srcDir
        elif len(sys.argv) == 2:
            srcDir = sys.argv[1]
            destDir = srcDir
        elif len(sys.argv) == 3:
            srcDir = sys.argv[1]
            destDir = sys.argv[2]
    
        convertAmc2Bvh(srcDir, destDir)
