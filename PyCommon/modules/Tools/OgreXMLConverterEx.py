'''
Convert *.mesh and *.skeleton files to *.mesh.xml and *.skeleton.xml files
'''

import psyco; psyco.full()
import glob, os, sys, getopt

def usuage(scriptName):
    print 'Usage: %s [src_bin_dir] [dest_xml_dir]'%scriptName
    
def convertOgreFiles(srcDir, destDir):
    binPaths = []
    binPaths += glob.glob(srcDir+'/*.mesh')
    binPaths += glob.glob(srcDir+'/*.skeleton')
    
    createdXmlPaths = []
    
    for binPath in binPaths:
        fileName = os.path.basename(binPath)
        xmlPath = destDir+'/'+fileName+'.xml'
        if not(os.path.lexists(xmlPath) and os.path.getmtime(binPath) < os.path.getmtime(xmlPath)):
            os.system('ogrexmlconverter %s %s'%(binPath, xmlPath))
            createdXmlPaths.append(xmlPath)
    
    print 'created files : '
    for xmlPath in createdXmlPaths:
        print xmlPath
    print '%d xml files created'%len(createdXmlPaths)
    

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
            
        convertOgreFiles(srcDir, destDir)