'''
Convert *.mesh & *.skeleton to *.mesh.xml & *.skeleton.xml
'''

import glob, os

binPaths = []
binPaths += glob.glob('*.mesh')
binPaths += glob.glob('*.skeleton')

createdXmlPaths = []

for binPath in binPaths:
     xmlPath = binPath+'.xml'
     if os.path.lexists(xmlPath) and os.path.getmtime(binPath) >= os.path.getmtime(xmlPath):
        os.system('ogrexmlconverter %s %s'%(binPath, xmlPath))
        createdXmlPaths.append(xmlPath)

print 'Created xml files : '
for xmlPath in createdXmlPaths:
    print xmlPath
print '%d xml files created'%len(createdXmlPaths)

