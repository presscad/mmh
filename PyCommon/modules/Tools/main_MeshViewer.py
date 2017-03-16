import psyco; psyco.full()
from fltk import *

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv
import Mesh.ysMeshUtil as ysu
import Resource.ysOgreDataLoader as yol

if __name__ == "__main__":
    mesh, motions= yol.readOgreDataFiles('../samples/woody2_15.mesh.xml', .01, None)
#    mesh2, motions= yol.readOgreDataFiles('Data/woody2_6.mesh.xml', .01, None)
#    ysu.mergePoints(mesh)
    print mesh
    
    motion = yf.readBvhFile('../samples/wd2_WalkSameSame00.bvh', .01)
#    motion, frameTime = yf.readBvhFile('../samples/wd2_left_turn.bvh', .01*2.53999905501)

    viewer = ysv.SimpleViewer()
    viewer.record(False)
    
    viewer.doc.addRenderer('mesh', yr.MeshRenderer(mesh, (0,255,255)))
    viewer.doc.addObject('mesh', mesh)
#    viewer.doc.addRenderer('mesh2', yr.MeshRenderer(mesh2, (255,255,0)))
#    viewer.doc.addObject('mesh2', mesh2)
    
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (127,127,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    
    
    def preFrameCallback(frame):
        mesh.update(motion[frame])
#        mesh2.update(motion[frame])
    
    viewer.setPreFrameCallback(preFrameCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()