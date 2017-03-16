import psyco; psyco.full()
import os
from fltk import *

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import GUI.ysSimpleViewer as ysv

def usuage(scriptName):
    print 'Usage: %s bvh_file'%scriptName

class TrcViewer(ysv.SimpleViewer):
    def __init__(self, rect=None, title='TrcViewer'):
        ysv.SimpleViewer.__init__(self, rect, title)
    def open(self, trcFilePath):
        fl_cursor(FL_CURSOR_WAIT)
        
        self.initialize()
        self.record(False)
        
        pointMotion = yf.readTrcFile(trcFilePath, .01)
        
        print pointMotion[0].skeleton

        self.doc.addRenderer('pointMotion', yr.PointMotionRenderer(pointMotion, (0,255,0)))
        self.doc.addObject('pointMotion', pointMotion)
        
        title = '%s - TrcViewer'%os.path.basename(trcFilePath)
        self.label(title)
        self.iconlabel(title)
        
        fl_cursor(FL_CURSOR_DEFAULT)
    
def dnd_handle(event):
    global trcViewer
    if event == FL_DND_DRAG:
        return 1
    if event == FL_DND_RELEASE:
        cl = Fl.event_text()
        ln = Fl.event_length()
        trcViewer.open(cl)
        return 1
    return 0

trcViewer = None    

if __name__ == "__main__":
    if len(sys.argv) > 2:
        usuage(os.path.basename(sys.argv[0]))
    else:
        if len(sys.argv) == 2:
            trcFilePath = sys.argv[1]
        else:
            trcFilePath = None
        
        trcViewer = TrcViewer()
        trcViewer.record(False)
        
        if trcFilePath!=None:
            trcViewer.open(trcFilePath)
            
        trcViewer.startTimer(1/30.)
        trcViewer.show()
        
        Fl.add_handler(dnd_handle)
        Fl.run()    
        