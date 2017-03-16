import fltk

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Util.ysPythonEx as ype

class Fl_Event:
    FL_NO_EVENT        = 0
    FL_PUSH        = 1
    FL_RELEASE        = 2
    FL_ENTER        = 3
    FL_LEAVE        = 4
    FL_DRAG        = 5
    FL_FOCUS        = 6
    FL_UNFOCUS        = 7
    FL_KEYDOWN        = 8
    FL_KEYUP        = 9
    FL_CLOSE        = 10
    FL_MOVE        = 11
    FL_SHORTCUT        = 12
    FL_DEACTIVATE        = 13
    FL_ACTIVATE        = 14
    FL_HIDE        = 15
    FL_SHOW        = 16
    FL_PASTE        = 17
    FL_SELECTIONCLEAR    = 18
    FL_MOUSEWHEEL        = 19
    FL_DND_ENTER        = 20
    FL_DND_DRAG        = 21
    FL_DND_LEAVE        = 22
    FL_DND_RELEASE    = 23
    text = ype.getReverseDict(locals())
    
class Fl_When:  #  Fl_Widget::when():
    FL_WHEN_NEVER        = 0
    FL_WHEN_CHANGED    = 1
    FL_WHEN_RELEASE    = 4
    FL_WHEN_RELEASE_ALWAYS= 6
    FL_WHEN_ENTER_KEY    = 8
    FL_WHEN_ENTER_KEY_ALWAYS=10
    FL_WHEN_ENTER_KEY_CHANGED=11
    FL_WHEN_NOT_CHANGED    = 2
    text = ype.getReverseDict(locals())
    
if __name__=='__main__':
    def test_text():
        print Fl_Event.text
        print Fl_Event.text[fltk.FL_DRAG]
        
    test_text()