#!/usr/bin/env python
from uav_alignment_lib import Unicorn_Docker
import sys

if __name__ == '__main__':
    try:
        action = sys.argv[1]
        print action
        if action.isalnum()==False:
            print 'enter {a1, a2 , h, e, r, run } '
            sys.exit()

        docke_ = Unicorn_Docker()
        print 'obj init done'
        if action == 'run':
            docke_.run()
        else:
            docke_.do_action(data=action) 
    except Exception as e:
        print "exception occured: ", e
