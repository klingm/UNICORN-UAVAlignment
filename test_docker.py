#!/usr/bin/env python
from uav_alignment_lib import Unicorn_Docker
import sys
import time

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
            while True:
                try:
                    print "running..."
                    docke_.run()
                except Exception as e:
                    print "Could not run: ", e
                    time.sleep(1)
                    continue
        else:
            docke_.do_action(data=action) 
    except Exception as e:
        print "exception occured: ", e
