#!/bin/sh
"exec" "blender" "--background" "--python-exit-code" "1" "--python" "$0" "--" "$@"
import os, sys
HERE = os.path.split(__file__)[0]
sys.path.append( HERE )
import robotsim as R
if __name__=='__main__':
    R.main()
