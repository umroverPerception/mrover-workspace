import os
import sys
my_dir = os.path.dirname(sys.argv[0])
os.system('%s %s' % (sys.executable, 
os.path.join(my_dir, 'timer.py')))