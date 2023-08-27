import os, sys
currentFolder = os.path.abspath('')
try:
    sys.path.remove(str(currentFolder))
except ValueError: # Already removed
    pass

# C:\Users\abjawad\Documents\GitHub\cogmod-driver-behavior-model\lib\highD
projectFolder = 'C:/Users/abjawad/Documents/GitHub/cogmod-driver-behavior-model/lib/highD'
sys.path.append(str(projectFolder))
os.chdir(projectFolder)
print( f"current working dir{os.getcwd()}")
