import os, sys
currentFolder = os.path.abspath('')
try:
    sys.path.remove(str(currentFolder))
except ValueError: # Already removed
    pass

projectFolder = 'C:/Users/abjawad/Documents/Github/cogmod-driver-behavior-model'
# projectFolder = 'E:\\AV\\Carla\\CARLA_0.9.13\\WindowsNoEditor\\PythonAPI\\experiments'
sys.path.append(str(projectFolder))
os.chdir(projectFolder)
print( f"current working dir{os.getcwd()}")
