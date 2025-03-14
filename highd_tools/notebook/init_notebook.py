import os, sys
currentFolder = os.path.abspath('')
try:
    sys.path.remove(str(currentFolder))
except ValueError: # Already removed
    pass

# projectFolder = 'C:\\Users\\abjawad\\Documents\\GitHub\\cogmod-driver-behavior-model'
COGMOD_LOGS = r"C:\Users\abjawad\Documents\GitHub\cogmod-driver-behavior-model\highd_tools\output"
projectFolder = r'C:\Users\abjawad\Documents\GitHub\cogmod-driver-behavior-model\highd_tools'
DATA_DIRECTORY = 'D:\\highD_data\\highD_dataset'
sys.path.append(str(projectFolder))
os.chdir(projectFolder)
print( f"current working dir{os.getcwd()}")
