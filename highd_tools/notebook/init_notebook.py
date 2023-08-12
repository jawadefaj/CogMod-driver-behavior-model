import os, sys
from IPython.display import Image
# This is the settings file for the program
# Set the directory addresses accordingly
# Directory of the project (path for the highD_tools folder)
PROJECT_DIR = r'C:\\Users\\abjawad\\Documents\\GitHub\\cogMod-driver-behavior-model\\highd_tools'
# Path for the HighD dataset
DATA_DIRECTORY = r'D:\\highD_data\\highD_dataset'
# Path for the output directory
OUTPUT_DIRECTORY = r"C:\\Users\\abjawad\\Documents\\GitHub\\cogMod-driver-behavior-model\\highd_tools\\output"
# Path for the dill directory
DILLDIR = r"C:\\Users\\abjawad\\Documents\\GitHub\\cogMod-driver-behavior-model\\highd_tools\\dill"

COGMOD_LOGS = r"C:\\Users\\abjawad\\Documents\\GitHub\\cogMod-driver-behavior-model\\logs"

currentFolder = os.path.abspath('')
try:
    sys.path.remove(str(currentFolder))
except ValueError: # Already removed
    pass
projectFolder = PROJECT_DIR
sys.path.append(str(projectFolder))
os.chdir(projectFolder)
print( f"current working dir{os.getcwd()}")
def display_animated_gif(gif_path):
    return Image(filename=gif_path, format='png')