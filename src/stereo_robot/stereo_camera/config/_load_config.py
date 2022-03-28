import json
import os
os.chdir(os.path.dirname(os.path.realpath(__file__)))
def load_sbm_config():
    try:
        with open("3dmap_set.txt",'r') as f:
            return json.load(f)
    except Exception as E:
        print(E)
        print("Please load config from calibration experiments using calibration/7_commit_to_config")
        exit(-1)

def load_camera_params():
    with open("camera_params.txt",'r') as f:
        return json.load(f)
