import os
import shutil

<<<<<<< HEAD:src/stereo_robot/stereo_camera/calibration/7_commit_to_config.py
current_folder = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
config_folder = os.path.abspath(os.path.join(current_folder,"..","config"))
=======
current_folder = os.path.abspath(os.path.dirname(__file__))
config_folder = os.path.abspath(os.path.join(current_folder, "..", "config"))
>>>>>>> master_update:src/stereo_robot/camera_utils/stereo_camera/stereo_camera_old/calibration/7_commit_to_config.py

calib_result_folder = os.path.join(current_folder, "calib_result")
calib_result_destination = os.path.join(config_folder, "calib_result")

camera_params = os.path.join(current_folder, "camera_params.txt")
camera_params_destination = os.path.join(config_folder, "camera_params.txt")

tdmap_set = os.path.join(current_folder, "3dmap_set.txt")
tdmap_set_destination = os.path.join(config_folder, "3dmap_set.txt")
verbose = False

def printv(msg):
    global verbose
    if verbose == True:
        print(msg)
if __name__ == "__main__":
    try:
        shutil.copytree(calib_result_folder, calib_result_destination)
    except Exception as E:
        printv(E)
        shutil.rmtree(calib_result_destination)
        shutil.copytree(calib_result_folder,calib_result_destination)
        printv(f"Removed existing {calib_result_destination}")
    printv(f"Moved {calib_result_folder} to {calib_result_destination}")

    try:
        shutil.copy(camera_params, camera_params_destination)
    except Exception as E:
        printv(E)
        shutil.rm(camera_params_destination)
        shutil.copy(camera_params,camera_params_destination)
        printv(f"Removed existing {camera_params_destination}")
    printv(f"Moved {camera_params} to {camera_params_destination}")

    try:
        shutil.copy(tdmap_set, tdmap_set_destination)
    except Exception as E:
        shutil.rm(tdmap_set_destination)
        shutil.copy(tdmap_set,tdmap_set_destination)
    printv(f"Moved {tdmap_set} to {tdmap_set_destination}")
