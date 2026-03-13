import os
import io
import sys
import subprocess
import contextlib
import time
from termcolor import cprint
import torch
import numpy as np
import matplotlib.pyplot as plt

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'grasp/build')) 

import grasp


def launch_grasp(robot_name, object_name, object_pc, batchid, 
                    folder_name='grasp_log', get_pics=True, 
                    adjustT=True, ini_base_id=None):
    """
    Wrap function for subprocess call (grasp).

    :param robot_name: str
    :param object_name: str
    :param object_pc: torch.Tensor, point cloud of the object
    :param batchid: int
    :return: torch.Tensor, predicted joint values
    """
    log_dir = os.path.join(ROOT_DIR, f'{folder_name}/{robot_name}/{object_name}/{batchid}')
    os.makedirs(log_dir, exist_ok=True)
    np.savetxt(str(os.path.join(log_dir, 'object_pc.txt')), object_pc.cpu().detach().numpy())

    args = [
        os.path.join(ROOT_DIR, 'grasp/build/test_grasp'),
        '--robot_name', robot_name,
        '--object_name', object_name,
        '--log_dir', str(log_dir),
        # '--verbose', str(1),
        '--verbose', str(0),
    ]

    if ini_base_id is not None:
        args += ['--ini_base_path', str(os.path.join(log_dir.strip(str(batchid)), str(ini_base_id), 'robot_base_mat.txt'))]
    else:
        args += ['--ini_base_path', '']

    start_time = time.time()
    ret = subprocess.run(args, capture_output=True, text=True)
    end_time = time.time()

    try:
        predict_q = np.loadtxt(os.path.join(log_dir, 'predict_q.txt'))
        robot_pts_ini = np.loadtxt(os.path.join(log_dir, 'robot_pts_ini.txt'))
        robot_pts_opt = np.loadtxt(os.path.join(log_dir, 'robot_pts_opt.txt'))
        robot_base_mat = np.loadtxt(os.path.join(log_dir, 'robot_base_mat.txt'))
    except FileNotFoundError as e:
        cprint(f"Caught a ValueError: {e}", 'yellow')
        cprint(ret.stdout.strip(), 'blue')
        cprint(ret.stderr.strip(), 'red')
        exit()
    
    # start_time = time.time()
    # with contextlib.redirect_stderr(io.StringIO()):
    #     ret = graspopt.grasp(
    #                         robot_name=robot_name,
    #                         object_name=object_name,
    #                         log_dir=str(log_dir),
    #                         verbose=0
    #                     )
    # end_time = time.time()

    # try:
    #     predict_q = np.loadtxt(os.path.join(log_dir, 'predict_q.txt'))
    #     robot_pts_ini = np.loadtxt(os.path.join(log_dir, 'robot_pts_ini.txt'))
    #     robot_pts_opt = np.loadtxt(os.path.join(log_dir, 'robot_pts_opt.txt'))
    #     robot_base_mat = np.loadtxt(os.path.join(log_dir, 'robot_base_mat.txt'))
    # except FileNotFoundError as e:
    #     cprint("graspopt.grasp() throw an error.", 'red')
    #     exit()
    
    newT = robot_base_mat.copy()
    if adjustT:
        if robot_name == 'shadowhand':
            newT[:3, 0] = -robot_base_mat[:3, 1]
            newT[:3, 1] = -robot_base_mat[:3, 2]
            newT[:3, 2] = robot_base_mat[:3, 0]
        elif robot_name == 'barrett':
            pass
        elif robot_name == 'allegro':
            pass
        elif robot_name == 'leaphand':
            pass
        else:
            pass

    return torch.tensor(predict_q, dtype=torch.float32), \
            torch.tensor(newT, dtype=torch.float32), \
            end_time - start_time, \
            log_dir

if __name__ == "__main__":
    # test launch_graspopt
    robot_name = 'shadowhand'
    object_name = 'dummy_object'
    object_pc = 0.05 * torch.rand(500, 3)  # Dummy point cloud
    predict_q = launch_grasp(robot_name, object_name, object_pc, 0)
    print(f"Predicted joint values for {robot_name} grasping {object_name}: {predict_q.shape}")