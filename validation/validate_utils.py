import os
import sys
import subprocess
from termcolor import cprint
import torch
import numpy as np

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)


def validate_isaac(robot_name, object_name, q_batch, gpu: int = 0):
    """
    Wrap function for subprocess call (isaac_main.py) to avoid Isaac Gym GPU memory leak problem.

    :param robot_name: str
    :param object_name: str
    :param q_batch: torch.Tensor, joint values to validate
    :param gpu: int
    :return: (list<bool>, list<float>), success list & info list
    """
    os.makedirs(os.path.join(ROOT_DIR, 'tmp'), exist_ok=True)
    q_file_path = str(os.path.join(ROOT_DIR, f'tmp/q_list_validate_{gpu}.pt'))
    torch.save(q_batch, q_file_path)
    batch_size = q_batch.shape[0]
    args = [
        'python',
        os.path.join(ROOT_DIR, 'validation/isaac_main.py'),
        '--mode', 'validation',
        '--robot_name', robot_name,
        '--object_name', object_name,
        '--batch_size', str(batch_size),
        '--q_file', q_file_path,
        '--gpu', str(gpu),
        # '--use_gui'
    ]
    ret = subprocess.run(args, capture_output=True, text=True)
    try:
        ret_file_path = os.path.join(ROOT_DIR, f'tmp/isaac_main_ret_{gpu}.pt')
        save_data = torch.load(ret_file_path)
        success = save_data['success']
        q_isaac = save_data['q_isaac']
        try: os.remove(q_file_path)
        except: pass
        try: os.remove(ret_file_path)
        except: pass
    except FileNotFoundError as e:
        cprint(f"Caught a ValueError: {e}", 'yellow')
        cprint(ret.stdout.strip(), 'blue')
        cprint(ret.stderr.strip(), 'red')
        exit()
    return success, q_isaac
