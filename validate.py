import os
import sys
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)

import time
import warnings
from termcolor import cprint
import argparse
import numpy as np
import torch
from scipy.spatial.transform import Rotation

from validation.CMapDataset import create_dataloader
from validation.validate_utils import validate_isaac
from grasp_pywrapper.wrapper import launch_grasp


def main(args):
    if torch.cuda.is_available():
        device = torch.device(f'cuda:{args.gpu}')
    else:
        device = torch.device('cpu')

    batch_size = args.ntrial_per_case
    dataloader = create_dataloader(args, is_train=False, full_dataset=True)

    print(f"Testing grasps using Isaac Gym on {device} with {args.ntrial_per_case} cases per test case...")

    global_robot_name = None
    all_success_q = []
    time_list = []
    success_num = 0
    total_num = 0
    vis_info = []
    for i, data in enumerate(dataloader):
        robot_name = data['robot_name']
        object_name = data['object_name']

        if robot_name != global_robot_name:
            if global_robot_name is not None:
                all_success_q = torch.cat(all_success_q, dim=0)
                diversity_std = torch.std(all_success_q, dim=0).mean()
                times = np.array(time_list)
                time_mean = np.mean(times)
                time_std = np.std(times)

                success_rate = success_num / total_num * 100
                cprint(f"[{global_robot_name}]", 'magenta', end=' ')
                cprint(f"Result: {success_num}/{total_num}({success_rate:.2f}%)", 'yellow', end=' ')
                cprint(f"Std: {diversity_std:.3f}", 'cyan', end=' ')
                cprint(f"Time: (mean) {time_mean:.2f} s, (std) {time_std:.2f} s", 'blue')

                all_success_q = []
                time_list = []
                success_num = 0
                total_num = 0
            global_robot_name = robot_name


        predict_q_list, object_pc_list, log_dir_list, data_count = [], [], [], 0
        
        while data_count != batch_size:
            split_num = min(batch_size - data_count, 20)

            object_pc = data['object_pc'][data_count : data_count + split_num].to(device)

            for idx in range(object_pc.shape[0]):
                object_pc_i = object_pc[idx]
                predict_q_i, predict_base_mat_i, runtime, log_dir_i = launch_grasp(robot_name, object_name, 
                                                                           object_pc_i, data_count + idx,
                                                                           folder_name='example_logs')
                
                new_euler_i = Rotation.from_matrix(predict_base_mat_i[:3, :3]).as_euler('XYZ', degrees=False)
                predict_q_i[3:6] = torch.tensor(new_euler_i, device=predict_q_i.device, dtype=predict_q_i.dtype)
                if idx == 0:
                    predict_q = predict_q_i.unsqueeze(0)
                else:
                    predict_q = torch.cat((predict_q, predict_q_i.unsqueeze(0)), dim=0)
                time_list.append(runtime)
                log_dir_list.append(log_dir_i)

            data_count += split_num
            print(f"[{data_count}/{batch_size}] Optimization time: {np.sum(np.array(time_list[-split_num:])):.4f} s")

            predict_q_list.append(predict_q)
            object_pc_list.append(object_pc)
            

        predict_q_batch = torch.cat(predict_q_list, dim=0)
        object_pc_batch = torch.cat(object_pc_list, dim=0)

        success, isaac_q = validate_isaac(robot_name, object_name, predict_q_batch, gpu=args.gpu)
        succ_num = success.sum().item() if success is not None else -1
        success_q = predict_q_batch[success]
        all_success_q.append(success_q)

        vis_info.append({
            'robot_name': robot_name,
            'object_name': object_name,
            'predict_q': predict_q_batch,
            'object_pc': object_pc_batch,
            'success': success,
            'isaac_q': isaac_q
        })

        cprint(f"[{robot_name}/{object_name}]", 'light_blue', end=' ')
        cprint(f"Result: {succ_num}/{batch_size}({succ_num / batch_size * 100:.2f}%)", 'green')

        success_num += succ_num
        total_num += batch_size

    all_success_q = torch.cat(all_success_q, dim=0)
    diversity_std = torch.std(all_success_q, dim=0).mean()

    times = np.array(time_list)
    time_mean = np.mean(times)
    time_std = np.std(times)

    success_rate = success_num / total_num * 100
    cprint(f"[{global_robot_name}]", 'magenta', end=' ')
    cprint(f"Result: {success_num}/{total_num}({success_rate:.2f}%)", 'yellow', end=' ')
    cprint(f"Std: {diversity_std:.3f}", 'cyan', end=' ')
    cprint(f"Time: (mean) {time_mean:.2f} s, (std) {time_std:.2f} s", 'blue')

    
if __name__ == "__main__":
    
    warnings.simplefilter(action='ignore', category=FutureWarning)
    torch.set_num_threads(8)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--gpu", type=str, default='0', help="GPU ID to use")
    parser.add_argument("--num_workers", type=int, default=8, help="Number of workers to use")
    parser.add_argument("--ntrial_per_case", type=int, default=100, help="Number of cases per test case")
    parser.add_argument("--robot_name", type=str, default=None, help="List of robot names to test", choices=[None, 'barrett', 'allegro', 'shadowhand', 'leaphand'])
    parser.add_argument("--object_pc_type", type=str, default='random', help="Type of object point cloud to use", choices=['fixed', 'random', 'partial'])
    args = parser.parse_args()

    if args.robot_name is not None:
        args.robot_names = [args.robot_name]
    else:
        args.robot_names = ['barrett', 'allegro', 'shadowhand', 'leaphand']
    
    main(args)
