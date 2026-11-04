#!/usr/bin/env python3
"""Use the installed Nav2 MPPI example with the same robot limits and scoring gates."""
import argparse
from copy import deepcopy
from pathlib import Path

import yaml


def mppi_parameters(base, official):
    result = deepcopy(base)
    controller = result['controller_server']['ros__parameters']
    dwb = controller['FollowPath']
    mppi = deepcopy(official['controller_server']['ros__parameters']['FollowPath'])
    if mppi['plugin'] != 'nav2_mppi_controller::MPPIController':
        raise ValueError('installed Nav2 example is not MPPI')
    mppi.update(vx_max=dwb['max_vel_x'], vx_min=dwb['min_vel_x'], vy_max=0.0,
                wz_max=dwb['max_vel_theta'], ax_max=dwb['acc_lim_x'],
                ax_min=dwb['decel_lim_x'], az_max=dwb['acc_lim_theta'],
                model_dt=1.0/controller['controller_frequency'], visualize=False)
    mppi['CostCritic']['consider_footprint'] = True
    # Grade real inflation normally; moving surfaces have a time-matched critic.
    mppi['CostCritic']['near_collision_cost'] = 253
    mppi['CostCritic']['cost_weight'] = 6.0
    mppi['CostCritic']['critical_cost'] = 1000.0
    # A farther follow target and stronger progress cost avoid persistent slow rollouts.
    mppi['PathFollowCritic'].update(cost_weight=15.0, offset_from_furthest=15)
    mppi['GoalCritic']['cost_weight'] = 10.0
    mppi['PathAlignCritic']['cost_weight'] = 14.0
    mppi['regenerate_noises'] = False
    mppi['GoalAngleCritic']['threshold_to_consider'] = controller['goal_checker']['xy_goal_tolerance']
    # Path bearing stops in the native near-goal region; final pose owns the endpoint.
    mppi['PathAngleCritic']['threshold_to_consider'] = mppi['PathAlignCritic']['threshold_to_consider']
    mppi['PathAngleCritic']['max_angle_to_furthest'] = 0.4
    mppi['PathAngleCritic']['cost_weight'] = 8.0
    mppi['critics'].insert(1,'TimedObstacleCritic')
    mppi['TimedObstacleCritic'] = {'enabled':True,'cost_weight':1000.0,'clearance_m':.75}
    result['local_costmap']['local_costmap']['ros__parameters']['motion_prediction']['enabled'] = False
    result['global_costmap']['global_costmap']['ros__parameters']['motion_prediction']['enabled'] = True
    # Shim aligns the initial path; MPPI scores later turns and goal heading in time.
    mppi.update(plugin='nav2_rotation_shim_controller::RotationShimController',
                primary_controller='nav2_mppi_controller::MPPIController',
                rotate_to_goal_heading=False, rotate_to_heading_once=True,
                rotate_to_heading_angular_vel=dwb['max_vel_theta'],
                max_angular_accel=dwb['acc_lim_theta'],
                angular_dist_threshold=controller['goal_checker']['yaw_goal_tolerance'],
                angular_disengage_threshold=controller['goal_checker']['yaw_goal_tolerance'])
    controller['FollowPath'] = mppi
    return result


if __name__ == '__main__':
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser()
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    official = Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml'
    parameters = mppi_parameters(yaml.safe_load(args.base.read_text()), yaml.safe_load(official.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        yaml.safe_dump(parameters, stream)
