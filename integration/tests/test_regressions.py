"""Small fixed-data checks; no ROS, Gazebo, network or historical-result writes."""
import importlib.util
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from benchmark_geometry import (base_pose, cpu_delta, dilate_occupied, covariance_valid,
                                dynamic_clearances, localization_metrics,
                                nearest_time_index, shortest_path, static_collision, tf_valid,
                                obstacle_observations_fresh, mapping_input_failure)
from sensor_geometry import self_returns, pose_twist, sensor_position, world_specific_force
from navigation_metrics import score_navigation,motion_metrics,mission_trajectory
from build_mppi_parameters import mppi_parameters
from run_development_set import run_development_set
from unittest.mock import patch


class GeometryTests(unittest.TestCase):
    def test_terminal_observation_drain_is_bounded_and_keeps_mission_end(self):
        source=(SCRIPTS/'run_nav2_goal.py').read_text()
        code=source[source.index('observation_deadline='):source.index("summary={'result':'PROCESS_FAILURE'",source.index('observation_deadline='))]
        class Clock:
            def __init__(self):self.now=0.
            def monotonic(self):self.now+=.2;return self.now
        for count,deliver in ((0,True),(2,True),(2,False)):
            stamps={};calls=[]
            def spin(*args,**kwargs):
                calls.append(1)
                if deliver:stamps[len(calls)-1]=5.
            namespace={'time':Clock(),'math':math,'obstacle_count':count,
                       'obstacle_stamps':stamps,'mission_end':5.,'navigator':object(),
                       'rclpy':SimpleNamespace(spin_once=spin)}
            exec(code,namespace)
            self.assertEqual(namespace['mission_end'],5.)
            if count==0 or deliver:self.assertEqual(len(calls),0 if count==0 else 2)
            else:self.assertTrue(0<len(calls)<=11)

    def test_development_runner_records_missing_map_and_continues_arena(self):
        manifest={'name':'mixed','navigation_case_count':2,'entries':[
            {'id':'barn','dataset':'BARN','world_id':22,'seed':0},
            {'id':'arena','dataset':'Arena','world':'map2','obstacles':5,'seed':0}]}
        calls=[]
        def execute(command, **kwargs):
            calls.append(Path(command[1]).name)
            if calls[-1]=='run_arena_navigation.sh':
                run=Path(command[-1]);run.mkdir(parents=True)
                (run/'nav2-goal.json').write_text(json.dumps({'result':'SUCCEEDED','scored_success':True,'collision':False}))
            return SimpleNamespace(returncode=0)
        with tempfile.TemporaryDirectory() as temp, patch('run_development_set.subprocess.run',side_effect=execute):
            output=Path(temp)/'results'
            summary=run_development_set(manifest,output)
            self.assertEqual(calls,['run_barn_mapping.sh','run_arena_navigation.sh'])
            self.assertEqual(summary['strict_successes'],1)
            self.assertEqual(summary['entries'][0]['result'],'MISSING_MAP')
            self.assertEqual(summary['valid_maps'],0)
            with self.assertRaises(FileExistsError):run_development_set(manifest,output)
            duplicate={**manifest,'entries':[manifest['entries'][0],{**manifest['entries'][0],'id':'other'}]}
            with self.assertRaises(ValueError):run_development_set(duplicate,Path(temp)/'duplicate')

    def test_mapping_requires_finite_fresh_inputs(self):
        valid = [[10., 0., 0., 0.]]
        self.assertIsNone(mapping_input_failure(valid, valid, 10.5))
        for samples, expected in (([], 'missing_lio'),
                                  ([[10., math.nan, 0., 0.]], 'invalid_lio'),
                                  ([[8., 0., 0., 0.]], 'stale_lio'),
                                  ([[11., 0., 0., 0.]], 'stale_lio')):
            self.assertEqual(mapping_input_failure(valid, samples, 10.), expected)
        self.assertEqual(mapping_input_failure([], valid, 10.), 'missing_ground_truth')

    def test_mapping_metadata_rejects_path_complete_without_lio(self):
        with tempfile.TemporaryDirectory() as temp:
            run=Path(temp)
            result={'terminal_state':'COMPLETE','lio_samples':0,'map_valid':False}
            (run/'mapping-result.json').write_text(json.dumps(result))
            (run/'map-metrics.json').write_text(json.dumps({'occupied_f1':.9,'occupied_iou':.8}))
            command=subprocess.run([sys.executable,str(SCRIPTS/'write_map_bundle_metadata.py'),
                                    '--run',temp,'--world-id','277'],capture_output=True,text=True)
            self.assertNotEqual(command.returncode,0)
            self.assertIn('incomplete mapping inputs or map bundle',command.stderr)
            self.assertFalse((run/'map-bundle-metadata.json').exists())
            self.assertEqual(json.loads((run/'mapping-result.json').read_text()),result)

    def test_obstacle_heartbeat_requires_every_actor(self):
        self.assertTrue(obstacle_observations_fresh({},0,10.))
        self.assertTrue(obstacle_observations_fresh({0:9.8,1:10.},2,10.))
        for stamps in ({0:10.},{0:9.,1:10.},{0:11.,1:10.},{0:math.nan,1:10.}):
            self.assertFalse(obstacle_observations_fresh(stamps,2,10.))

    def test_navigation_exit_uses_strict_score(self):
        tree=ast.parse((SCRIPTS/'run_nav2_goal.py').read_text())
        status=tree.body[-1].exc.args[0]
        for strict,expected in ((False,1),(True,0)):
            self.assertEqual(eval(compile(ast.Expression(status),'<exit status>','eval'),
                                  {'summary':{'scored_success':strict}}),expected)
        for script in ('run_arena_navigation.sh','run_barn_navigation.sh'):
            self.assertTrue((SCRIPTS/script).read_text().rstrip().endswith('exit "$STATUS"'))

    def test_barn_missing_mapping_is_a_failed_command(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);mapping=root/'mapping';maps=mapping/'nav-map';maps.mkdir(parents=True)
            (mapping/'mapping-result.json').write_text('{}')
            path=maps/'map.yaml';path.write_text('{}')
            output=root/'navigation'
            result=subprocess.run(['bash',str(SCRIPTS/'run_barn_navigation.sh'),
                                   '22','0',str(path),str(output)],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            summary=json.loads((output/'nav2-goal.json').read_text())
            self.assertEqual(summary['result'],'MISSING_LIO_ODOMETRY')
            self.assertFalse(summary['scored_success'])

    def test_barn_navigation_rejects_incomplete_mapping_with_final_pose(self):
        for data in ({'terminal_state':'TIMEOUT','lio_final_xyyaw':[10.,0.,0.]},
                     {'terminal_state':'COMPLETE','map_valid':False,'lio_final_xyyaw':[10.,0.,0.]}):
            with self.subTest(data=data), tempfile.TemporaryDirectory() as temp:
                root=Path(temp);mapping=root/'mapping';maps=mapping/'nav-map';maps.mkdir(parents=True)
                (mapping/'mapping-result.json').write_text(json.dumps(data))
                path=maps/'map.yaml';path.write_text('{}')
                output=root/'navigation'
                result=subprocess.run(['bash',str(SCRIPTS/'run_barn_navigation.sh'),
                                       '22','0',str(path),str(output)],capture_output=True,text=True)
                self.assertNotEqual(result.returncode,0)
                self.assertFalse(json.loads((output/'nav2-goal.json').read_text())['scored_success'])
                self.assertFalse((output/'gazebo.log').exists())

    def test_mapping_exit_requires_complete_path(self):
        tree=ast.parse((SCRIPTS/'follow_barn_path.py').read_text())
        status=tree.body[-1].exc.args[0]
        for terminal,expected in (('COMPLETE',0),('TIMEOUT',1)):
            self.assertEqual(eval(compile(ast.Expression(status),'<mapping exit>','eval'),
                                  {'result':{'terminal_state':terminal}}),expected)

    def test_mapping_requires_nonempty_saved_cloud(self):
        source=(SCRIPTS/'run_barn_mapping.sh').read_text()
        gate=source[source.index('if [ -s "$PCD" ]; then'):source.index('kill -USR1 "$PM"',source.index('if [ -s "$PCD" ]; then'))]
        with tempfile.TemporaryDirectory() as temp:
            pcd=Path(temp)/'empty.pcd';pcd.touch()
            for path in (pcd,Path(temp)/'missing.pcd'):
                result=subprocess.run(['bash','-c','PCD=$1; OUTPUT=$2; '+gate,
                                       '_',str(path),temp],capture_output=True,text=True,timeout=5)
                self.assertNotEqual(result.returncode,0)
                self.assertIn('Missing or empty saved point cloud',result.stderr)

    def test_mapping_batch_refuses_existing_incomplete_bundle(self):
        source=SCRIPTS/'run_barn30_mapping.sh'
        for terminal in ('TIMEOUT','COMPLETE'):
            with self.subTest(terminal=terminal), tempfile.TemporaryDirectory() as temp:
                mapping=Path(temp)/'world_000/mapping';mapping.mkdir(parents=True)
                result_path=mapping/'mapping-result.json'
                content=json.dumps({'terminal_state':terminal});result_path.write_text(content)
                relative=str(source.relative_to(SCRIPTS.parents[1]))
                (mapping/'run-provenance.json').write_text(json.dumps({'schema':2,
                    'sources':{relative:hashlib.sha256(source.read_bytes()).hexdigest()}}))
                result=subprocess.run(['bash',str(source),temp],capture_output=True,text=True,timeout=5)
                self.assertNotEqual(result.returncode,0)
                self.assertIn('Incomplete mapping bundle',result.stderr)
                self.assertEqual(result_path.read_text(),content)
                self.assertEqual(list(Path(temp).glob('world_*')),[mapping.parent])

    def test_mapping_batch_records_failure_and_runs_remaining_worlds(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);scripts=root/'scripts';scripts.mkdir()
            manifests=root/'benchmarks/manifests';manifests.mkdir(parents=True)
            (manifests/'barn30.json').write_text(json.dumps({'entries':[{'world_id':0},{'world_id':1}]}))
            runner=scripts/'run_barn30_mapping.sh';runner.write_bytes((SCRIPTS/runner.name).read_bytes())
            (scripts/'run_barn_mapping.sh').write_text('mkdir -p "$2"\nif [ "$1" = 0 ]; then exit 1; fi\necho \'{"terminal_state":"COMPLETE","map_valid":true}\' > "$2/mapping-result.json"\n')
            (scripts/'score_v0.py').write_bytes((SCRIPTS/'score_v0.py').read_bytes())
            output=root/'results'
            result=subprocess.run(['bash',str(runner),str(output)],capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,1)
            summary=json.loads((output/'mapping-summary.json').read_text())
            self.assertEqual(summary['mapping_runs_scored'],2)
            self.assertEqual(summary['mapping_terminal_states'],{'PROCESS_FAILURE':1,'COMPLETE':1})

    def test_mapping_summary_counts_missing_and_nonfinite_metrics(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            for world,ate in ((0,.2),(1,math.nan),(2,.01)):
                run=root/f'world_{world:03d}/mapping';run.mkdir(parents=True)
                data={'terminal_state':'COMPLETE','map_valid':True,'paired_samples':6,
                      'ate_rmse_m':ate,'translation_p95_m':.3,'yaw_rmse_rad':.01}
                (run/'mapping-result.json').write_text(json.dumps(data))
                if world==2:
                    (run/'map-metrics.json').write_text(json.dumps({'occupied_f1':1.,'occupied_iou':1.}))
                if world==0:
                    for name in ('mapping-trajectories.npz','map.pcd','nav-map/map.yaml',
                                 'nav-map/map.pgm','map-bundle-metadata.json'):
                        path=run/name;path.parent.mkdir(exist_ok=True);path.write_text('fixture')
                    (run/'map-metrics.json').write_text(json.dumps({'occupied_f1':.9,'occupied_iou':.8}))
            result=subprocess.run([sys.executable,str(SCRIPTS/'score_v0.py'),'--results',temp,
                                   '--mapping-only'],capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            summary=json.loads((root/'mapping-summary.json').read_text())
            self.assertEqual(summary['valid_mapping_runs'],1)
            self.assertEqual(summary['invalid_mapping_runs'],2)
            self.assertEqual(summary['ate_rmse_m']['count'],1)
            self.assertEqual(summary['ate_rmse_m']['missing_count'],2)
            self.assertEqual(summary['ate_rmse_m']['mean'],.2)
            self.assertEqual(summary['occupied_f1']['count'],1)
            self.assertEqual(summary['occupied_f1']['mean'],.9)
            self.assertEqual(summary['occupied_f1']['missing_count'],2)

    def test_mission_window_is_inclusive_and_legacy_window_is_unchanged(self):
        rows=[[0.,0.,0.,float(i)] for i in range(4)]
        self.assertIs(mission_trajectory(rows,{}),rows)
        self.assertEqual(mission_trajectory(rows,{'mission_start_stamp_s':1.,'mission_end_stamp_s':2.}),rows[1:3])
        self.assertEqual(mission_trajectory(rows,{'mission_start_stamp_s':3.,'mission_end_stamp_s':2.}),[])

    def test_mission_scoring_excludes_warmup_but_not_missing_data_or_collision(self):
        args=SimpleNamespace(x=1.,y=0.,score_goal_x=None,score_goal_y=None,
            score_goal_is_local=False,goal_is_local=False,initial_x=0.,initial_y=0.,initial_yaw=0.,
            ground_truth_origin_x=None,ground_truth_origin_y=None,ground_truth_origin_yaw=None,
            ground_truth_is_world=True,localization_reference_is_world=True,
            goal_tolerance=.25,shortest_path=1.,barn_world=None,timeout=150.)
        with tempfile.TemporaryDirectory() as directory:
            args.arena_world=Path(directory)/'world.sdf';args.arena_scenario=Path(directory)/'scenario.json'
            args.arena_world.write_text('<sdf><world><model name="wall_0"><pose>20 20 0 0 0 0</pose><link><collision><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>')
            args.arena_scenario.write_text(json.dumps({'pedsim_agents':[{}]}))
            samples={'gt':[[0,0,0,0],[0,0,0,1],[1,0,0,2]],'lio':[[1,0,0,2]],
                'fused':[[1,0,0,2]],'amcl':[[1,0,0,2,.1,.1,.1]],'map_odom':[],
                'dynamic':[[1,0,10,10,.3],[2,0,10,10,.3]]}
            terminal={'result':'SUCCEEDED','elapsed_s':1.,'input_failure':None}
            self.assertFalse(score_navigation(samples,args,0.,terminal)['scoring_valid'])
            terminal['mission_start_stamp_s']=1.
            self.assertTrue(score_navigation(samples,args,0.,terminal)['scored_success'])
            samples['dynamic']=[[1,0,10,10,.3]]
            self.assertFalse(score_navigation(samples,args,0.,terminal)['scoring_valid'])
            samples['dynamic'].append([2,0,1,0,.3])
            result=score_navigation(samples,args,0.,terminal)
            self.assertTrue(result['collision']);self.assertFalse(result['scored_success'])
            samples['dynamic']=[[1,0,10,10,.3],[2,0,10,10,.3]]
            samples['gt'].append([1,0,0,3])
            self.assertFalse(score_navigation(samples,args,0.,terminal)['scoring_valid'])
            terminal['mission_end_stamp_s']=2.
            result=score_navigation(samples,args,0.,terminal)
            self.assertTrue(result['scoring_valid'])
            self.assertFalse(result['collision'])
            samples['dynamic'][1]=[2,0,1,0,.3]
            self.assertTrue(score_navigation(samples,args,0.,terminal)['collision'])
            terminal['mission_start_stamp_s']=3.
            self.assertFalse(score_navigation(samples,args,0.,terminal)['scored_success'])

    def test_mppi_preserves_robot_limits_and_noncontroller_gates(self):
        import yaml
        from copy import deepcopy
        base=yaml.safe_load((SCRIPTS.parents[1]/'src/jt16_bringup/config/nav2_params.yaml').read_text())
        before=deepcopy(base)
        official={'controller_server':{'ros__parameters':{'FollowPath':{
            'plugin':'nav2_mppi_controller::MPPIController','CostCritic':{'consider_footprint':False},
            'critics':['ConstraintCritic','CostCritic','PathFollowCritic'],
            'GoalCritic':{'cost_weight':5.0},'PathAlignCritic':{'cost_weight':14.0,'threshold_to_consider':.5},
            'regenerate_noises':True,
            'PathFollowCritic':{'cost_weight':5.0},'GoalAngleCritic':{'threshold_to_consider':.5},
            'PathAngleCritic':{'threshold_to_consider':.5,'max_angle_to_furthest':1.0}}}}}
        result=mppi_parameters(base,official)
        self.assertEqual(base,before)
        mppi=result['controller_server']['ros__parameters'].pop('FollowPath')
        expected=deepcopy(before);dwb=expected['controller_server']['ros__parameters'].pop('FollowPath')
        expected['local_costmap']['local_costmap']['ros__parameters']['motion_prediction']['enabled']=False
        expected['global_costmap']['global_costmap']['ros__parameters']['motion_prediction']['enabled']=True
        self.assertEqual(result,expected)
        self.assertEqual(mppi['vx_max'],dwb['max_vel_x'])
        self.assertEqual(mppi['vx_min'],dwb['min_vel_x'])
        self.assertEqual(mppi['wz_max'],dwb['max_vel_theta'])
        self.assertEqual(mppi['model_dt'],1./expected['controller_server']['ros__parameters']['controller_frequency'])
        self.assertTrue(mppi['CostCritic']['consider_footprint'])
        self.assertEqual(mppi['CostCritic']['near_collision_cost'],253)
        self.assertEqual(mppi['CostCritic']['cost_weight'],6.0)
        self.assertEqual(mppi['CostCritic']['critical_cost'],1000.0)
        self.assertEqual(mppi['PathFollowCritic']['cost_weight'],15.0)
        self.assertEqual(mppi['PathFollowCritic']['offset_from_furthest'],15)
        self.assertEqual(mppi['GoalCritic']['cost_weight'],10.0)
        self.assertEqual(mppi['PathAlignCritic']['cost_weight'],14.0)
        self.assertFalse(mppi['regenerate_noises'])
        self.assertEqual(mppi['GoalAngleCritic']['threshold_to_consider'],
                         before['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'])
        self.assertEqual(mppi['PathAngleCritic']['threshold_to_consider'],
                         mppi['PathAlignCritic']['threshold_to_consider'])
        self.assertEqual(mppi['PathAngleCritic']['max_angle_to_furthest'],.4)
        self.assertEqual(mppi['critics'][1],'TimedObstacleCritic')
        self.assertEqual(mppi['TimedObstacleCritic'],{'enabled':True,'cost_weight':1000.0,'clearance_m':.75})
        self.assertEqual(mppi['plugin'],'nav2_rotation_shim_controller::RotationShimController')
        self.assertEqual(mppi['primary_controller'],'nav2_mppi_controller::MPPIController')
        self.assertFalse(mppi['rotate_to_goal_heading'])
        self.assertTrue(mppi['rotate_to_heading_once'])
        self.assertEqual(mppi['rotate_to_heading_angular_vel'],dwb['max_vel_theta'])
        self.assertEqual(mppi['max_angular_accel'],dwb['acc_lim_theta'])
        self.assertEqual(mppi['angular_dist_threshold'],before['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'])
        official['controller_server']['ros__parameters']['FollowPath']['plugin']='other'
        with self.assertRaises(ValueError):mppi_parameters(base,official)

    def test_runners_reject_existing_evidence_before_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/'evidence';output.mkdir()
            evidence=output/'goal.log';evidence.write_text('preserve me')
            invocations=[('run_arena_navigation.sh',['small_warehouse','5','1',str(output)]),
                         ('run_barn_navigation.sh',['22','0',str(evidence),str(output)]),
                         ('run_barn_mapping.sh',['22',str(output)])]
            for script,args in invocations:
                result=subprocess.run(['bash',str(SCRIPTS/script),*args],capture_output=True,text=True,timeout=5)
                self.assertEqual(result.returncode,2,result.stderr)
                self.assertIn('Refusing to overwrite',result.stderr)
                self.assertEqual(evidence.read_text(),'preserve me')

    def test_navigation_defaults_generate_mppi_before_children_launch(self):
        for script in ['run_arena_navigation.sh','run_barn_navigation.sh']:
            source=(SCRIPTS/script).read_text()
            self.assertIn('PARAMS="${NAV2_PARAMS_FILE:-$OUTPUT/nav2-params.yaml}"',source)
            self.assertIn('if [ -z "${NAV2_PARAMS_FILE:-}" ]',source)
            self.assertLess(source.index('build_mppi_parameters.py'),source.index('> "$OUTPUT/gazebo.log"'))
            self.assertIn('--nav2-params-file "$PARAMS"',source)

    def test_dynamic_episode_uses_single_finite_mission_epoch(self):
        tree=ast.parse((SCRIPTS/'sim_state_adapter.py').read_text());cls=next(item for item in tree.body if isinstance(item,ast.ClassDef))
        method=next(item for item in cls.body if isinstance(item,ast.FunctionDef) and item.name=='start_episode')
        namespace={'math':math};exec(compile(ast.Module(body=[method],type_ignores=[]),'<episode epoch>','exec'),namespace)
        node=SimpleNamespace(obstacle_start_time=None)
        for stamp in [float('nan'),-1.]:namespace['start_episode'](node,SimpleNamespace(data=stamp))
        self.assertIsNone(node.obstacle_start_time)
        namespace['start_episode'](node,SimpleNamespace(data=12.))
        namespace['start_episode'](node,SimpleNamespace(data=15.))
        self.assertEqual(node.obstacle_start_time,12.)

    def test_rectangular_footprint_critic_and_goal_heading(self):
        import yaml
        params=yaml.safe_load((SCRIPTS.parents[1]/'src/jt16_bringup/config/nav2_params.yaml').read_text())
        critics=params['controller_server']['ros__parameters']['FollowPath']['critics']
        self.assertIn('ObstacleFootprint',critics);self.assertNotIn('BaseObstacle',critics)
        controller=params['controller_server']['ros__parameters']
        mount=json.loads((SCRIPTS.parents[1]/'src/jt16_bringup/config/jt16_mount.json').read_text())
        for costmap in ['local_costmap','global_costmap']:
            layer=params[costmap][costmap]['ros__parameters']['obstacle_layer']
            scan=layer['scan']
            self.assertLess(scan['min_obstacle_height'],-mount['position_m'][2])
            self.assertEqual(layer['min_obstacle_height'],scan['min_obstacle_height'])
        self.assertLessEqual(controller['goal_checker']['xy_goal_tolerance'],.25)
        self.assertEqual(controller['FollowPath']['xy_goal_tolerance'],controller['goal_checker']['xy_goal_tolerance'])
        self.assertGreaterEqual(controller['FollowPath']['min_vel_x'],params['velocity_smoother']['ros__parameters']['min_velocity'][0])
        self.assertLessEqual(controller['FollowPath']['max_vel_theta'],params['velocity_smoother']['ros__parameters']['max_velocity'][2])
        monitor=params['collision_monitor']['ros__parameters']
        self.assertEqual(monitor['polygons'],['FootprintApproach'])
        approach=monitor['FootprintApproach']
        self.assertEqual(approach['action_type'],'approach')
        self.assertEqual(approach['footprint_topic'],'/local_costmap/published_footprint')
        self.assertGreaterEqual(approach['time_before_collision'],2.)
        self.assertIn('scan',monitor['observation_sources'])
        tree=ast.parse((SCRIPTS/'run_nav2_goal.py').read_text())
        assignment=next(item for item in tree.body if isinstance(item,ast.Assign) and
                        isinstance(item.targets[0],ast.Name) and item.targets[0].id=='goal_heading')
        namespace={'math':math,'args':SimpleNamespace(x=0.,y=-1.,initial_x=0.,initial_y=0.)}
        exec(compile(ast.Module(body=[assignment],type_ignores=[]),'<goal heading>','exec'),namespace)
        self.assertAlmostEqual(namespace['goal_heading'],-math.pi/2)

    def test_amcl_waits_for_scenario_initial_pose(self):
        import yaml
        params=yaml.safe_load((SCRIPTS.parents[1]/'src/jt16_bringup/config/nav2_params.yaml').read_text())['amcl']['ros__parameters']
        self.assertIs(params['set_initial_pose'],False)
        self.assertNotIn('initial_pose',params)

    def test_covariance_roundoff_does_not_hide_invalid_variance(self):
        self.assertTrue(covariance_valid([-9.8e-13,0.,.1]))
        for value in [-1e-6,1.01,float('nan'),float('inf')]:
            self.assertFalse(covariance_valid([value,0.,.1]))

    def test_dynamic_collision_is_json_serializable(self):
        clearance=dynamic_clearances([[0.,0.,0.,1.]],[[1.,0,1.,0.,.3]])
        self.assertIs(type(clearance[0]),float)
        self.assertEqual(json.loads(json.dumps({'collision':min(clearance)<=0.})),{'collision':False})

    def test_terminal_localization_refresh_waits_for_new_observation(self):
        tree=ast.parse((SCRIPTS/'run_nav2_goal.py').read_text())
        cls=next(item for item in tree.body if isinstance(item,ast.ClassDef))
        method=next(item for item in cls.body if isinstance(item,ast.FunctionDef) and item.name=='refreshLocalization')
        clock=[0.];poses=[[0,0,0,1.]];truth=[[0,0,0,1.]];requests=[]
        client=SimpleNamespace(wait_for_service=lambda timeout_sec:True,
            call_async=lambda request:requests.append(request) or SimpleNamespace(done=lambda:True))
        node=SimpleNamespace(create_client=lambda typ,topic:client)
        def spin_once(node,timeout_sec):
            clock[0]+=.05;poses.append([0,0,0,2.])
            if clock[0]>=.1:truth.append([0,0,0,2.])
        namespace={'time':SimpleNamespace(monotonic=lambda:clock[0]),'Empty':SimpleNamespace(Request=lambda:'request'),
            'rclpy':SimpleNamespace(spin_until_future_complete=lambda *a,**k:None,spin_once=spin_once)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<terminal AMCL>','exec'),namespace)
        namespace['refreshLocalization'](node,poses,truth)
        self.assertEqual(requests,['request']);self.assertEqual(poses[-1][3],2.)
        self.assertEqual(clock[0],.1)
        self.assertGreaterEqual(truth[-1][3],poses[-1][3])
        client.wait_for_service=lambda timeout_sec:False
        namespace['refreshLocalization'](node,poses,truth)
        self.assertEqual(requests,['request'])

    def test_obstacle_read_poll_does_not_release_or_change_movement_requests(self):
        tree=ast.parse((SCRIPTS/'sim_state_adapter.py').read_text())
        cls=next(item for item in tree.body if isinstance(item,ast.ClassDef))
        methods=[item for item in cls.body if isinstance(item,ast.FunctionDef) and item.name in
                 ('poll_obstacles','publish_polled','publish_observation')]
        callbacks=[];requests=[];published=[]
        response=SimpleNamespace(result=SimpleNamespace(result=0,RESULT_OK=0),
                 state=SimpleNamespace(header=SimpleNamespace(frame_id='world',stamp=7.),
                                       pose=SimpleNamespace(position=SimpleNamespace(x=4.))))
        future=SimpleNamespace(add_done_callback=callbacks.append,result=lambda:response)
        def query(request):requests.append(request.entity);return future
        namespace={'GetEntityState':SimpleNamespace(Request=lambda entity:SimpleNamespace(entity=entity)),
                   'Odometry':lambda:SimpleNamespace(header=None,pose=SimpleNamespace(pose=None))}
        exec(compile(ast.Module(body=methods,type_ignores=[]),'<independent GT read>','exec'),namespace)
        node=SimpleNamespace(scenario={'pedsim_agents':[{},{}]},observation_pending={},obstacle_pending={0:True,1:True},
             get=SimpleNamespace(call_async=query),dynamic_publisher=SimpleNamespace(publish=published.append))
        node.publish_observation=lambda f,i:namespace['publish_observation'](node,f,i)
        node.publish_polled=lambda f,i:namespace['publish_polled'](node,f,i)
        namespace['poll_obstacles'](node);namespace['poll_obstacles'](node)
        self.assertEqual(requests,['dynamic_0','dynamic_1'])
        callbacks[0](future)
        self.assertEqual(node.obstacle_pending,{0:True,1:True})
        self.assertIs(published[0].pose.pose,response.state.pose)
        self.assertEqual(published[0].header.stamp,7.)
        self.assertEqual(published[0].child_frame_id,'dynamic_0')
        namespace['poll_obstacles'](node)
        self.assertEqual(requests,['dynamic_0','dynamic_1','dynamic_0'])
        response.result.result=1;callbacks[-1](future)
        self.assertEqual(len(published),1)

    def test_collision_bottom_matches_ground_frame(self):
        tree=ast.parse((SCRIPTS/'build_robot.py').read_text())
        collision=next(item for item in ast.walk(tree) if isinstance(item,ast.Assign) and
                       isinstance(item.targets[0],ast.Name) and item.targets[0].id=='chassis_collision')
        self.assertFalse(collision.value.keywords,'custom collision name drops sdformat friction')
        def attribute(parent,tag,key):
            call=next(item for item in ast.walk(tree) if isinstance(item,ast.Call) and
                isinstance(item.func,ast.Attribute) and item.func.attr=='SubElement' and
                len(item.args)>1 and isinstance(item.args[0],ast.Name) and item.args[0].id==parent and
                isinstance(item.args[1],ast.Constant) and item.args[1].value==tag)
            return next(ast.literal_eval(item.value) for item in call.keywords if item.arg==key)
        center=float(attribute('chassis_collision','origin','xyz').split()[2])
        height=float(attribute('chassis_geometry','box','size').split()[2])
        ground=float(attribute('base_joint','origin','xyz').split()[2])
        self.assertAlmostEqual(center-height/2,ground)

    def test_3d_sensor_acceleration_and_lever_arm(self):
        q=[0.,math.sqrt(.5),0.,math.sqrt(.5)]
        np.testing.assert_allclose(sensor_position([1,2,3],q,[0,0,.2]),[1.2,2,3])
        np.testing.assert_allclose(world_specific_force([1,2,3],q),[-12.80665,2,1],atol=1e-12)
        np.testing.assert_allclose(world_specific_force([0,0,0],[0,0,0,1]),[0,0,9.80665])

    def test_initial_pose_wait_does_not_flood_localizer(self):
        tree=ast.parse((SCRIPTS/'run_nav2_goal.py').read_text())
        cls=next(item for item in tree.body if isinstance(item,ast.ClassDef))
        self.assertTrue(all(isinstance(item,ast.FunctionDef) for item in cls.body),
                        'navigator logic escaped its method scope')
        method=next(item for item in cls.body if isinstance(item,ast.FunctionDef) and item.name=='_waitForInitialPose')
        clock=[0.];published=[]
        node=SimpleNamespace(initial_pose_received=False,remaining=lambda:2-clock[0],
                             _setInitialPose=lambda:published.append(clock[0]))
        def spin_once(node,timeout_sec):
            clock[0]+=.001
            node.initial_pose_received=clock[0]>=1.01
        namespace={'time':SimpleNamespace(monotonic=lambda:clock[0]),'rclpy':SimpleNamespace(spin_once=spin_once)}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<initial pose wait>','exec'),namespace)
        namespace['_waitForInitialPose'](node)
        self.assertLessEqual(len(published),2)

    def test_planar_model_has_no_free_joints_or_mixed_gravity(self):
        import xml.etree.ElementTree as ET
        tree=ast.parse((SCRIPTS/'build_robot.py').read_text())
        function=next(item for item in tree.body if isinstance(item,ast.FunctionDef) and item.name=='rigid_planar_model')
        namespace={'ET':ET}
        exec(compile(ast.Module(body=[function],type_ignores=[]),'<robot generator>','exec'),namespace)
        robot=ET.fromstring('<robot><link name="chassis"/><link name="arm"/><joint name="arm_joint" type="revolute"/></robot>')
        namespace['rigid_planar_model'](robot)
        self.assertTrue(all(joint.get('type')=='fixed' for joint in robot.findall('joint')))
        flags={item.get('reference'):item.findtext('gravity') for item in robot.findall('gazebo')}
        self.assertEqual(flags,{'chassis':'true','arm':'true'})
        chassis=robot.find("gazebo[@reference='chassis']")
        self.assertEqual(chassis.findtext('mu1'),'0')
        self.assertEqual(chassis.findtext('mu2'),'0')
        self.assertIsNone(robot.find("gazebo[@reference='arm']/mu1"))

    def test_stationary_tilt_rotates_gravity_not_fake_motion(self):
        angle=.1
        force=world_specific_force([0.,0.,0.],[0.,math.sin(angle/2),0.,math.cos(angle/2)])
        np.testing.assert_allclose(force,[-9.80665*math.sin(angle),0.,9.80665*math.cos(angle)])
        np.testing.assert_allclose(world_specific_force([1.,2.,0.],[0.,0.,0.,1.]),[1.,2.,9.80665])
    def test_offline_scoring_matches_terminal_failure(self):
        args=SimpleNamespace(x=1.,y=0.,score_goal_x=None,score_goal_y=None,
             score_goal_is_local=False,goal_is_local=False,initial_x=0.,initial_y=0.,initial_yaw=0.,
             ground_truth_origin_x=None,ground_truth_origin_y=None,ground_truth_origin_yaw=None,
             ground_truth_is_world=True,localization_reference_is_world=True,
             arena_scenario=None,goal_tolerance=.25,shortest_path=1.,timeout=150.)
        with tempfile.TemporaryDirectory() as temp:
            args.barn_world=Path(temp)/'world.sdf'
            args.barn_world.write_text('<sdf><world><model name="unit_cylinder_0"><pose>5 5 0 0 0 0</pose></model></world></sdf>')
            samples={'gt':[[0,0,0,1],[1,0,0,2]],'lio':[[1,0,0,2]],'fused':[[1,0,0,2]],
                     'amcl':[[1,0,0,2,.1,.1,.1]],'map_odom':[],'dynamic':[]}
            terminal={'result':'SUCCEEDED','elapsed_s':1.,'input_failure':None}
            result=score_navigation(samples,args,1.,terminal)
            self.assertTrue(result['scored_success']);self.assertTrue(result['localization_success'])
            for elapsed in [150.0001,-1.,float('nan'),float('inf')]:
                self.assertFalse(score_navigation(samples,args,1.,dict(terminal,elapsed_s=elapsed))['scored_success'])
            self.assertTrue(score_navigation(samples,args,1.,dict(terminal,elapsed_s=150.))['scored_success'])
            for state in ['FAILED','TIMEOUT','CANCELED','PROCESS_FAILURE',None]:
                self.assertFalse(score_navigation(samples,args,1.,dict(terminal,result=state))['scored_success'])
            result=score_navigation(samples,args,1.,dict(terminal,input_failure='stale LIO'))
            self.assertFalse(result['scored_success'])
            samples['gt'][1][0]=float('nan')
            self.assertFalse(score_navigation(samples,args,1.,terminal)['scoring_valid'])

    def test_arena_summary_rejects_failed_near_goal(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);run=root/'map2_obs05/seed_00';run.mkdir(parents=True)
            result={'result':'TIMEOUT','scored_success':True,'scoring_valid':True,
                    'localization_success':True,'collision':False,'input_failure':None}
            (run/'nav2-goal.json').write_text(json.dumps(result))
            subprocess.run([sys.executable,str(SCRIPTS/'score_arena.py'),'--results',str(root)],check=True,capture_output=True)
            self.assertEqual(json.loads((root/'arena-summary.json').read_text())['success_rate'],0.)

    def test_motion_metrics_use_actual_sample_times(self):
        result=motion_metrics([[0,0,0,0],[1,0,0,1],[3,0,0,3],[6,0,0,6],[10,0,0,10]])
        self.assertAlmostEqual(result['mean_acceleration_m_s2'],0.)
        self.assertAlmostEqual(result['mean_jerk_m_s3'],0.)

    def test_lio_pose_twist_without_ground_truth(self):
        previous=(np.array([0.,0.,0.]),[0,0,0,1],1.)
        current=(np.array([1.,0.,0.]),[0,0,0,1],2.)
        linear,angular=pose_twist(previous,current)
        np.testing.assert_allclose(linear,[1,0,0])
        np.testing.assert_allclose(angular,[0,0,0])
        current=(np.array([0.,1.,0.]),[0,0,math.sqrt(.5),math.sqrt(.5)],2.)
        linear,angular=pose_twist(previous,current)
        np.testing.assert_allclose(linear,[1,0,0],atol=1e-12)
        np.testing.assert_allclose(angular,[0,0,math.pi/2],atol=1e-12)
        for stamp in [1.,.5]:
            with self.assertRaises(ValueError):pose_twist(previous,(current[0],current[1],stamp))

    def test_stationary_pose_generates_zero_inertial_acceleration(self):
        tree=ast.parse((SCRIPTS/'sim_imu_adapter.py').read_text())
        cls=next(item for item in tree.body if isinstance(item,ast.ClassDef))
        method=next(item for item in cls.body if isinstance(item,ast.FunctionDef) and item.name=='odom')
        namespace={'np':np,'sensor_position':sensor_position}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<IMU odom>','exec'),namespace)
        node=SimpleNamespace(previous_stamp=None,previous_velocity=None,previous_pose=None,
                             acceleration=np.zeros(3),chassis_offset=np.zeros(3))
        def sample(stamp,z):
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0)),
                pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.,z=z),
                     orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
        for stamp in [0,1,2]:namespace['odom'](node,sample(stamp,3.))
        np.testing.assert_allclose(node.acceleration,[0,0,0])
        namespace['odom'](node,sample(2,99.))
        np.testing.assert_allclose(node.previous_pose,[1,2,3])
        namespace['odom'](node,sample(4,9.))
        np.testing.assert_allclose(node.acceleration,[0,0,2.])

    def test_lio_self_filter_keeps_ground_and_external_obstacle(self):
        mount=json.loads((SCRIPTS.parents[1]/'src/jt16_bringup/config/jt16_mount.json').read_text())
        points=np.array([[.03,0,-.2],[.03,0,-.36],[.5,0,-.2],[.24,0,-.2],
                         [-.20695075+.03,-.17694305,.18694042-.36]])
        np.testing.assert_array_equal(self_returns(points,mount),[True,False,False,False,True])

    def test_dilation_does_not_wrap(self):
        grid = np.zeros((5, 5), dtype=bool)
        grid[2, 0] = True
        self.assertTrue(dilate_occupied(grid)[2, 1])
        self.assertFalse(dilate_occupied(grid)[2, -1])

    def test_blocked_corner_and_invalid_endpoints(self):
        grid = np.array([[False, True], [True, False]])
        for start, goal in [((0, 0), (1, 1)), ((-1, 0), (1, 1)), ((0, 1), (1, 1))]:
            with self.assertRaises(ValueError):
                shortest_path(grid, start, goal)
        self.assertAlmostEqual(shortest_path(np.zeros((2, 2), bool), (0, 0), (1, 1)), math.sqrt(2))

    def test_absolute_obstacle_time(self):
        self.assertEqual(nearest_time_index([9, 10, 11], 11), 2)
        with self.assertRaises(ValueError):
            nearest_time_index([9, 10], 11)
        robot = [[0, 0, 0, 11]]
        observations = [[9, 0, 5, 0, .3], [10, 0, 5, 0, .3], [11, 0, 0, 0, .3]]
        self.assertLess(min(dynamic_clearances(robot, observations)), 0)
        with self.assertRaises(ValueError):
            dynamic_clearances(robot, [])

    def test_static_wall_and_clear_path(self):
        robot = [[0, 0, 0, 1]]
        self.assertTrue(static_collision(robot, [(0, 0, .1, .1)]))
        self.assertFalse(static_collision(robot, [(2, 0, .1, .1)]))
        self.assertTrue(static_collision([[0, 0, math.pi/4, 1]], [(.2, 0, .05, .05)]))

    def test_localization_is_not_message_presence(self):
        truth = [[0, 0, 0, 10]]
        valid = [0, 0, 0, 10, .1, .1, .1]
        self.assertTrue(localization_metrics([valid], truth, 10)['localization_success'])
        for sample, now in [([5, 0, 0, 10, .1, .1, .1], 10), (valid, 12),
                            ([0, 0, 0, 10, -1, .1, .1], 10)]:
            self.assertFalse(localization_metrics([sample], truth, now)['localization_success'])
        self.assertFalse(localization_metrics([], truth, 10)['localization_success'])

    def test_extrinsic_translation_and_rotation(self):
        mount = {'position_m': [1, 0, 0], 'ros_urdf_rpy_rad': [0, 0, 0]}
        p, q = base_pose([1, 0, 0], [0, 0, 0, 1], mount)
        np.testing.assert_allclose(p, [0, 0, 0])
        p, q = base_pose([0, 1, 0], [0, 0, math.sqrt(.5), math.sqrt(.5)], mount)
        np.testing.assert_allclose(p, [0, 0, 0], atol=1e-12)
        mount['ros_urdf_rpy_rad'] = [0, 0, math.pi/2]
        p, q = base_pose([1, 0, 0], [0, 0, math.sqrt(.5), math.sqrt(.5)], mount)
        np.testing.assert_allclose(p, [0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(q, [0, 0, 0, 1], atol=1e-12)

    def test_tf_missing_and_duplicate_fail(self):
        edges = {'map->odom': ['/amcl@a'], 'odom->base_link': ['/lio_odom_bridge@b']}
        self.assertTrue(tf_valid(edges))
        self.assertFalse(tf_valid({}))
        edges['map->odom'].append('/amcl@c')
        self.assertFalse(tf_valid(edges))

    def test_cpu_exit_does_not_subtract_work(self):
        previous = {}
        self.assertEqual(cpu_delta(previous, {(1, 0): 2., (2, 0): 5.}), 0)
        self.assertEqual(cpu_delta(previous, {(1, 0): 3., (2, 0): 7.}), 3.)
        self.assertEqual(cpu_delta(previous, {(1, 0): 4.}), 1.)

    def test_failed_obstacle_move_never_publishes_requested_pose(self):
        tree = ast.parse((SCRIPTS/'sim_state_adapter.py').read_text())
        cls = next(item for item in tree.body if isinstance(item, ast.ClassDef))
        method = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == 'move_obstacles')
        namespace = {'Pose_V':object,'Boolean':object}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<obstacle callback>', 'exec'), namespace)
        errors = []
        node = SimpleNamespace(move_service='/world/test/set_pose_vector',
                               get_logger=lambda: SimpleNamespace(error=errors.append))
        for ok,ack in ((False,False),(True,False)):
            node.gz=SimpleNamespace(request=lambda *args:(ok,SimpleNamespace(data=ack)))
            with self.assertRaisesRegex(RuntimeError,'Obstacle move failed'):
                namespace['move_obstacles'](node, object())
        self.assertEqual(len(errors), 2)
        node.gz=SimpleNamespace(request=lambda *args:(True,SimpleNamespace(data=True)))
        namespace['move_obstacles'](node, object())

    def test_map_keeps_nearby_wall_and_unknown_space(self):
        from PIL import Image
        import yaml
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            points = np.array([[0., 0., 0.]] * 100 + [[.35, 0., .2]] * 10, dtype='<f4')
            header = ('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                      f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n')
            (root/'map.pcd').write_bytes(header.encode() + points.tobytes())
            np.savez(root/'trajectory.npz', lio=np.array([[0., 0., 0., 0.]]))
            subprocess.run([sys.executable, str(SCRIPTS/'pcd_to_occupancy.py'),
                '--pcd', str(root/'map.pcd'), '--trajectory', str(root/'trajectory.npz'),
                '--output', str(root/'output'), '--trajectory-padding', '1', '--min-hits', '1'],
                check=True, capture_output=True)
            info = yaml.safe_load((root/'output/map.yaml').read_text())
            grid = np.flipud(np.asarray(Image.open(root/'output/map.pgm')))
            x, y = np.floor((np.array([.35, 0.]) - info['origin'][:2]) / info['resolution']).astype(int)
            self.assertEqual(grid[y, x], 0)
            self.assertIn(205, grid)

    def test_quick_gate_exact_members_and_collision_evidence(self):
        for worlds, collision, timeout, expected in [([22, 55, 77], False, 150, True),
                                            ([0, 11, 33], False, 150, False),
                                            ([22, 55, 77], None, 150, False),
                                            ([22, 55, 77], False, 45, False)]:
            with tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                for world in worlds:
                    run = root/f'world_{world:03d}/navigation/seed_00'
                    run.mkdir(parents=True)
                    (run/'nav2-goal.json').write_text(json.dumps({'result': 'SUCCEEDED',
                        'final_goal_distance_m': .2, 'collision': collision, 'gt_samples': 10,
                        'scoring_schema': 2, 'navigation_timeout_s': timeout,
                        'scoring_valid': True, 'scored_success': True,
                        'localization_success': True}))
                subprocess.run([sys.executable, str(SCRIPTS/'score_quick3.py'),
                    '--root', temp, '--output', str(root/'summary.json')], check=True, capture_output=True)
                self.assertEqual(json.loads((root/'summary.json').read_text())['promotion_gate_pass'], expected)

    def test_empty_acceptance_evidence_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            result = subprocess.run([sys.executable, str(SCRIPTS/'audit_v0.py'), '--root', temp],
                                    capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            report = json.loads((Path(temp)/'v0-audit.json').read_text())
            self.assertFalse(report['pass'])

    def test_quick_gate_rejects_contradictory_success(self):
        for changes in [{'scored_success':False},{'scoring_valid':None},
                        {'localization_success':False},{'input_failure':'stale LIO'},
                        {'result':'CANCELED'}]:
            with tempfile.TemporaryDirectory() as temp:
                root=Path(temp)
                for world in [22,55,77]:
                    run=root/f'world_{world:03d}/navigation/seed_00';run.mkdir(parents=True)
                    record={'result':'SUCCEEDED','final_goal_distance_m':.2,'collision':False,
                            'gt_samples':10,'scoring_schema':2,'navigation_timeout_s':150,
                            'scoring_valid':True,'scored_success':True,'localization_success':True}
                    if world==22:record.update(changes)
                    (run/'nav2-goal.json').write_text(json.dumps(record))
                subprocess.run([sys.executable,str(SCRIPTS/'score_quick3.py'),'--root',temp,
                                '--output',str(root/'summary.json')],check=True,capture_output=True)
                self.assertFalse(json.loads((root/'summary.json').read_text())['promotion_gate_pass'])

    def test_changed_world_or_map_invalidates_provenance(self):
        from run_provenance import write_provenance,verify_provenance
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);output=root/'run';output.mkdir()
            world=root/'world.sdf';world.write_text('world one')
            write_provenance(output,[world],{'timeout':150})
            verify_provenance(output)
            runtime=json.loads((output/'run-provenance.json').read_text())['runtime_hashes']
            for filename in ('libmppi_controller.so','libnav2_smac_planner_2d.so',
                             'librmw_cyclonedds_cpp.so'):
                mppi=Path('/opt/ros/jazzy/lib')/filename
                if not mppi.is_file():continue
                self.assertIn(str(mppi),runtime)
                identity=json.loads((output/'run-provenance.json').read_text())
                identity['runtime_hashes'][str(mppi)]='0'*64
                (output/'run-provenance.json').write_text(json.dumps(identity))
                with self.assertRaisesRegex(ValueError,'runtime changed'):verify_provenance(output)
                identity['runtime_hashes']=runtime
                (output/'run-provenance.json').write_text(json.dumps(identity))
            world.write_text('world two')
            with self.assertRaises(ValueError):verify_provenance(output)

    def test_provenance_cannot_overwrite_candidate(self):
        from run_provenance import write_provenance, verify_provenance
        with tempfile.TemporaryDirectory() as temp:
            write_provenance(temp)
            verify_provenance(temp)
            with self.assertRaises(FileExistsError):
                write_provenance(temp)


if __name__ == '__main__':
    unittest.main()
