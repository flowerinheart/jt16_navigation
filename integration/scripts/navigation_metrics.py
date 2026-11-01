"""Offline navigation metrics, separated from ROS action execution."""
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from benchmark_geometry import dynamic_clearances, static_collision, localization_metrics, nearest_time_index


def origin(args):
    return (args.ground_truth_origin_x if args.ground_truth_origin_x is not None else args.initial_x,
            args.ground_truth_origin_y if args.ground_truth_origin_y is not None else args.initial_y,
            args.ground_truth_origin_yaw if args.ground_truth_origin_yaw is not None else args.initial_yaw)


def world_trajectory(gt, args):
    if args.ground_truth_is_world:
        return gt
    x, y, yaw = origin(args)
    c, s = math.cos(yaw), math.sin(yaw)
    return [[x+c*row[0]-s*row[1], y+s*row[0]+c*row[1], yaw+row[2], row[3]] for row in gt]


def local_trajectory(gt, args):
    x, y, yaw = origin(args)
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c*(row[0]-x)+s*(row[1]-y), -s*(row[0]-x)+c*(row[1]-y),
             row[2]-yaw, row[3]] for row in gt]


def goal_metrics(gt, args):
    x=args.score_goal_x if args.score_goal_x is not None else args.x
    y=args.score_goal_y if args.score_goal_y is not None else args.y
    is_local=args.score_goal_is_local or (args.score_goal_x is None and args.score_goal_y is None and args.goal_is_local)
    if is_local:
        ox,oy,yaw=origin(args);c,s=math.cos(yaw),math.sin(yaw)
        x,y=ox+c*x-s*y,oy+s*x+c*y
    length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(gt[:-1],gt[1:]))
    return {'gt_start_xy_m':gt[0][:2],'gt_final_xy_m':gt[-1][:2],
            'gt_displacement_m':math.hypot(gt[-1][0]-gt[0][0],gt[-1][1]-gt[0][1]),
            'gt_path_length_m':length,'gt_goal_xy_m':[x,y],
            'final_goal_distance_m':math.hypot(gt[-1][0]-x,gt[-1][1]-y)}


def barn_collision(gt, path):
    centers=[]
    for model in ET.parse(path).getroot().find('world').findall('model'):
        if model.get('name','').startswith('unit_cylinder_'):
            centers.append(tuple(map(float,model.findtext('pose').split()[:2])))
    if not centers:
        raise ValueError('missing BARN collision geometry')
    centers=np.asarray(centers);minimum=math.inf
    for x,y,yaw,_ in gt:
        dx,dy=centers[:,0]-x,centers[:,1]-y;c,s=math.cos(yaw),math.sin(yaw)
        bx,by=c*dx+s*dy,-s*dx+c*dy
        values=np.hypot(np.maximum(np.abs(bx)-.20,0),np.maximum(np.abs(by)-.235,0))-.075
        minimum=min(minimum,float(values.min()))
    return {'minimum_obstacle_clearance_m':minimum,'collision':minimum<=0.,'scoring_valid':True}


def arena_collision(gt, dynamic, args):
    if not args.arena_world:
        raise ValueError('missing Arena wall geometry')
    expected=len(json.loads(args.arena_scenario.read_text())['pedsim_agents'])
    if {int(row[1]) for row in dynamic} != set(range(expected)):
        raise ValueError('missing obstacle identities')
    clearances=dynamic_clearances(gt,dynamic)
    rectangles=[]
    for model in ET.parse(args.arena_world).getroot().find('world').findall('model'):
        if model.get('name','').startswith('wall_'):
            x,y=map(float,model.findtext('pose').split()[:2])
            sx,sy=map(float,model.findtext('link/collision/geometry/box/size').split()[:2])
            rectangles.append((x,y,sx/2,sy/2))
    if not rectangles:
        raise ValueError('missing wall geometry')
    wall_hit=static_collision(gt,rectangles)
    return {'static_collision':wall_hit,'dynamic_collision':min(clearances)<=0.,
            'minimum_obstacle_clearance_m':min(clearances),'mean_obstacle_clearance_m':float(np.mean(clearances)),
            'collision':wall_hit or min(clearances)<=0.,'scoring_valid':True}


def motion_metrics(gt):
    if len(gt)<4:
        return {}
    data=np.asarray(gt);dt=np.diff(data[:,3]);valid=dt>1e-5
    displacement=np.diff(data[:,:2],axis=0)[valid]
    velocity=displacement/dt[valid,None]
    if len(velocity)<3:
        return {}
    velocity_times=(data[1:,3]+data[:-1,3])[valid]/2
    vdt=np.diff(velocity_times);vvalid=vdt>1e-5
    acceleration=np.diff(velocity,axis=0)[vvalid]/vdt[vvalid,None]
    result={}
    if len(acceleration):result['mean_acceleration_m_s2']=float(np.linalg.norm(acceleration,axis=1).mean())
    acceleration_times=((velocity_times[1:]+velocity_times[:-1])/2)[vvalid]
    if len(acceleration)>1:result['mean_jerk_m_s3']=float(np.linalg.norm(np.diff(acceleration,axis=0)/np.diff(acceleration_times)[:,None],axis=1).mean())
    headings=np.unwrap(np.arctan2(velocity[:,1],velocity[:,0]));distance=np.linalg.norm(displacement,axis=1);usable=distance[1:]>.005
    curvature=np.abs(np.diff(headings)[usable]/distance[1:][usable])
    if len(curvature):result.update(mean_curvature_rad_m=float(curvature.mean()),roughness=float(np.mean(curvature**2)))
    return result


def pose_error(samples, truth, prefix):
    if not samples:
        return {}
    sample=samples[-1]
    try:reference=truth[nearest_time_index([row[3] for row in truth],sample[3],.1)]
    except ValueError:return {f'{prefix}_error_valid':False}
    yaw=abs(math.atan2(math.sin(sample[2]-reference[2]),math.cos(sample[2]-reference[2])))
    return {f'{prefix}_translation_error_m':math.hypot(sample[0]-reference[0],sample[1]-reference[1]),
            f'{prefix}_yaw_error_rad':yaw,f'{prefix}_error_valid':True}


def localization_summary(amcl, reference, now, initial_stamp):
    result=localization_metrics(amcl,reference,now)
    first_valid=next((row for row in amcl if localization_metrics([row],reference,row[3])['localization_success']),None)
    result['localization_initialization_latency_s']=max(0.,first_valid[3]-initial_stamp) if first_valid else None
    return result


def tf_jump_metrics(samples):
    if len(samples)<2:return {}
    pairs=list(zip(samples[:-1],samples[1:]))
    return {'map_odom_max_translation_jump_m':max(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in pairs),
            'map_odom_max_yaw_jump_rad':max(abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))) for a,b in pairs)}


def completion_metrics(result,args,terminal):
    elapsed=terminal['elapsed_s']
    result['scored_success']=bool(terminal.get('result')=='SUCCEEDED' and
        math.isfinite(elapsed) and 0<=elapsed<=args.timeout and
        result['scoring_valid'] and result['collision'] is False and
        result['localization_success'] is True and not terminal['input_failure'] and
        result['final_goal_distance_m']<=args.goal_tolerance)
    if args.shortest_path is not None:
        if args.shortest_path<=0:raise ValueError('invalid shortest path')
        optimal=args.shortest_path/2.
        result.update(shortest_path_m=args.shortest_path,path_ratio=result['gt_path_length_m']/args.shortest_path,
                      barn_score=optimal/min(max(elapsed,2*optimal),8*optimal) if result['scored_success'] else 0.)


def mission_trajectory(full_world,terminal):
    start=terminal.get('mission_start_stamp_s')
    end=terminal.get('mission_end_stamp_s')
    rows=full_world if start is None else [row for row in full_world if row[3]>=start]
    return rows if end is None else [row for row in rows if row[3]<=end]


def score_navigation(samples, args, initial_stamp, terminal):
    gt=samples['gt'];amcl=samples['amcl']
    if len(gt)<2:
        return {'scoring_valid':False,'scoring_error':'missing robot trajectory','scored_success':False}
    data=np.asarray(gt,dtype=float)
    if not np.isfinite(data).all() or np.any(np.diff(data[:,3])<=0):
        return {'scoring_valid':False,'scoring_error':'invalid robot trajectory','scored_success':False}
    full_world=world_trajectory(gt,args);local=local_trajectory(full_world,args)
    # Keep warmup truth for localization initialization, not mission collision/motion scores.
    world=mission_trajectory(full_world,terminal)
    if len(world)<2:
        return {'scoring_valid':False,'scoring_error':'missing mission trajectory','scored_success':False}
    result=goal_metrics(world,args)
    try:
        if args.arena_scenario:result.update(arena_collision(world,samples['dynamic'],args))
        elif args.barn_world:result.update(barn_collision(world,args.barn_world))
        else:raise ValueError('missing collision geometry')
    except ValueError as error:
        result.update(scoring_valid=False,scoring_error=str(error),collision=None)
    result.update(motion_metrics(world))
    reference=full_world if args.localization_reference_is_world else local
    result.update(localization_summary(amcl,reference,full_world[-1][3],initial_stamp))
    result.update(pose_error(samples['lio'],local,'lio'));result.update(pose_error(samples['fused'],local,'fused'))
    result.update(tf_jump_metrics(samples['map_odom']))
    completion_metrics(result,args,terminal)
    return result
