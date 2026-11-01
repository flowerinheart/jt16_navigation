"""Compact LiDAR clusters tracked in the FAST-LIO odometry frame."""
from collections import deque
import numpy as np
from scipy.spatial import cKDTree


def scan_groups(base, indices, bins):
    groups = []
    for i, point in enumerate(base):
        if not groups or indices[i] != indices[i-1]+1 or np.linalg.norm(point-base[i-1]) > .18:
            groups.append([])
        groups[-1].append(point)
    if len(groups)>1 and indices[0]==0 and indices[-1]==bins-1 and np.linalg.norm(base[0]-base[-1])<.18:
        groups[0] = groups[-1]+groups[0]
        groups.pop()
    return groups


def scan_clusters(ranges, angle_min, angle_increment, pose, range_min=.15):
    ranges = np.asarray(ranges, dtype=float)
    pose = np.asarray(pose, dtype=float)
    if ranges.ndim!=1 or pose.shape != (3,) or not np.isfinite(pose).all() or not np.isfinite([angle_min,angle_increment]).all() or angle_increment <= 0:
        raise ValueError('invalid scan pose/angles')
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= 4.)
    indices = np.flatnonzero(valid)
    base = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
    groups = scan_groups(base,indices,len(ranges))
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.asarray([[c,-s],[s,c]])
    # shortcut: track separated compact clusters only; extend for merged crowds/large movers.
    return [np.asarray(group)@rotation.T+pose[:2] for group in groups
            if len(group)>=4 and np.linalg.norm(np.ptp(group,axis=0))<=1.2]


def track_velocity(history, points):
    if len(history)<4 or history[-1][0]-history[0][0]<.3:
        return None
    stamps = np.asarray([row[0] for row in history])
    centers = np.asarray([row[1] for row in history])
    times = stamps-stamps.mean()
    velocity = (times[:,None]*(centers-centers.mean(axis=0))).sum(axis=0)/(times@times)
    residual = np.linalg.norm(centers-centers.mean(axis=0)-times[:,None]*velocity,axis=1)
    if not .08<=np.linalg.norm(velocity)<=1.2 or residual.max()>.04:
        return None
    eigenvalues,axes = np.linalg.eigh(np.cov(points.T))
    # A visible fragment sliding along a straight surface does not prove surface motion.
    if eigenvalues[0]<.03*eigenvalues[1] and abs(velocity@axes[:,0])<.05:
        return None
    reference = history[0][2]
    static_error = np.median(cKDTree(reference).query(points)[0])
    moving_error = np.median(cKDTree(reference+velocity*(stamps[-1]-stamps[0])).query(points)[0])
    if static_error-moving_error<=.02:
        return None
    return velocity


def match_clusters(clusters, centers, old):
    candidates = sorted((np.linalg.norm(center-track['history'][-1][1]),i,j)
                        for i,center in enumerate(centers) for j,track in enumerate(old))
    matches, used = {}, set()
    for distance,i,j in candidates:
        if distance>.3 or i in matches or j in used:
            continue
        shift = centers[i]-old[j]['history'][-1][1]
        error = cKDTree(old[j]['points']+shift).query(clusters[i])[0]
        if np.median(error)<=.04 and np.linalg.norm(np.ptp(clusters[i],axis=0)-np.ptp(old[j]['points'],axis=0))<=.15:
            matches[i]=j
            used.add(j)
    return matches


def validated_clusters(clusters):
    result = [np.asarray(points,dtype=float) for points in clusters]
    if any(points.ndim!=2 or points.shape[1]!=2 or not len(points) or not np.isfinite(points).all() for points in result):
        raise ValueError('invalid cluster')
    return result


def moving_surfaces(tracks, stamp):
    moving = []
    for track in tracks:
        if 'motion' not in track:
            continue
        observed,points,velocity = track['motion']
        # shortcut: bridge at most 0.5 s of lost confirmation, not persistent occlusion.
        if stamp-observed <= .5:
            moving.append(np.column_stack((points+velocity*(stamp-observed),
                                           np.tile(velocity,(len(points),1)))))
    return np.concatenate(moving) if moving else np.empty((0,4))


def update_track(points, center, stamp, previous):
    history = previous['history'] if previous is not None else deque(maxlen=6)
    history.append((stamp,center,points))
    track = {'history':history,'points':points}
    if previous is not None and 'motion' in previous:
        track['motion'] = previous['motion']
    velocity = track_velocity(history,points)
    if velocity is not None:
        track['motion'] = (stamp,points,velocity)
    return track,velocity


def unobserved_motion(previous, matched, stamp):
    return [track for track in previous if id(track) not in matched and
            'motion' in track and stamp-track['motion'][0]<=.5]


class MotionPredictor:
    def __init__(self):
        self.tracks = []
        self.stamp = None
        self.moving_points = np.empty((0,4))

    def update(self, clusters, stamp):
        if not np.isfinite(stamp) or stamp<0 or (self.stamp is not None and stamp<=self.stamp):
            raise ValueError('non-increasing/invalid scan stamp')
        clusters = validated_clusters(clusters)
        previous = self.tracks
        old = [track for track in previous if stamp-track['history'][-1][0]<=.25]
        centers = [points.mean(axis=0) for points in clusters]
        matches = match_clusters(clusters,centers,old)
        self.tracks = []
        predicted, velocities = [], []
        for i,points in enumerate(clusters):
            track,velocity = update_track(points,centers[i],stamp,
                                          old[matches[i]] if i in matches else None)
            self.tracks.append(track)
            if velocity is not None:
                velocities.append([*centers[i],*velocity])
                # Cover the controller's 3.7 s rollout, including lateral arrivals at a stop.
                predicted.extend(points+velocity*t for t in np.arange(.2,4.01,.2))
        self.stamp = stamp
        matched = {id(old[j]) for j in matches.values()}
        self.tracks.extend(unobserved_motion(previous,matched,stamp))
        self.moving_points = moving_surfaces(self.tracks,stamp)
        return (np.concatenate(predicted) if predicted else np.empty((0,2)),
                np.asarray(velocities,dtype=float).reshape(-1,4))
