"""Compact LiDAR clusters tracked in the FAST-LIO odometry frame."""
from collections import deque
import numpy as np
from scipy.spatial import cKDTree


def pose_at_stamp(poses, stamp):
    if not poses:
        return None
    left = next((row for row in reversed(poses) if row[0] <= stamp), None)
    right = next((row for row in poses if row[0] >= stamp), None)
    if left is not None and right is not None and 0 < right[0] - left[0] <= .5:
        delta = np.asarray(right[1]) - left[1]
        delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
        return np.asarray(left[1]) + (stamp - left[0]) / (right[0] - left[0]) * delta
    nearest = min(poses, key=lambda row: abs(row[0] - stamp))
    return nearest[1] if abs(nearest[0] - stamp) <= .03 else None


def static_scan_ranges(ranges, angle_min, angle_increment, pose, tracks, stamp):
    result = np.asarray(ranges, dtype=float).copy()
    points = [track['motion'][1] for track in tracks
              if 'motion' in track and track['motion'][0] == stamp]
    if not points:
        return result
    c, s = np.cos(pose[2]), np.sin(pose[2])
    base = (np.concatenate(points)-pose[:2])@np.array([[c,-s],[s,c]])
    # Current confirmed surfaces came from this full-revolution scan, not leased forecasts.
    bins = np.rint(np.mod(np.arctan2(base[:,1],base[:,0])-angle_min,2*np.pi)/angle_increment).astype(int)%len(result)
    result[bins] = np.inf
    return result


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


def scan_clusters(ranges, angle_min, angle_increment, pose, range_min=.15, range_max=np.inf, minimum_points=4):
    ranges = np.asarray(ranges, dtype=float)
    pose = np.asarray(pose, dtype=float)
    if ranges.ndim!=1 or pose.shape != (3,) or not np.isfinite(pose).all() or not np.isfinite([angle_min,angle_increment,range_min]).all() or angle_increment <= 0 or not 0<=range_min<range_max:
        raise ValueError('invalid scan pose/angles')
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    indices = np.flatnonzero(valid)
    base = np.column_stack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
    groups = scan_groups(base,indices,len(ranges))
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rotation = np.asarray([[c,-s],[s,c]])
    # shortcut: track separated compact clusters only; extend for merged crowds/large movers.
    return [np.asarray(group)@rotation.T+pose[:2] for group in groups
            if len(group)>=minimum_points and np.linalg.norm(np.ptp(group,axis=0))<=1.2]


def surface_translation(reference, points):
    """Fit translation against surface normals instead of a visibility-dependent centroid."""
    tree = cKDTree(reference)
    # Global PCA on a small surface leaves translation rank deficient.
    neighbors = reference[tree.query(reference, k=min(5, len(reference)-1))[1]]
    centered = neighbors-neighbors.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)
    normals = np.linalg.eigh(covariance)[1][:, :, 0]
    shift = points.mean(axis=0)-reference.mean(axis=0)
    for _ in range(8):
        distances, indices = tree.query(points-shift)
        if np.max(distances) < 1e-9:
            break
        n = normals[indices]
        residual = np.sum(n*(points-shift-reference[indices]), axis=1)
        delta = np.linalg.lstsq(n, residual, rcond=None)[0]
        shift += delta
        if np.linalg.norm(delta) < 1e-6:
            break
    return shift, np.median(tree.query(points-shift)[0])


def track_velocity(history, points):
    if len(history)<4 or history[-1][0]-history[0][0]<.3:
        return None
    stamps = np.asarray([row[0] for row in history])
    reference = history[0][2]
    registered = [surface_translation(reference, row[2]) for row in history]
    if max(row[1] for row in registered) > .04:
        return None
    centers = np.asarray([row[0] for row in registered])
    times = stamps-stamps.mean()
    velocity = (times[:,None]*(centers-centers.mean(axis=0))).sum(axis=0)/(times@times)
    residual = np.linalg.norm(centers-centers.mean(axis=0)-times[:,None]*velocity,axis=1)
    if not .08<=np.linalg.norm(velocity)<=1.2 or residual.max()>.04:
        return None
    eigenvalues,axes = np.linalg.eigh(np.cov(reference.T))
    # A visible fragment sliding along a straight surface does not prove surface motion.
    if eigenvalues[0]<.03*eigenvalues[1] and abs(velocity@axes[:,0])<.05:
        return None
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
        shift,error = surface_translation(old[j]['points'],clusters[i])
        # Shrinking visibility does not mean the object changed shape.
        growth = np.ptp(clusters[i],axis=0)-np.ptp(old[j]['points'],axis=0)
        if np.linalg.norm(shift)<=.3 and error<=.04 and np.max(growth)<=.15:
            matches[i]=j
            used.add(j)
    return matches


def validated_clusters(clusters):
    result = [np.asarray(points,dtype=float) for points in clusters]
    if any(points.ndim!=2 or points.shape[1]!=2 or not len(points) or not np.isfinite(points).all() for points in result):
        raise ValueError('invalid cluster')
    return result


def static_map_exclusions(clusters, cells, distances, resolution, origin, robot, variance, margin):
    """Reject a whole compact surface only when the static map clearly contradicts it."""
    c, s = np.cos(origin[2]), np.sin(origin[2])
    exclusions = []
    for points in clusters:
        grid = np.floor((points-origin[:2])@np.array([[c,-s],[s,c]])/resolution).astype(int)
        x, y = grid.T
        inside = (x>=0)&(x<cells.shape[1])&(y>=0)&(y<cells.shape[0])
        if not inside.all() or not (cells[y,x]==0).all():
            exclusions.append(False)
            continue
        radius = np.linalg.norm(points-robot[:2],axis=1)
        # Triangle bound includes translation/yaw correlation without assuming independence.
        uncertainty = 3*(np.sqrt(np.maximum(variance[:2],0).sum())+radius*np.sqrt(max(variance[2],0)))
        exclusions.append(bool((distances[y,x]>margin+uncertainty+np.sqrt(2)*resolution).all()))
    return exclusions


def moving_surfaces(tracks, stamp, key='motion'):
    moving = []
    for track in tracks:
        if key not in track:
            continue
        observed,points,velocity = track[key]
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
        track['forecast_motion'] = previous['forecast_motion']
    velocity = track_velocity(history,points)
    if velocity is not None:
        # Retain recently observed sides without extending motion confirmation or its lease.
        surfaces = np.concatenate([points, *[surface+velocity*(stamp-observed)
            for observed,_,surface in history if 0 < stamp-observed <= .5]])
        _, first = np.unique(surfaces.astype(np.float32), axis=0, return_index=True)
        track['motion'] = (stamp,points,velocity)
        track['forecast_motion'] = (stamp,surfaces[np.sort(first)],velocity)
    return track,velocity


def unobserved_motion(previous, matched, stamp):
    return [track for track in previous if id(track) not in matched and
            'motion' in track and stamp-track['motion'][0]<=.5]


class MotionPredictor:
    def __init__(self):
        self.tracks = []
        self.stamp = None
        self.moving_points = np.empty((0,4))
        self.forecast_points = np.empty((0,4))

    def update(self, clusters, stamp):
        if not np.isfinite(stamp) or stamp<0 or (self.stamp is not None and stamp<=self.stamp):
            raise ValueError('non-increasing/invalid scan stamp')
        clusters = validated_clusters(clusters)
        previous = self.tracks
        old = [track for track in previous if stamp-track['history'][-1][0]<=.25]
        centers = [points.mean(axis=0) for points in clusters]
        matches = match_clusters(clusters,centers,old)
        self.tracks = []
        velocities = []
        for i,points in enumerate(clusters):
            track,velocity = update_track(points,centers[i],stamp,
                                          old[matches[i]] if i in matches else None)
            self.tracks.append(track)
            if velocity is not None:
                velocities.append([*centers[i],*velocity])
        self.stamp = stamp
        matched = {id(old[j]) for j in matches.values()}
        self.tracks.extend(unobserved_motion(previous,matched,stamp))
        self.moving_points = moving_surfaces(self.tracks,stamp)
        self.forecast_points = moving_surfaces(self.tracks,stamp,'forecast_motion')
        # Forecast geometry includes past sides; SLAM keeps actual retained observations.
        predicted = [self.forecast_points[:,:2]+self.forecast_points[:,2:]*t
                     for t in np.arange(.2,4.01,.2)]
        return (np.concatenate(predicted),
                np.asarray(velocities,dtype=float).reshape(-1,4))
