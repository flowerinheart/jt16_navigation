"""Pure geometry used by navigation scoring and regression tests."""
import math
import heapq
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation


def dilate_occupied(occupied):
    return binary_dilation(occupied, structure=np.ones((3, 3), dtype=bool))


def shortest_path(occupied, start, goal):
    def free(cell):
        return (0 <= cell[0] < occupied.shape[0] and
                0 <= cell[1] < occupied.shape[1] and not occupied[cell])
    if not free(start) or not free(goal):
        raise ValueError('invalid start or goal')
    queue, cost = [(0., start)], {start: 0.}
    while queue:
        distance, current = heapq.heappop(queue)
        if distance != cost[current]:
            continue
        if current == goal:
            return distance
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nxt = (current[0] + dy, current[1] + dx)
            if not free(nxt):
                continue
            if dy and dx and (not free((current[0] + dy, current[1])) or
                              not free((current[0], current[1] + dx))):
                continue
            value = distance + math.hypot(dx, dy)
            if value < cost.get(nxt, math.inf):
                cost[nxt] = value
                heapq.heappush(queue, (value, nxt))
    raise ValueError('no feasible path')


def base_pose(position, quaternion, mount):
    """FAST-LIO body is IMU; simulation IMU is collocated with LiDAR."""
    world_imu = Rotation.from_quat(quaternion)
    base_imu = Rotation.from_euler('xyz', mount['ros_urdf_rpy_rad'])
    world_base = world_imu * base_imu.inv()
    return (np.asarray(position) - world_base.apply(mount['position_m']),
            world_base.as_quat())


def nearest_time_index(times, stamp, max_gap=.25):
    times = np.asarray(times, dtype=float)
    if not len(times) or not np.isfinite(times).all():
        raise ValueError('missing/invalid observation timestamps')
    index = int(np.argmin(np.abs(times - stamp)))
    if abs(times[index] - stamp) > max_gap:
        raise ValueError('observation timestamp gap')
    return index


def obstacle_observations_fresh(stamps, count, now):
    return all(i in stamps and math.isfinite(stamps[i]) and
               -.03 <= now-stamps[i] <= .5 for i in range(count))


def dynamic_clearances(robot, observations):
    """observations rows: [absolute stamp, id, x, y, radius]."""
    data = np.asarray(observations, dtype=float)
    if data.ndim != 2 or data.shape[1] != 5 or not np.isfinite(data).all():
        raise ValueError('missing/invalid obstacle observations')
    ids = np.unique(data[:, 1])
    clearances = []
    for agent in ids:
        samples = data[data[:, 1] == agent]
        for x, y, yaw, stamp in robot:
            index = nearest_time_index(samples[:, 0], stamp)
            _, _, ox, oy, radius = samples[index]
            dx, dy = ox - x, oy - y
            bx = math.cos(yaw) * dx + math.sin(yaw) * dy
            by = -math.sin(yaw) * dx + math.cos(yaw) * dy
            clearances.append(math.hypot(max(abs(bx) - .20, 0),
                                         max(abs(by) - .235, 0)) - float(radius))
    return clearances


def static_collision(robot, rectangles):
    """SAT intersection between robot footprint and axis-aligned wall boxes."""
    boxes = np.asarray(rectangles, dtype=float).reshape(-1, 4)
    for x, y, yaw, _ in robot:
        u = np.array([math.cos(yaw), math.sin(yaw)])
        v = np.array([-math.sin(yaw), math.cos(yaw)])
        candidates = boxes[(np.abs(boxes[:, 0] - x) <= boxes[:, 2] + .31) &
                           (np.abs(boxes[:, 1] - y) <= boxes[:, 3] + .31)]
        for cx, cy, hx, hy in candidates:
            delta = np.array([cx - x, cy - y])
            separated = False
            for axis in (u, v, np.array([1., 0.]), np.array([0., 1.])):
                robot_radius = .20 * abs(axis @ u) + .235 * abs(axis @ v)
                wall_radius = hx * abs(axis[0]) + hy * abs(axis[1])
                if abs(delta @ axis) > robot_radius + wall_radius:
                    separated = True
                    break
            if not separated:
                return True
    return False


def covariance_valid(values):
    # AMCL's E[x*x]-E[x]^2 can produce tiny negative roundoff at zero variance.
    return all(math.isfinite(value) and -1e-9 <= value <= 1. for value in values)


def localization_metrics(amcl, truth, now):
    if not amcl:
        return {'localization_success': False, 'amcl_message_received': False}
    pose = amcl[-1]
    try:
        index = nearest_time_index([row[3] for row in truth], pose[3], .1)
    except ValueError:
        return {'localization_success': False, 'amcl_message_received': True}
    reference = truth[index]
    distance = math.hypot(pose[0] - reference[0], pose[1] - reference[1])
    yaw = abs(math.atan2(math.sin(pose[2] - reference[2]),
                         math.cos(pose[2] - reference[2])))
    fresh = 0 <= now - pose[3] <= 1.
    valid = len(pose) >= 7 and covariance_valid(pose[4:7])
    return {'localization_success': bool(fresh and valid and distance <= .5 and yaw <= .35),
            'amcl_message_received': True, 'localization_translation_error_m': distance,
            'localization_yaw_error_rad': yaw}


def tf_valid(edges):
    required = {'map->odom': 'amcl', 'odom->base_link': 'lio_odom_bridge'}
    return (all(len(nodes) == 1 for nodes in edges.values()) and
            all(len(edges.get(edge, [])) == 1 and
                edges[edge][0].split('/')[-1].split('@')[0] == owner
                for edge, owner in required.items()))


def cpu_delta(previous, samples):
    delta = 0.
    for identity, value in samples.items():
        if identity in previous:
            delta += max(0., value - previous[identity])
        previous[identity] = value
    return delta
