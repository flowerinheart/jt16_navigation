"""Arena's existing seeded kinematic path, shared with diagnostic state output."""
import math


def motion_state(agent, seed, elapsed):
    points = [agent['pos'], *agent.get('waypoints', [])]
    points = [points[(i+seed) % len(points)] for i in range(len(points))]
    if seed % 2:
        points = list(reversed(points))
    speed = max(float(agent.get('vmax', .3)), .1)
    lengths = [math.dist(x, y) for x, y in zip(points, points[1:]+points[:1])]
    period = sum(lengths)/speed
    phase = elapsed % period if period else 0.
    for x, y, length in zip(points, points[1:]+points[:1], lengths):
        duration = length/speed
        if phase <= duration:
            ratio = phase/duration if duration else 0.
            position = [x[0]+ratio*(y[0]-x[0]), x[1]+ratio*(y[1]-x[1])]
            velocity = [speed*(y[0]-x[0])/length, speed*(y[1]-x[1])/length] if length else [0., 0.]
            return position, velocity
        phase -= duration
    return points[0], [0., 0.]
