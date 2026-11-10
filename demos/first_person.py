"""CPU perspective replay of upright SDF boxes/cylinders, using recorded world poses."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def scene(path):
    shapes = []
    for model in ET.parse(path).getroot().findall('world/model'):
        if model.get('name') == 'xlerobot':
            continue
        pose = np.fromstring(model.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        for link in model.findall('link'):
            lp = np.fromstring(link.findtext('pose', '0 0 0 0 0 0'), sep=' ')
            for visual in link.findall('visual'):
                vp = np.fromstring(visual.findtext('pose', '0 0 0 0 0 0'), sep=' ')
                p = pose + lp + vp
                geometry = visual.find('geometry')
                if geometry is None or geometry.find('plane') is not None:
                    continue
                # shortcut: benchmark geometry is upright; reject other SDF scenes until transforms are supported.
                if np.any(np.abs(p[3:5]) > 1e-8):
                    raise ValueError('Replay supports upright benchmark geometry only')
                box, cylinder = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    size = np.fromstring(box.findtext('size'), sep=' ')
                    kind = 'box'
                elif cylinder is not None:
                    size = np.array([float(cylinder.findtext('radius')), 0, float(cylinder.findtext('length'))])
                    kind = 'cylinder'
                else:
                    raise ValueError(f'Unsupported visual geometry in {model.get("name")}')
                color = np.fromstring(visual.findtext('material/diffuse', '.65 .68 .72 1'), sep=' ')[:3]
                shapes.append((model.get('name'), kind, p, size, color))
    assert shapes, 'No supported scene geometry'
    return shapes


def image(shapes, pose, actors=(), width=360, height=240):
    """Virtual level camera at 0.38 m, horizontal FOV 90 degrees, +X forward."""
    x, y, yaw = pose[:3]
    camera_z = .38
    u = ((np.arange(width) + .5) - width / 2) / (width / 2)
    v = (height / 2 - (np.arange(height) + .5)) / (width / 2)
    dx, dy = math.cos(yaw) + u * math.sin(yaw), math.sin(yaw) - u * math.cos(yaw)
    with np.errstate(divide='ignore', invalid='ignore'):
        floor = -camera_z / v[:, None] * np.ones((1, width))
    depth = np.where(floor > 0, floor, np.inf)
    rgb = np.empty((height, width, 3)); rgb[:] = [.65, .73, .82]
    floor_mask = np.isfinite(depth)
    fx, fy = x + depth * dx, y + depth * dy
    checker = (np.floor(np.where(floor_mask, fx, 0)) + np.floor(np.where(floor_mask, fy, 0))) % 2
    rgb[floor_mask] = np.repeat((.46 + .035 * checker[floor_mask])[:, None], 3, axis=1)
    positions = {int(a[1]): a for a in actors}
    for name, kind, p, size, color in shapes:
        p = p.copy()
        if name.startswith('dynamic_'):
            actor = positions.get(int(name.split('_')[-1]))
            if actor is None:
                continue
            p[:2] = actor[2:4]
            size = size.copy(); size[0] = actor[4]
            color = np.array([.82, .38, .16])
        c, s = math.cos(p[5]), math.sin(p[5])
        ox, oy = c*(x-p[0])+s*(y-p[1]), -s*(x-p[0])+c*(y-p[1])
        rx, ry = c*dx+s*dy, -s*dx+c*dy
        with np.errstate(divide='ignore', invalid='ignore'):
            if kind == 'box':
                tx = np.sort(np.array([(-size[0]/2-ox)/rx, (size[0]/2-ox)/rx]), axis=0)
                ty = np.sort(np.array([(-size[1]/2-oy)/ry, (size[1]/2-oy)/ry]), axis=0)
                near, far = np.maximum(tx[0], ty[0]), np.minimum(tx[1], ty[1])
            else:
                a = rx*rx+ry*ry; b = ox*rx+oy*ry
                disc = b*b-a*(ox*ox+oy*oy-size[0]**2)
                root = np.sqrt(np.maximum(disc, 0))
                near, far = (-b-root)/a, (-b+root)/a
                near = np.where(disc >= 0, near, np.inf)
            tz = np.sort(np.array([(p[2]-size[2]/2-camera_z)/v, (p[2]+size[2]/2-camera_z)/v]), axis=0)
        hit = np.maximum(near[None, :], tz[0, :, None])
        end = np.minimum(far[None, :], tz[1, :, None])
        visible = (hit > .03) & (hit <= end) & (hit < depth)
        shade = .55 + .45 / (1 + .04 * np.maximum(hit, 0))
        rgb[visible] = shade[visible, None] * color
        depth[visible] = hit[visible]
    return np.uint8(np.clip(rgb, 0, 1)*255)


if __name__ == '__main__':
    # Occlusion and +X forward/yaw conventions on a synthetic upright wall.
    wall = [('wall', 'box', np.array([3., 0, 1, 0, 0, 0]), np.array([.2, 2, 2]), np.array([1., 0, 0]))]
    front = image(wall, [0, 0, 0]); back = image(wall, [0, 0, math.pi])
    assert front[120, 180, 0] > 150 and front[120, 180, 1] == 0
    assert back[120, 180, 1] > 0
    far = [('far', 'box', np.array([5., 0, 1, 0, 0, 0]), np.array([.2, 2, 2]), np.array([0., 1, 0]))]
    assert np.array_equal(image(far + wall, [0, 0, 0]), image(wall + far, [0, 0, 0]))
    print('Perspective and occlusion checks passed')
