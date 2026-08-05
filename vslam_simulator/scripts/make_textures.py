#!/usr/bin/env python3
"""Generate high-frequency textures for the SLAM world.

Monocular ORB-SLAM3 needs corners to track. Flat-shaded surfaces produce
features only at silhouette edges, which is too sparse to initialise. These
patterns are chosen for corner density, and each pillar gets a DIFFERENT one so
place recognition cannot confuse one pillar for another.

  python3 vslam_simulator/scripts/make_textures.py vslam_simulator/materials/textures
"""
import sys
import pathlib
import numpy as np
from PIL import Image

S = 512
rng = np.random.default_rng(7)  # fixed seed: textures must be reproducible


def _rgb(a):
    return np.clip(a, 0, 255).astype(np.uint8)


def checker(n, c1, c2):
    i = (np.arange(S) * n // S)[:, None]
    j = (np.arange(S) * n // S)[None, :]
    m = ((i + j) % 2).astype(bool)
    out = np.zeros((S, S, 3))
    out[m] = c1
    out[~m] = c2
    return _rgb(out)


def blocks(n, base):
    """Random colour blocks - dense corners at every block junction.

    n must be reasonably high: coarse blocks are nearly featureless to ORB
    (n=8 measured at only 97 features vs >1000 for n=28).
    """
    idx = (np.arange(S) * n // S)
    small = rng.integers(40, 235, size=(n, n, 3))
    out = small[idx[:, None], idx[None, :]].astype(float)
    out = out * 0.55 + np.array(base, float) * 0.45
    return _rgb(out + rng.normal(0, 14, (S, S, 1)))


def stripes_dots(base, accent):
    out = np.tile(np.array(base, float), (S, S, 1))
    out[:, ::16] = accent
    out[::16, :] = accent
    yy, xx = np.mgrid[0:S, 0:S]
    out[((yy % 64) - 32) ** 2 + ((xx % 64) - 32) ** 2 < 100] = accent
    return _rgb(out)


def rings(base, accent):
    yy, xx = np.mgrid[0:S, 0:S]
    r = np.sqrt((yy - S / 2) ** 2 + (xx - S / 2) ** 2)
    m = ((r // 18) % 2).astype(bool)
    out = np.zeros((S, S, 3))
    out[m] = base
    out[~m] = accent
    return _rgb(out)


def noise(base, amp=70):
    n = rng.normal(0, amp, (S, S, 1))
    return _rgb(np.array(base, float) + n)


def grid_floor():
    out = np.array([116, 110, 100], float) + rng.normal(0, 16, (S, S, 1))
    out[::64, :] = [58, 54, 48]
    out[:, ::64] = [58, 54, 48]
    out[1::64, :] = [58, 54, 48]
    out[:, 1::64] = [58, 54, 48]
    return _rgb(out)


# nine visually distinct pillar textures
PILLARS = [
    lambda: checker(16, (220, 60, 50), (245, 240, 230)),
    lambda: blocks(28, (60, 90, 160)),
    lambda: stripes_dots((240, 200, 60), (40, 40, 50)),
    lambda: rings((30, 140, 120), (240, 240, 235)),
    lambda: checker(24, (40, 45, 55), (200, 205, 215)),
    lambda: blocks(36, (150, 70, 150)),
    lambda: stripes_dots((90, 170, 70), (25, 30, 25)),
    lambda: rings((200, 90, 40), (250, 235, 200)),
    lambda: noise((150, 150, 155), 80),
]


def main(outdir):
    out = pathlib.Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    for i, fn in enumerate(PILLARS, 1):
        p = out / f'pillar_{i}.png'
        Image.fromarray(fn()).save(p)
        print(f'  {p}')
    Image.fromarray(grid_floor()).save(out / 'floor.png')
    print(f'  {out / "floor.png"}')
    Image.fromarray(blocks(32, (170, 160, 145))).save(out / 'wall.png')
    print(f'  {out / "wall.png"}')


def demo():
    """Self-check: textures are the right size, distinct, and ORB-rich.

    The feature-count assert is the one that matters - a texture can look busy
    to a human and still be nearly invisible to ORB.
    """
    import cv2
    orb = cv2.ORB_create(nfeatures=2000)
    imgs = [fn() for fn in PILLARS] + [grid_floor(), blocks(32, (170, 160, 145))]
    for i, im in enumerate(imgs):
        assert im.shape == (S, S, 3), f'{i}: {im.shape}'
        n = len(orb.detect(cv2.cvtColor(im, cv2.COLOR_RGB2GRAY), None))
        assert n > 500, f'texture {i} only {n} ORB features'
    for i in range(len(PILLARS)):
        for j in range(i + 1, len(PILLARS)):
            d = np.abs(imgs[i].astype(int) - imgs[j].astype(int)).mean()
            assert d > 8, f'pillar {i} and {j} too similar: {d:.1f}'
    print('demo ok')


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'demo':
        demo()
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else 'materials/textures')
