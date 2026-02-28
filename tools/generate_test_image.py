#!/usr/bin/env python3
"""Generate a synthetic road image with simple lane curves for demo/testing."""
import cv2
import numpy as np
import sys

def make_image(w=1280, h=720, out='test.jpg'):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    # sky
    img[:int(h*0.5), :] = (50, 50, 50)
    # road
    cv2.rectangle(img, (0, int(h*0.5)), (w, h), (60,60,60), -1)

    # draw left/right lane as quadratic curves
    def draw_lane(a, b, c, color):
        pts = []
        for y in range(int(h*0.5), h, 5):
            x = int(a*y*y + b*y + c)
            pts.append((x, y))
        for i in range(len(pts)-1):
            cv2.line(img, pts[i], pts[i+1], color, 8)

    # left curve
    draw_lane(1e-6, -0.3, 600, (255,255,255))
    # right curve
    draw_lane(1e-6, 0.28, 700, (255,255,255))

    cv2.imwrite(out, img)
    print('Wrote', out)

if __name__ == '__main__':
    out = 'test.jpg'
    if len(sys.argv) > 1:
        out = sys.argv[1]
    make_image(out=out)
