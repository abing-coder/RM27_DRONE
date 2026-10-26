#!/usr/bin/env python3
"""INT8 calibration from training images only (OpenVINO + NNCF 2.19.0)."""
import argparse
from pathlib import Path
import cv2
import numpy as np
import openvino as ov
import nncf


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', type=Path, required=True)
    ap.add_argument('--train-images', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    files = sorted(args.train_images.glob('*.jpg'))
    if not files or args.output.exists() or args.output.with_suffix('.bin').exists() or args.output.suffix != '.xml':
        ap.error('Provide nonempty training images and an unused .xml output')
    def transform(file):
        im = cv2.imread(str(file))
        h, w = im.shape[:2]
        scale = min(640/w, 640/h)
        nw, nh = int(w*scale), int(h*scale)
        dx, dy = (640-nw)//2, (640-nh)//2
        image = np.full((640,640,3), 114, np.uint8)
        image[dy:dy+nh, dx:dx+nw] = cv2.resize(im, (nw,nh))
        return np.transpose(image[:,:,::-1], (2,0,1))[None].astype(np.float32)/255
    model = ov.Core().read_model(str(args.model))
    quantized = nncf.quantize(model, nncf.Dataset(files, transform), subset_size=256,
                             preset=nncf.QuantizationPreset.MIXED)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ov.save_model(quantized, str(args.output))


if __name__ == '__main__':
    main()
