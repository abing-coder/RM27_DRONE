#!/usr/bin/env python3
"""Fine-tune YOLO11n-pose on render_sim_dataset.py output, then export static ONNX.

Training is optional and uses a separate CPU virtualenv, never the ROS runtime.
Ultralytics 8.3.0 / torch 2.5.1+cpu / torchvision 0.20.1+cpu / onnx 1.17.0.
The resulting model inherits Ultralytics AGPL-3.0; see models/sim_armor/SOURCE.md.
"""
import argparse
import os
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--pretrained', default='yolo11n-pose.pt')
    ap.add_argument('--threads', type=int, default=8)
    args = ap.parse_args()
    if not args.data.is_file() or args.threads < 1:
        ap.error('Existing dataset YAML and a positive thread count are required')
    if args.output.exists() and any(args.output.iterdir()):
        ap.error('Output must be empty; existing training results are never overwritten')
    os.environ.update(OMP_NUM_THREADS=str(args.threads), OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS=str(args.threads))
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(args.threads)
    model = YOLO(args.pretrained)
    model.train(data=str(args.data.resolve()), epochs=45, imgsz=416, batch=16, device='cpu', workers=0,
                project=str(args.output.resolve()), name='sim-armor', exist_ok=True, pretrained=True,
                seed=2701, cache=True, amp=False, plots=False, fliplr=0, flipud=0, mosaic=.35,
                close_mosaic=10, hsv_h=.015, hsv_s=.35, hsv_v=.3, degrees=10, translate=.12,
                scale=.3, patience=15)
    YOLO(str(args.output/'sim-armor/weights/best.pt')).export(format='onnx', imgsz=640,
                dynamic=False, simplify=False, opset=13)


if __name__ == '__main__':
    main()
