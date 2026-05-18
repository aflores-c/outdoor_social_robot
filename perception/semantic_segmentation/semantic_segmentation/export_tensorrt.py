"""
export_tensorrt.py
──────────────────
Exports YOLO and SegFormer models to TensorRT .engine format for
maximum performance on Jetson Orin (and RTX desktop).

Run once before deploying:
  ros2 run semantic_segmentation export_tensorrt --ros-args \\
      -p model:=yolo -p yolo_model:=yolov8n-seg.pt -p half:=true

  ros2 run semantic_segmentation export_tensorrt --ros-args \\
      -p model:=segformer -p segformer_model:=nvidia/segformer-b0-finetuned-cityscapes-512-1024 \\
      -p segformer_width:=640 -p segformer_height:=320 -p half:=true

Output files
────────────
  YOLO     → same directory as input .pt, with .engine extension
             e.g.  yolov8n-seg.engine
  SegFormer → ./segformer_b0_cityscapes.onnx  (then use trtexec separately)

After export, point the config yaml to the .engine / .onnx file:
  yolo_model: "/path/to/yolov8n-seg.engine"
"""

import argparse
import sys
from pathlib import Path


def export_yolo(yolo_model: str, half: bool, img_size: int,
                device: str) -> None:
    print(f'\n[YOLO TensorRT export]')
    print(f'  model    : {yolo_model}')
    print(f'  img_size : {img_size}')
    print(f'  half     : {half}')
    print(f'  device   : {device}')

    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit('ERROR: ultralytics not installed.  pip install ultralytics')

    model  = YOLO(yolo_model)
    engine = model.export(
        format='engine',
        imgsz=img_size,
        half=half,
        device=device,
        simplify=True,
        verbose=True,
    )
    print(f'\nExported TensorRT engine → {engine}')
    print('Update your config yaml:')
    print(f'  yolo_model: "{engine}"')


def export_segformer(model_name: str, width: int, height: int,
                      half: bool, output_dir: str) -> None:
    print(f'\n[SegFormer ONNX export]')
    print(f'  model    : {model_name}')
    print(f'  size     : {width}×{height}')
    print(f'  half     : {half}')

    try:
        import torch
        from transformers import (SegformerImageProcessor,
                                   SegformerForSemanticSegmentation)
    except ImportError:
        sys.exit('ERROR: transformers or torch not installed.')

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'  device   : {device}')

    model = SegformerForSemanticSegmentation.from_pretrained(model_name)
    model.eval().to(device)
    if half and device.type == 'cuda':
        model = model.half()
        dtype = torch.float16
    else:
        dtype = torch.float32

    dummy = torch.zeros(1, 3, height, width, dtype=dtype, device=device)

    out_dir  = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    safe_name = model_name.replace('/', '_').replace('-', '_')
    onnx_path = out_dir / f'{safe_name}_{width}x{height}.onnx'

    torch.onnx.export(
        model,
        {'pixel_values': dummy},
        str(onnx_path),
        input_names=['pixel_values'],
        output_names=['logits'],
        dynamic_axes={'pixel_values': {0: 'batch'}},
        opset_version=17,
    )
    print(f'\nExported ONNX → {onnx_path}')
    print('\nConvert to TensorRT with trtexec (run on the target device):')
    fp_flag = '--fp16' if half else ''
    print(f'  trtexec --onnx={onnx_path} {fp_flag} \\')
    print(f'          --saveEngine={onnx_path.with_suffix(".engine")} \\')
    print(f'          --minShapes=pixel_values:1x3x{height}x{width} \\')
    print(f'          --optShapes=pixel_values:1x3x{height}x{width} \\')
    print(f'          --maxShapes=pixel_values:1x3x{height}x{width}')


def main() -> None:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('export_tensorrt')

    node.declare_parameter('model',            'yolo')
    node.declare_parameter('yolo_model',       'yolov8n-seg.pt')
    node.declare_parameter('yolo_img_size',    640)
    node.declare_parameter('segformer_model',  'nvidia/segformer-b0-finetuned-cityscapes-512-1024')
    node.declare_parameter('segformer_width',  640)
    node.declare_parameter('segformer_height', 320)
    node.declare_parameter('half',             True)
    node.declare_parameter('device',           '0')
    node.declare_parameter('output_dir',       '.')

    p = lambda n: node.get_parameter(n).value

    model_type = p('model').lower()
    if model_type == 'yolo':
        export_yolo(
            yolo_model = p('yolo_model'),
            half       = p('half'),
            img_size   = p('yolo_img_size'),
            device     = p('device'),
        )
    elif model_type == 'segformer':
        export_segformer(
            model_name = p('segformer_model'),
            width      = p('segformer_width'),
            height     = p('segformer_height'),
            half       = p('half'),
            output_dir = p('output_dir'),
        )
    else:
        node.get_logger().error(f'Unknown model type: {model_type}  (use yolo or segformer)')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
