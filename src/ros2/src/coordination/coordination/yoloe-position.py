from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

from ultralytics import YOLOE


@dataclass(frozen=True)
class CameraParameters:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


def _normalize_prompts(target_prompt: str | list[str]) -> list[str]:
    """统一 text prompt 输入格式。"""
    prompts = [target_prompt] if isinstance(target_prompt, str) else list(target_prompt)
    prompts = [prompt for prompt in prompts if prompt]
    if not prompts:
        raise ValueError("target_prompt must contain at least one class name")
    return prompts


def load_camera_parameters(
    settings_path: str | Path,
    vehicle_name: str = "UAV_1",
    camera_name: str = "front_center",
    image_type: int = 0,
) -> CameraParameters:
    """从 AirSim settings.json 读取相机内参并转成针孔模型参数。"""
    with Path(settings_path).expanduser().open("r", encoding="utf-8") as file:
        settings = json.load(file)

    vehicle = settings["Vehicles"][vehicle_name]
    camera = vehicle["Cameras"][camera_name]
    capture = next(item for item in camera["CaptureSettings"] if int(item["ImageType"]) == image_type)

    width = int(capture["Width"])
    height = int(capture["Height"])
    fov_deg = float(capture["FOV_Degrees"])
    fx = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    fy = fx

    return CameraParameters(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=width * 0.5,
        cy=height * 0.5,
    )


def pixel_to_camera_2d(u: float, v: float, camera: CameraParameters) -> dict[str, float]:
    """把像素中心点解算成相对摄像头的 2D 方向坐标。"""
    return {
        "x": (float(u) - camera.cx) / camera.fx,
        "y": (float(v) - camera.cy) / camera.fy,
    }


def pixel_to_camera_3d(
    u: float,
    v: float,
    camera: CameraParameters,
    depth_m: float,
) -> dict[str, float]:
    """用中心点深度把像素反投影到相机系 xyz。"""
    if not math.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError(f"invalid depth value: {depth_m}")

    # z 为中心点深度。
    z = float(depth_m)

    # 针孔反投影。
    return {
        "x": (float(u) - camera.cx) * z / camera.fx,
        "y": (float(v) - camera.cy) * z / camera.fy,
        "z": z,
    }


def sample_depth(depth_image: Any, u: float, v: float) -> float:
    """读取目标中心点的深度值。"""
    x = int(round(u))
    y = int(round(v))
    return float(depth_image[y][x])


def yoloe_2d_detact(
    image: Any,
    target_prompt: str | list[str],
    camera: CameraParameters,
    model_path: str = "yoloe-26x-seg.pt",
    conf: float = 0.25,
    iou: float = 0.7,
    return_annotated: bool = False,
) -> list[dict[str, Any]] | tuple[list[dict[str, Any]], Any | None]:
    """执行 YOLOE 检测并返回目标相对摄像头的 2D 坐标。

    Args:
        image: 输入2D图像
        target_prompt: 文本提示词
        camera: 相机内参
        model_path: YOLOE模型权重路径
        conf: 检测置信度阈值
        iou: NMS使用的IOU阈值
        return_annotated: 是否同时返回YOLOE自动标注后的图像

    Returns:
        detections: 目标2D检测结果列表，包含target、confidence、x、y、u、v、h_px。
        annotated_image: return_annotated=True时返回的自动标注图像，否则不返回该值。
    """
    prompts = _normalize_prompts(target_prompt)

    # 初始化模型并设置 text prompt。
    model = YOLOE(model_path)
    model.set_classes(prompts)

    # 执行检测，不保存图片。
    results = model.predict(source=image, conf=conf, iou=iou, verbose=False)
    if not results or results[0].boxes is None:
        return ([], None) if return_annotated else []

    # YOLOE 自动绘制检测框和类别标签；只返回图像，不保存文件。
    annotated_image = results[0].plot() if return_annotated else None

    # 取第一张图结果。
    boxes = results[0].boxes
    names = results[0].names
    xyxy = boxes.xyxy.tolist()
    classes = boxes.cls.tolist() if boxes.cls is not None else [None] * len(xyxy)
    confidences = boxes.conf.tolist() if boxes.conf is not None else [None] * len(xyxy)
    detections: list[dict[str, Any]] = []

    for index, bbox in enumerate(xyxy):
        # 由 bbox 得到中心像素，再解算相对相机 2D 方向坐标。
        x1, y1, x2, y2 = [float(value) for value in bbox]
        u = (x1 + x2) * 0.5
        v = (y1 + y2) * 0.5
        h_px = y2 - y1
        pose_2d = pixel_to_camera_2d(u, v, camera)

        class_value = classes[index]
        class_id = None if class_value is None else int(class_value)
        confidence_value = confidences[index]

        detections.append(
            {
                "target": names.get(class_id, str(class_id)) if class_id is not None else "unknown",
                "confidence": None if confidence_value is None else float(confidence_value),
                "x": pose_2d["x"],
                "y": pose_2d["y"],
                "u": u,
                "v": v,
                "h_px": h_px,
            }
        )

    return (detections, annotated_image) if return_annotated else detections


def yoloe_3d_detact(
    image: Any,
    depth_image: Any,
    target_prompt: str | list[str],
    camera: CameraParameters,
    model_path: str = "yoloe-26x-seg.pt",
    conf: float = 0.25,
    iou: float = 0.7,
) -> dict[str, Any]:
    """2D图像+文本提示直接输出目标相对相机的3D位置xyz。

    Args:
        image: 输入的2D图像
        depth_image: 与2D图像对齐的深度图，单位米
        target_prompt: 文本检测提示，指定需要检测的目标类别
        camera: 相机内参
        model_path: 模型路径
        conf: 检测置信度阈值。
        iou: NMS使用的IOU阈值

    Returns:
        detections: 目标3D检测结果列表，包含target、confidence、x、y、z
        annotated_image: YOLOE自动标注目标后的图像
    """
    # 复用现有2D检测函数，同时取回 YOLOE 自动标注图。
    detections_2d, annotated_image = cast(
        tuple[list[dict[str, Any]], Any | None],
        yoloe_2d_detact(
            image=image,
            target_prompt=target_prompt,
            camera=camera,
            model_path=model_path,
            conf=conf,
            iou=iou,
            return_annotated=True,
        ),
    )

    outputs: list[dict[str, float | str | None]] = []

    for det in detections_2d:
        depth_m = sample_depth(depth_image, float(det["u"]), float(det["v"]))
        xyz = pixel_to_camera_3d(
            u=float(det["u"]),
            v=float(det["v"]),
            camera=camera,
            depth_m=depth_m,
        )
        outputs.append(
            {
                "target": str(det["target"]),
                "confidence": det["confidence"],
                "x": xyz["x"],
                "y": xyz["y"],
                "z": xyz["z"],
            }
        )

    return {
        "detections": outputs,
        "annotated_image": annotated_image,
    }
