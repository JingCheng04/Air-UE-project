"""YOLOE 检测发布节点.

订阅 AirSim 前下视相机的 Scene + DepthPerspective 两路图像, 配对后调用
``yoloe-position.py`` 中的 ``yoloe_3d_detact`` 取得目标在相机系下的 xyz 位置,
然后把结果作为 ROS 话题发布:

发布:
    {out_prefix}/detections        std_msgs/String        (JSON 字符串, 含全部目标)
    {out_prefix}/target_pose       geometry_msgs/PoseStamped  (置信度最高目标在相机系)
    {out_prefix}/annotated_image   sensor_msgs/Image      (YOLOE 自动标注的可视化图)
    {out_prefix}/detected          std_msgs/Bool          (本帧是否识别到目标)

设计原则:
    1. 不修改 yoloe-position.py. 该文件名带 '-' 无法直接 import, 所以这里用
       importlib.util.spec_from_file_location 按路径加载.
    2. 缺少 ultralytics / cv_bridge 等可选依赖时, 节点不崩溃, 只打印 warning
       并停发结果, 让上层 RTL 节点自然退化为"无识别"巡航.
    3. 只做识别和发布, 任何状态切换 / 控制决策都留给上层 (uav_with_rtl_node).
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String


def _load_yoloe_module(yoloe_path: str):
    """按文件路径加载 yoloe-position.py 模块 (文件名带 '-' 不能直接 import)."""
    path = Path(yoloe_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"yoloe script not found: {path}")
    spec = importlib.util.spec_from_file_location("coordination_yoloe_position", str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot create import spec for {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _CachedYoloeFactory:
    """让 yoloe-position.py 里的 ``YOLOE(model_path)`` 调用复用同一个模型实例.

    原 ``yoloe-position.py`` 在每次 ``yoloe_2d_detact`` 里都要 ``YOLOE(model_path)``
    + ``set_classes(prompts)``, 这两步合起来要重读权重 + 加载 MobileCLIP text
    encoder, 单帧上百 ms 内存占用还很高, 30Hz 调一次直接把整个 ROS 图拖崩
    (RViz 一起卡). 这里用一个轻量代理替换原模块里的 ``YOLOE`` 符号:

    - 第一次调用真正构造模型并缓存
    - 后续调用返回同一实例
    - ``set_classes`` 也做同 prompts 缓存, prompt 没变就跳过

    保持对原识别程序零修改.
    """

    def __init__(self, real_yoloe):
        self._real_yoloe = real_yoloe
        self._model = None
        self._model_path: str | None = None
        self._classes_key: tuple | None = None

    def __call__(self, model_path: str):
        # 同一 path 复用; 切到新 path 时重新加载 (本节点目前用不到, 留个口).
        if self._model is None or self._model_path != model_path:
            self._model = self._real_yoloe(model_path)
            self._model_path = model_path
            self._classes_key = None
        return self  # 让 yoloe-position.py 后续 .set_classes / .predict 继续走代理

    # ---- 透明转发 ----
    def set_classes(self, prompts):
        key = tuple(prompts) if not isinstance(prompts, str) else (prompts,)
        if key != self._classes_key:
            self._model.set_classes(list(prompts))
            self._classes_key = key

    def predict(self, *args, **kwargs):
        return self._model.predict(*args, **kwargs)

    def __getattr__(self, name):
        # 兜底: 任何其他属性 / 方法直接转发到真实模型, 保持兼容性.
        return getattr(self._model, name)


class YoloeDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yoloe_detector_node")

        # ---- 参数 ----
        self.declare_parameter("uav_prefix", "/uav/airsim_node/UAV_1")
        self.declare_parameter("camera_name", "front_center")
        # AirSim wrapper 的图像话题命名: <prefix>/<camera>_<ImageType>/image
        # ImageType 0 -> Scene, ImageType 2 -> DepthPerspective.
        self.declare_parameter("scene_image_topic", "")
        self.declare_parameter("depth_image_topic", "")
        self.declare_parameter("out_prefix", "/uav/yoloe")
        self.declare_parameter("target_prompt", "unmanned ground vehicle")
        self.declare_parameter("model_path", "yoloe-26x-seg.pt")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.7)
        # 同步两路图像时间戳的最大容差 (秒). AirSim Scene/Depth 同帧渲染,
        # 实测延迟可能 > 200ms, 0.5s 留够余量.
        self.declare_parameter("sync_tolerance", 0.5)
        # 标注图像最大发布频率 (Hz). 推理本身可达 30Hz, 但 RViz 端在某些
        # 显卡/驱动 (尤其是 Wayland + 多 GL 客户端 + AirSim 占 GPU) 上对
        # 30Hz BGR8 sensor_data 的 Image 流会触发"failed to create drawable"
        # 直至 segfault. 默认 10Hz 即可让人眼连续, 也大幅降低 RViz 渲染压力.
        self.declare_parameter("publish_rate", 10.0)
        # 是否发布 raw Image. 在 RViz 上有 GL 崩溃倾向时建议设 False, 让
        # RViz 只订阅 ``annotated_image/compressed``, 走 image_transport 的
        # JPEG 路径, CPU 解压, 不走原始 BGR8 GL 上传.
        self.declare_parameter("publish_raw_image", True)
        # 是否发布 CompressedImage (JPEG). 默认开. 不需要时可关.
        self.declare_parameter("publish_compressed_image", True)
        # JPEG 质量 1-100, 越高越大. 80 在云模拟里压缩比和锯齿都合适.
        self.declare_parameter("jpeg_quality", 80)
        # 是否开启 OpenCV 本地弹窗 (cv2.imshow). 默认关闭:
        # 单线程 ROS 执行器下, GTK/Qt 后端会让 waitKey 偶发阻塞 200ms+,
        # 进而堵住 scene/depth 订阅回调, 导致 yoloe 推理掉帧 / 跟踪卡死.
        # 推荐用 rqt_image_view 订阅 .../annotated_image/compressed 看图.
        self.declare_parameter("show_window", False)
        self.declare_parameter("window_name", "yoloe")
        self.declare_parameter("settings_path", "~/Documents/AirSim/settings.json")
        self.declare_parameter("vehicle_name", "UAV_1")
        self.declare_parameter("yoloe_script_path",
                               "~/Air-UE-project/src/ros2/src/coordination/coordination/yoloe-position.py")

        uav_prefix = str(self.get_parameter("uav_prefix").value).rstrip("/")
        camera_name = str(self.get_parameter("camera_name").value)
        scene_topic = str(self.get_parameter("scene_image_topic").value) \
            or f"{uav_prefix}/{camera_name}_Scene/image"
        depth_topic = str(self.get_parameter("depth_image_topic").value) \
            or f"{uav_prefix}/{camera_name}_DepthPerspective/image"
        self.out_prefix = str(self.get_parameter("out_prefix").value).rstrip("/")
        # 多 prompt: 用 '|' 或 ',' 分隔; 每条都会作为单独类别送进 set_classes,
        # YOLOE 取并集. ROS 参数原生不支持 list[str] 在 launch 里方便传入,
        # 因此用一个字符串拼起来再 split.
        raw_prompt = str(self.get_parameter("target_prompt").value)
        prompts = [p.strip() for p in raw_prompt.replace("|", ",").split(",") if p.strip()]
        self.target_prompt: list[str] | str = prompts if len(prompts) > 1 else (prompts[0] if prompts else raw_prompt)
        self.model_path = str(self.get_parameter("model_path").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.sync_tolerance = float(self.get_parameter("sync_tolerance").value)
        settings_path = os.path.expanduser(str(self.get_parameter("settings_path").value))
        vehicle_name = str(self.get_parameter("vehicle_name").value)
        yoloe_script = os.path.expanduser(str(self.get_parameter("yoloe_script_path").value))

        # ---- 延迟加载: 依赖在 import 时即可能失败. 失败仅 warn, 不阻塞 ROS spin. ----
        self._ready = False
        self._yoloe = None
        self._camera = None
        self._bridge = None
        try:
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
            self._yoloe = _load_yoloe_module(yoloe_script)
            # 用缓存代理替换原模块里的 YOLOE 符号. 这样 yoloe-position.py 内部
            # 每次 yoloe_2d_detact() 里 model = YOLOE(model_path) 会拿到同一实例,
            # set_classes 也只在 prompt 变化时跑一次. 不修改原识别程序源码.
            real_yoloe = getattr(self._yoloe, "YOLOE", None)
            if real_yoloe is not None:
                self._yoloe.YOLOE = _CachedYoloeFactory(real_yoloe)
            self._camera = self._yoloe.load_camera_parameters(
                settings_path=settings_path,
                vehicle_name=vehicle_name,
                camera_name=camera_name,
                image_type=0,
            )
            self._ready = True
            self.get_logger().info(
                f"YOLOE ready: prompts={self.target_prompt!r}, model='{self.model_path}', "
                f"camera={self._camera.width}x{self._camera.height}"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"YOLOE detector disabled (missing dep or settings): {exc!r}. "
                f"Node will spin idly so the rest of the pipeline is unaffected."
            )

        # ---- 订阅 ----
        self.create_subscription(Image, scene_topic, self._scene_cb, qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, self._depth_cb, qos_profile_sensor_data)

        # ---- 发布 ----
        self.det_pub = self.create_publisher(String, f"{self.out_prefix}/detections", 10)
        self.pose_pub = self.create_publisher(PoseStamped, f"{self.out_prefix}/target_pose", 10)
        self.flag_pub = self.create_publisher(Bool, f"{self.out_prefix}/detected", 10)
        # raw + compressed 分两路, 让 RViz 可以选 compressed 减轻 GL 压力.
        self._publish_raw = bool(self.get_parameter("publish_raw_image").value)
        self._publish_compressed = bool(self.get_parameter("publish_compressed_image").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._show_window = bool(self.get_parameter("show_window").value)
        self._window_name = str(self.get_parameter("window_name").value)
        self._window_initialized = False
        # 弹窗刷新解耦: 推理回调里只把最新一帧 BGR 写到这个缓存,
        # 由独立定时器 5Hz 调 cv2.imshow + waitKey, 避免 imshow 阻塞推理.
        self._latest_imshow_frame = None
        if self._show_window:
            self.create_timer(0.2, self._imshow_tick)
        self.image_pub = (
            self.create_publisher(Image, f"{self.out_prefix}/annotated_image", 10)
            if self._publish_raw else None
        )
        self.compressed_pub = (
            self.create_publisher(
                CompressedImage, f"{self.out_prefix}/annotated_image/compressed", 10
            )
            if self._publish_compressed else None
        )

        # 最近一帧缓存. 收到一对配对成功的帧就触发一次推理.
        self._last_scene: Image | None = None
        self._last_depth: Image | None = None
        # 诊断计数器: 收到第一帧时打 log, 之后用 throttle 节流.
        self._scene_frames = 0
        self._depth_frames = 0
        self._inference_count = 0
        # 限频: 推理本身不限, 仅控制 image 发布最大频率.
        publish_rate = float(self.get_parameter("publish_rate").value)
        self._publish_min_interval = 1.0 / max(publish_rate, 0.1)
        self._last_image_publish_time = 0.0

        self.get_logger().info(
            f"sub: scene={scene_topic}, depth={depth_topic}; "
            f"pub prefix={self.out_prefix}, sync_tol={self.sync_tolerance:.2f}s, "
            f"image_rate={publish_rate:.1f}Hz, raw={self._publish_raw}, "
            f"compressed={self._publish_compressed} (q={self._jpeg_quality})"
        )

    # ------------------------- 工具 -------------------------
    @staticmethod
    def _stamp_seconds(image: Image) -> float:
        return image.header.stamp.sec + image.header.stamp.nanosec * 1e-9

    def _try_run(self) -> None:
        if not self._ready or self._last_scene is None or self._last_depth is None:
            return
        dt = abs(self._stamp_seconds(self._last_scene) - self._stamp_seconds(self._last_depth))
        if dt > self.sync_tolerance:
            # 诊断: 配对失败时也打 throttle log.
            self.get_logger().warn(
                f"image pair out of sync dt={dt:.3f}s > tol={self.sync_tolerance:.2f}s",
                throttle_duration_sec=2.0,
            )
            return
        scene = self._last_scene
        depth = self._last_depth
        # 防止同一帧反复推理.
        self._last_scene = None
        self._last_depth = None
        self._run_inference(scene, depth)

    def _run_inference(self, scene_msg: Image, depth_msg: Image) -> None:
        # 1) 解码 Scene/Depth. 用 passthrough 避免编码不匹配 segfault.
        try:
            import numpy as np
            scene_cv = self._bridge.imgmsg_to_cv2(scene_msg, desired_encoding="passthrough")
            depth_cv = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"image decode failed: {exc!r}", throttle_duration_sec=2.0,
            )
            return

        # 确保 scene_cv 是 3 通道 BGR uint8.
        if scene_cv is None or scene_cv.size == 0:
            return
        if scene_cv.ndim == 2:
            import cv2
            scene_cv = cv2.cvtColor(scene_cv, cv2.COLOR_GRAY2BGR)
        elif scene_cv.ndim == 3 and scene_cv.shape[2] == 4:
            scene_cv = scene_cv[:, :, :3].copy()  # BGRA -> BGR
        elif scene_cv.ndim == 3 and scene_cv.shape[2] != 3:
            return
        if scene_cv.dtype != np.uint8:
            scene_cv = scene_cv.astype(np.uint8)
        if not scene_cv.flags['C_CONTIGUOUS']:
            scene_cv = np.ascontiguousarray(scene_cv)

        # 2) 调 YOLOE. 失败 / 异常时把 detections 视作空, annotated 视作 None,
        #    走和"识别成功但 0 命中"完全一致的发布路径, 这样 RViz 始终能拿到画面.
        detections: list[dict[str, Any]] = []
        annotated = None
        # has_3d=True 表示 detections 含可信的 (x, y, z); False 表示只有 2D bbox,
        # 此时不发 target_pose.
        has_3d = False
        try:
            result: dict[str, Any] = self._yoloe.yoloe_3d_detact(
                image=scene_cv,
                depth_image=depth_cv,
                target_prompt=self.target_prompt,
                camera=self._camera,
                model_path=self.model_path,
                conf=self.conf,
                iou=self.iou,
            )
            detections = list(result.get("detections", []))
            annotated = result.get("annotated_image", None)
            has_3d = True
        except ValueError as exc:
            # yoloe-position.pixel_to_camera_3d 在 bbox 中心像素深度=0 / NaN 时
            # 直接抛 ValueError, 整帧的 detections 全部丢失. 退回到 2D 路径:
            # 仍能拿到 bbox + annotated 图, 用于可视化和上层"是否检测到"判断,
            # 但不再有 (x,y,z) -> 不发 target_pose.
            self.get_logger().info(
                f"3D depth invalid ({exc}); falling back to 2D detection only",
                throttle_duration_sec=2.0,
            )
            try:
                two_d = self._yoloe.yoloe_2d_detact(
                    image=scene_cv,
                    target_prompt=self.target_prompt,
                    camera=self._camera,
                    model_path=self.model_path,
                    conf=self.conf,
                    iou=self.iou,
                    return_annotated=True,
                )
                # 2D 函数返回 (list, annotated) 当 return_annotated=True.
                if isinstance(two_d, tuple):
                    dets_2d, annotated = two_d
                else:
                    dets_2d, annotated = two_d, None
                detections = list(dets_2d)
            except Exception as exc2:  # noqa: BLE001
                self.get_logger().warn(
                    f"yoloe 2d fallback failed: {exc2!r}", throttle_duration_sec=2.0,
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"yoloe inference failed: {exc!r}", throttle_duration_sec=2.0,
            )

        self._inference_count += 1
        # throttle log: 每秒最多一条, 让"识别有没有跑"在控制台立刻可见.
        self.get_logger().info(
            f"yoloe inference #{self._inference_count}: detections={len(detections)}",
            throttle_duration_sec=1.0,
        )

        # ---- 终端打印识别结果: 每次检测到目标都输出名称和坐标, 方便排查 ----
        if detections:
            for det in detections:
                name = det.get("class", det.get("name", "unknown"))
                conf = det.get("confidence", 0.0)
                if has_3d and all(k in det for k in ("x", "y", "z")):
                    self.get_logger().info(
                        f"\n  [DETECTED] {name} (conf={conf:.2f}) "
                        f"pos=({det['x']:.3f}, {det['y']:.3f}, {det['z']:.3f})"
                    )
                else:
                    bbox = det.get("bbox", None)
                    self.get_logger().info(
                        f"\n  [DETECTED] {name} (conf={conf:.2f}) bbox={bbox}"
                    )

        # 3) 列表 (JSON 字符串).
        det_msg = String()
        det_msg.data = json.dumps(detections, ensure_ascii=False)
        self.det_pub.publish(det_msg)

        # 4) 是否识别到.
        flag = Bool()
        flag.data = bool(detections)
        self.flag_pub.publish(flag)

        # 5) 置信度最高的目标 -> PoseStamped (相机系).
        # 仅当本帧走的是 3D 路径时才发 pose; 2D fallback 没有 (x,y,z).
        if detections and has_3d:
            best = max(
                detections,
                key=lambda d: float(d.get("confidence") or 0.0),
            )
            # 兼容: 极少数情况下即便 has_3d=True, 个别 detection 可能也缺字段.
            if all(k in best for k in ("x", "y", "z")):
                pose = PoseStamped()
                pose.header = scene_msg.header  # 复用图像 header, frame_id 通常是相机.
                pose.pose.position.x = float(best["x"])
                pose.pose.position.y = float(best["y"])
                pose.pose.position.z = float(best["z"])
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

        # 6) 可视化:
        #    - 检测成功且 YOLOE 给了 annotated_image -> 发标注图
        #    - 否则 (无命中 / 推理失败 / 标注图缺失) -> 直接透传原始 scene_msg
        #      (绕过 cv_bridge BGR8 重编, 避免 30Hz raw 流让 RViz GL 卡退)
        annotated_for_pub = annotated if (detections and annotated is not None) else None
        self._publish_image(scene_msg, annotated_for_pub)

    # ------------------------- 回调 -------------------------
    def _scene_cb(self, msg: Image) -> None:
        self._last_scene = msg
        self._scene_frames += 1
        if self._scene_frames == 1:
            self.get_logger().info("first scene image received")
        # 无论推理是否就绪, 都尝试转发原始 scene 给 RViz, 保证连续帧.
        # 只有在 _run_inference 成功时才会被标注图覆盖.
        if not self._ready or self._last_depth is None:
            self._publish_scene_passthrough(msg)
        else:
            self._try_run()

    def _depth_cb(self, msg: Image) -> None:
        self._last_depth = msg
        self._depth_frames += 1
        if self._depth_frames == 1:
            self.get_logger().info("first depth image received")
        self._try_run()

    def _publish_scene_passthrough(self, msg: Image) -> None:
        """推理不可用时, 把原始 scene 图像透传给 RViz.

        实现统一走 ``_publish_image(scene_msg, annotated=None)``: raw 流
        直接 republish, compressed 用 cv2 重编 (AirSim 不发 jpeg).
        """
        self._publish_image(msg, annotated=None)

    # ------------------------- 图像发布 -------------------------
    def _publish_image(self, scene_msg: Image, annotated) -> None:
        """发布 raw + compressed 图像, 限频共用 ``_last_image_publish_time``.

        - 有命中且 annotated 有效 -> 发标注图 (cv_bridge 重编 BGR8).
        - 否则 -> raw 直接 republish 原始 ``scene_msg`` (不经 cv_bridge),
          compressed 仍由本节点重编 JPEG.

        无命中场景下绕过 cv_bridge 是为了避免 30Hz BGR8 raw 流让 RViz
        在 GL 端反复上传纹理触发崩溃 / 卡退.
        """
        now = time.time()
        if (now - self._last_image_publish_time) < self._publish_min_interval:
            return
        self._last_image_publish_time = now

        # annotated 有效性: ndim==3, 通道数 3 或 4, 宽高>0.
        use_annotated = (
            annotated is not None
            and getattr(annotated, "ndim", 0) == 3
            and annotated.shape[2] in (3, 4)
            and annotated.shape[0] > 0
            and annotated.shape[1] > 0
        )

        if not use_annotated:
            # raw: 直接透传原始 msg, 完全绕过 cv_bridge 重编.
            if self.image_pub is not None:
                self.image_pub.publish(scene_msg)
            # compressed: 仍需 encode JPEG, 失败仅 warn.
            self._publish_compressed_from_msg(scene_msg)
            # OpenCV 弹窗: 把原始 msg 解一次给本地窗口看.
            self._imshow_from_msg(scene_msg)
            return

        # 标注图路径: 走 cv_bridge 把 numpy 编回 Image, 再 encode JPEG.
        try:
            img = annotated[:, :, :3] if annotated.shape[2] == 4 else annotated
            if not img.flags['C_CONTIGUOUS']:
                import numpy as np
                img = np.ascontiguousarray(img)
            if self.image_pub is not None:
                ann_msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ann_msg.header = scene_msg.header
                self.image_pub.publish(ann_msg)
            if self.compressed_pub is not None:
                import cv2
                ok, buf = cv2.imencode(
                    ".jpg", img,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
                )
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header = scene_msg.header
                    cmsg.format = "jpeg"
                    cmsg.data = buf.tobytes()
                    self.compressed_pub.publish(cmsg)
            # OpenCV 弹窗: annotated 已经是 BGR ndarray, 直接 imshow.
            self._imshow(img)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"annotated image publish failed: {exc!r}", throttle_duration_sec=2.0,
            )

    def _imshow(self, bgr) -> None:
        """缓存最新帧给 _imshow_tick. 真正的 cv2.imshow 在独立定时器里调,
        避免 imshow / waitKey 阻塞 ROS 推理回调.
        """
        if not self._show_window or bgr is None:
            return
        # 直接保留引用即可, ndarray 不可变 (推理生成后没人再改).
        self._latest_imshow_frame = bgr

    def _imshow_tick(self) -> None:
        """5Hz 定时器: 取出最新帧, 调 cv2.imshow + waitKey(1)."""
        frame = self._latest_imshow_frame
        if not self._show_window or frame is None:
            return
        try:
            import cv2
            if not self._window_initialized:
                cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
                self._window_initialized = True
            cv2.imshow(self._window_name, frame)
            cv2.waitKey(1)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"cv2.imshow disabled ({exc!r})", throttle_duration_sec=10.0,
            )
            self._show_window = False

    def _imshow_from_msg(self, msg: Image) -> None:
        """从原始 sensor_msgs/Image 解一帧, 缓存给 _imshow_tick."""
        if not self._show_window:
            return
        try:
            import numpy as np
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if cv_img is None or cv_img.size == 0:
                return
            if cv_img.ndim == 2:
                import cv2 as _cv2
                cv_img = _cv2.cvtColor(cv_img, _cv2.COLOR_GRAY2BGR)
            elif cv_img.ndim == 3 and cv_img.shape[2] == 4:
                cv_img = cv_img[:, :, :3]
            if cv_img.dtype != np.uint8:
                cv_img = cv_img.astype(np.uint8)
            self._imshow(cv_img)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"imshow decode failed: {exc!r}", throttle_duration_sec=5.0,
            )

    def _publish_compressed_from_msg(self, msg: Image) -> None:
        """把原始 Image msg 编 JPEG 发到 compressed_pub. 失败仅 warn."""
        if self.compressed_pub is None:
            return
        try:
            import numpy as np
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if cv_img is None or cv_img.size == 0:
                return
            if cv_img.ndim == 2:
                import cv2 as _cv2
                cv_img = _cv2.cvtColor(cv_img, _cv2.COLOR_GRAY2BGR)
            elif cv_img.ndim == 3 and cv_img.shape[2] == 4:
                cv_img = cv_img[:, :, :3]
            if cv_img.dtype != np.uint8:
                cv_img = cv_img.astype(np.uint8)
            if not cv_img.flags['C_CONTIGUOUS']:
                cv_img = np.ascontiguousarray(cv_img)
            import cv2
            ok, buf = cv2.imencode(
                ".jpg", cv_img,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
            if ok:
                cmsg = CompressedImage()
                cmsg.header = msg.header
                cmsg.format = "jpeg"
                cmsg.data = buf.tobytes()
                self.compressed_pub.publish(cmsg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"compressed encode failed: {exc!r}", throttle_duration_sec=5.0,
            )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = YoloeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 关闭 OpenCV 弹窗 (本进程内创建的所有 cv2 窗口).
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
