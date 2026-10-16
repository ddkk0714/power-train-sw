"""Jetson: L515 depth(ROS2) -> terrain.py -> packet.py -> UDP(laptop:5003), 10Hz.
실행: python3 terrain_node.py --laptop-ip <노트북IP> --meta meta.json
- 새 프레임이 없어도/에러여도 매 틱 패킷을 보낸다(패킷 자체가 생존 신호).
- 최신 깊이 프레임이 0.5초보다 오래되면 SENSOR_ERROR."""

import argparse, json, socket, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from terrain import TerrainDetector
from packet import ColorProjector, build_packet

ap = argparse.ArgumentParser()
ap.add_argument("--laptop-ip", required=True)
ap.add_argument("--port", type=int, default=5003)
ap.add_argument("--meta", required=True)
ap.add_argument(
    "--depth-topic", default="/l515/depth/image_raw"
)  # ros2 topic list | grep l515 로 확인
ap.add_argument("--hz", type=float, default=10.0)
args = ap.parse_args()

meta = json.load(open(args.meta))
print("meta keys:", list(meta.keys()))  # 컬러 intrinsics 키 이름 확인용

# meta.json에서 깊이 및 컬러 카메라 파라미터(fx, fy, cx, cy) 파싱
d_intr = meta["depth_intrinsics"]
c_intr = meta["color_intrinsics"]

depth_params = (d_intr["fx"], d_intr["fy"], d_intr["ppx"], d_intr["ppy"])
color_params = (c_intr["fx"], c_intr["fy"], c_intr["ppx"], c_intr["ppy"])


class TerrainNode(Node):
    def __init__(self):
        super().__init__("terrain_node")
        self.det = TerrainDetector()
        # meta.json에서 읽어온 값 적용
        self.proj = ColorProjector(depth_params, color_params)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (args.laptop_ip, args.port)
        self.depth, self.t_rx, self.seq = None, 0.0, 0
        self.create_subscription(
            Image, args.depth_topic, self.on_depth, qos_profile_sensor_data
        )
        self.create_timer(1.0 / args.hz, self.tick)

    def on_depth(self, m):
        a = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        self.depth, self.t_rx = a, time.time()  # 최신 것만 보관

    def tick(self):
        self.seq += 1
        t0 = time.time()
        try:
            if self.depth is None or t0 - self.t_rx > 0.5:
                status, mask, d, zones, near = "SENSOR_ERROR", None, None, {}, None
            else:
                d = self.depth.astype(np.float32) * 0.00025
                r = self.det.process(d)  # dict 형태/키 이름은 terrain.py 에 맞춰 조정
                status, zones, near = (
                    r["status"],
                    r.get("zones", {}),
                    r.get("nearest_obstacle_m"),
                )
                mask = self.det.obst_mask
            pkt, b = build_packet(
                self.proj,
                self.seq,
                status,
                mask,
                d,
                getattr(self.det, "fwd", None),
                getattr(self.det, "lat", None),
                zones,
                near,
                self.det.half_width,
                proc_ms=round((time.time() - t0) * 1000, 1),
            )
        except Exception as e:  # 어떤 에러여도 패킷은 계속 보낸다
            self.get_logger().error(str(e))
            _, b = build_packet(
                self.proj,
                self.seq,
                "SENSOR_ERROR",
                None,
                None,
                None,
                None,
                {},
                None,
                0.52,
            )
        self.sock.sendto(b, self.addr)


rclpy.init()
rclpy.spin(TerrainNode())
