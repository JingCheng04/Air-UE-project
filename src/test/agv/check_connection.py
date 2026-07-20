"""
check_connection.py — Cosys-AirSim AGV (SkidVehicle / Husky) 连接自检

确认:
  1. 41451 RPC 端口可达
  2. cosysairsim 包能导入
  3. CarClient 能 ping (SkidVehicle 复用 CarClient 接口)
  4. 能拿到 CarState

运行:
    python3 src/test/agv/check_connection.py
"""

import socket
import sys

HOST = "127.0.0.1"
PORT = 41451  # 单实例默认端口; 如果走"双实例双端口"协同方案, 车端用 41452


def tcp_probe(host: str, port: int, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError as e:
        print(f"[FAIL] 无法连接到 {host}:{port} — {e}")
        return False


def main() -> int:
    print(f"[1/4] TCP 探测 {HOST}:{PORT} ...")
    if not tcp_probe(HOST, PORT):
        print("\n请确认 UE 已 Play, 且 settings.json 的 SimMode 是 'SkidVehicle'.")
        return 1
    print("    OK\n")

    print("[2/4] 导入 cosysairsim ...")
    try:
        import cosysairsim as airsim  # type: ignore
    except ImportError:
        print("    FAIL — 未安装 cosysairsim")
        return 2
    print("    OK\n")

    print("[3/4] 建立 CarClient + ping ...")
    client = airsim.CarClient(ip=HOST, port=PORT)
    try:
        client.confirmConnection()
    except Exception as e:
        print(f"    FAIL — {e}")
        print("    可能原因: SimMode 不是 'SkidVehicle', VehicleType 不是 'CPHusky', 或插件版本不匹配")
        return 3
    print("    OK\n")

    print("[4/4] 读取 CarState ...")
    try:
        client.enableApiControl(True)
        state = client.getCarState()
        p = state.kinematics_estimated.position
        print(f"    speed={state.speed:.2f} m/s, gear={state.gear}")
        print(f"    position (NED): x={p.x_val:.2f}, y={p.y_val:.2f}, z={p.z_val:.2f}")
        client.enableApiControl(False)
    except Exception as e:
        print(f"    FAIL — {e}")
        return 4

    print("\n全部通过, 可以运行 hello_agv.py / keyboard_agv.py / drive_square.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
