"""
check_connection.py — Cosys-AirSim 连接自检脚本

用途:
    在运行其它控制脚本前，先用本脚本验证：
      1. AirSim RPC 服务是否在 127.0.0.1:41451 可达
      2. 是否能成功 enableApiControl / armDisarm
      3. settings.json 是否为 Multirotor 模式

运行:
    python3 check_connection.py
"""

import sys
import time
import socket

HOST = "127.0.0.1"
PORT = 41451  # Cosys-AirSim 默认 RPC 端口


def tcp_probe(host: str, port: int, timeout: float = 2.0) -> bool:
    """先做一次 TCP 探测，避免 msgpack-rpc 阻塞太久。"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError as e:
        print(f"[FAIL] 无法连接到 {host}:{port} — {e}")
        return False


def main() -> int:
    print(f"[1/4] TCP 探测 {HOST}:{PORT} ...")
    if not tcp_probe(HOST, PORT):
        print("\n请先在 UE 中启动包含 Cosys-AirSim 插件的关卡 (PIE 或 Standalone)。")
        print("如果端口被改过，请检查 ~/Documents/AirSim/settings.json 中的 ApiServerPort。")
        return 1
    print("    OK — 端口可达\n")

    print("[2/4] 导入 cosysairsim 包 ...")
    try:
        import cosysairsim as airsim  # type: ignore
    except ImportError:
        print("    FAIL — 未安装 cosysairsim")
        print("    安装: python3 -m pip install cosysairsim msgpack-rpc-python")
        return 2
    print("    OK\n")

    print("[3/4] 建立 MultirotorClient 并 ping ...")
    client = airsim.MultirotorClient(ip=HOST, port=PORT)
    try:
        client.confirmConnection()
    except Exception as e:
        print(f"    FAIL — confirmConnection 异常: {e}")
        print("    可能原因: settings.json 中 SimMode 不是 'Multirotor'")
        return 3
    print("    OK — 已连接\n")

    print("[4/4] 申请 API 控制权 + 解锁 ...")
    try:
        client.enableApiControl(True)
        api_ok = client.isApiControlEnabled()
        print(f"    enableApiControl -> isApiControlEnabled = {api_ok}")
        client.armDisarm(True)
        print("    armDisarm(True) 已发送")
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"    当前位置 (NED): x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")
        time.sleep(0.5)
        client.armDisarm(False)
        client.enableApiControl(False)
    except Exception as e:
        print(f"    FAIL — {e}")
        return 4

    print("\n全部通过。可以运行 hello_drone.py / keyboard_control.py 了。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
