import requests
import time

API_BASE_URL = "http://192.168.10.90:5000/api/extaxis"
ENABLE_URL = f"{API_BASE_URL}/enable"
MOVETO_URL = f"{API_BASE_URL}/moveto"
STATUS_URL = f"{API_BASE_URL}/status"

HEIGHT_OFFSET = 10.0  # 高度升降偏移量（单位：mm或根据API定义）

# 1. 使能
try:
    resp = requests.post(ENABLE_URL, json={"enable": 1}, timeout=1.0)
    if resp.status_code == 200:
        print("升降电机已使能")
    else:
        print(f"使能失败: {resp.status_code}")
except Exception as e:
    print(f"使能请求异常: {e}")
    exit(1)

# 2. 获取当前状态
try:
    resp = requests.get(STATUS_URL, timeout=1.0)
    state = resp.json()
    current_height = state[0]["pos"]
    num1 = state[1]["pos"]
    current_rotation = state[2]["pos"]
    num3 = state[3]["pos"]
    print(f"当前高度: {current_height}, 当前其他参数: num1={num1}, 旋转={current_rotation}, num3={num3}")
except Exception as e:
    print(f"获取状态异常: {e}")
    exit(1)

# 3. 升高动作
move_cmd = {
    "pos": [current_height, num1, current_rotation, num3-10],
    "vel": 100,
    "acc": 100
}
try:
    resp = requests.post(MOVETO_URL, json=move_cmd, timeout=1.0)
    if resp.status_code == 200:
        print(f"升高：高度增加 {HEIGHT_OFFSET} 到 {current_height + HEIGHT_OFFSET}")
    else:
        print(f"升高动作失败: {resp.status_code}")
except Exception as e:
    print(f"升高动作异常: {e}")
    exit(1)
