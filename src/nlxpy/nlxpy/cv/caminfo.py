import subprocess
import re


def get_max_fps(device):
    """使用 v4l2-ctl 查询摄像头支持的最大帧率"""
    try:
        # 调用 v4l2-ctl 列出支持的格式和帧率
        result = subprocess.run(
            ["v4l2-ctl", "--list-formats-ext", "-d", f"/dev/{device}"],
            capture_output=True,
            text=True,
            check=True,
        )
        output = result.stdout

        # 使用正则表达式匹配最大帧率
        fps_matches = re.findall(r"Interval: .*\((\d+\.\d+) fps\)", output)
        if not fps_matches:
            print(f"未能找到 {device} 的帧率信息")
            return None

        # 获取最高帧率
        max_fps = max(float(fps) for fps in fps_matches)
        return max_fps

    except subprocess.CalledProcessError:
        print(f"无法访问设备 {device}")
        return None
    
def get_camera_info(device):
    """获取摄像头信息
    返回一个 list, 格式：
    {w}x{h}@{fps}.{fourcc}
    """
    
class CameraInfo:
    pass