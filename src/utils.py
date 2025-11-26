"""工具函数模块"""
import math
import numpy as np
from typing import Optional, List, Tuple


def find_nearest_time(time_list: List[float], target: float) -> Optional[float]:
    """
    在时间列表中查找小于目标时间的最近时间戳
    
    Args:
        time_list: 时间戳列表
        target: 目标时间戳
    
    Returns:
        最近的时间戳，如果不存在则返回None
    """
    filtered_array = np.array(list(time_list))[np.array(list(time_list)) < target]
    return None if len(filtered_array) == 0 else np.max(filtered_array)


def convert_velocity_to_speed(velocity: dict) -> float:
    """
    将速度向量转换为标量速度
    
    Args:
        velocity: 包含x, y, z的速度字典
    
    Returns:
        速度标量值
    """
    x = velocity.get('x', 0)
    y = velocity.get('y', 0)
    z = velocity.get('z', 0)
    return math.sqrt(x*x + y*y + z*z)


def rotate_point(x: float, y: float, angle: float, center_x: float, center_y: float) -> Tuple[float, float]:
    """
    围绕中心点旋转坐标
    
    Args:
        x, y: 待旋转点坐标
        angle: 旋转角度（弧度）
        center_x, center_y: 旋转中心坐标
    
    Returns:
        旋转后的坐标 (x, y)
    """
    angle = -angle
    rotated_x = (x - center_x) * math.cos(angle) + (y - center_y) * math.sin(angle) + center_x
    rotated_y = (y - center_y) * math.cos(angle) - (x - center_x) * math.sin(angle) + center_y
    return rotated_x, rotated_y


def calculate_polygon_points(center_x: float, center_y: float, 
                            length: float, width: float, 
                            heading: float, wheelbase: float = 0) -> List[Tuple[float, float]]:
    """
    计算矩形物体的四个角点坐标
    
    Args:
        center_x, center_y: 中心点坐标
        length: 长度
        width: 宽度
        heading: 朝向角度（弧度）
        wheelbase: 轴距（如果是自车需要考虑）
    
    Returns:
        四个角点坐标列表
    """
    result = []
    
    # 计算四个角点（相对于中心点）
    if wheelbase > 0:
        # 自车需要考虑轴距
        half_length_front = (length - wheelbase) / 2 + wheelbase
        half_length_back = (length - wheelbase) / 2
    else:
        # 障碍物直接用长度的一半
        half_length_front = length / 2
        half_length_back = length / 2
    
    half_width = width / 2
    
    # 四个角点：前右、前左、后左、后右
    corners = [
        (center_x + half_length_front, center_y + half_width),
        (center_x + half_length_front, center_y - half_width),
        (center_x - half_length_back, center_y - half_width),
        (center_x - half_length_back, center_y + half_width)
    ]
    
    # 应用旋转
    for corner_x, corner_y in corners:
        rotated = rotate_point(corner_x, corner_y, heading, center_x, center_y)
        result.append(rotated)
    
    return result


def normalize_angle(angle: float) -> float:
    """
    将角度归一化到[0, 2π)范围
    
    Args:
        angle: 输入角度（弧度）
    
    Returns:
        归一化后的角度
    """
    if angle < 0:
        return angle + 2 * math.pi
    return angle


def get_map_name_from_path(record_path: str) -> str:
    """
    根据record文件路径推断地图名称
    
    Args:
        record_path: record文件路径
    
    Returns:
        地图名称
    """
    from .config import MAP_NAME_RULES
    
    for keyword, map_name in MAP_NAME_RULES.items():
        if keyword == 'default':
            continue
        if keyword in record_path:
            return map_name
    
    return MAP_NAME_RULES['default']
