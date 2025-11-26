"""消息处理器基类和各类消息处理器"""
import copy
import math
from typing import Dict, Any, Optional, List
from shapely.geometry import Polygon, Point
from google.protobuf.json_format import MessageToDict

from .config import TURN_SIGNAL_MAP, GEAR_MAP, DEFAULT_DISTANCE, EGO_VEHICLE
from .utils import convert_velocity_to_speed, calculate_polygon_points, find_nearest_time


class MessageProcessor:
    """消息处理器基类"""
    
    def __init__(self, map_info=None):
        """
        初始化消息处理器
        
        Args:
            map_info: 地图信息对象
        """
        self.map_info = map_info
    
    def process(self, message: Any, timestamp: float, context: Optional[Dict] = None) -> Dict:
        """
        处理消息的抽象方法
        
        Args:
            message: protobuf消息对象
            timestamp: 时间戳
            context: 上下文信息（如其他消息数据）
        
        Returns:
            处理后的字典数据
        """
        raise NotImplementedError


class PoseProcessor(MessageProcessor):
    """Pose消息处理器"""
    
    def process(self, message: Any, timestamp: float, context: Optional[Dict] = None) -> Dict:
        """处理pose消息"""
        msg_dict = MessageToDict(message)
        
        # 添加车辆尺寸信息
        msg_dict['size'] = {
            'length': EGO_VEHICLE['length'],
            'width': EGO_VEHICLE['width']
        }
        
        # 计算自车的多边形区域
        position = msg_dict['pose']['position']
        heading = msg_dict['pose']['heading']
        
        polygon_points = calculate_polygon_points(
            position['x'], position['y'],
            EGO_VEHICLE['length'], EGO_VEHICLE['width'],
            heading, EGO_VEHICLE['wheelbase']
        )
        
        msg_dict['area'] = Polygon(polygon_points)
        
        return msg_dict


class ChassisProcessor(MessageProcessor):
    """Chassis消息处理器"""
    
    def process(self, message: Any, timestamp: float, context: Optional[Dict] = None) -> Dict:
        """处理chassis消息"""
        msg_dict = MessageToDict(message)
        
        # 转换档位信息
        if 'gearLocation' in msg_dict:
            gear_str = msg_dict['gearLocation']
            msg_dict['gearLocation'] = GEAR_MAP.get(gear_str, 0)
        
        return msg_dict


class PlanningProcessor(MessageProcessor):
    """Planning消息处理器"""
    
    def process(self, message: Any, timestamp: float, context: Optional[Dict] = None) -> Dict:
        """处理planning消息"""
        msg_dict = MessageToDict(message)
        result = {}
        
        try:
            # 提取转向信号
            vehicle_signal = msg_dict.get('decision', {}).get('vehicleSignal', {})
            turn_signal = vehicle_signal.get('turnSignal', 'TURN_NONE')
            result['planning_turn_signal'] = TURN_SIGNAL_MAP.get(turn_signal, 0)
            
            # 提取超车决策
            is_overtaking = False
            decisions = msg_dict.get('decision', {}).get('objectDecision', {})
            decision_list = decisions.get('decision', [])
            
            for item in decision_list:
                object_decisions = item.get('objectDecision', [])
                if len(object_decisions) > 0:
                    item0 = object_decisions[0]
                    # 检查超车决策
                    if 'overtake' in item0 and item0.get('overtake', {}).get('distanceS', 0) != 0:
                        is_overtaking = True
                    # 检查微调决策
                    if 'nudge' in item0 and item0.get('nudge', {}).get('distanceL', 0) != 0:
                        is_overtaking = True
            
            result['is_overtaking'] = is_overtaking
            
        except Exception:
            # 返回默认值
            result['is_overtaking'] = False
            result['planning_turn_signal'] = 0
        
        return result


class ObstaclesProcessor(MessageProcessor):
    """Obstacles消息处理器"""
    
    def __init__(self, map_info=None):
        super().__init__(map_info)
        self.agent_ids = []
    
    def process(self, message: Any, timestamp: float, context: Optional[Dict] = None) -> Dict:
        """处理obstacles消息"""
        msg_dict = MessageToDict(message)
        result = {}
        
        # 获取障碍物列表（支持两种命名格式）
        perception_obstacles = msg_dict.get('perceptionObstacle', msg_dict.get('perception_obstacle', []))
        
        if len(perception_obstacles) == 0:
            result["minDistToEgo"] = DEFAULT_DISTANCE
            result["nearestGtObs"] = None
            result["NearestNPC"] = None
            result["obsList"] = []
            return result
        
        # 如果有pose数据，则计算距离
        if context is not None and 'pose' in context:
            pose_data = context['pose']
            
            # 处理每个障碍物
            for obs in perception_obstacles:
                self._process_single_obstacle(obs, pose_data)
            
            # 找到最近的障碍物
            min_dist = DEFAULT_DISTANCE
            nearest_id = None
            
            for obs in perception_obstacles:
                obs_id = obs.get("id", "unknown")
                if obs_id not in self.agent_ids:
                    self.agent_ids.append(obs_id)
                
                dist = obs.get("distToEgo", DEFAULT_DISTANCE)
                if dist < min_dist:
                    min_dist = dist
                    nearest_id = obs_id
            
            result["minDistToEgo"] = min_dist
            result["nearestGtObs"] = nearest_id
            result["NearestNPC"] = nearest_id
        else:
            # 初次提取时没有pose，不计算距离，但保留obsList
            result["minDistToEgo"] = DEFAULT_DISTANCE
            result["nearestGtObs"] = None
            result["NearestNPC"] = None
        
        result["obsList"] = perception_obstacles
        
        return result
    
    def _process_single_obstacle(self, obs: Dict, pose_data: Dict):
        """处理单个障碍物"""
        # 初始化字段
        if "currentLane" not in obs:
            obs["currentLane"] = {}
        obs["currentLane"]["currentLaneId"] = None
        obs["currentLane"]["type"] = None
        obs["currentLane"]["area"] = None
        
        # 计算速度
        velocity = obs.get('velocity', {})
        obs['speed'] = convert_velocity_to_speed(velocity)
        
        # 处理多边形点
        polygon_points = obs.get("polygonPoint", obs.get("polygon_point", []))
        
        # 如果没有polygonPoint，从位置和尺寸计算
        if not polygon_points or len(polygon_points) == 0:
            if all(k in obs for k in ['position', 'length', 'width', 'theta']):
                position = obs['position']
                polygon_points = self._calculate_obstacle_polygon(
                    position.get('x', 0), position.get('y', 0),
                    obs['length'], obs['width'], obs['theta']
                )
                obs["polygonPoint"] = polygon_points
            else:
                obs["polygonPoint"] = []
        else:
            if "polygonPoint" not in obs and "polygon_point" in obs:
                obs["polygonPoint"] = obs["polygon_point"]
        
        # 计算到自车的距离
        try:
            obs["distToEgo"] = self._calculate_dist_to_ego(obs, pose_data)
        except Exception:
            obs["distToEgo"] = DEFAULT_DISTANCE
        
        # 确定障碍物所在车道/路口
        if self.map_info and len(obs.get("polygonPoint", [])) > 0:
            try:
                points = [(p.get("x", 0), p.get("y", 0)) for p in obs["polygonPoint"]]
                obs_area = Polygon(points)
                obs["currentLane"]["area"] = obs_area
                
                result = self.map_info.find_which_area_the_ego_is_in(obs_area)
                
                if result and len(result) > 0:
                    if "lane_id" in result[0]:
                        obs["currentLane"]["currentLaneId"] = result[0]["lane_id"]
                        obs["currentLane"]["type"] = 'lane'
                        obs["currentLane"]["turn"] = result[0].get("turn", 0)
                    elif "junction_id" in result[0]:
                        obs["currentLane"]["currentLaneId"] = result[0]["junction_id"]
                        obs["currentLane"]["type"] = 'junction'
            except Exception:
                pass
    
    def _calculate_obstacle_polygon(self, center_x: float, center_y: float,
                                   length: float, width: float, theta: float) -> List[Dict]:
        """计算障碍物的多边形角点"""
        points = calculate_polygon_points(center_x, center_y, length, width, theta)
        return [{'x': p[0], 'y': p[1]} for p in points]
    
    def _calculate_dist_to_ego(self, obstacle: Dict, pose_data: Dict) -> float:
        """计算障碍物到自车的距离"""
        ego_area = pose_data.get('area')
        if ego_area is None:
            return DEFAULT_DISTANCE
        
        polygon_points = obstacle.get("polygonPoint", [])
        if len(polygon_points) == 0:
            return DEFAULT_DISTANCE
        
        try:
            points = [(p.get("x", 0), p.get("y", 0)) for p in polygon_points]
            obs_area = Polygon(points)
            return ego_area.distance(obs_area)
        except Exception:
            return DEFAULT_DISTANCE


class TrafficLightProcessor(MessageProcessor):
    """TrafficLight消息处理器"""
    
    def process(self, message: Any, timestamp: float, context: Optional[Dict] = None) -> Dict:
        """处理traffic_light消息"""
        msg_dict = MessageToDict(message)
        result = {}
        
        # 需要pose数据来计算距离
        if context is None or 'pose' not in context:
            result["trafficLightList"] = []
            result["nearest"] = None
            result["trafficLightStopLine"] = None
            return result
        
        pose_data = context['pose']
        traffic_lights = msg_dict.get("trafficLight", [])
        result["trafficLightList"] = traffic_lights
        
        # 找到最近的红绿灯
        if not self.map_info or len(traffic_lights) == 0:
            result["nearest"] = None
            result["trafficLightStopLine"] = None
            return result
        
        min_distance = None
        nearest_idx = None
        
        for idx, light in enumerate(traffic_lights):
            try:
                distance = self._calculate_distance_to_stopline(pose_data, light.get('id', ''))
                if distance is not None:
                    if min_distance is None or distance < min_distance:
                        min_distance = distance
                        nearest_idx = idx
            except Exception:
                continue
        
        result["nearest"] = nearest_idx
        result["trafficLightStopLine"] = min_distance
        
        return result
    
    def _calculate_distance_to_stopline(self, pose_data: Dict, light_id: str) -> Optional[float]:
        """计算到红绿灯停止线的距离"""
        if not self.map_info:
            return None
        
        ego_area = pose_data.get('area')
        if ego_area is None:
            return None
        
        traffic_signals = self.map_info.get_traffic_signals()
        
        for signal in traffic_signals:
            if signal["id"] == light_id:
                stop_line = signal.get("stop_line")
                if stop_line:
                    return ego_area.distance(stop_line)
        
        return None
