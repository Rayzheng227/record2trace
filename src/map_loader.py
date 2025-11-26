"""地图信息加载模块"""
import json
import warnings
import numpy as np
from typing import Dict, List, Optional, Tuple
from shapely.geometry import LineString, Point, Polygon


class MapInfo:
    """地图信息类，负责加载和查询地图数据"""
    
    def __init__(self, map_name: str, maps_dir: str = 'maps/'):
        """
        初始化地图信息
        
        Args:
            map_name: 地图名称
            maps_dir: 地图文件目录
        """
        self.map_name = map_name
        self.file_path = f"{maps_dir}{map_name}.json"
        
        # 地图数据结构
        self.lane_config: Dict[str, float] = {}
        self.lane_waypoints: Dict[str, List[np.ndarray]] = {}
        self.lane_turn: Dict[str, int] = {}
        self.areas: Dict[str, Dict[str, Polygon]] = {
            "lane_areas": {},
            "junction_areas": {}
        }
        self.crosswalk_config: Dict[str, Polygon] = {}
        self.traffic_sign: List[Dict] = []
        self.traffic_signals: List[Dict] = []
        self.lane_to_road: Dict[str, str] = {}
        self.road_to_lane: Dict[str, List[str]] = {}
        self.left_lane: List[str] = []
        
        # 加载地图
        self._load_map()
    
    def _load_map(self):
        """从JSON文件加载地图数据"""
        with open(self.file_path, 'r') as f:
            map_config = json.load(f)
        
        # 处理车道信息
        self._load_lanes(map_config.get('lane', []))
        
        # 处理路口信息
        self._load_junctions(map_config.get('junction', []))
        
        # 处理人行横道
        self._load_crosswalks(map_config.get('crosswalk', []))
        
        # 处理交通标志
        self._load_traffic_signs(map_config.get('stopSign', []))
        
        # 处理交通信号灯
        self._load_traffic_signals(map_config.get('signal', []))
        
        # 处理道路信息
        self._load_roads(map_config.get('road', []))
    
    def _load_lanes(self, lanes: List[Dict]):
        """加载车道信息"""
        for lane in lanes:
            lane_id = lane['id']['id']
            self.lane_config[lane_id] = lane['length']
            self.lane_waypoints[lane_id] = []
            self.lane_turn[lane_id] = lane.get("turn", 0)
            
            # 获取中心线（支持两种命名格式）
            central_curve = lane.get('centralCurve') or lane.get('central_curve')
            if central_curve:
                for segment in central_curve.get('segment', []):
                    line_segment = segment.get('lineSegment') or segment.get('line_segment')
                    if line_segment:
                        for point in line_segment.get('point', []):
                            self.lane_waypoints[lane_id].append(
                                np.array([point['x'], point['y']])
                            )
            
            # 获取边界信息构建车道多边形
            left_points = self._extract_boundary_points(
                lane.get('leftBoundary') or lane.get('left_boundary')
            )
            right_points = self._extract_boundary_points(
                lane.get('rightBoundary') or lane.get('right_boundary')
            )
            
            if left_points and right_points:
                right_points.reverse()
                polygon_points = left_points + right_points
                self.areas["lane_areas"][lane_id] = Polygon(polygon_points)
            
            # 检查是否为最左车道
            if 'leftNeighborForwardLaneId' not in lane and 'left_neighbor_forward_lane_id' not in lane:
                self.left_lane.append(lane_id)
    
    def _extract_boundary_points(self, boundary: Optional[Dict]) -> List[Tuple[float, float]]:
        """提取边界点坐标"""
        points = []
        if boundary:
            for segment in boundary.get('curve', {}).get('segment', []):
                line_segment = segment.get('lineSegment') or segment.get('line_segment')
                if line_segment:
                    for point in line_segment.get('point', []):
                        points.append((point['x'], point['y']))
        return points
    
    def _load_junctions(self, junctions: List[Dict]):
        """加载路口信息"""
        for junction in junctions:
            junction_id = junction['id']['id']
            points = [(p['x'], p['y']) for p in junction['polygon']['point']]
            self.areas["junction_areas"][junction_id] = Polygon(points)
    
    def _load_crosswalks(self, crosswalks: List[Dict]):
        """加载人行横道信息"""
        for idx, crosswalk in enumerate(crosswalks):
            polygon_points = crosswalk['polygon']['point']
            if len(polygon_points) == 4:
                points = [(p['x'], p['y']) for p in polygon_points]
                self.crosswalk_config[f'crosswalk{idx+1}'] = Polygon(points)
    
    def _load_traffic_signs(self, signs: List[Dict]):
        """加载交通标志信息"""
        for sign in signs:
            sign_data = {
                "id": sign['id']['id'],
                "type": "stopsign" if "stopsign" in sign['id']['id'] or "stop_sign" in sign['id']['id'] else None
            }
            
            # 提取停止线
            if 'stopLine' in sign:
                stop_line_points = sign['stopLine'][0]['segment'][0]['lineSegment']['point']
                points = [(p['x'], p['y']) for p in stop_line_points]
                sign_data["stop_line"] = LineString(points)
                sign_data["stop_line_points"] = stop_line_points
            else:
                sign_data["stop_line_points"] = []
            
            self.traffic_sign.append(sign_data)
    
    def _load_traffic_signals(self, signals: List[Dict]):
        """加载交通信号灯信息"""
        for signal in signals:
            signal_data = {
                "id": signal['id']['id'],
                "sub_signal_type_list": []
            }
            
            # 提取子信号类型
            if 'subsignal' in signal:
                signal_data["sub_signal_type_list"] = [
                    sub['type'] for sub in signal['subsignal']
                ]
            
            # 提取停止线
            if 'stopLine' in signal:
                stop_line_points = signal['stopLine'][0]['segment'][0]['lineSegment']['point']
                points = [(p['x'], p['y']) for p in stop_line_points]
                signal_data["stop_line"] = LineString(points)
                signal_data["stop_line_points"] = stop_line_points
            
            self.traffic_signals.append(signal_data)
    
    def _load_roads(self, roads: List[Dict]):
        """加载道路信息"""
        for road in roads:
            road_id = road['id']['id']
            lane_ids = []
            
            # 获取车道ID列表（支持两种命名格式）
            lane_id_list = road['section'][0].get('laneId') or road['section'][0].get('lane_id', [])
            for lane_id_obj in lane_id_list:
                lane_id = lane_id_obj['id']
                lane_ids.append(lane_id)
                self.lane_to_road[lane_id] = road_id
            
            self.road_to_lane[road_id] = lane_ids
    
    # 查询方法
    def get_lane_config(self) -> Dict[str, float]:
        """获取车道配置"""
        return self.lane_config
    
    def get_crosswalk_config(self) -> Dict[str, Polygon]:
        """获取人行横道配置"""
        return self.crosswalk_config
    
    def get_traffic_sign(self) -> List[Dict]:
        """获取交通标志"""
        return self.traffic_sign
    
    def get_traffic_signals(self) -> List[Dict]:
        """获取交通信号灯"""
        return self.traffic_signals
    
    def find_which_area_the_ego_is_in(self, ego: Polygon) -> Optional[List[Dict]]:
        """
        查找自车/障碍物所在的区域
        
        Args:
            ego: 车辆的多边形区域
        
        Returns:
            区域信息列表，如果不在任何区域则返回None
        """
        # 先检查是否在路口
        result = self._check_junction_area(ego)
        if result:
            return result
        
        # 再检查是否在车道
        result = self._check_lane_area(ego)
        if result:
            return result
        
        return None
    
    def _check_lane_area(self, point: Polygon) -> List[Dict]:
        """检查是否在车道区域"""
        result = []
        for lane_id, area in self.areas["lane_areas"].items():
            if point.distance(area) == 0:
                result.append({
                    "lane_id": lane_id,
                    "turn": self.lane_turn.get(lane_id, 0),
                    "laneNumber": self._get_lane_number_of_road(lane_id)
                })
        return result
    
    def _check_junction_area(self, point: Polygon) -> List[Dict]:
        """检查是否在路口区域"""
        result = []
        for junction_id, area in self.areas["junction_areas"].items():
            if point.distance(area) == 0:
                result.append({
                    "junction_id": junction_id
                })
        return result
    
    def _get_lane_number_of_road(self, lane_id: str) -> int:
        """获取道路的车道数"""
        try:
            road_id = self.lane_to_road[lane_id]
            num = len(self.road_to_lane[road_id])
            if lane_id in self.left_lane:
                return num * 100 + 1
            return num
        except KeyError:
            return 0
    
    def check_whether_two_lanes_are_in_the_same_road(self, lane_1: str, lane_2: str) -> bool:
        """检查两条车道是否在同一道路上"""
        try:
            return self.lane_to_road[lane_1] == self.lane_to_road[lane_2]
        except KeyError:
            return False
