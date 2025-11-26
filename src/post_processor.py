"""后处理模块 - 计算派生字段"""
import math
from typing import Dict, List, Optional
from shapely.geometry import Polygon, Point

from .config import DEFAULT_DISTANCE


class PostProcessor:
    """轨迹后处理器，计算派生字段"""
    
    def __init__(self, map_info):
        """
        初始化后处理器
        
        Args:
            map_info: 地图信息对象
        """
        self.map_info = map_info
    
    def process_trace(self, trace: Dict) -> Dict:
        """
        处理完整轨迹，添加派生字段
        
        Args:
            trace: 轨迹字典 {timestamp: trace_point}
        
        Returns:
            处理后的轨迹
        """
        timestamps = sorted(trace.keys())
        
        # 第一遍：计算单时刻的字段
        for ts in timestamps:
            self._process_single_timestamp(trace[ts])
        
        # 第二遍：计算需要历史信息的字段
        for i, ts in enumerate(timestamps):
            self._process_temporal_fields(trace, timestamps, i)
        
        return trace
    
    def _process_single_timestamp(self, trace_point: Dict):
        """处理单个时间戳的派生字段"""
        ego = trace_point["ego"]
        truth = trace_point["truth"]
        
        # 计算自车当前车道
        ego["currentLane"] = self._get_current_lane(ego)
        
        # 计算前方距离信息
        self._calculate_ahead_distances(ego)
        
        # 计算前方障碍物
        self._calculate_ahead_obstacles(ego, truth)
        
        # 分类障碍物
        truth["npcClassification"] = self._classify_obstacles(ego, truth)
        
        # 判断交通拥堵
        ego["isTrafficJam"] = self._check_traffic_jam(ego, truth)
    
    def _process_temporal_fields(self, trace: Dict, timestamps: List[float], current_idx: int):
        """处理需要历史信息的字段"""
        current_ts = timestamps[current_idx]
        trace_point = trace[current_ts]
        ego = trace_point["ego"]
        
        # 初始化
        ego["isLaneChanging"] = False
        ego["isTurningAround"] = False
        ego["PriorityNPCAhead"] = False
        ego["PriorityPedsAhead"] = False
        ego["reach_destinaton"] = False  # 暂时设为False，需要destination坐标
        
        # 检查变道
        if current_idx >= 10:
            self._check_lane_changing(trace, timestamps, current_idx)
        
        # 检查掉头
        if current_idx >= 20:
            self._check_turning_around(trace, timestamps, current_idx)
        
        # 检查优先级车辆和行人
        self._find_priority_npcs_and_peds(trace, timestamps, current_idx)
    
    def _get_current_lane(self, ego: Dict) -> Dict:
        """获取自车当前车道信息"""
        result = {
            "currentLaneId": None,
            "type": None,
            "turn": 0,
            "number": 0
        }
        
        ego_area = ego.get('area')
        if ego_area is None:
            print("警告: ego area 为 None")
            return result
        
        lane_result = self.map_info.find_which_area_the_ego_is_in(ego_area)
        
        # 调试信息
        if lane_result is None or len(lane_result) == 0:
            # 检查是否有地图数据
            lane_count = len(self.map_info.areas["lane_areas"])
            junction_count = len(self.map_info.areas["junction_areas"])
            if lane_count == 0 and junction_count == 0:
                print("错误: 地图数据为空！")
                return result
            
            # 不打印每个时间戳的警告，只打印一次
            if not hasattr(self, '_lane_warning_printed'):
                print(f"警告: 自车不在任何车道/路口内 (地图有 {lane_count} 条车道, {junction_count} 个路口)")
                print("尝试查找最近的车道...")
                self._lane_warning_printed = True
            
            # 尝试查找最近的车道
            nearest = self._find_nearest_lane(ego_area)
            if nearest:
                return nearest
            
            return result
        
        if lane_result and len(lane_result) > 0:
            if "lane_id" in lane_result[0]:
                result["currentLaneId"] = lane_result[0]["lane_id"]
                result["type"] = 'lane'
                
                # 计算转向和车道数
                forward = left = right = u_turn = 0
                number = 0
                for item in lane_result:
                    number += item.get("laneNumber", 0)
                    turn = item.get("turn", 0)
                    if turn == 1:  # NO_TURN
                        forward = 1
                    elif turn == 2:  # LEFT_TURN
                        left = 1
                    elif turn == 3:  # RIGHT_TURN
                        right = 1
                    elif turn == 4:  # U_TURN
                        u_turn = 1
                
                # 编码转向类型
                if forward:
                    if left and right:
                        result["turn"] = 6
                    elif left:
                        result["turn"] = 4
                    elif right:
                        result["turn"] = 5
                    else:
                        result["turn"] = 0
                else:
                    if left and right:
                        result["turn"] = 7
                    elif left:
                        result["turn"] = 1
                    elif right:
                        result["turn"] = 2
                    elif u_turn:
                        result["turn"] = 3
                
                result["number"] = number
                
            elif "junction_id" in lane_result[0]:
                result["currentLaneId"] = lane_result[0]["junction_id"]
                result["type"] = 'junction'
        
        return result
    
    def _calculate_ahead_distances(self, ego: Dict):
        """计算前方各种距离"""
        ego_area = ego.get('area')
        if ego_area is None:
            ego["crosswalkAhead"] = DEFAULT_DISTANCE
            ego["junctionAhead"] = DEFAULT_DISTANCE
            ego["stopSignAhead"] = DEFAULT_DISTANCE
            ego["stoplineAhead"] = DEFAULT_DISTANCE
            ego["junction_ahead"] = None
            return
        
        # 计算前方区域
        pose = ego.get('pose', {})
        position = pose.get('position', {})
        heading = pose.get('heading', 0)
        size = ego.get('size', {})
        width = size.get('width', 2.06)
        
        head_point = self._get_head_middle_point(
            position.get('x', 0), position.get('y', 0),
            heading, size.get('length', 4.7), width
        )
        
        ahead_area = self._calculate_area_of_ahead(head_point, heading, width)
        
        # 人行横道
        ego["crosswalkAhead"] = self._distance_to_crosswalk(ego_area, ahead_area)
        
        # 路口
        ego["junctionAhead"], ego["junction_ahead"] = self._distance_to_junction(ego_area, ahead_area)
        
        # 停止标志
        ego["stopSignAhead"] = self._distance_to_stop_sign(ego_area, ahead_area)
        
        # 停止线（红绿灯）
        ego["stoplineAhead"] = self._distance_to_stopline(ego_area, ahead_area, ego["stopSignAhead"])
    
    def _calculate_ahead_obstacles(self, ego: Dict, truth: Dict):
        """计算前方障碍物"""
        ego_area = ego.get('area')
        obs_list = truth.get('obsList', [])
        current_lane = ego.get('currentLane', {})
        
        if ego_area is None:
            truth["NPCAhead"] = None
            truth["PedAhead"] = None
            truth["NPCOpposite"] = None
            return
        
        # 计算前方区域
        pose = ego.get('pose', {})
        position = pose.get('position', {})
        heading = pose.get('heading', 0)
        size = ego.get('size', {})
        width = size.get('width', 2.06)
        
        head_point = self._get_head_middle_point(
            position.get('x', 0), position.get('y', 0),
            heading, size.get('length', 4.7), width
        )
        
        ahead_area = self._calculate_area_of_ahead(head_point, heading, width)
        ahead_area_opposite = self._calculate_area_of_ahead(head_point, heading, width, dist=30)
        
        # 前方车辆
        truth["NPCAhead"] = self._find_npc_ahead(obs_list, ahead_area, ego_area, current_lane)
        
        # 前方行人
        truth["PedAhead"] = self._find_ped_ahead(obs_list, ahead_area, ego_area)
        
        # 对向车辆
        truth["NPCOpposite"] = self._find_npc_opposite(obs_list, ahead_area_opposite, ego_area, heading)
    
    def _classify_obstacles(self, ego: Dict, truth: Dict) -> Dict:
        """分类障碍物"""
        result = {
            "NextToEgo": [],
            "OntheDifferentRoad": [],
            "IntheJunction": [],
            "EgoInjunction_Lane": [],
            "EgoInjunction_junction": []
        }
        
        ego_lane = ego.get('currentLane', {})
        obs_list = truth.get('obsList', [])
        
        for obs in obs_list:
            obs_lane = obs.get('currentLane', {})
            temp = {"name": obs.get("id", "unknown")}
            
            if ego_lane.get('type') == 'lane':
                if obs_lane.get('type') == 'lane':
                    temp["laneId"] = obs_lane.get("currentLaneId")
                    temp["turn"] = obs_lane.get("turn", 0)
                    temp["type"] = "lane"
                    
                    if self.map_info.check_whether_two_lanes_are_in_the_same_road(
                        ego_lane.get("currentLaneId"), obs_lane.get("currentLaneId")
                    ):
                        result["NextToEgo"].append(temp)
                    else:
                        result["OntheDifferentRoad"].append(temp)
                        
                elif obs_lane.get('type') == 'junction':
                    temp["junctionId"] = obs_lane.get("currentLaneId")
                    temp["type"] = "junction"
                    result["IntheJunction"].append(temp)
                    
            elif ego_lane.get('type') == 'junction':
                if obs_lane.get('type') == 'lane':
                    temp["laneId"] = obs_lane.get("currentLaneId")
                    temp["turn"] = obs_lane.get("turn", 0)
                    temp["type"] = "lane"
                    result["EgoInjunction_Lane"].append(temp)
                    
                elif obs_lane.get('type') == 'junction':
                    temp["junctionId"] = obs_lane.get("currentLaneId")
                    temp["type"] = "junction"
                    result["EgoInjunction_junction"].append(temp)
        
        return result
    
    def _check_traffic_jam(self, ego: Dict, truth: Dict) -> bool:
        """检查是否交通拥堵"""
        junction_ahead = ego.get('junction_ahead')
        if junction_ahead is None:
            return False
        
        obs_list = truth.get('obsList', [])
        count = 0
        
        for obs in obs_list:
            if obs.get('speed', 0) < 1 and obs.get('type') == 'VEHICLE':
                obs_lane = obs.get('currentLane', {})
                if obs_lane.get('type') == 'junction':
                    if obs_lane.get('currentLaneId') == junction_ahead:
                        count += 1
        
        return count >= 6
    
    def _check_lane_changing(self, trace: Dict, timestamps: List[float], current_idx: int):
        """检查是否正在变道"""
        current_ts = timestamps[current_idx]
        prev_ts = timestamps[current_idx - 10]
        
        current_ego = trace[current_ts]["ego"]
        prev_ego = trace[prev_ts]["ego"]
        
        if prev_ego.get("isLaneChanging", False):
            return
        
        prev_lane = prev_ego.get("currentLane", {})
        prev_lane_id = prev_lane.get("currentLaneId")
        prev_turn = prev_ego.get("planning_of_turn", 0)
        
        if prev_turn == 0:
            return
        
        # 检查后续10个时间戳
        for i in range(current_idx - 10 + 1, min(current_idx + 1, len(timestamps))):
            ts = timestamps[i]
            ego = trace[ts]["ego"]
            current_lane_id = ego.get("currentLane", {}).get("currentLaneId")
            
            if current_lane_id != prev_lane_id and current_lane_id is not None:
                in_same_road = self.map_info.check_whether_two_lanes_are_in_the_same_road(
                    prev_lane_id, current_lane_id
                )
                if in_same_road:
                    prev_ego["isLaneChanging"] = True
                    break
    
    def _check_turning_around(self, trace: Dict, timestamps: List[float], current_idx: int):
        """检查是否正在掉头"""
        current_ts = timestamps[current_idx]
        prev_ts = timestamps[current_idx - 20]
        
        current_ego = trace[current_ts]["ego"]
        prev_ego = trace[prev_ts]["ego"]
        
        prev_heading = prev_ego.get("pose", {}).get("heading", 0)
        prev_heading = self._normalize_angle(prev_heading)
        prev_turn = prev_ego.get("planning_of_turn", 0)
        
        if prev_turn == 0:
            return
        
        # 检查后续20个时间戳
        for i in range(current_idx - 20 + 1, min(current_idx + 1, len(timestamps))):
            ts = timestamps[i]
            ego = trace[ts]["ego"]
            current_heading = ego.get("pose", {}).get("heading", 0)
            current_heading = self._normalize_angle(current_heading)
            
            if abs(current_heading - prev_heading) > 3 * math.pi / 4:
                prev_ego["isTurningAround"] = True
                break
    
    def _find_priority_npcs_and_peds(self, trace: Dict, timestamps: List[float], current_idx: int):
        """查找优先级车辆和行人"""
        current_ts = timestamps[current_idx]
        ego = trace[current_ts]["ego"]
        truth = trace[current_ts]["truth"]
        
        ego_area = ego.get('area')
        if ego_area is None:
            return
        
        pose = ego.get('pose', {})
        position = pose.get('position', {})
        heading = pose.get('heading', 0)
        size = ego.get('size', {})
        
        back_point = self._get_back_middle_point(
            position.get('x', 0), position.get('y', 0),
            heading, size.get('length', 4.7), size.get('width', 2.06)
        )
        
        ahead_area = self._calculate_area_of_ahead2(back_point, heading)
        left_area = self._calculate_area_of_ahead_left(back_point, heading)
        right_area = self._calculate_area_of_ahead_right(back_point, heading)
        back_left = self._calculate_area_of_back_left(back_point, heading, size.get('width', 2.06))
        back_right = self._calculate_area_of_back_right(back_point, heading, size.get('width', 2.06))
        
        obs_list = truth.get('obsList', [])
        planning_turn = ego.get('planning_of_turn', 0)
        is_lane_changing = ego.get('isLaneChanging', False)
        
        for obs in obs_list:
            obs_area = obs.get('currentLane', {}).get('area')
            if obs_area is None:
                continue
            
            obs_type = obs.get('type')
            dist = obs.get('distToEgo', 200)
            
            # 检查前方车辆
            if truth.get('NPCAhead') == obs.get('id') and dist < 3:
                ego['PriorityNPCAhead'] = True
            
            if obs_type == 'VEHICLE':
                obs_heading = self._normalize_angle(obs.get('theta', 0))
                ego_heading = self._normalize_angle(heading)
                heading_diff = abs(obs_heading - ego_heading)
                
                # 转向时的优先级判定
                if planning_turn != 0 and not is_lane_changing:
                    if math.pi/4 < heading_diff < 3*math.pi/4:
                        if dist < 30 and ahead_area.distance(obs_area) == 0:
                            ego['PriorityNPCAhead'] = True
                
                # 变道时的优先级判定
                if is_lane_changing and heading_diff < math.pi/4:
                    ego_speed = self._get_ego_speed(ego)
                    obs_speed = obs.get('speed', 0)
                    
                    if planning_turn == 1 and back_left.distance(obs_area) == 0:
                        if dist < 10 and obs_speed > ego_speed:
                            ego['PriorityNPCAhead'] = True
                    
                    if planning_turn == 2 and back_right.distance(obs_area) == 0:
                        if dist < 10 and obs_speed > ego_speed:
                            ego['PriorityNPCAhead'] = True
            
            elif obs_type == 'PEDESTRIAN':
                if planning_turn == 0 and dist < 3 and ahead_area.distance(obs_area) == 0:
                    ego['PriorityPedsAhead'] = True
                elif planning_turn == 1 and dist < 10 and left_area.distance(obs_area) == 0:
                    ego['PriorityPedsAhead'] = True
                elif planning_turn == 2 and dist < 10 and right_area.distance(obs_area) == 0:
                    ego['PriorityPedsAhead'] = True
    
    # 辅助几何计算方法
    def _get_head_middle_point(self, x: float, y: float, heading: float, length: float, width: float) -> tuple:
        """获取车头中点"""
        wheelbase = 2.697298
        dx = (length - wheelbase) / 2 + wheelbase
        
        rotated_x = dx * math.cos(heading)
        rotated_y = dx * math.sin(heading)
        
        return (x + rotated_x, y + rotated_y)
    
    def _get_back_middle_point(self, x: float, y: float, heading: float, length: float, width: float) -> tuple:
        """获取车尾中点"""
        wheelbase = 2.697298
        dx = -(length - wheelbase) / 2
        
        rotated_x = dx * math.cos(heading)
        rotated_y = dx * math.sin(heading)
        
        return (x + rotated_x, y + rotated_y)
    
    def _calculate_area_of_ahead(self, point: tuple, heading: float, width: float, dist: float = 200) -> Polygon:
        """计算前方区域"""
        x, y = point
        points = []
        
        # 四个角点
        corners = [
            (x + dist, y + dist),
            (x + dist, y - dist),
            (x, y - width/2),
            (x, y + width/2)
        ]
        
        for cx, cy in corners:
            rx = (cx - x) * math.cos(-heading) + (cy - y) * math.sin(-heading) + x
            ry = (cy - y) * math.cos(-heading) - (cx - x) * math.sin(-heading) + y
            points.append((rx, ry))
        
        return Polygon(points)
    
    def _calculate_area_of_ahead2(self, point: tuple, heading: float) -> Polygon:
        """计算前方大区域"""
        return self._calculate_area_of_ahead(point, heading, 200, 200)
    
    def _calculate_area_of_ahead_left(self, point: tuple, heading: float) -> Polygon:
        """计算左前方区域"""
        x, y = point
        points = []
        corners = [(x + 30, y), (x + 30, y - 30), (x, y - 30), (x, y)]
        
        for cx, cy in corners:
            rx = (cx - x) * math.cos(-heading) + (cy - y) * math.sin(-heading) + x
            ry = (cy - y) * math.cos(-heading) - (cx - x) * math.sin(-heading) + y
            points.append((rx, ry))
        
        return Polygon(points)
    
    def _calculate_area_of_ahead_right(self, point: tuple, heading: float) -> Polygon:
        """计算右前方区域"""
        x, y = point
        points = []
        corners = [(x + 30, y + 30), (x + 30, y), (x, y), (x, y + 30)]
        
        for cx, cy in corners:
            rx = (cx - x) * math.cos(-heading) + (cy - y) * math.sin(-heading) + x
            ry = (cy - y) * math.cos(-heading) - (cx - x) * math.sin(-heading) + y
            points.append((rx, ry))
        
        return Polygon(points)
    
    def _calculate_area_of_back_left(self, point: tuple, heading: float, width: float) -> Polygon:
        """计算左后方区域"""
        x, y = point
        points = []
        offset = width/2 + 0.3
        corners = [(x, y - offset), (x, y - offset - 3), (x - 30, y - offset - 3), (x - 30, y - offset)]
        
        for cx, cy in corners:
            rx = (cx - x) * math.cos(-heading) + (cy - y) * math.sin(-heading) + x
            ry = (cy - y) * math.cos(-heading) - (cx - x) * math.sin(-heading) + y
            points.append((rx, ry))
        
        return Polygon(points)
    
    def _calculate_area_of_back_right(self, point: tuple, heading: float, width: float) -> Polygon:
        """计算右后方区域"""
        x, y = point
        points = []
        offset = width/2 + 0.3
        corners = [(x, y + offset + 3), (x, y + offset), (x - 30, y + offset), (x - 30, y + offset + 3)]
        
        for cx, cy in corners:
            rx = (cx - x) * math.cos(-heading) + (cy - y) * math.sin(-heading) + x
            ry = (cy - y) * math.cos(-heading) - (cx - x) * math.sin(-heading) + y
            points.append((rx, ry))
        
        return Polygon(points)
    
    def _distance_to_crosswalk(self, ego_area: Polygon, ahead_area: Polygon) -> float:
        """计算到人行横道的距离"""
        crosswalks = self.map_info.get_crosswalk_config()
        min_dist = DEFAULT_DISTANCE
        
        for cw_area in crosswalks.values():
            if ahead_area.distance(cw_area) == 0:
                dist = ego_area.distance(cw_area)
                min_dist = min(min_dist, dist)
        
        return min_dist
    
    def _distance_to_junction(self, ego_area: Polygon, ahead_area: Polygon) -> tuple:
        """计算到路口的距离"""
        junctions = self.map_info.areas["junction_areas"]
        result = {}
        
        for junc_id, junc_area in junctions.items():
            if ahead_area.distance(junc_area) == 0:
                result[junc_id] = ego_area.distance(junc_area)
        
        if result:
            nearest_id = min(result.keys(), key=lambda k: result[k])
            return result[nearest_id], nearest_id
        
        return DEFAULT_DISTANCE, None
    
    def _distance_to_stop_sign(self, ego_area: Polygon, ahead_area: Polygon) -> float:
        """计算到停止标志的距离"""
        signs = self.map_info.get_traffic_sign()
        min_dist = DEFAULT_DISTANCE
        
        for sign in signs:
            stop_line = sign.get("stop_line")
            if stop_line and ahead_area.distance(stop_line) == 0:
                dist = ego_area.distance(stop_line)
                min_dist = min(min_dist, dist)
        
        return min_dist
    
    def _distance_to_stopline(self, ego_area: Polygon, ahead_area: Polygon, stop_sign_dist: float) -> float:
        """计算到停止线的距离"""
        signals = self.map_info.get_traffic_signals()
        min_dist = DEFAULT_DISTANCE
        
        for signal in signals:
            stop_line = signal.get("stop_line")
            if stop_line and ahead_area.distance(stop_line) == 0:
                dist = ego_area.distance(stop_line)
                min_dist = min(min_dist, dist)
        
        return min(min_dist, stop_sign_dist)
    
    def _find_npc_ahead(self, obs_list: List, ahead_area: Polygon, ego_area: Polygon, current_lane: Dict) -> Optional[str]:
        """查找前方车辆"""
        candidates = {}
        
        for obs in obs_list:
            if obs.get('type') != 'VEHICLE':
                continue
            
            obs_area = obs.get('currentLane', {}).get('area')
            if obs_area is None or ahead_area.distance(obs_area) != 0:
                continue
            
            obs_lane = obs.get('currentLane', {})
            
            if current_lane.get('type') == 'lane':
                if obs_lane.get('type') == 'lane' and \
                   obs_lane.get('currentLaneId') == current_lane.get('currentLaneId'):
                    candidates[obs['id']] = ego_area.distance(obs_area)
            elif current_lane.get('type') == 'junction':
                candidates[obs['id']] = ego_area.distance(obs_area)
        
        return min(candidates.keys(), key=lambda k: candidates[k]) if candidates else None
    
    def _find_ped_ahead(self, obs_list: List, ahead_area: Polygon, ego_area: Polygon) -> Optional[str]:
        """查找前方行人"""
        candidates = {}
        
        for obs in obs_list:
            if obs.get('type') != 'PEDESTRIAN':
                continue
            
            obs_area = obs.get('currentLane', {}).get('area')
            if obs_area is None or ahead_area.distance(obs_area) != 0:
                continue
            
            candidates[obs['id']] = ego_area.distance(obs_area)
        
        return min(candidates.keys(), key=lambda k: candidates[k]) if candidates else None
    
    def _find_npc_opposite(self, obs_list: List, ahead_area: Polygon, ego_area: Polygon, ego_heading: float) -> Optional[str]:
        """查找对向车辆"""
        candidates = {}
        ego_heading = self._normalize_angle(ego_heading)
        
        for obs in obs_list:
            if obs.get('type') != 'VEHICLE':
                continue
            
            obs_area = obs.get('currentLane', {}).get('area')
            if obs_area is None or ahead_area.distance(obs_area) != 0:
                continue
            
            obs_heading = self._normalize_angle(obs.get('theta', 0))
            heading_diff = abs(obs_heading - ego_heading)
            
            if 3*math.pi/4 < heading_diff < 5*math.pi/4:
                candidates[obs['id']] = ego_area.distance(obs_area)
        
        return min(candidates.keys(), key=lambda k: candidates[k]) if candidates else None
    
    def _normalize_angle(self, angle: float) -> float:
        """归一化角度到[0, 2π)"""
        if angle < 0:
            return angle + 2 * math.pi
        return angle
    
    def _find_nearest_lane(self, ego_area: Polygon) -> Optional[Dict]:
        """查找最近的车道"""
        min_dist = float('inf')
        nearest_lane_id = None
        
        # 先检查车道
        for lane_id, area in self.map_info.areas["lane_areas"].items():
            dist = ego_area.distance(area)
            if dist < min_dist:
                min_dist = dist
                nearest_lane_id = lane_id
        
        # 再检查路口
        for junction_id, area in self.map_info.areas["junction_areas"].items():
            dist = ego_area.distance(area)
            if dist < min_dist:
                min_dist = dist
                nearest_lane_id = junction_id
        
        # 如果距离小于5米，认为基本在该车道上
        if nearest_lane_id and min_dist < 5.0:
            if nearest_lane_id in self.map_info.areas["lane_areas"]:
                return {
                    "currentLaneId": nearest_lane_id,
                    "type": 'lane',
                    "turn": self.map_info.lane_turn.get(nearest_lane_id, 0),
                    "number": self.map_info._get_lane_number_of_road(nearest_lane_id)
                }
            else:
                return {
                    "currentLaneId": nearest_lane_id,
                    "type": 'junction',
                    "turn": 0,
                    "number": 0
                }
        
        return None
    
    def _get_ego_speed(self, ego: Dict) -> float:
        """获取自车速度"""
        vel = ego.get('pose', {}).get('linearVelocity', {})
        vx = vel.get('x', 0)
        vy = vel.get('y', 0)
        return math.sqrt(vx*vx + vy*vy)
