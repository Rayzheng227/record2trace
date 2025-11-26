"""轨迹提取器主模块"""
import pickle
from typing import Dict, List, Optional
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm
from cyber_record.record import Record

from .config import TOPIC_MAP
from .processors import (
    PoseProcessor, ChassisProcessor, PlanningProcessor,
    ObstaclesProcessor, TrafficLightProcessor
)
from .map_loader import MapInfo
from .utils import find_nearest_time, get_map_name_from_path
from .post_processor import PostProcessor


class TraceExtractor:
    """轨迹提取器，负责从record文件提取并对齐轨迹数据"""
    
    def __init__(self, record_path: str, map_name: Optional[str] = None):
        """
        初始化轨迹提取器
        
        Args:
            record_path: record文件路径
            map_name: 地图名称，如果不指定则从路径推断
        """
        self.record_path = record_path
        
        # 确定地图名称
        if map_name is None:
            map_name = get_map_name_from_path(record_path)
        self.map_name = map_name
        
        # 加载地图信息
        print(f"加载地图: {map_name}")
        self.map_info = MapInfo(map_name)
        
        # 初始化消息处理器
        self.processors = {
            'pose': PoseProcessor(self.map_info),
            'chassis': ChassisProcessor(self.map_info),
            'planning': PlanningProcessor(self.map_info),
            'obstacles': ObstaclesProcessor(self.map_info),
            'traffic_light': TrafficLightProcessor(self.map_info)
        }
        
        # 存储提取的消息数据
        self.messages: Dict[str, Dict[float, Dict]] = {}
        
        # 最终的对齐轨迹
        self.trace: Dict[float, Dict] = {}
        
        # 初始化后处理器
        self.post_processor = PostProcessor(self.map_info)
        
        # Agent名称列表
        self.agent_names: List[str] = []
    
    def extract(self) -> Dict:
        """
        提取轨迹数据
        
        Returns:
            包含完整轨迹信息的字典
        """
        print(f"开始处理record文件: {self.record_path}")
        
        # 步骤1: 提取各类消息
        self._extract_all_messages()
        
        # 步骤2: 时间戳对齐
        self._align_timestamps()
        
        # 步骤3: 后处理，计算派生字段
        print("开始后处理...")
        self.trace = self.post_processor.process_trace(self.trace)
        print("后处理完成")
        
        # 步骤4: 整理输出结果
        result = self._build_result()
        
        print(f"提取完成，共 {len(self.trace)} 个时间戳")
        return result
    
    def _extract_all_messages(self):
        """提取所有类型的消息"""
        # 先提取pose作为基准时间戳
        print("提取pose消息...")
        self._extract_messages('pose')
        
        # 并行提取其他消息
        other_topics = [k for k in TOPIC_MAP.keys() if k != 'pose']
        
        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [executor.submit(self._extract_messages, topic) for topic in other_topics]
            for future in futures:
                future.result()
    
    def _extract_messages(self, topic_name: str):
        """
        提取单类消息
        
        Args:
            topic_name: 消息类型名称
        """
        self.messages[topic_name] = {}
        processor = self.processors[topic_name]
        topic = TOPIC_MAP[topic_name]
        
        record = Record(self.record_path)
        
        # 读取所有消息
        messages_list = [(t, message) for _, message, t in record.read_messages(topic)]
        
        print(f"处理 {topic_name} 消息: {len(messages_list)} 条")
        
        # 处理每条消息
        for timestamp, message in tqdm(messages_list, desc=f"Processing {topic_name}"):
            try:
                processed = processor.process(message, timestamp)
                if processed:
                    self.messages[topic_name][timestamp] = processed
            except Exception as e:
                # 忽略处理错误，继续下一条
                continue
        
        record.close()
    
    def _align_timestamps(self):
        """基于pose的时间戳对齐所有数据"""
        print("开始时间戳对齐...")
        
        pose_timestamps = sorted(self.messages['pose'].keys())
        chassis_timestamps = sorted(self.messages['chassis'].keys())
        
        # 确保pose和chassis时间戳数量一致
        min_len = min(len(pose_timestamps), len(chassis_timestamps))
        pose_timestamps = pose_timestamps[:min_len]
        chassis_timestamps = chassis_timestamps[:min_len]
        
        print(f"对齐时间戳数量: {len(pose_timestamps)}")
        
        for pose_ts, chassis_ts in tqdm(zip(pose_timestamps, chassis_timestamps), 
                                        total=len(pose_timestamps),
                                        desc="对齐时间戳"):
            self._align_single_timestamp(pose_ts, chassis_ts)
    
    def _align_single_timestamp(self, pose_ts: float, chassis_ts: float):
        """
        对齐单个时间戳的数据
        
        Args:
            pose_ts: pose时间戳
            chassis_ts: chassis时间戳
        """
        # 查找最近的obstacles时间戳
        obstacles_ts = find_nearest_time(list(self.messages['obstacles'].keys()), pose_ts)
        if obstacles_ts is None:
            return
        
        # 构建单个时间点的轨迹数据
        trace_point = {
            "timestamp": pose_ts,
            "ego": {},
            "truth": {},
            "traffic_lights": {}
        }
        
        # 添加pose数据
        pose_data = self.messages['pose'][pose_ts]
        trace_point["ego"].update(pose_data)
        
        # 添加chassis数据
        chassis_data = self.messages['chassis'][chassis_ts]
        trace_point["ego"]["Chassis"] = chassis_data
        
        # 添加planning数据
        planning_ts = find_nearest_time(list(self.messages['planning'].keys()), pose_ts)
        if planning_ts is not None:
            planning_data = self.messages['planning'][planning_ts]
            trace_point["ego"]["planning_of_turn"] = planning_data.get('planning_turn_signal', 0)
            trace_point["ego"]["isOverTaking"] = planning_data.get('is_overtaking', False)
        else:
            trace_point["ego"]["planning_of_turn"] = 0
            trace_point["ego"]["isOverTaking"] = False
        
        # 添加obstacles数据（需要pose作为上下文）
        obstacles_processor = self.processors['obstacles']
        
        # 获取原始obstacles数据的obsList
        obstacles_data_orig = self.messages['obstacles'][obstacles_ts]
        obs_list = obstacles_data_orig.get('obsList', [])
        
        # 创建上下文
        context = {'pose': pose_data}
        
        # 重新处理每个障碍物以计算距离
        for obs in obs_list:
            obstacles_processor._process_single_obstacle(obs, pose_data)
        
        # 重新计算minDistToEgo和nearestGtObs
        min_dist = 200
        nearest_id = None
        
        for obs in obs_list:
            obs_id = obs.get("id", "unknown")
            if obs_id not in self.agent_names:
                self.agent_names.append(obs_id)
            
            dist = obs.get("distToEgo", 200)
            if dist < min_dist:
                min_dist = dist
                nearest_id = obs_id
        
        # 构建truth数据
        trace_point["truth"] = {
            "minDistToEgo": min_dist,
            "nearestGtObs": nearest_id,
            "NearestNPC": nearest_id,
            "obsList": obs_list
        }
        
        # 添加traffic_light数据
        traffic_light_ts = find_nearest_time(list(self.messages['traffic_light'].keys()), pose_ts)
        if traffic_light_ts is not None:
            traffic_light_data = self.messages['traffic_light'][traffic_light_ts]
            
            # 重新处理以计算距离
            traffic_light_processor = self.processors['traffic_light']
            traffic_light_data = traffic_light_processor.process(
                None, traffic_light_ts, context={'pose': pose_data}
            )
            # 更新原始数据
            if traffic_light_ts in self.messages['traffic_light']:
                orig_data = self.messages['traffic_light'][traffic_light_ts]
                if 'trafficLightList' in orig_data:
                    traffic_light_data['trafficLightList'] = orig_data['trafficLightList']
            
            trace_point["traffic_lights"] = traffic_light_data
        
        # 保存到trace
        self.trace[pose_ts] = trace_point
    
    def _build_result(self) -> Dict:
        """构建最终输出结果"""
        # 计算是否到达目的地和最小障碍物距离
        min_ego_obs_dist = 200
        reach_destination = False
        
        for timestamp, trace_point in self.trace.items():
            truth = trace_point.get("truth", {})
            min_dist = truth.get("minDistToEgo", 200)
            if min_dist < min_ego_obs_dist:
                min_ego_obs_dist = min_dist
        
        # 判断是否发生碰撞
        # 阈值可调整：0表示接触即判定碰撞，负值表示允许一定重叠
        COLLISION_THRESHOLD = 0  # 可以改为 -0.5 等负值来放宽判定
        test_failures = []
        if min_ego_obs_dist <= COLLISION_THRESHOLD:
            test_failures.append('Accident')
        
        # 构建结果字典
        result = {
            "testFailures": test_failures,
            "trace": self.trace,
            "completed": reach_destination,
            "destinationReached": reach_destination,
            "groundTruthPerception": True,
            "AgentNames": self.agent_names,
            "mapName": self.map_name
        }
        
        return result
    
    def save_to_pickle(self, output_path: str):
        """
        保存轨迹到pickle文件
        
        Args:
            output_path: 输出文件路径
        """
        result = self.extract()
        
        print(f"保存轨迹到: {output_path}")
        with open(output_path, 'wb') as f:
            pickle.dump(result, f)
        
        print("保存完成")
        return result


def extract_trace(record_path: str, output_path: str, map_name: Optional[str] = None) -> Dict:
    """
    提取轨迹的便捷函数
    
    Args:
        record_path: record文件路径
        output_path: 输出pickle文件路径
        map_name: 地图名称（可选）
    
    Returns:
        提取的轨迹字典
    """
    extractor = TraceExtractor(record_path, map_name)
    return extractor.save_to_pickle(output_path)
