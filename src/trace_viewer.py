"""轨迹查看器 - 将pickle轨迹转换为CSV等格式便于查看"""
import pickle
import csv
from pathlib import Path
from typing import Dict, List, Any, Optional
import json


class TraceViewer:
    """轨迹查看器，提供多种格式的轨迹展示"""
    
    def __init__(self, pickle_path: str):
        """
        初始化轨迹查看器
        
        Args:
            pickle_path: pickle文件路径
        """
        self.pickle_path = pickle_path
        self.data = self._load_pickle()
    
    def _load_pickle(self) -> Dict:
        """加载pickle文件"""
        with open(self.pickle_path, 'rb') as f:
            return pickle.load(f)
    
    def show_summary(self):
        """显示轨迹摘要信息"""
        print("=" * 60)
        print("轨迹摘要信息")
        print("=" * 60)
        print(f"时间戳数量: {len(self.data.get('trace', {}))}")
        print(f"Agent数量: {len(self.data.get('AgentNames', []))}")
        print(f"地图名称: {self.data.get('mapName', 'N/A')}")
        print(f"测试失败: {self.data.get('testFailures', [])}")
        print(f"到达目的地: {self.data.get('destinationReached', False)}")
        print(f"完成状态: {self.data.get('completed', False)}")
        
        if self.data.get('AgentNames'):
            print(f"\nAgent列表 (前10个):")
            for agent in self.data['AgentNames'][:10]:
                print(f"  - {agent}")
        
        print("=" * 60)
    
    def export_to_csv(self, output_path: str, max_rows: int = 50):
        """
        导出轨迹数据到CSV文件
        
        Args:
            output_path: 输出CSV文件路径
            max_rows: 最大导出行数
        """
        trace = self.data.get('trace', {})
        if not trace:
            print("警告: 轨迹数据为空")
            return
        
        # 获取时间戳列表
        timestamps = sorted(trace.keys())[:max_rows]
        
        # 准备CSV数据
        rows = []
        for ts in timestamps:
            point = trace[ts]
            ego = point.get('ego', {})
            truth = point.get('truth', {})
            traffic_lights = point.get('traffic_lights', {})
            
            # 提取关键信息
            pose = ego.get('pose', {})
            position = pose.get('position', {})
            chassis = ego.get('Chassis', {})
            
            row = {
                'timestamp': ts,
                'ego_x': position.get('x', 0),
                'ego_y': position.get('y', 0),
                'ego_z': position.get('z', 0),
                'ego_heading': pose.get('heading', 0),
                'ego_speed': chassis.get('speed', 0),
                'ego_gear': chassis.get('gearLocation', 0),
                'planning_turn': ego.get('planning_of_turn', 0),
                'is_overtaking': ego.get('isOverTaking', False),
                'is_lane_changing': ego.get('isLaneChanging', False),
                'is_turning_around': ego.get('isTurningAround', False),
                'current_lane_id': ego.get('currentLane', {}).get('currentLaneId', 'None'),
                'current_lane_type': ego.get('currentLane', {}).get('type', 'None'),
                'crosswalk_ahead': ego.get('crosswalkAhead', 999),
                'junction_ahead': ego.get('junctionAhead', 999),
                'stop_sign_ahead': ego.get('stopSignAhead', 999),
                'stopline_ahead': ego.get('stoplineAhead', 999),
                'is_traffic_jam': ego.get('isTrafficJam', False),
                'priority_npc_ahead': ego.get('PriorityNPCAhead', False),
                'priority_peds_ahead': ego.get('PriorityPedsAhead', False),
                'reach_destination': ego.get('reach_destinaton', False),
                'min_dist_to_ego': truth.get('minDistToEgo', 999),
                'nearest_obs_id': truth.get('nearestGtObs', 'None'),
                'npc_ahead': truth.get('NPCAhead', 'None'),
                'ped_ahead': truth.get('PedAhead', 'None'),
                'npc_opposite': truth.get('NPCOpposite', 'None'),
                'obs_count': len(truth.get('obsList', [])),
                'traffic_light_dist': traffic_lights.get('trafficLightStopLine', 999),
                # 新增的高级信号
                'v_ego': ego.get('v_ego', 0),
                'front_dist': ego.get('front_dist', 999),
                'd_safe': ego.get('d_safe', 0),
                'v_rel_front': ego.get('v_rel_front', 0),
                'thw_front': ego.get('thw_front', 999),
                'ttc_front': ego.get('ttc_front', 999),
                'a_ego': ego.get('a_ego', 0),
                'hard_brake': ego.get('HardBrake', False),
                'lane_change_started': ego.get('LaneChangeStarted', False),
                'lane_change_finished': ego.get('LaneChangeFinished', False),
                'brake_or_lane_change': ego.get('BrakeOrLaneChange', False),
                'lat_offset': ego.get('lat_offset', 0),
                'in_lane': ego.get('InLane', True),
                'gap_safe': ego.get('GapSafe', True),
                'red_light_ahead': ego.get('RedLightAhead', False),
                'dist_to_red_stopline': ego.get('DistToRedStopLine', 999),
                'should_stop_for_red': ego.get('ShouldStopForRed', False),
                'stop_sign_ahead_flag': ego.get('StopSignAhead', False),
                'dist_to_stop_sign': ego.get('DistToStopSign', 999),
                'dist_to_stopline': ego.get('DistToStopLine', 999),
                'should_stop_at_stop_sign': ego.get('ShouldStopAtStopSign', False),
                'ped_in_crosswalk': ego.get('PedInCrosswalk', False),
                'dist_to_crosswalk': ego.get('DistToCrosswalk', 999),
                'stopped_duration': ego.get('StoppedDuration', 0),
                'unjustified_stop': ego.get('UnjustifiedStop', False),
            }
            rows.append(row)
        
        # 写入CSV
        if rows:
            fieldnames = rows[0].keys()
            with open(output_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            
            print(f"✓ 已导出 {len(rows)} 行数据到: {output_path}")
        else:
            print("警告: 没有数据可导出")
    
    def export_obstacles_to_csv(self, output_path: str, max_rows: int = 50):
        """
        导出障碍物详细信息到CSV
        
        Args:
            output_path: 输出CSV文件路径
            max_rows: 最大导出时间戳数
        """
        trace = self.data.get('trace', {})
        if not trace:
            print("警告: 轨迹数据为空")
            return
        
        timestamps = sorted(trace.keys())[:max_rows]
        
        rows = []
        for ts in timestamps:
            point = trace[ts]
            truth = point.get('truth', {})
            obs_list = truth.get('obsList', [])
            
            for obs in obs_list:
                position = obs.get('position', {})
                current_lane = obs.get('currentLane', {})
                
                row = {
                    'timestamp': ts,
                    'obs_id': obs.get('id', 'unknown'),
                    'obs_type': obs.get('type', 'unknown'),
                    'obs_x': position.get('x', 0),
                    'obs_y': position.get('y', 0),
                    'obs_speed': obs.get('speed', 0),
                    'dist_to_ego': obs.get('distToEgo', 999),
                    'lane_type': current_lane.get('type', 'unknown'),
                    'lane_id': current_lane.get('currentLaneId', 'unknown'),
                }
                rows.append(row)
        
        if rows:
            fieldnames = rows[0].keys()
            with open(output_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            
            print(f"✓ 已导出 {len(rows)} 条障碍物数据到: {output_path}")
        else:
            print("警告: 没有障碍物数据可导出")
    
    def export_to_json(self, output_path: str, max_timestamps: int = 10):
        """
        导出部分轨迹到JSON格式（便于人类阅读）
        
        Args:
            output_path: 输出JSON文件路径
            max_timestamps: 最大导出时间戳数
        """
        trace = self.data.get('trace', {})
        if not trace:
            print("警告: 轨迹数据为空")
            return
        
        timestamps = sorted(trace.keys())[:max_timestamps]
        
        # 构建JSON数据（需要处理Shapely对象）
        export_data = {
            'summary': {
                'timestamp_count': len(self.data.get('trace', {})),
                'agent_count': len(self.data.get('AgentNames', [])),
                'map_name': self.data.get('mapName', 'N/A'),
                'test_failures': self.data.get('testFailures', []),
                'completed': self.data.get('completed', False)
            },
            'trace_sample': {}
        }
        
        for ts in timestamps:
            point = trace[ts]
            # 移除Shapely对象（area字段）
            clean_point = self._clean_shapely_objects(point)
            export_data['trace_sample'][str(ts)] = clean_point
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(export_data, f, indent=2, ensure_ascii=False)
        
        print(f"✓ 已导出 {len(timestamps)} 个时间戳的数据到: {output_path}")
    
    def _clean_shapely_objects(self, obj: Any) -> Any:
        """递归清理Shapely对象，转换为可JSON序列化的格式"""
        if isinstance(obj, dict):
            cleaned = {}
            for k, v in obj.items():
                if k == 'area':
                    # Shapely对象转换为坐标列表
                    try:
                        if hasattr(v, 'exterior'):
                            cleaned[k] = list(v.exterior.coords)
                        else:
                            cleaned[k] = str(v)
                    except:
                        cleaned[k] = 'Polygon'
                else:
                    cleaned[k] = self._clean_shapely_objects(v)
            return cleaned
        elif isinstance(obj, list):
            return [self._clean_shapely_objects(item) for item in obj]
        else:
            return obj
    
    def show_first_n_timestamps(self, n: int = 5):
        """
        显示前N个时间戳的详细信息
        
        Args:
            n: 显示的时间戳数量
        """
        trace = self.data.get('trace', {})
        if not trace:
            print("警告: 轨迹数据为空")
            return
        
        timestamps = sorted(trace.keys())[:n]
        
        print("\n" + "=" * 60)
        print(f"前 {len(timestamps)} 个时间戳的详细信息")
        print("=" * 60)
        
        for i, ts in enumerate(timestamps, 1):
            point = trace[ts]
            ego = point.get('ego', {})
            truth = point.get('truth', {})
            
            pose = ego.get('pose', {})
            position = pose.get('position', {})
            chassis = ego.get('Chassis', {})
            
            print(f"\n[{i}] 时间戳: {ts}")
            print(f"  自车位置: ({position.get('x', 0):.2f}, {position.get('y', 0):.2f}, {position.get('z', 0):.2f})")
            print(f"  自车朝向: {pose.get('heading', 0):.4f} rad")
            print(f"  自车速度: {chassis.get('speed', 0):.2f} m/s")
            print(f"  档位: {chassis.get('gearLocation', 0)}")
            print(f"  转向信号: {ego.get('planning_of_turn', 0)}")
            print(f"  是否超车: {ego.get('isOverTaking', False)}")
            print(f"  障碍物数量: {len(truth.get('obsList', []))}")
            print(f"  最近障碍物距离: {truth.get('minDistToEgo', 999):.2f} m")
            print(f"  最近障碍物ID: {truth.get('nearestGtObs', 'None')}")


def view_trace(pickle_path: str, export_csv: bool = True, max_rows: int = 50):
    """
    查看和导出轨迹的便捷函数
    
    Args:
        pickle_path: pickle文件路径
        export_csv: 是否导出CSV
        max_rows: CSV最大行数
    """
    viewer = TraceViewer(pickle_path)
    
    # 显示摘要
    viewer.show_summary()
    
    # 显示前几个时间戳
    viewer.show_first_n_timestamps(5)
    
    if export_csv:
        # 自动生成CSV文件名
        pickle_file = Path(pickle_path)
        csv_path = pickle_file.parent / f"{pickle_file.stem}_ego.csv"
        obs_csv_path = pickle_file.parent / f"{pickle_file.stem}_obstacles.csv"
        json_path = pickle_file.parent / f"{pickle_file.stem}_sample.json"
        
        print("\n" + "=" * 60)
        print("导出数据文件")
        print("=" * 60)
        
        # 导出自车轨迹
        viewer.export_to_csv(str(csv_path), max_rows)
        
        # 导出障碍物数据
        viewer.export_obstacles_to_csv(str(obs_csv_path), max_rows)
        
        # 导出JSON样例
        viewer.export_to_json(str(json_path), max_timestamps=10)
        
        print("\n完成! 可以使用Excel或其他工具查看CSV文件。")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("用法: uv run python -m src.trace_viewer <pickle文件路径> [max_rows]")
        sys.exit(1)
    
    pickle_path = sys.argv[1]
    max_rows = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    
    view_trace(pickle_path, export_csv=True, max_rows=max_rows)
