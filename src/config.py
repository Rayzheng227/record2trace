"""配置文件 - 定义消息topic和常量映射"""

# Apollo消息topic映射
TOPIC_MAP = {
    'planning': '/apollo/planning',
    'traffic_light': '/apollo/perception/traffic_light',
    'obstacles': '/apollo/perception/obstacles',
    'pose': '/apollo/localization/pose',
    'chassis': '/apollo/canbus/chassis'
}

# 转向信号映射
TURN_SIGNAL_MAP = {
    'TURN_NONE': 0,
    'TURN_LEFT': 1,
    'TURN_RIGHT': 2
}

# 档位映射
GEAR_MAP = {
    'GEAR_NEUTRAL': 0,
    'GEAR_DRIVE': 1,
    'GEAR_REVERSE': 2,
    'GEAR_PARKING': 3,
    'GEAR_LOW': 4,
    'GEAR_INVALID': 5,
    'GEAR_NONE': 6
}

# 地图名称映射规则
MAP_NAME_RULES = {
    'Sunnyvale': 'sunnyvale',
    'SanMateo': 'san',
    'default': 'lanechange'
}

# 自车尺寸配置
EGO_VEHICLE = {
    'length': 4.7,
    'width': 2.06,
    'wheelbase': 2.697298  # 轴距
}

# 默认距离阈值
DEFAULT_DISTANCE = 200
REACH_DESTINATION_THRESHOLD = 2.0
