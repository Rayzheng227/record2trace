# Record2trace

Record2trace的目的是将Apollo的CyberRT录制的`.record`文件转换为trace，便于使用STL进行相关性质的验证。




## Before Use
### Map Info
需要在Apollo的容器中将编译好的地图文件`map.bin`转换为json文件；具体来说，通过运行以下的`convert_pb_to_json`脚本即可得到Json格式的地图文件。
```python
from modules.common_msgs.map_msgs import map_pb2
def load_map_proto(path: str) -> map_pb2.Map:
    """
    Load Map proto from either binary (.bin) or text (.txt) file.
    """
    m = map_pb2.Map()
    ext = os.path.splitext(path)[1].lower()

    if ext == ".bin":
        with open(path, "rb") as f:
            data = f.read()
        m.ParseFromString(data)
    else:
        with open(path, "r") as f:
            txt = f.read()
        text_format.Merge(txt, m)

    return m


def convert_pb_to_json(input_pb: str, output_json: str):
    print("Loading map proto from:", input_pb)

    hdmap = load_map_proto(input_pb)

    print("Converting proto to JSON...")

    json_str = json_format.MessageToJson(
        hdmap,
        preserving_proto_field_name=True,
        including_default_value_fields=True,
    )

    with open(output_json, "w") as f:
        f.write(json_str)

    print("Done. Output written to:", output_json)
```
### Message Info
由于各版本的Apollo的Proto定义有所差异，需要在构建好的Apollo容器中，运行以下代码：
```shell
protoc -I. \
  --python_out=. \
  $(find modules/common_msgs -name "*.proto")
```
之后将common_msgs文件夹覆盖该项目中的同名文件。

### Data prepare
将需要转换的Apollo录制结果`.record`文件放入项目目录`raw_record`中。
注意，`raw_record`文件夹中可以包含多个`.record`文件，但目前仅支持转换一个`.record`文件。
此外，需要按照地图进行存放，如`a.record`使用地图`sunnyvale`，则需要将`a.record`放入`sunnyvale`文件夹中。

## 使用方法

本项目使用 `uv` 作为包管理器。在GitHub上克隆项目后，进入项目目录并运行以下命令进行安装：
```bash
uv sync
```

### 基本使用

```bash
# 提取轨迹（自动选择raw_record下最新的record文件，保存到output/目录）
uv run python main.py

# 指定record文件
uv run python main.py <record文件路径>

# 指定输出路径
uv run python main.py <record文件> <输出文件>

# 指定地图
uv run python main.py <record文件> --map <地图名称>

# 提取后自动查看并导出CSV
uv run python main.py --view
```

**说明**：
- 如果不指定record文件路径，程序会自动在`raw_record`目录下递归查找最新的`.record.*`文件
- 输出文件名自动使用record文件的完整名称，格式为：`<record完整名称>.pickle`
- 例如：`raw_record/lanechange/20251015164431.record.00000.20251015164431` → `output/20251015164431.record.00000.20251015164431.pickle`
- 如果同名文件已存在，会自动覆盖

### 查看轨迹

```bash
# 查看最新生成的pickle文件并导出CSV
uv run python view_trace.py

# 查看指定pickle文件并导出CSV
uv run python view_trace.py <pickle文件路径>

# 自定义CSV行数
uv run python view_trace.py <pickle文件> --max-rows 100

# 仅查看摘要，不导出CSV
uv run python view_trace.py <pickle文件> --no-csv
```

**说明**：
- 如果不指定pickle文件路径，程序会自动在`output`目录下递归查找最新的`.pickle`文件
- 导出的CSV和JSON文件会保存在pickle文件所在目录

## 提取的轨迹内容

### Pickle文件结构

提取的pickle文件是一个Python字典，包含以下顶层结构：

```python
{
    "testFailures": [],           # 测试失败列表（如碰撞检测）
    "trace": {...},               # 按时间戳索引的完整轨迹
    "completed": bool,            # 是否完成
    "destinationReached": bool,   # 是否到达目的地
    "groundTruthPerception": True,# 是否使用真值感知
    "AgentNames": [...],          # 所有障碍物ID列表
    "mapName": str                # 使用的地图名称
}
```

### 单个时间戳的数据组成

每个时间戳包含三部分数据：

#### 1. ego - 自车数据

**原始数据（直接从消息提取）：**
- `pose` - 位置、朝向、速度、加速度等
- `size` - 车辆尺寸（长4.7m，宽2.06m）
- `area` - 车辆多边形区域（Shapely对象）
- `Chassis` - 底盘状态（速度、档位、油门、刹车等）
- `planning_of_turn` - 转向信号（0=直行，1=左转，2=右转）
- `isOverTaking` - 是否正在超车

**后处理字段（计算得出）：**
- `currentLane` - 当前车道信息（ID、类型、转向、车道数）
- `crosswalkAhead` - 前方人行横道距离（米）
- `junctionAhead` - 前方路口距离（米）
- `stopSignAhead` - 前方停止标志距离（米）
- `stoplineAhead` - 前方停止线距离（米）
- `junction_ahead` - 前方路口ID
- `isLaneChanging` - 是否正在变道
- `isTurningAround` - 是否正在掉头
- `isTrafficJam` - 是否交通拥堵
- `PriorityNPCAhead` - 前方是否有优先车辆
- `PriorityPedsAhead` - 前方是否有优先行人
- `reach_destinaton` - 是否到达目的地

#### 2. truth - 障碍物真值数据

**原始数据：**
- `obsList` - 障碍物列表，每个障碍物包含：
  - `id` - 障碍物唯一标识
  - `type` - 类型（VEHICLE/PEDESTRIAN/BICYCLE等）
  - `position` - 位置坐标
  - `velocity` - 速度向量
  - `speed` - 标量速度
  - `theta` - 朝向角
  - `length/width/height` - 尺寸
  - `polygonPoint` - 四个角点坐标
  - `distToEgo` - 到自车的距离（米）
  - `currentLane` - 所在车道信息

**后处理字段：**
- `minDistToEgo` - 最近障碍物距离（米）
- `nearestGtObs` - 最近障碍物ID
- `NearestNPC` - 最近车辆ID
- `NPCAhead` - 前方车辆ID
- `PedAhead` - 前方行人ID
- `NPCOpposite` - 对向车辆ID
- `npcClassification` - 障碍物分类（同车道、不同道路、路口内等）

#### 3. traffic_lights - 红绿灯数据

- `trafficLightList` - 红绿灯列表
- `nearest` - 最近红绿灯索引
- `trafficLightStopLine` - 到最近红绿灯停止线的距离（米）

### 默认值说明

在CSV导出中，您可能看到这些默认值：

- **999** - 表示该数据不可用或无穷大（如没有检测到该类对象）
  - 例如：`crosswalk_ahead=999` 表示前方200米内没有人行横道
  - 例如：`traffic_light_dist=999` 表示没有检测到红绿灯

- **200** - 默认最大检测距离（米）
  - 例如：`min_dist_to_ego=200` 表示最近障碍物在200米以外
  - 这是系统设定的感知范围上限

- **None** - 表示该字段不适用或无法计算
  - 例如：`current_lane_id=None` 表示自车不在任何已知车道上
  - 例如：`npc_ahead=None` 表示前方没有车辆

## 后处理说明

### 后处理流程

轨迹提取分为两个阶段：

1. **原始数据提取**（`TraceExtractor`）
   - 从record文件读取各类消息（pose、chassis、planning、obstacles、traffic_light）
   - 基于时间戳对齐数据
   - 计算基础几何信息（多边形、距离等）

2. **后处理计算**（`PostProcessor`）
   - 计算派生字段
   - 分析车辆行为
   - 判定交通状态

### 后处理计算内容

#### 1. 车道信息计算

- **方法**：`_get_current_lane()`
- **输入**：自车多边形区域
- **处理**：
  - 查询地图，判断自车是否在车道或路口内
  - 如果不在任何区域内，查找最近的车道（距离<5米）
  - 计算车道转向类型和车道数
- **输出**：`currentLane`（ID、类型、转向、车道数）

#### 2. 前方距离计算

- **方法**：`_calculate_ahead_distances()`
- **输入**：自车位置、朝向、车头位置
- **处理**：
  - 计算车头前方200米的扇形区域
  - 检测该区域内的人行横道、路口、停止标志、停止线
  - 计算多边形到自车的距离
- **输出**：`crosswalkAhead`, `junctionAhead`, `stopSignAhead`, `stoplineAhead`

#### 3. 前方障碍物识别

- **方法**：`_calculate_ahead_obstacles()`
- **输入**：障碍物列表、自车车道、前方区域
- **处理**：
  - 判断障碍物是否在前方区域内
  - 对于车辆：检查是否在同一车道
  - 对于行人：检查是否在前方任意位置
  - 对向车辆：检查朝向差异（135°-225°）
- **输出**：`NPCAhead`, `PedAhead`, `NPCOpposite`

#### 4. 障碍物分类

- **方法**：`_classify_obstacles()`
- **输入**：障碍物列表、自车车道
- **处理**：根据自车和障碍物的车道关系分类
- **输出**：`npcClassification`
  - `NextToEgo` - 同道路不同车道
  - `OntheDifferentRoad` - 不同道路
  - `IntheJunction` - 路口内
  - `EgoInjunction_Lane` - 自车在路口，障碍物在车道
  - `EgoInjunction_junction` - 都在路口

#### 5. 变道检测

- **方法**：`_check_lane_changing()`
- **输入**：当前和过去10个时间戳的数据
- **处理**：
  - 检查转向信号是否激活
  - 对比当前车道ID与10个时间戳前的车道ID
  - 判断两个车道是否在同一道路上
- **输出**：`isLaneChanging`

#### 6. 掉头检测

- **方法**：`_check_turning_around()`
- **输入**：当前和过去20个时间戳的数据
- **处理**：
  - 检查转向信号是否激活
  - 计算朝向角变化
  - 判断是否旋转超过135度
- **输出**：`isTurningAround`

#### 7. 优先级判定

- **方法**：`_find_priority_npcs_and_peds()`
- **输入**：障碍物列表、自车状态、前方/侧方/后方区域
- **处理**：
  - 前方3米内的车辆
  - 转向时的交叉路径车辆
  - 变道时后方10米内速度更快的车辆
  - 行人在前方或转向路径上
- **输出**：`PriorityNPCAhead`, `PriorityPedsAhead`

#### 8. 交通拥堵判定

- **方法**：`_check_traffic_jam()`
- **输入**：前方路口、障碍物列表
- **处理**：统计前方路口内速度<1m/s的车辆数量
- **输出**：`isTrafficJam`（≥6辆车时为True）

## trace_viewer 使用说明

`trace_viewer` 提供了查看和导出轨迹数据的功能。

### 生成的文件

运行 `view_trace.py` 会生成三个文件：

#### 1. `<名称>_ego.csv` - 自车轨迹

包含29列数据，每行代表一个时间戳：
- 基础信息：timestamp, ego_x, ego_y, ego_z, ego_heading
- 运动状态：ego_speed, ego_gear
- 决策信息：planning_turn, is_overtaking, is_lane_changing, is_turning_around
- 车道信息：current_lane_id, current_lane_type
- 前方距离：crosswalk_ahead, junction_ahead, stop_sign_ahead, stopline_ahead
- 交通状态：is_traffic_jam, priority_npc_ahead, priority_peds_ahead
- 障碍物信息：min_dist_to_ego, nearest_obs_id, npc_ahead, ped_ahead, npc_opposite, obs_count
- 红绿灯：traffic_light_dist
- 目标：reach_destination

**用途**：Excel打开，分析自车行为、速度变化、决策过程

#### 2. `<名称>_obstacles.csv` - 障碍物详情

每行代表一个障碍物在某时刻的状态：
- timestamp, obs_id, obs_type
- obs_x, obs_y, obs_speed
- dist_to_ego, lane_type, lane_id

**用途**：分析障碍物运动轨迹、交互行为

#### 3. `<名称>_sample.json` - 完整数据样例

包含前10个时间戳的完整JSON数据，展示：
- 数据结构
- 所有字段含义
- 嵌套关系

**用途**：理解完整数据格式、调试、开发

### 查看器功能

```python
from src.trace_viewer import TraceViewer

viewer = TraceViewer("output/trace.pickle")

# 显示摘要信息
viewer.show_summary()

# 显示前N个时间戳的详细信息
viewer.show_first_n_timestamps(5)

# 导出CSV
viewer.export_to_csv("output/ego.csv", max_rows=50)
viewer.export_obstacles_to_csv("output/obstacles.csv", max_rows=50)

# 导出JSON样例
viewer.export_to_json("output/sample.json", max_timestamps=10)
```