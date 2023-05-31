---
layout: post
title: Highway Env usage
category: rl
---

- [Github](https://github.com/Farama-Foundation/HighwayEnv)
- [Documentation](https://farama-foundation.github.io/HighwayEnv/)
- [Examples](https://github.com/Farama-Foundation/HighwayEnv/tree/master/scripts)

<br>

Highway Env는 gym 기반의 자율주행 환경 라이브러리이다.

이를 통해 다양한 도로 환경을 구축하고 강화학습 알고리즘을 테스트해 볼 수 있다.

----

# Table of contents

- [Installation](#installation)
- [Usage](#usage)
  - [Code Structure](#code-structure)
  - [Code Analysis](#code-analysis)
    - [Create Road Environment](#create-road-environment)
    - [Create Vehicle](#create-vehicle)
    - [Define Reward Function](#define-reward-function)
- [Issue](#issue)


-----

# Installation

### Window11

- Gymnasium

```
conda install -c conda-forge gymnasium
```

<br>

- Stable Baselines3

```
conda install -c conda-forge stable-baselines3
```

<br>

- Highway Env

```
pip install highway-env
```

----------

# Usage

### Code Structure

```
HighwayEnv
├─📁.github
│  └─📁workflows
├─📁docs  # HighwayEnv의 공식 Documentation이 저장된 폴더
│  ├─📁actions
│  ├─📁bibliography
│  ├─📁dynamics
│  │  ├─📁road
│  │  └─📁vehicle
│  ├─📁environments
│  ├─📁graphics
│  ├─📁observations
│  ├─📁rewards
│  ├─📁statsscripts
│  └─📁_static
│      └─📁img
├─📁highway_env  # ⭐HighwayEnv 라이브러리
│  ├─📁envs
│  │  └─📁common
│  ├─📁road
│  └─📁vehicle
│      └─📁uncertainty
├─📁scripts
└─📁tests  # HighwayEnv 라이브러리 테스트 코드
    ├─📁envs
    ├─📁graphics
    ├─📁road
    └─📁vehicle
```

우리가 중점적으로 봐야하는 디렉토리는 highway_env이다.

여기에 도로 환경 및 차량 관련 모듈들이 구현돼 있다.

<br>

**📁 highway_env**

```
highway_env
├─📁envs  # 환경 패키지
│  └─📁common
│  │  ├─📝__init__.py
│  │  ├─📝exit_env.py
│  │  ├─📝highway_env.py
│  │  ├─📝intersection_env.py
│  │  ├─📝lane_keeping_env.py
│  │  ├─📝merge_env.py
│  │  ├─📝parking_env.py
│  │  ├─📝racetrack_env.py
│  │  ├─📝roundabout_env.py
│  │  ├─📝two_way_env.py
│  │  ├─📝u_turn_env.py
├─📁road  # 도로 및 차선 패키지
│  └─📝__init__.py
│  └─📝graphics.py
│  └─📝lane.py
│  └─📝regulation.py
│  └─📝road.py
│  └─📝spline.py
└─📁vehicle  # 차량 패키지
    └─📁uncertainty
    └─📝__init__.py
    └─📝behavior.py
    └─📝controller.py
    └─📝dynamics.py
    └─📝graphics.py
    └─📝kinematics.py
    └─📝objects.py
```

envs는 환경 클래스가 정의된 디렉토리이다. [공식 문서](https://farama-foundation.github.io/HighwayEnv/quickstart/)에 Highway, Merge, Roundabout, Parking, Intersection, Raccetrack에 관한 설명이 있는데 얘네들이 정의된 디렉토리이다.

<br>

road는 도로 및 차선 클래스가 정의된 디렉토리이고, vehicle은 차량 클래스가 정의된 디렉토리이다.

<br>

### Code Analysis

##### Create Road Environment

Custom 환경 구성 방법을 알아보기 전에, Highway Env 라이브러리에서 환경을 어떻게 정의했는지 코드를 살펴볼 것이다.

앞서 환경 클래스는 highway\_env/envs 디렉토리에 정의돼 있다고 했는데, 그 부분을 보기 전에 먼저 도로 및 차선이 정의된 패키지부터 보도록하자.

**📁 highway_env/road**

```
📁road
└─📝__init__.py
└─📝graphics.py
└─📝lane.py  # 차선 모듈
└─📝regulation.py
└─📝road.py  # 도로 모듈
└─📝spline.py
```

**차선 생성하기**

차선(lane)은 lane.py 파일에 정의돼 있다.

AbstractLane이라는 추상 클래스가 있고, 이를 상속 받은 StraightLane, CircularLane, PolyLaneFixedWidth 클래스들이 있다.

모든 차선들이 갖는 공통 기능은 모두 AbstractLane에 정의돼 있고, 각 차선별 특징에 따른 메소드들이 overriding 돼 있다.

<br>

도로에 차선을 하나 추가하기 위해선 생성하려는 차선의 종류(직선, 원형 등)에 따라 해당 클래스의 인스턴스를 생성하면 된다.

직선 차선을 만드는 경우 StraightLane 클래스의 인스턴스를 생성하면 된다. *생성 전에 initial parameter를 잘 살펴보자*

<br>

StraightLane 클래스의 생성자를 보면,

📝 [StraightLane.\_\_init\_\_()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/road/lane.py#L144)

```python
class StraightLane(AbstractLane):

    """A lane going in straight line."""

    def __init__(self,
                 start: Vector,
                 end: Vector,
                 width: float = AbstractLane.DEFAULT_WIDTH,
                 line_types: Tuple[LineType, LineType] = None,
                 forbidden: bool = False,
                 speed_limit: float = 20,
                 priority: int = 0) -> None:
        """
        New straight lane.

        :param start: the lane starting position [m]
        :param end: the lane ending position [m]
        :param width: the lane width [m]
        :param line_types: the type of lines on both sides of the lane
        :param forbidden: is changing to this lane forbidden
        :param priority: priority level of the lane, for determining who has right of way
        """
        self.start = np.array(start)
        self.end = np.array(end)
        self.width = width
        self.heading = np.arctan2(self.end[1] - self.start[1], self.end[0] - self.start[0])
        self.length = np.linalg.norm(self.end - self.start)
        self.line_types = line_types or [LineType.STRIPED, LineType.STRIPED]
        self.direction = (self.end - self.start) / self.length
        self.direction_lateral = np.array([-self.direction[1], self.direction[0]])
        self.forbidden = forbidden
        self.priority = priority
        self.speed_limit = speed_limit
```

1. start, end: 차선의 시작 위치 및 종료 위치

2. line\_type: 차선의 양 옆이 무슨 종류인지 (실선인지, 점선인지)
   line\_type은 lane.py 134 ~ 141 line에 다음과 같이 LineType 클래스로 정의돼 있다.

   ```python
   class LineType:
   
       """A lane side line type."""
   
       NONE = 0  # 선 없음
       STRIPED = 1  # 점선
       CONTINUOUS = 2  # 실선
       CONTINUOUS_LINE = 3
   ```

<br>

**도로 생성하기**

생성한 차선들을 연결해 하나의 도로로 만드는 것이  road 모듈이다.

road.py에는 [RoadNetwork 클래스](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/road/road.py#L17)가 정의돼 있는데, RoadNetwork는 다음과 같은 클래스 변수를 갖는다.

```python
class RoadNetwork(object):
    graph: Dict[str, Dict[str, List[AbstractLane]]]
```

<br>

차선을 생성한 뒤, RoadNetwork의 add\_lane 메소드로 해당 차선을 추가한다.

📝 [RoadNetwork.add\_lane()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/road/road.py#L37)

```python
def add_lane(self, _from: str, _to: str, lane: AbstractLane) -> None:
    """
    A lane is encoded as an edge in the road network.

    :param _from: the node at which the lane starts.
    :param _to: the node at which the lane ends.
    :param AbstractLane lane: the lane geometry.
    """
    if _from not in self.graph:
        self.graph[_from] = {}
    if _to not in self.graph[_from]:
        self.graph[_from][_to] = []
    self.graph[_from][_to].append(lane)
```

\_from, \_to는 각각 도로의 시작 부분과 종료 부분을 나타내는 별칭이라고 생각하면 된다. 추후에 merge 환경을 구성할 때 자세히 설명하겠다.

<br>

**📁 highway_env/envs**

```
highway_env
├─📁envs  # 환경 패키지
	└─📁common
		├─📝__init__.py
		├─📝exit_env.py
		├─📝highway_env.py
		├─📝intersection_env.py
		├─📝lane_keeping_env.py
		├─📝merge_env.py
		├─📝parking_env.py
		├─📝racetrack_env.py
		├─📝roundabout_env.py
		├─📝two_way_env.py
		├─📝u_turn_env.py
```

HighwayEnv 라이브러리에서 제공하는 기본 환경들은 모두 highway\_env/envs 디렉토리에 저장돼 있다.

위에 정의된 것들말고 완전히 새로운 환경을 구축하고 싶다면 여기에 새로운 파이썬 파일을 생성한 뒤 시작하면 된다.

<br>

![image](https://github.com/seominseok00/comments/assets/110466566/69e7e8a8-c8fe-42d1-9236-9fe875ad7389)

나는 고속도로 합류 구간을 만들 것이므로 merge\_env.py 파일을 살펴보겠다.

📝 [MergeEnv](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/merge_env.py)

```python
class MergeEnv(AbstractEnv):

    """
    A highway merge negotiation environment.

    The ego-vehicle is driving on a highway and approached a merge, with some vehicles incoming on the access ramp.
    It is rewarded for maintaining a high speed and avoiding collisions, but also making room for merging
    vehicles.
    """

    @classmethod
    def default_config(cls) -> dict:
        cfg = super().default_config()
        cfg.update({
            "collision_reward": -1,
            "right_lane_reward": 0.1,
            "high_speed_reward": 0.2,
            "merging_speed_reward": -0.5,
            "lane_change_reward": -0.05,
        })
        return cfg
```

모든 환경들은 [highway_env/envs/common/abstract.py](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/common/abstract.py)에 정의된 AbstractEnv 클래스를 상속받아 정의됐다.

default\_config 메소드부터 살펴보면, 먼저 부모 클래스(AbstractEnv)의 default\_config 메소드를 호출한다.

📝 [AbstractEnv.default\_config()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/common/abstract.py#L84)

```python
@classmethod
def default_config(cls) -> dict:
    """
    Default environment configuration.

    Can be overloaded in environment implementations, or by calling configure().
    :return: a configuration dict
    """
    return {
        "observation": {
            "type": "Kinematics"
        },
        "action": {
            "type": "DiscreteMetaAction"
        },
        "simulation_frequency": 15,  # [Hz]
        "policy_frequency": 1,  # [Hz]
        "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
        "screen_width": 600,  # [px]
        "screen_height": 150,  # [px]
        "centering_position": [0.3, 0.5],
        "scaling": 5.5,
        "show_trajectories": False,
        "render_agent": True,
        "offscreen_rendering": os.environ.get("OFFSCREEN_RENDERING", "0") == "1",
        "manual_control": False,
        "real_time_rendering": False
    }
```

default\_config 메소드는 다음과 같이 observation, action space를 정의하고, 환경에 관련된 다양한 설정들을 저장하는 딕셔너리를 생성한다.

<br>

[MergeEnv.\_make\_road()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/merge_env.py#L74)

```python
def _make_road(self) -> None:
    """
    Make a road composed of a straight highway and a merging lane.

    :return: the road
    """
    net = RoadNetwork()

    # Highway lanes
    ends = [150, 80, 80, 150]  # Before, converging, merge, after
    c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
    y = [0, StraightLane.DEFAULT_WIDTH]
    line_type = [[c, s], [n, c]]
    line_type_merge = [[c, s], [n, s]]
    for i in range(2):
        net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
        net.add_lane("b", "c", StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge[i]))
        net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends), y[i]], line_types=line_type[i]))

    # Merging lane
    amplitude = 3.25
    ljk = StraightLane([0, 6.5 + 4 + 4], [ends[0], 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
    lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
                   amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
    lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
                       line_types=[n, c], forbidden=True)
    net.add_lane("j", "k", ljk)
    net.add_lane("k", "b", lkb)
    net.add_lane("b", "c", lbc)
    road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
    road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))
    self.road = road
```

\_make\_road는 도로를 만드는 메소드이다.

한 줄씩 보면,

먼저 RoadNetwork 클래스의 인스턴스를 생성한다.

```python
net = RoadNetwork()
```

<br>

ends 리스트는 도로의 마지막 위치를 나타내는 x좌표들의 리스트이고, 각 차선의 타입(실선, 점선, 차선 없음)을 각각 c, s, n으로 정의했다.

y는 도로의 높이, line\_type, line\_type\_merge는 각각 도로의 타입을 의미한다.

```python
ends = [150, 80, 80, 150]
c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
y = [0, StraightLane.DEFAULT_WIDTH]  # StraightLane.DEFAULT_WIDTH = 4
line_type = [[c, s], [n, c]]
line_type_merge = [[c, s], [n, s]]
```

<br>

이 코드는 직선 도로를 만들어서 앞서 생성한 RoadNetwork 클래스의 인스턴스에 추가하는 부분이다.

```python
for i in range(2):
    net.add_lane("a", "b", StraightLane([0, y[i]], [sum(ends[:2]), y[i]], line_types=line_type[i]))
    net.add_lane("b", "c", StraightLane([sum(ends[:2]), y[i]], [sum(ends[:3]), y[i]], line_types=line_type_merge[i]))
    net.add_lane("c", "d", StraightLane([sum(ends[:3]), y[i]], [sum(ends), y[i]], line_types=line_type[i]))
```

![image](https://github.com/seominseok00/comments/assets/110466566/75930040-38b5-4311-8b84-ed31ef498792)

위 코드를 실행하면 다음과 같은 도로가 만들어지게 된다.

for loop가 두 번 돌면서 구간별로 다른 타입의 도로가 만들어지게 되는데, c(LineType.CONTINUOUS_LINE)는 실선을 의미하고, s(LineType.STRIPED)는 점선, n(LineType.NONE)은 차선 없음을 의미한다.

두 번째 for loop 때(i가 1일 때) line\_type을 [n, c]로 하는 이유는 첫 번째 for loop에서(i가 0일 때) 점선을 이미 그었기 때문이다.

<br>

이어서 합류 차선을 생성한다.

```python
# Merging lane
amplitude = 3.25
ljk = StraightLane([0, 6.5 + 4 + 4], [ends[0], 6.5 + 4 + 4], line_types=[c, c], forbidden=True)
lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude),
               amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, line_types=[c, c], forbidden=True)
lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0],
                   line_types=[n, c], forbidden=True)
net.add_lane("j", "k", ljk)
net.add_lane("k", "b", lkb)
net.add_lane("b", "c", lbc)
```

position 메소드는 위치 벡터를 계산하는 함수로, 각 차선 클래스에 정의돼 있다.

![image](https://github.com/seominseok00/comments/assets/110466566/f195ebb3-c011-4a42-b830-f28dd3d4aaed)

<br>

```python
road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
road.objects.append(Obstacle(road, lbc.position(ends[2], 0)))
self.road = road
```

이후 합류 차선의 마지막에 장애물을 설치해주고, 앞서 생성한 RoadNetwork의 인스턴스를 파라미터로 넘겨 Road 인스턴스트를 생성하면 다음과 같은 환경이 완성된다.

![image](https://github.com/seominseok00/comments/assets/110466566/fd53610a-8324-4f40-b6f6-e43532f76acd)

<br>

##### Create Vehicle

[MergeEnv.\_make\_vehicles()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/merge_env.py#L107)

```python
def _make_vehicles(self) -> None:
    """
    Populate a road with several vehicles on the highway and on the merging lane, as well as an ego-vehicle.

    :return: the ego-vehicle
    """
    road = self.road
    ego_vehicle = self.action_type.vehicle_class(road,
                                                 road.network.get_lane(("a", "b", 1)).position(30, 0),
                                                 speed=30)
    road.vehicles.append(ego_vehicle)

    other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
    road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(90, 0), speed=29))
    road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 1)).position(70, 0), speed=31))
    road.vehicles.append(other_vehicles_type(road, road.network.get_lane(("a", "b", 0)).position(5, 0), speed=31.5))

    merging_v = other_vehicles_type(road, road.network.get_lane(("j", "k", 0)).position(110, 0), speed=20)
    merging_v.target_speed = 30
    road.vehicles.append(merging_v)
    self.vehicle = ego_vehicle
```

차량은 \_make\_vehicles 메소드로 생성한다.

먼저 강화학습으로 제어할 ego\_vehicle은 action type에 정의된 vehicle\_class로 생성하는데, default_config 메소드에서 정의된 action space에 따라 달라진다.

*vehicle\_class는 highway_env/envs/common/action.py에 정의돼 있다.*

Merge env에서는 DiscreteMetaAction을 사용하기 때문에 [vehicle_class](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/common/action.py#L236)가 MDPVehicle이다.

*각 환경에서 정의한 observation, action space는 documentation에서 확인할 수 있다(소스코드를 봐도 됨)*

<br>

그 외 다른 차량들은 default\_config 메소드에서 정의한 other\_vehicles\_type인 [IDMVehicle](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/vehicle/behavior.py#L12)을 사용한다.

각 차량들은 초기 위치와 속도를 파라미터로 넘겨주어 생성하고, 생성한 모든 차량들을 앞서 정의한 Road 인스턴스의 vehicles에 추가해준다.

❗**강화학습으로 제어하려는 ego\_vehicle은 0번째 인덱스에 추가해야 한다.**

<br>

##### Define Reward Function

보상함수는 \_rewards 메소드로 각각의 보상을 계산한 뒤 딕셔너리 형태로 반환하면, \_reward 메소드로 하나의 값으로 합친다.

[MergeEnv.\_rewards\()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/envs/merge_env.py#L50)

```python
def _rewards(self, action: int) -> Dict[Text, float]:
    return {
        "collision_reward": self.vehicle.crashed,
        "right_lane_reward": self.vehicle.lane_index[2] / 1,
        "high_speed_reward": self.vehicle.speed_index / (self.vehicle.target_speeds.size - 1),
        "lane_change_reward": action in [0, 2],
        "merging_speed_reward": sum(  # Altruistic penalty
            (vehicle.target_speed - vehicle.speed) / vehicle.target_speed
            for vehicle in self.road.vehicles
            if vehicle.lane_index == ("b", "c", 2) and isinstance(vehicle, ControlledVehicle)
        )
    }
```

각각의 key 값들은 앞서 default\_config 메소드로 정의한 key 값들이다. 

default_config로 정의한 값들은 각 보상들의 가중치이다. *이후 \_reward 메소드에서 다시 설명*

하나씩 살펴보면

**1. collision\_reward**

차량이 충돌할 경우 주는 패널티로, boolean 값이라 충돌할 경우 1, 충돌하지 않았을 경우 0이다.

**2. right\_lane\_reward**

*커스텀 환경을 만들 때 이 보상을 사용하지 않아서 모르겠다 ㅎ..*

**3. high\_speed\_reward**

ego_vehicle에서 속도를 컨트롤 할 때 속도 값에 접근하는게 아니라, 속도를 리스트로 만든 다음 인덱스로 접근한다. *👉 [MDPVehicle.act()](https://github.com/Farama-Foundation/HighwayEnv/blob/master/highway_env/vehicle/controller.py#L234) 참조*
디폴트 속도 범위가 [20, 25, 30]인데 만약 속도가 30에 가까우면 vehicle.speed\_index는 30을 가리키는 인덱스를 리턴한다. 따라서 속도가 높을수록 1의 값을 갖고, 속도가 낮을 수록 1/2에 가까운 값을 갖게 된다.

**4. lane\_change\_reward**

MergeEnv에서는 [DiscreteMetaAction](https://farama-foundation.github.io/HighwayEnv/actions/#discrete-meta-actions)을 사용한다. 0, 2번 인덱스의 행동이 차선을 변경하는 행동인데 에이전트가 0번이나 2번 행동을 취했을 경우 1의 보상을 준다.

**5. merging\_speed\_reward**

*이것도 커스텀 환경을 만들 때 사용하지 않아서 잘 모르겠다*

<br>

```python
def _reward(self, action: int) -> float:
    """
    The vehicle is rewarded for driving with high speed on lanes to the right and avoiding collisions

    But an additional altruistic penalty is also suffered if any vehicle on the merging lane has a low speed.

    :param action: the action performed
    :return: the reward of the state-action transition
    """
    reward = sum(self.config.get(name, 0) * reward for name, reward in self._rewards(action).items())
    return utils.lmap(reward,
                      [self.config["collision_reward"] + self.config["merging_speed_reward"],
                       self.config["high_speed_reward"] + self.config["right_lane_reward"]],
                      [0, 1])
```

\_rewards 메소드에서 계산한 보상에 앞서 default\_config 메소드에서 configuration으로 정의한 각 값들을 가중치로 곱한 뒤 더한다.

<br>

```python
cfg.update({
            "collision_reward": -1,
            "right_lane_reward": 0.1,
            "high_speed_reward": 0.2,
            "merging_speed_reward": -0.5,
            "lane_change_reward": -0.05,
        })
```

-------------

# Issue

**AssertionError: The algorithm only supports (<class 'gym.spaces.discrete.Discrete'>,) as action spaces but Discrete(5) was provided.**

예제 코드를 실행하면, 저렇게 space가 맞지 않는다는 에러가 발생한다. issue 탭에 역시 같은 오류를 가진 사람이 있었다.

아래 명령어를 통해 stable baselines를 업데이트 하면 해결된다.

```
pip install stable-baselines3[extra] --pre --upgrade
```

➡️ [AssertionError: The algorithm only supports (<class 'gym.spaces.box.Box'>, <class 'gym.spaces.discrete.Discrete'>, <class 'gym.spaces.multi_discrete.MultiDiscrete'>, <class 'gym.spaces.multi_binary.MultiBinary'>) as action spaces but Discrete(2) was provided](http://archive.today/WT72L)

<br>

**ModuleNotFoundError: No module named 'moviepy'**

RecordVideo를 실행하면 moviepy 모듈이 없다는 에러가 발생하는데, 아래 명령어를 통해 moviepy를 설치하면 해결된다.

```
pip install moviepy
```

<br>

**RuntimeError: Calling torch.geqrf on a CPU tensor requires compiling PyTorch with LAPACK. Please use PyTorch built with LAPACK support.**

1. [PyTorch](https://pytorch.org/get-started/locally/)를 CPU only로 설치

```
conda install pytorch torchvision torchaudio cpuonly -c pytorch
```

<br>

2. 에러 메시지를 봤을 때 Highway Env 문제는 아닌 것 같고, PyTorch 문제인 것 같아 PyTorch 깃허브 이슈 탭을 뒤졌더니 mkl을 설치해보라고 해서 설치했더니 됐다.

2번까지 해서 해결되긴 했는데, mkl을 설치해서 된건지는 확실하지 않다. (나중에 gpu로 돌렸을 때 되나 확인해봐야 할 듯)

```
conda install -c anaconda mkl
```

➡️ [Calling torch.linalg.cholesky on a CPU tensor requires compiling PyTorch with LAPACK](http://archive.today/7AerN)

<br>

**gymnasium.error.NameNotFound: Environment `merge` doesn't exist.**

새로 생성한 환경을 찾을 수 없다고 할 때

HIGHWAY_ENV/highway_env/\_\_init\_\_.py에 새로 생성한 환경을 아래처럼 등록한 뒤,

```
register(
        id='merge-v1',
        entry_point='highway_env.envs:MergeEnv1',
    )
```

<br>

새로 생성한 환경을 테스트하는 코드에다가 아래 코드를 추가하면 된다.

```
import highway_env

highway_env.register_highway_envs()
```
