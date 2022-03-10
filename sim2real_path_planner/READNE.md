# パッケージ概要
学生ロボコン2021のフィールドにおいて，経路生成，経路追従を統括的に行うパッケージです．

# ノード
## nut_navigation_node
現在のロボットの自己位置と行き先座標リストとそのidを受け取り，ダイクストラ法と台形加速度プロファイルに基づいたロボットの走行経路の決定を行います．また，決定された走行経路の追従制御をモデル予測制御に基づいて実行します．

<br>

### 購読トピック
#### navigation_start ([nut_navigation_msgs / NavigationStart](https://gitlab.com/robopro_nut/2021nhkrobocon/nut_ros_pkg_2021/-/blob/master/nut_navigation_msgs/msg/NavigationStart.msg))
- 経路生成及び経路追従の開始指示
- 経路追従を行いたいときに開始時に一回だけ送信してください．

#### sub_navigation_cancel ([std_msgs / Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html))
- 経路追従の終了指示
- ロボットの走行を停止したい場合に送信してください．
- データの中身はtrueでもfalseでも構いません．

#### now_vel ([geometry_msgs / Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
- ロボットの現在の速度
- ロボット座標系でのx, y, theta の速度を送信してください．
- この情報は経路追従用のMPCの最適化計算に用いられます．

<br>

### 配信トピック

#### nut_navigation/trajectory ([nav_msgs / Path](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html))
- 生成されたロボットの走行経路

#### nut_navigation/prediction_interval ([geometry_msgs / PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html))
- MPC制御器に入力されている予測区間

#### nut_navigation/state ([nut_navigation_msgs / NavigationState](https://gitlab.com/robopro_nut/2021nhkrobocon/nut_ros_pkg_2021/-/blob/master/nut_navigation_msgs/msg/NavigationState.msg))
- 現在のnavigation_nodeの状態

#### cmd_vel ([geometry_msgs / Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
- ロボットの速度司令

#### nut_navigation/boundary_line ([visualization_msgs / Marker](http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html))
- 経路生成に使用するロボットが侵入可能な領域

#### nut_navigation/node_poses ([geometry_msgs / PoseArray](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseArray.html))
- 経路生成に使用するノードリスト（目標点リスト）

<br>

### パラメータ
#### nut_navigation_node/target_poses_filePath (string)
- 経路生成に使用するノードリストCSVファイルのパス

#### nut_navigation_node/robot_radius (float)
- ロボットの最大半径 [m]
- このパラメータは経路生成時に使用されます．



#### nut_navigation_node/tolerance/linear (float)
- 並進成分の許容収束半径 [m]

#### nut_navigation_node/tolerance/angular (float)
- 回転成分の許容収束半径 [rad]

#### nut_navigation_node/max_speed/linear (float)
- 並進成分の最大速度 [m/s]

#### nut_navigation_node/max_speed/angular (float)
- 回転成分の最大速度 [rad/s]

#### nut_navigation_node/max_acceleration/linear (float)
- 並進成分の最大加速度 [m/s^2]

#### nut_navigation_node/max_acceleration/angular (float)
- 回転成分の最大加速度 [rad/s^2]

#### nut_navigation_node/max_jerk/linear (float)
- 並進成分の最大躍度 [m/s^3]

#### nut_navigation_node/max_jerk/angular (float)
- 回転成分の最大躍度 [rad/s^3]
