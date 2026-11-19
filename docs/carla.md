# CARLA 0.9.16 与 Stage II

目标是官方 CARLA 0.9.16（UE 4.26 分支），Python 3.10–3.12。官方发布提供这些版本的 wheel；SiGMA 通过独立环境安装匹配版本的 wheel。[官方发布说明](https://carla.org/2025/09/16/release-0.9.16/)、[官方构建指南](https://carla.readthedocs.io/en/0.9.16/build_linux/)

本迁移只读审计原 `CARLA_sensor_storage`。原入口依赖 `trajectory_generator`、`carla_session`、`scenario_geometry`、`collection_manager`、`sensor_manager`、`traffic_actor_manager`、`annotation_manager`、`frame_sync` 和数据写入工具。迁移按确定性采集调用链提取模块。`sigma/sensing` 提取原 A*、ordered-via、相机投影、bbox/corner FoV 与边缘入场计算、严格帧检查、地理变换和 world-state payload 构造（原文件哈希见 [stage2_sources.json](stage2_sources.json)）；以普通函数/类连接新的官方 API 运行器。

| 原实现类别 | 官方兼容实现与处理 |
| --- | --- |
| `carla.Client` / `World` / `Map` / Blueprints | 保留标准 API；连接后先检验 client/server 版本一致，再加载世界。 |
| 环境中自动追加 PythonAPI 路径 | 通过独立环境安装匹配 wheel。 |
| 外部脚本/集群日志自动发现、重启服务 | 用户明确启动服务器；运行状态写入 manifest，并保留已经写入的数据。 |
| 同步世界、Traffic Manager、随机种子 | 单一 `world.tick()`；世界与 TM 同步；固定步长和物理子步；退出时恢复世界设置。 |
| 预定义路由与道路可达性 | 提取原 bounded A*、方向一致的终点车道选择和 ordered-via 精确拼接；沿道路行驶方向的 forward 拓扑生成路径，并向官方 TM 下发方向一致的路线。 |
| 动态交通 | 标准 vehicle blueprints、TM autopilot、`set_path`、`auto_lane_change(False)` 与 `set_desired_speed`。实际车辆数需达到请求数量，以保持配置的动态遮挡条件；数量不足触发错误。 |
| RGB / Depth / LiDAR / GNSS / IMU | 标准 camera、ray_cast、gnss、imu blueprints；每个请求属性在 spawn 前验证。|
| 部分定制版本的 actor 属性和 unsubscribe 崩溃回避 | 按官方 sensor.stop()/destroy() 生命周期处理本次创建的 actor。 |
| 自定义 static mesh、灯光、窗户及运行时插件接口 | 静态细节在 Stage I 的官方 UE 工程中构建/cook。Stage II 使用标准接口；RSU/RIS 作为固定地理参考和相机挂载节点。 |
| Radar IQ 的旧仿真后端 | Stage II 写入 `radar_backend: sionna_rcs` 虚拟雷达 calibration；Stage III 从同帧场景求 RCS CIR 和 FMCW IQ。 |
| 原 Depth 灰度图 / 语义调色板 | 新 schema 保存 metric depth float32 NPY 和原始 semantic tag uint8 NPY，保留深度精度和类别 ID；RGB 为 PNG。 |

标准调用已逐项核对 [0.9.16 Python API](https://carla.readthedocs.io/en/0.9.16/python_api/)。世界和 TM 的同步关系、物理子步约束遵循 [同步文档](https://carla.readthedocs.io/en/0.9.16/adv_synchrony_timestep/)。

## 安装与启动

在 Python 3.10–3.12 环境安装 `carla==0.9.16` 及本项目 sensing 依赖。服务器需要 GPU 和匹配的官方 0.9.16 数据包。启动命令由用户在 CARLA 安装目录执行：

```bash
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

`-RenderOffScreen` 保留传感器渲染。SiGMA 设置 `no_rendering_mode=False`，启用相机渲染。同步世界由单一客户端推进，snapshot.frame 必须与当前 tick 一致。client/server 版本要求为相同的 `0.9.16` 标签，或在 wheel metadata 同为 `0.9.16` 时使用相同的官方干净 release commit `294096e…`。[官方 release commit](https://github.com/carla-simulator/carla/releases/tag/0.9.16)

```bash
python -m sigma doctor -c configs/town03.yaml --connect-carla
python -m sigma validate -c configs/town03.yaml
python -m sigma stage2 -c configs/town03.yaml --output outputs/town03 --seed 42
```

这些是实际 CLI 接口。运行前应按 [配置来源说明](configuration.md) 设置站点地图、资产和解释器路径。

## 自定义地图必须重新构建

保留原始地理数据、OpenDRIVE、FBX/纹理及坐标参考，在官方 0.9.16 的 CARLA UE 4.26 工程重新导入、设置碰撞、语义/材料并 cook。通过实际 client/server 版本及构建来源确认目标环境。

官方标准 map package 含同名 FBX 与 XODR、纹理，以及 `maps` / `props` JSON 描述。该常规导入可重建静态网格；Stage I 程序建筑、外立面、道路、植被与材质的 UE 构建还需使用本项目保留的构建脚本与其明确的补丁前置条件。[地图包结构](https://carla.readthedocs.io/en/0.9.16/tuto_M_manual_map_package/)

```text
Import/SiGMASite/
  SiGMASite.json
  SiteMap/SiteMap.fbx
  SiteMap/SiteMap.xodr
  SiteMap/<textures>
```

源代码构建使用官方对应版本的导入流程；二进制发行包使用官方 `docker_tools.py` cook 包，再在该发行包根目录运行 `./ImportAssets.sh`。这些步骤使用 CARLA 外部工具。[二进制地图导入](https://carla.readthedocs.io/en/0.9.16/tuto_M_add_map_package/)

配置 `stage2.map: {name: SiteMap, assets_version: '0.9.16'}` 表示用户已完成官方重导入；运行器仍检查服务器可用地图。真实 Cook 与加载结果纳入部署验收。

显式 `stage2.map.opendrive: site.xodr` 使用官方 `generate_opendrive_world`，输入是文件内容，生成可行驶道路和人行道。此模式用于道路 smoke test。完整 SiGMA 场景采用 Stage I 重建的建筑、纹理和地理环境，并通过命名地图加载。

## Town03 路由选择

`configs/town03.yaml` 采用 `route.selection: seeded_spawn_points`。运行器从
实际地图的 Driving spawn points 中按种子排序，通过已有 forward 道路规划器
逐个验证起止组合；`max_route_candidates` 默认 64。`route.json` 保存地图原始
spawn 索引、选择种子、实际起止坐标、候选次数及道路 waypoints。

`configs/ris.yaml` 采用 `route.selection: explicit`，通过地理坐标指定起点、终点
与有序 `via`。两种模式共享道路可达性检查。配置来源与资源路径见
[configuration.md](configuration.md)。

## YAML 与数据契约

配置字段以 `sigma/sensing/config.py` 与 `runner.py` 为准。RSU 可给 `location: [x,y,z]` 或 `geolocation: {latitude,longitude,altitude}`；`rotation` 为 `[pitch,yaw,roll]`（度）。route 的 `start`、`destination` 和有序 `via` 可给 `x/y/z` 或 `geolocation`。静态位置与路线必须共享同一个站点参考。

`georeference_y_south: true` 明确启用源站点地图的 tmerc 东向 X/南向 Y 约定和真实 GNSS 测量纠正；CARLA Town 地图保留原生 GNSS。无线坐标统一为 `(x,-y,z)`。路线点投影需位于道路容差内，路段之间需连通；运行车辆的位置需符合道路与预定义路线约束。

`sensors.cav/rsu/ris.items` 使用原确定性 sensor item：`name`、`type`、blueprint 属性、`transform.location/rotation`。`ris` 相机需显式配置，通过独立 `ris_001` 节点完整旋转矩阵挂载。顶级 `ris` 配置由源 `mosaic_core.ris` 普通算法验证。`z_mode: absolute|relative_to_ground` 保留原实现：后者以最近 Driving 车道的高程加配置高度计算，要求存在参考道路。

`sample_every_n_frames` 指定统一采样步长，`world_state_every_n_frames` 必须为其整数倍。`every_n_samples` 指定各传感器保存间隔；sensor_tick 固定为 0，让 callback 精确对应 world tick，并在保存时按该间隔采样。FoV 开始帧作为所有采样节奏共同原点。`sensor_samples.jsonl` 独立索引各模态；world-state 与无线结果仅在配置的 state cadence 下生成。默认两者 stride 均为 1。

`fov.enabled` 开启保留的 bbox top/center/corner 投影入场与离场规则，`node_kind: rsu|ris` 选择相机节点。支持 `require_entry_edge`、`entry_side`、`motion_direction`、`require_entry_heading_alignment`、`entry_heading_min_dot`、`entry_timeout_frames`、`stop_after_exit_frames`、`margin_px` 及源 future-motion 参数。关闭 FoV 时沿预定义路线采集，直至 destination 或 max_frames。采样帧必须与 sensor callback 的 frame/timestamp 精确匹配，差异触发错误。

```text
stage1_multimodal/
  classes.yaml
  sensor_config.yaml
  ris_metadata.yaml                      # 配置 RIS 时
  000001/
    manifest.json                         # running / complete / failed
    route.json
    trajectory_meta.yaml
    object_registry.yaml
    samples.csv
    sensor_samples.jsonl
    calibration/{cav_001,rsu_001,ris_001}.yaml
    nodes/<node>/<modality>/<sensor>/*
    nodes/<node>/pose/pose_<frame_idx>.json
    world_state/world_state_<frame_idx>.yaml
    annotations/camera_bbox2d/<node>/<camera>/bbox.jsonl
```

world_state 保留 `schema_version=1`、`site_id`、整数 `trajectory_id`、`frame_idx`、`world_frame`、真实仿真绝对秒 `timestamp` / `time_s`（与 `sensor_timestamp` 一致）和独立相对秒 `trajectory_time_s`；`nodes.rsu/cav/ris` 和 `scene.vehicles` 使用源无线转换器的结构。每个输出文件由 sensor receipt 保留 site/trajectory/world-frame/time 和文件路径。CAV 的 actor transform/bbox 是几何真值；GNSS 来自同帧真实 sensor 测量。

calibration 的 `sensors` 是以 sensor name 为键的映射，包含 `type`、`config`、坐标变换、intrinsic 和虚拟 Radar 标记。RIS metadata 保存分量求解所需的 element world coordinates；RIS 通过分量信道建模。相机 bbox 标注提供投影框、YOLO 归一化框、稳定 object ID 与目标 actor 对应；`visibility` 定义为图像裁剪比例；遮挡真值需独立计算。

## 验证记录

单元测试执行 `python -m pytest -q tests/test_stage2.py tests/test_stage2_route_selection.py`。它验证精确帧、采样原点、Depth 解码、tmerc 往返、真实 GNSS 约束、版本一致性、自定义地图校验、OpenDRIVE 内容传入、同步设置、ordered-via 可达性、路线方向检查、FoV 轴向，以及 Stage II 虚拟雷达校准到 Stage III pose 的数值衔接。

已核实的官方 Python 3.10.20 环境返回 client version 0.9.16；官方 server、
UE 4.26、编译后的 DTT bridge 和外部 actor 库均已定位。SLURM GPU 探测在
RTX 4090 D 上运行 Sionna 成功，详细路径与证据见 [资源清单](remaining_work.md)。

官方 server 实际启动、地图 Cook/加载、真实短轨迹、传感器同步与 live FoV
仍待执行。源码树中的历史补丁需要按 DTT 构建需求审计；现有源码 fixture 和
数值测试的验收范围为本节列出的独立契约。
