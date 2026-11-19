# SiGMA

[English](README.md) | [简体中文](README_CN.md)

**Site-Grounded Multi-Modal ISAC Data Generation for Environment-Aware Communications**

SiGMA 是命令行 ISAC 数据生成框架，实现 IEEE WCNC 2027 论文
“SiGMA: Site-Grounded Multi-Modal ISAC Data Generation for Environment-Aware
Communications”中的三阶段数据生成流程：站点重建、同步多模态采集以及无线/RIS/Radar
数据生成。站点、轨迹、传感器、传播参数和输出位置均由 YAML 配置控制。

## 系统架构

| 阶段 | 功能 | 产物 |
| --- | --- | --- |
| I · 站点地图构建 | OSM 道路与主要建筑轮廓、Overture 补充与冲突筛选、统一投影、OpenDRIVE、DTT 程序建筑/外立面/纹理/植被/地形，以及地理、道路和语义检查 | 融合地理输入、构建报告、XODR 及供 Cook/Import 使用的源地图资产 |
| II · 多模态感知采集 | 道路约束轨迹、有序途经点、背景交通、FoV 规则和 CARLA 同步采集 | RGB、米制深度、LiDAR、GNSS/IMU、车辆状态、校准及帧索引 |
| III · 无线与 Radar 生成 | UE–Blender–Sionna 转换、无线材料、冻结动态场景、路径/CSI/Radio Map、synthetic/explicit arrays、RIS 分量与 RCS Radar | 场景 XML/mesh、复信道、路径元数据、CIR 与 FMCW IQ |

Stage II 与 Stage III 共享已保存场景状态，通过站点、轨迹、帧和仿真时间标识关联各模态。
模块来源见 [source_audit.md](docs/source_audit.md)，论文对应关系见
[paper_correspondence.md](docs/paper_correspondence.md)。

## 环境准备

采用 Python 3.10 或 3.11。在项目根目录安装 CLI、地图/采集依赖、测试工具与各后端环境：

```bash
python -m venv .venv
.venv/bin/python -m pip install -e '.[map,sensing,test]'
python -m venv .venv-communication
.venv-communication/bin/python -m pip install -e '.[communication]'
python -m venv .venv-radar
.venv-radar/bin/python -m pip install -e '.[radar]'
source .venv/bin/activate
```

通信和 Radar 分别使用独立后端环境：

| 后端 | 配置的解释器 | 固定运行版本 |
| --- | --- | --- |
| 通信 | `../.venv-communication/bin/python` | Sionna RT 2.0.1、Mitsuba 3.8.0、Dr.Jit 1.3.1 |
| Radar | `../.venv-radar/bin/python` | Sionna RT 2.2.0 |

将解释器路径填写到 `stage3.communication_python` 与 `stage3.radar_python`。
GPU 运行需要兼容驱动；CPU 运行需要 LLVM 与匹配的 C++ runtime。随机种子控制显式随机参数；
外部 UE 程序生成及并行 RT 的逐位复现性待验证。

外部工具与资产：

- 官方 CARLA 0.9.16 server 与匹配的 Python wheel。
- 地图构建/导出所需的 UE 4.26、匹配的 CARLA 源码/内容、CarlaTools、StreetMap 和 Python 编辑器插件。
- Blender 与匹配的 Mitsuba 导出插件；Blender Python 需要 PyYAML。
  本地转换 fixture 已通过 Blender 4.2.10 验证。
- 动态场景组装所需的兼容 actor 模型库。

在所选 YAML 中填写本地资源路径。YAML 内路径相对于该文件，CLI `--output` 相对于调用目录。
在 CARLA 安装目录启动服务：

```bash
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

运行器校验官方 client/server 版本及其一致性。地图准备与导入步骤见
[carla.md](docs/carla.md) 和 [stage1.md](docs/stage1.md)。

## 配置与工作流

| 配置 | 用途 |
| --- | --- |
| [town03.yaml](configs/town03.yaml) | 官方 Town03 感知与信道部署 |
| [ris.yaml](configs/ris.yaml) | 源工程 Arizona RIS 部署与站点构建 |
| [town03_export.yaml](configs/town03_export.yaml) | 独立导出官方 Town03 源地图 |
| [ris_export.yaml](configs/ris_export.yaml) | 独立导出重建后的 RIS 源地图 |
| [rf_materials.yaml](configs/rf_materials.yaml) | 无线材料定义 |

`python -m sigma` 与安装后的 `sigma` 命令提供相同接口。`doctor` 检查依赖与配置的解释器，
`validate` 验证 YAML schema。阶段命令支持 `--config`、`--output` 和 `--seed`。

### 官方 Town03

Town03 配置采用 10 Hz 仿真步进、4 s 采集窗口、随机种子 42、1 辆 CAV 和 29 辆背景车。
RSU 固定在 `[0,0,15]` m，yaw 为 180°。`route.selection: seeded_spawn_points`
基于地图真实 spawn points 与道路拓扑选择确定性的前向路线。
CAV 配置共 8 项，包括 7 个 CARLA 传感器和 1 个虚拟 RCS Radar；
RSU 配置共 17 项，包括 16 个 CARLA 传感器和 1 个虚拟 RCS Radar。
`sionna_rcs` calibration 将这两个 Radar 配置衔接至 Stage III 处理。无线参数为 28 GHz、1 GHz 带宽、256 子载波，
`radio.arrays: [[4,4,8,8]]`；Radar 生成已启用。

准备官方地图、匹配的源地图导出与 actor 库，在同一数据集目录中执行 Stage II 和 III：

```bash
python -m sigma validate -c configs/town03.yaml
python -m sigma doctor -c configs/town03.yaml --connect-carla
python -m sigma stage2 -c configs/town03.yaml -o outputs/town03
python -m sigma stage3 -c configs/town03.yaml -o outputs/town03
```

### 站点 RIS 部署

RIS 配置为 Arizona 部署重建 `SigmaRIS`。在 `assets/local/ris/` 中准备站点的
OSM/Overture/GHS 输入，并在 `ris.yaml` 中填写本地构建资源。
高度补充采用 `overture-ghs`，植被补充采用 `overture-conservative`。

| 部署参数 | 配置值 |
| --- | --- |
| RIS 纬度 / 经度 | `33.417827, -111.926134` |
| RIS 高度 / yaw | 相对本地道路参考 2 m / 180° |
| RIS 面板 | 32×32 元素、半波长间距、0.02 m 厚度 |
| RSU 纬度 / 经度 | `33.41920874636455, -111.92717732926977` |
| RSU 高度 / yaw | 相对本地道路参考 2 m / 0° |
| 载波 / 带宽 / 子载波数 | 60 GHz / 100 MHz / 1 |
| 阵列配置 / `rx_antenna_height_m` | `[[1,1,16,1]]` / 0.3 m |
| 信道 / 相位 / 反射幅度 | `effective` / `zero` / 0.8 |
| 分量 / 路线 | 保存完整分量 / southbound，30 km/h |
| 随机种子 / 仿真步长 / 最大帧数 | 1477 / 0.1 s / 150 |
| 背景交通 / 采集 | 24 辆 / RIS 相机 RGB 与场景状态 |

每次调用采集一条轨迹，后续采集使用新的 `trajectory_id`。
[configuration.md](docs/configuration.md) 记录参数来源与显式确定性设置。
Stage I 按 `stage1.mode` 执行：`prepare` 生成融合地理输入，`build` 进一步构建
OpenDRIVE 与 UE 源地图。

```bash
python -m sigma validate -c configs/ris.yaml
python -m sigma doctor -c configs/ris.yaml
python -m sigma stage1 -c configs/ris.yaml -o outputs/ris
```

使用目标官方 CARLA 0.9.16 的源码、内容和工具对生成地图执行 Cook/Import。
在对应服务器验证地图后，继续采集与信道生成：

```bash
python -m sigma doctor -c configs/ris.yaml --connect-carla
python -m sigma stage2 -c configs/ris.yaml -o outputs/ris
python -m sigma stage3 -c configs/ris.yaml -o outputs/ris
```

`pipeline` 按 I→II→III 执行，并记录 Stage I 地理准备或 Cook 交接状态。
目标地图就绪后从 `stage2` 接续。显式随机种子覆盖与 pipeline 命令为：

```bash
python -m sigma pipeline -c configs/ris.yaml -o outputs/ris --seed 1477
```

各阶段写入 `manifests/stageN.json`。阶段错误记录失败原因，返回非零退出码并保留已完成的中间文件。
添加轨迹时使用新的 `trajectory_id`。Stage III 复用配置、种子和输入内容匹配的结果；
修改场景或配置时使用新输出目录。

### 静态地图导出

独立导出器使用匹配的 UE 源码工程和地图源资产。所提供配置从
`assets/local/town03/Town03.fbx` 和 `assets/local/ris/SigmaRIS.fbx` 读取静态几何。
直接导出到对应资产目录：

```bash
python -m sigma.map_export.cli -c configs/town03_export.yaml -o assets/local/town03
python -m sigma.map_export.cli -c configs/ris_export.yaml -o assets/local/ris
```

导出 manifest 记录 FBX 路径。`stage3.blender.static_scene` 选择静态输入，
数据集输出目录可独立指定。启用 `stage3.map_export` 可将导出集成到 Stage III。
Blender 转换支持 FBX 和 `.blend` 场景，详见 [map_export.md](docs/map_export.md)。
`blender.enabled: false` 可复用数据集已有的 XML。

## 数据目录与状态标识

输出沿用数据协议中的阶段目录名称：

```text
outputs/<site>/
  manifests/stage{1,2,3}.json
  stage1/
  map_export/
  stage1_multimodal/
    sensor_config.yaml
    ris_metadata.yaml
    000001/
      calibration/<node>.yaml
      nodes/<node>/<modality>/<sensor>/*
      nodes/<node>/radar/<prefix>_<frame_idx>.npz
      world_state/world_state_<frame_idx>.yaml
      annotations/camera_bbox2d/...
      sensor_samples.jsonl
      samples.csv
  stage2_xml/
    static/<site>.xml
    000001/dynamic/dynamic_<frame_idx>.xml
  stage3_radio/
    radio_index.yaml
    runtime_signature.json
    000001/{paths,channels,explicit_paths,radio_maps,ris,...}/
    000001/radar_paths/<node>/<sensor>/p_<frame_idx>.npz
  logs/
```

`stage1/` 保存地理输入与建图报告，`stage1_multimodal/` 保存 Stage II 采集，
`stage2_xml/` 保存 Stage III 场景转换结果。

统一状态键为 `site_id`、整数 `trajectory_id`、轨迹内 `frame_idx`、CARLA `world_frame`
及绝对仿真秒 `time_s`。`trajectory_time_s` 记录轨迹相对时间。传感器回调按 world frame/time
精确匹配，缺帧触发错误。CARLA 使用米和度，无线坐标为 `(x,-y,z)`。
校准文件保存传感器安装矩阵、相机内参与输出轴约定。

`sample_every_n_frames` 设置共享采样步长，`world_state_every_n_frames` 为该步长的整数倍。
传感器级 `every_n_samples` 控制各模态的保存间隔，FoV 入场帧作为共同采样起点。

| 模态 | 格式 |
| --- | --- |
| RGB | PNG |
| Depth | float32 米，`.npy` |
| 语义 | 类别标签 NPY |
| LiDAR | 传感器坐标下的 `[x,y,z,intensity]` |
| GNSS/IMU | 同帧传感器测量 JSON |
| Actor 状态 | 冻结的变换、几何、身份及仿真时间 |

数据集一致性检查与索引导出：

```bash
python -m sigma.audit_dataset outputs/ris
python -m CSV_data_export.generate_samples_csv --dataset-root outputs/ris
```

## 无线、RIS 与 Radar 输出

CSI `h` 为 complex64 `[subcarrier,rx_antenna,tx_antenna]`。路径 NPZ 保存复系数、
绝对时延、角度、LoS/路径类型、interaction 顶点、去重 ID 与原始路径记录。
Explicit CIR 维度为 `[rx_antenna,tx_antenna,path]`。

`radio.reconstruction: farfield` 基于中心路径执行 synthetic-array 重建；`nearfield`
配置物理 PlanarArrays，通过 `synthetic_array=False` 逐天线对追踪。
`radio.arrays` 各项顺序为 `[rx_cols,rx_rows,tx_cols,tx_rows]`。
Radio Map 的 `center`、`size` 和 `cell_size` 使用 Sionna 米坐标，输出保存线性
`path_gain`、cell centers、传播设置和共享帧身份。

### RIS

RIS 保存 `h_br[K,Nris,Ntx]`、`h_ru[K,Nrx,Nris]`、direct/cascaded/effective
`[K,Nrx,Ntx]` 及 `phi[K,Nris]`，遵循
`H_effective = H_direct + H_ru diag(phi) H_br`。

启用 `save_full_component_channels: true` 后，可从保存的分量重评相位配置：

```bash
python -m Sionna_channel_storage.rephase_ris \
  --input components.npz --output rephased.npz --phase-mode focus --reflection-amplitude 0.8
```

[experiments/](experiments/) 中的可选用例适配器提供 beam 子集、RIS-only 标签及 grouped-fold
导出。相位码本和固定 RSU beam 通过显式文件输入，见 [experiments.md](docs/experiments.md)。

### Radar

Stage II 写入虚拟 `sionna_rcs` calibration。启用 `stage3.radar.enabled: true` 后，
Stage III 对保存场景运行多散射点目标/RCS 模型，并合成可配置 FMCW IQ。
波形、阵列与 RCS 参数位于 Stage II sensor item。

CIR 包含 `[Nrx,Npaths]` 系数、绝对时延、Doppler、目标与 interaction 元数据。
IQ 为 complex64 `[Nrx,Nadc,Nchirps]`，附带 ADC、chirp slope/repetition、载波频率及帧元数据。
接收端处理可使用已保存路径与波形参数。传感器说明见 [carla.md](docs/carla.md)。Radar 路径保存在
`stage3_radio/<trajectory>/radar_paths/`，IQ 保存在
`stage1_multimodal/<trajectory>/nodes/<node>/radar/`。

## 验证

2026-11-19 回归测试结果为 **124 项通过、0 项失败、0 项跳过**。
Town03、RIS 部署及地理 fixture 的 CLI 配置验证通过，fixture 环境检查和本地
OSM–Overture 地理准备通过，生成 3 栋建筑和 1 条道路。138 个 Python 文件静态编译通过，
原工程源码与部署来源输入的校验和均与记录一致。

运行本地测试与小区域地理 fixture：

```bash
python -m pytest -q
python -m sigma stage1 -c tests/fixtures/stage1/geography.yaml -o /tmp/sigma-geography-smoke
```

在对应 Sionna 环境中分别运行后端检查：

```bash
python tests/backend_smoke.py --output /tmp/sigma-wireless-smoke
python tests/ris_backend_smoke.py --output /tmp/sigma-ris-smoke
PYTHONPATH=. python tests/radar_backend_smoke.py
```

已有验证覆盖本地地理处理、CLI/schema 契约、Blender–Sionna 转换、CPU
信道/Radio Map/RIS/Radar、打包以及 RTX 4090 D GPU Sionna 探测。
官方 CARLA 真实站点构建/Cook、实时同步采集、同站点多模态闭环与完整 1,024 元素 RIS RT
仍待验证。测试证据与资源详情见 [validation.md](docs/validation.md) 和
[remaining_work.md](docs/remaining_work.md)。

## 致谢与许可

感谢 CARLA 团队提供 [Digital Twin Tool](https://carla.readthedocs.io/en/0.9.16/adv_digital_twin/)。
SiGMA 基于其程序化地图构建能力，扩展了 OSM–Overture 地理融合、命令行自动构建、
地理/道路/语义检查、同步感知、无线材料分配以及无线/RIS/Radar 数据生成。

迁移的平台代码保留 [MIT 许可证](LICENSES/platform-MIT.txt)。地图构建与导出代码保留原版权头，
公开发布前需要权利人确认分发授权。CARLA、Unreal、地理数据及其他外部资源适用各自条款。
来源与归属见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)，原工程完整性记录见
[source_integrity.json](docs/source_integrity.json)。
