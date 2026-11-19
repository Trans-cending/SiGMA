# SiGMA

SiGMA 是命令行 ISAC 数据生成框架，对应 WCNC 论文 **“SiGMA: Site-Grounded Multi-Modal ISAC Data Generation for Environment-Aware Communications”**（[main.pdf](main.pdf)）。工程通过 YAML 和确定性参数控制站点重建、多模态采集、无线信道及 Radar 数据生成，源码来源见 [迁移审计](docs/source_audit.md)。

当前交付范围为三阶段数据生成。源码迁移、CLI、数值回归、小场景 Blender–Sionna 后端及服务器 GPU 验证已完成；官方 CARLA 0.9.16 的真实站点构建、Cook、同步采集和同站点全链路验收待完成。详见 [功能对应表](docs/paper_correspondence.md)、[验收报告](docs/validation.md) 和 [工作清单](docs/remaining_work.md)。

## 三阶段

| 阶段 | 实现 | 主要产物 |
| --- | --- | --- |
| I：Site-Grounded Map Construction | OSM 道路与建筑优先、Overture 补充、空间冲突筛选、坐标统一、OpenDRIVE、程序建筑、外立面、纹理、植被、地形和语义检查 | 地理输入、构建审计、XODR、待 Cook 的源地图 |
| II：Multi-Modal Sensing Acquisition | 官方 CARLA 同步 tick、预定义道路轨迹、有序 via、背景交通、FoV、RGB/Depth/LiDAR/GNSS/IMU、车辆状态与 bbox | 帧级传感器数据、校准、world_state 和索引 |
| III：Wireless and Radar Data Generation | UE–Blender–Sionna 转换、冻结动态场景、路径、CSI、Radio Map、synthetic/explicit arrays、RIS 分量与级联、RCS Radar CIR/FMCW IQ | XML/mesh、复信道、路径、IQ 和元数据 |

用户入口统一为 CLI。底层 `mosaic_core` 包名、部分 Unreal 类型和地理标签沿用原技术标识，以保持数据与资产协议。

## 环境

基础环境采用 Python 3.10 或 3.11。在项目目录中安装：

```bash
cd /home/ybianaf/David_PROGRAM/Python/SiGMA
python -m venv .venv
.venv/bin/python -m pip install -e '.[map,sensing,test]'
.venv/bin/python -m sigma doctor -c configs/site.example.yaml
.venv/bin/python -m sigma validate -c configs/site.example.yaml
```

通信与 Radar 使用各自独立环境：

- 通信：`pip install -e '.[communication]'`，Sionna RT 2.0.1 / Mitsuba 3.8.0 / Dr.Jit 1.3.1。
- Radar：`pip install -e '.[radar]'`，Sionna RT 2.2.0。

把两个解释器路径分别填入 `stage3.communication_python` 和 `stage3.radar_python`。运行环境需提供 GPU 驱动或 CPU LLVM 动态库。随机种子控制显式参数；外部 UE 程序生成及并行 RT 的逐位复现性待验证。

其他外部依赖包括官方 CARLA 0.9.16 server/wheel、匹配的 UE 4.26 与 CARLA 内容、CarlaTools/StreetMap/Python 编辑器插件、Blender 及匹配的 `mitsuba-blender` 插件。Blender 内置 Python 需支持 PyYAML。各项资源按其许可证独立取得。

在 CARLA 安装目录启动服务：

```bash
./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000
```

回到 SiGMA 目录检查连接：

```bash
python -m sigma doctor -c configs/site.example.yaml --connect-carla
```

运行器校验官方版本和 client/server 一致性。自定义地图需使用目标 0.9.16 的源码、内容和插件重新生成、Cook 并导入。标准 API、扩展及操作步骤见 [CARLA 说明](docs/carla.md) 与 [Stage I 构建说明](docs/stage1.md)。

## 配置与运行

复制 [site.example.yaml](configs/site.example.yaml)，填写地理输入、地图、道路起终点、RSU 和外部资源路径。示例坐标用于展示配置结构，真实站点坐标由部署者填写。YAML 内路径相对于该文件，`--output` 相对于调用目录。实验协议 YAML 用于记录参数；pipeline 使用站点配置。

```bash
python -m sigma validate --config configs/site.example.yaml
python -m sigma stage1 --config configs/site.example.yaml --output outputs/example_site --seed 42
python -m sigma stage2 --config configs/site.example.yaml --output outputs/example_site --seed 42
python -m sigma stage3 --config configs/site.example.yaml --output outputs/example_site --seed 42
python -m sigma pipeline --config configs/site.example.yaml --output outputs/example_site --seed 42
```

安装后也可使用 `sigma` console script。`stage1.mode: prepare` 生成 OSM–Overture 融合输入；`build` 进一步执行 OpenDRIVE 与 UE 重建。`pipeline` 按 I→II→III 调用，在地理准备或地图 Cook 交接处返回明确状态。Cook/Import 采用官方 CARLA 工具；地图加载就绪后继续执行 `stage2` 和 `stage3`。

本地小区域测试使用构造的地理 fixture：

```bash
python -m sigma stage1 -c configs/geography-smoke.yaml -o /tmp/sigma-geography-smoke
```

Blender 转换测试使用 Blender 4.2.10 与匹配的 Mitsuba 插件。设置 `blender.executable` 指向对应版本，通过 `BLENDER_USER_SCRIPTS` 提供插件。

每阶段结果写入 `manifests/stageN.json`。阶段失败返回非零退出码，并保存错误、日志及已完成的中间文件。Stage II 以新的 `trajectory_id` 添加轨迹。Stage III 根据配置、种子和输入内容复用结果；修改场景或配置时使用新输出目录。

Stage III 支持 `blender.static_scene: rebuilt_map.fbx` 或 `.blend`，也支持通过 `stage3.map_export` 调用 UE 静态 FBX 导出器，参数见 [导出说明](docs/map_export.md)。`actor_assets` 指向与官方 CARLA 0.9.16 对齐的车辆等模型库。已有 XML 可通过 `blender.enabled: false` 复用。

`radio.reconstruction: farfield` 复用中心几何路径进行 synthetic-array 重建；`nearfield` 配置实际 PlanarArrays，以 `synthetic_array=False` 逐天线追踪。`radio.arrays` 顺序为 `[rx_cols,rx_rows,tx_cols,tx_rows]`。Radio Map 的 `center`、`size`、`cell_size` 使用 Sionna 米坐标，并共享冻结帧与传播设置。

## 数据格式

```text
outputs/<site>/
  manifests/stage{1,2,3}.json
  stage1/                         # Stage I：地理输入与地图构建审计
  map_export/                     # 可选 UE 静态导出
  stage1_multimodal/               # Stage II；沿用原数据协议目录名
    sensor_config.yaml
    ris_metadata.yaml
    000001/
      calibration/<node>.yaml
      nodes/<node>/<modality>/<sensor>/*
      world_state/world_state_<frame_idx>.yaml
      annotations/camera_bbox2d/...
      sensor_samples.jsonl
      samples.csv
  stage2_xml/                      # Stage III：Blender 转换
    static/<site>.xml
    000001/dynamic/dynamic_<frame_idx>.xml
  stage3_radio/
    radio_index.yaml
    runtime_signature.json
    000001/{paths,channels,explicit_paths,radio_maps,ris,...}/
  logs/
```

统一状态键为 `site_id`、整数 `trajectory_id`、轨迹内 `frame_idx`、CARLA `world_frame` 和绝对仿真秒 `time_s`；相对时间另存 `trajectory_time_s`。传感器回调按相同 world frame/timestamp 匹配，缺帧触发错误。CARLA 使用米/度，无线空间坐标为 `(x,-y,z)`。安装矩阵、相机 intrinsics 和输出轴顺序保存在校准及索引内。

RGB 为 PNG；Depth 为 float32 米 `.npy`；语义为标签 NPY；LiDAR 为传感器坐标下的 `[x,y,z,intensity]`；GNSS/IMU 为 JSON。GNSS 来源为同帧传感器测量。

CSI `h` 为 complex64 `[subcarrier,rx_antenna,tx_antenna]`。路径 NPZ 保存复 `coefficient`、绝对 `delay_s`、角度、LoS/路径类型、interaction 顶点、去重标识和原始路径。Explicit CIR 为 `[rx_antenna,tx_antenna,path]`。Radio Map 保存线性 `path_gain`、cell centers、网格、传播设置和共享帧身份。

RIS 保存 `h_br[K,Nris,Ntx]`、`h_ru[K,Nrx,Nris]`、direct/cascade/effective `[K,Nrx,Ntx]` 和 `phi[K,Nris]`，遵循 `H_direct + H_ru diag(phi) H_br`。启用 `save_full_component_channels: true` 后可从已保存分量重评相位：

```bash
python -m Sionna_channel_storage.rephase_ris \
  --input components.npz --output rephased.npz --phase-mode focus
```

Radar 使用虚拟 `sionna_rcs` calibration，Stage III 根据同帧场景运行多散射点目标/RCS 求解。`stage3.radar.enabled: true` 启用后处理；波形、阵列和 RCS 参数放在 Stage II sensor item。CIR 保存 `[Nrx,Npaths]` 系数、绝对时延、Doppler、目标与 interaction 元数据；IQ 为 complex64 `[Nrx,Nadc,Nchirps]`，附 ADC、chirp slope、repetition、频率和帧元数据。详见 [传感器配置](docs/carla.md)。

```bash
python -m sigma.audit_dataset outputs/example_site
python -m CSV_data_export.generate_samples_csv --dataset-root outputs/example_site
```

## 数据导出与验证

[experiments/](experiments/) 提供可追溯用例参数、32/64 beam 子集适配、RIS-only label 与 grouped-fold 导出接口。角域码本与固定 RSU beam 通过显式文件输入，详见 [数据导出说明](docs/experiments.md)。

```bash
python -m pytest -q
python tests/backend_smoke.py --output /tmp/sigma-wireless-smoke
python tests/ris_backend_smoke.py --output /tmp/sigma-ris-smoke
PYTHONPATH=. python tests/radar_backend_smoke.py
```

后面三个命令分别使用对应 Sionna 环境。CPU 需配置 LLVM/libstdc++；GPU 需驱动。小场景后端、数值测试和真实站点验收状态分别记录在 [验收报告](docs/validation.md)。

## 来源和授权

平台代码沿用 [MIT 许可证](LICENSES/platform-MIT.txt)。地图构建与导出子项目保留原版权头，公开发布前需权利人确认分发授权。CARLA、Unreal、地理数据及其他外部资源适用各自条款，见 [第三方与来源说明](THIRD_PARTY_NOTICES.md)。原工程只读校验记录见 [source_integrity.json](docs/source_integrity.json)；当前交付保存在本地。
# SiGMA
