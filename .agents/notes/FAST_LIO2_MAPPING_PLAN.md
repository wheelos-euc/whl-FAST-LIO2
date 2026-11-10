# FAST-LIO2 建图规范化计划

**目标仓库：** `/home/wfh/01code/whl-FAST-LIO2`
**关联项目：** `/home/wfh/01code/map`
**文档性质：** 实施计划；本文件不代表下述阶段已完成。

## 1. 目标与原则

将 `whl-FAST-LIO2` 规范化为一个可通过 Apollo Cyber `.record` 离线回放或 Apollo 在线话题运行的建图生产者，并将结果交给当前项目已有的地图处理、评估和可视化流程。

核心原则：

1. 离线 `.record` 与在线 Apollo 话题共用同一个 Cyber 组件、输入契约和 FAST-LIO2 核心，不维护两套估计器。
2. 以 `optimized_frame_dataset` 作为 FAST-LIO2 和地图下游之间的稳定边界；运行时话题和拼接 PCD 是观测/辅助产物，不代替规范化数据集。
3. 明确时间戳、单位、坐标系、外参方向和点时间语义；未经验证的输入不得靠默认值伪装成有效建图。
4. 将输入有效性、算法运行健康、轨迹质量、地图几何和地图产品一致性分开验收。
5. 从静态检查和短段回放逐级验证。未过前置门槛，不启动长记录或全量昂贵回放。

## 2. 当前基线与待补齐工作

### 已有基础

- `modules/fast_lio2` 已包含 Cyber 组件、Apollo 点云/IMU 适配、FAST-LIO2 runtime core、DAG、配置和 Bazel 构建入口。
- `tools/prepare_apollo_host_run.py` 可生成 host replay 所需配置和 DAG，并支持 `cyber_recorder` 回放命令。
- 组件已有位姿、里程计、运行指标、注册点云和局部地图等运行时输出。
- `fast_lio2/map_infra` 已定义 `optimized_frame_dataset` 与地图产品的契约；数据集指纹、地图坐标系和地图原点的一致性已有验证逻辑。
- 当前项目已有 ROS1 FAST-LIO 流程与数据导出/评估工具，可作为地图下游能力和流程先例；ROS1 bag 与 Apollo Cyber `.record` 是不同输入格式，不应混为同一播放实现。

### 不能视为已完成的部分

- 组件能够启动、订阅并发布点云，不等于已完成离线建图产物交付。
- 目前的地图契约不等于已实现从每帧处理结果自动生成 `optimized_frame_dataset`、瓦片和定位地图包的端到端流水线。
- 已有 Cyber 回放基线曾被审计为偏向 LiDAR-only/初始化阶段，不能直接作为 IMU 融合 FAST-LIO2 等价性或长期地图质量的证明。
- 实际代码路径、初始化状态、传感器输入语义、局部地图管理和输出坐标语义必须针对当前构建版本重新验证，不能仅凭 README、历史记录或“成功启动”判断。

## 3. 目标数据流

```text
Apollo Cyber .record ── cyber_recorder play ──┐
                                              ├─ Apollo 传感器适配器
Apollo 在线 LiDAR / IMU topics ──────────────┘
                                                   │
                                                   ▼
                                      统一 FAST-LIO2 Cyber 组件
                                      同步 / 初始化 / 去畸变
                                      scan-to-map / 局部地图更新
                                                   │
                           ┌───────────────────────┴──────────────────────┐
                           ▼                                              ▼
             运行时 pose / odometry / metrics                 逐帧规范化数据集
             registered cloud / local map            optimized_frame_dataset
                                                                      │
                                                                      ▼
                                                      当前项目地图下游与评估
                                                                      │
                                      tile_bundle / localization_map_package
                                      / map_product_bundle manifest
```

首期 `.record` 入口使用 Cyber recorder 发布原有 Apollo 消息，由现有 reader 消费；不另写绕过 Cyber 组件的文件解析器。在线与离线的差异限于播放速度、队列容量、退出/落盘策略和运行监控，不能改变传感器消息解释。

ROS1 bag 如需纳入 `whl-FAST-LIO2`，另行设计 ROS1-to-canonical-input bridge 或格式转换工具；不属于 Apollo `.record` 与在线 Apollo 双入口的首期范围。

## 4. 输入与坐标契约

### LiDAR

- 输入为 Apollo `PointCloud`，点坐标有限、单位为米，坐标处于声明的 LiDAR frame。
- 点时间必须按选定驱动的真实契约解释；验证 epoch/unit、单帧时间范围、单调性及其与 IMU 时间轴的重叠。不得将点时间戳单位或 epoch 泛化到未经验证的驱动。
- 保留与时间同步有关的传感器时间；接收时间只允许在明确配置的模式中使用，且必须记录模式和依据。
- 对每帧记录输入/过滤点数、时间跨度、拒绝原因和源 frame。

### IMU

- FAST-LIO2 输入必须是 IMU 机体系下的原始角速度（rad/s）和比力（m/s²），并与 LiDAR 使用同一传感器时间轴。
- `CorrectedImu` 仅在上游 producer 的 body-frame、单位和 sensor-time 语义经过实证后启用；配置确认是操作员声明，不能代替证据。
- Raw GNSS IMU 的时间字段优先级须按实际消息定义和数据核实；GPS time、header time 和 receive time 不可混用。
- 检查有限性、单调性、频率、与 LiDAR 的覆盖和初始化所需运动/静止条件；统计丢弃与队列溢出。

### 外参及输出 frame

- 配置契约固定为 `p_imu = R_li * p_lidar + t_li`。
- 必须显式提供已验证的 LiDAR-to-IMU 外参。缺失、部分设置、非法 quaternion 或不匹配时预检失败；不允许无提示地使用 identity。
- 明确 FAST-LIO2 输出是局部 `map` frame 还是已对齐 Apollo 全局地图 frame。局部里程计不得标记或消费为全局 Apollo localization。
- `optimized_frame_dataset` 的每帧 pose 表示帧局部坐标到统一 map frame 的变换；PCD 保存帧局部去畸变点，禁止先变换到世界坐标后再由下游二次应用 pose。

## 5. 分阶段实施与退出门槛

| 阶段 | 实施内容 | 退出门槛 |
|---|---|---|
| **0. 冻结输入/产物契约** | 对支持的 record 和在线 topics 盘点消息类型、频率、时间戳、点字段/点时间、IMU 单位/frame、静态 TF/外参和参考轨迹。为每种驱动记录配置来源和适用范围。定义数据集 schema、map frame、origin 和 manifest。 | 每种首期输入均有可复现的传感器契约和配置；关键字段未知、外参缺失或时间语义不一致会明确阻断，而不是生成成功形状的结果。 |
| **1. 核验核心算法路径** | 针对实际 Bazel 构建的 FAST-LIO2 代码核验初始化、IMU 传播、去畸变、scan-to-map、地图插入/裁剪、失败恢复和退出处理。检查算法审计中记录的差异，确认当前代码的实际行为。 | 短段能证明处理进入预期 IMU 融合与 scan-to-map 阶段；关键状态、拒绝原因、帧计数、残差和地图更新可观测。只启动成功或只发布点云不算通过。 |
| **2. 统一 Apollo 双入口** | 用 `cyber_recorder play` 对 `.record` 回放；在线模式订阅相同的 Apollo topic。将共同的解码、校验、时间和 frame 转换放在相同适配器/契约边界。提供明确的离线/在线队列、回放速度、停止与输出目录配置。 | 两种入口对同一消息的语义与配置一致；短段回放具有有效且单调的输出时间、预期同步覆盖率，并能记录丢帧、拒绝原因和 IMU 覆盖。 |
| **3. 实现逐帧规范化数据集写出** | 在每帧处理结果处持久化 sensor timestamp、map-frame pose、帧局部去畸变点云和 diagnostics。采用 `optimized_frame_dataset/manifest.json`、`frames.csv`、`diagnostics.csv`、`pcd/...`。写入过程要可检测失败，避免静默丢帧或留下看似完整的半成品。 | 帧索引唯一；每条 frame 记录与 PCD 一一对应；时间、点数、pose、frame 语义和 schema 校验通过；异常退出有明确的不完整状态，不标记为成功产物。 |
| **4. 接入现有地图下游** | 通过薄适配层将规范化数据集交给当前项目已有的地图生成、轨迹评估和可视化工具。由同一数据集生成瓦片、定位地图和 map-product bundle；不另建与当前项目并行的地图流水线。 | 各产品引用同一 `source_dataset_fingerprint`、`map_frame`、`map_origin_id`；产品合同验证通过；抽样验证变换没有重复应用，源帧可追溯。 |
| **5. 分级回归与质量验收** | 按“单测/合成数据 → 短段 Apollo record → 含转弯/重访的代表数据 → 长记录或性能测试”执行。逐次记录源 revision/dirty diff、配置 hash、输入 hash、命令、日志和结果。 | 输入有效性、运行健康、轨迹、地图几何和产品一致性分别结论明确；所有门槛以预先约定的指标与阈值判定。只有短段接线通过时，结论只能是 smoke pass。 |

## 6. 数据集及地图产品

推荐交付目录：

```text
run_output/
  input_audit.json
  runtime/
    fast_lio2_metrics.csv
    runtime.log
  trajectory/
    fast_lio2_pose.csv
  optimized_frame_dataset/
    manifest.json
    frames.csv
    diagnostics.csv
    diagnostics/
    pcd/<bucket>/<frame_index>.pcd
  tile_bundle/
  localization_map_package/
  map_product_bundle/
    manifest.json
  scorecard.json
  run_manifest.json
```

`frames.csv` 至少应包含稳定帧索引、传感器时间、PCD 相对路径、map-frame origin pose、点数和有效性状态。PCD 内容应采用 `frame_local` 坐标；强度等保留字段需在 schema 中声明。`manifest.json` 记录 schema/version、producer/source、输入 fingerprint、map frame、map origin、配置与外参、frame 数和产物完整性信息。

拼接 PCD 可作为浏览或兼容产物，但不能取代逐帧数据集：逐帧来源、时间、位姿和诊断是重新渲染、几何评估及追查坏帧所必需的信息。

## 7. 验收与证据

验收分为五个互不替代的维度：

1. **输入有效性：** topic/schema、有效点比例、点时间跨度及单调性、IMU 频率/单位/frame、时间重叠、外参完整性。
2. **运行健康：** 核心阶段、成功/拒绝帧数、IMU/LiDAR 丢失与队列溢出、有效特征、残差、scan-match 状态、地图点数、处理延迟。
3. **轨迹：** 时间连续性、速度/跳变、path length、适用的 ATE/RPE 和闭环/重访一致性。说明对齐方法、区间和参考源质量。
4. **地图几何：** 重复结构偏移、墙/杆/路缘厚度、重访重影、局部残差分布、动态物体污染。不能只看截图或平均残差。
5. **地图产品：** 数据集 schema、帧/点云完整性、fingerprint/frame/origin 一致性和源数据 lineage。

Apollo Localization/GNSS 或其他估计器只有在独立性及质量得到证明后才能用作准确度真值。否则结果标注为 reference-relative 或 inter-estimator consistency，不报告为绝对精度。GNSS 因子启用需有独立质量门槛，不以有效 fix 标志单独放行。

## 8. 风险及处理

- **回放接线成功但算法未进入融合阶段：** 指标必须报告状态机阶段；将初始化和有效 scan-to-map 作为门槛。
- **IMU/点云时间轴或单位错误：** 在估计器启动前运行 schema/quality audit 与 runtime-input gate，并留存报告。
- **Corrected IMU 语义未知：** 默认阻断该输入类型，直到 producer frame、单位与 sensor-time 得到核验。
- **外参错误或缺失：** 使用明确的变换方向和完整性检查；禁止静默 identity。
- **历史算法行为与 upstream FAST-LIO 不一致：** 对当前构建版本做实现审阅和长程地图管理检查，不能将 upstream 文档直接视为本地实现证据。
- **积压导致离线 record 丢帧或在线延迟：** 把离线吞吐与在线实时性分开验收；同时记录处理延迟和队列丢弃，不以加大队列掩盖长期算力不足。
- **只写累计地图而丢失逐帧 provenance：** 规范化数据集为必需交付，累计 PCD 降为派生物。
- **地图/定位产品坐标不一致：** 产品必须由同一数据集派生，并以 fingerprint、frame 和 origin 做强校验。
- **ROS1 bag 与 Apollo record 混淆：** 首期仅定义 Apollo record/话题；ROS1 输入桥接另立范围并单独验证。

## 9. 推荐实施顺序

1. 将本文件对应的输入契约、数据集 schema 和验收指标整理为可执行检查项。
2. 检查当前代码与最近基线，跑纯单测/契约测试，不回放长记录。
3. 选一个已有短 Apollo record，完成输入审计、初始化/算法阶段核验和 Cyber replay smoke。
4. 实现并测试逐帧 writer，验证 PCD/pose 对应、时间关联和失败状态。
5. 打通一个 FAST-LIO2 `optimized_frame_dataset` 到当前项目地图下游的端到端短流程。
6. 用代表性重访数据评估轨迹与地图几何，明确通过、失败或证据不足及原因。
7. 前置门槛通过且获得运行授权后，才进行长 record、吞吐/延迟和资源评估。

## 10. 暂不纳入首期范围

- FAST-LIO2 回环检测或完整全局 pose-graph/BA；FAST-LIO2 局部里程计并不自动保证长程全局一致性。
- 将局部 FAST-LIO2 pose 直接转换为 Apollo 全局定位。
- 新建 ROS1 bag 原生 reader 或 ROS1 runtime。
- 未经单独方案和验收的 GNSS 因子融合、在线定位、生产级 map tile manager。
- 未经批准的长时段/全量数据运行或参数 sweep。
