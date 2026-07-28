# QMI8658B Shell 测试用例

模块路径：`ext_module/QMI8658B`

测试命令实现：`src/my_shell.c`

数据手册：`QMI8658B/QMI8658B-Datasheet.pdf`

## 1. 测试准备

1. 确认 QMI8658B 的 I2C 地址为 `0x6B`，并已连接到设备树别名 `gsensor_i2c`。
2. 确认 QMI8658B INT1 已连接到设备树别名 `gsensor_int`，INT1 空闲电平及中断极性应符合板级硬件设计。
3. 打开 `src/my_shell.c`，将宏 `QMI8658B_SHELL_TEST_ENABLE` 改为 `1`；测试结束后恢复为 `0`，Shell 中将不再注册 `qmi8658b` 命令。
4. 烧录启用 Shell 的固件，通过 RTT Shell 连接设备，输入 `app qmi8658b` 可查看命令摘要。

测试期间请保持传感器供电稳定。`cali` 与 `selftest` 会暂时改变芯片内部工作状态，执行前应停止依赖该 IMU 的业务流程。

## 2. 命令列表

| 命令 | 功能 | 使用前提 |
|---|---|---|
| `app qmi8658b init` | 初始化端口、软复位芯片并应用默认配置 | 无 |
| `app qmi8658b config <normal\|low>` | 测试运行期配置更新 | 已初始化 |
| `app qmi8658b power <normal\|snooze\|suspend\|down>` | 测试电源模式切换 | 已初始化 |
| `app qmi8658b id` | 读取 WHO_AM_I | 已初始化 |
| `app qmi8658b info` | 读取固件版本和 USID | 已初始化 |
| `app qmi8658b read [1-10]` | 读取换算后的六轴与温度数据 | 已初始化 |
| `app qmi8658b raw [1-10]` | 读取原始六轴与温度数据 | 已初始化 |
| `app qmi8658b temp` | 单独读取温度 | 已初始化 |
| `app qmi8658b status` | 读取 STATUS0 数据就绪标志 | 已初始化 |
| `app qmi8658b timestamp` | 读取 24 位传感器时间戳 | 已初始化 |
| `app qmi8658b reg ctrl1` | 翻转 FIFO_INT1 位、读回校验并恢复 CTRL1 | 已初始化 |
| `app qmi8658b int callback` | 注册 INT1 GPIO 回调并清零计数 | 已初始化 |
| `app qmi8658b int map <fifo\|any\|no\|sig\|tap>` | 将中断源映射至 INT1 | 已初始化 |
| `app qmi8658b int pin <on\|off>` | 使能或禁用芯片 INT1 输出 | 已初始化 |
| `app qmi8658b int status` / `int count` / `int clear` | 读取 STATUSINT、读取或清零 GPIO 回调计数 | 已初始化 |
| `app qmi8658b fifo start` | 配置 128 样本、六轴 Stream FIFO，水印为 32 | 已初始化 |
| `app qmi8658b fifo status` | 读取 FIFO_STATUS | FIFO 已配置 |
| `app qmi8658b fifo read` | 最多读取并打印 16 帧原始 FIFO 数据 | FIFO 已配置 |
| `app qmi8658b fifo flush` | 清空 FIFO | FIFO 已配置 |
| `app qmi8658b fifo flush_read` | 原子清空 FIFO 后立即读取帧数 | FIFO 已配置 |
| `app qmi8658b motion config` | 写入运动检测测试参数 | 已初始化 |
| `app qmi8658b motion status` | 读取 Any/No/Sig-Motion 与 Tap 状态 | 已初始化 |
| `app qmi8658b tap config` | 写入敲击检测测试参数 | 已初始化 |
| `app qmi8658b tap status` | 读取敲击次数、轴和方向 | 已初始化 |
| `app qmi8658b feature <any\|no\|sig\|tap> <on\|off>` | 开关嵌入式检测特性 | 先写入对应配置 |
| `app qmi8658b sync <on\|off>` | 开关同步采样 | 已初始化 |
| `app qmi8658b offset delta_zero` | 对加速度和陀螺仪提交零增量偏置 | 已初始化 |
| `app qmi8658b cali` | 执行片内 COD 校准并输出 6 字节陀螺仪增益 | 已初始化，传感器静止 |
| `app qmi8658b gain apply` | 应用缓存的 COD 增益并读回校验 | 已成功校准 |
| `app qmi8658b selftest <acc\|gyr\|all>` | 执行加速度计、陀螺仪或六轴硬件自检 | 已初始化，传感器静止 |

所有成功操作均输出 `QMI8658B <operation>: PASS`；失败时输出 `QMI8658B <operation>: FAIL, ret=<错误码>`。`ret=1` 表示未初始化，`ret=2` 表示总线通信失败，`ret=3` 表示芯片 ID 错误，`ret=4` 表示参数错误，`ret=5` 表示超时，`ret=7` 表示硬件自检未通过。除事件触发测试外，每个用例均以对应 `PASS` 行作为通过判定。

## 3. 基础通信与数据测试

| 编号 | 操作步骤 | 预期结果 |
|---|---|---|
| TC-01 | 执行 `app qmi8658b init` | 必须打印 `QMI8658B init: PASS`。未出现该行即不通过。 |
| TC-02 | 执行 `app qmi8658b id` | 必须同时打印 `QMI8658B chip_id: PASS` 和 `chip_id=0x05, expected=0x05`。ID 不为 `0x05` 即不通过。 |
| TC-03 | 执行 `app qmi8658b info` | 必须打印 `QMI8658B chip_info: PASS`，并打印 `firmware=xx.xx.xx, usid=xxxxxxxxxxxx`；USID 必须为 12 个十六进制字符。 |
| TC-04 | 将模块静置在桌面，执行 `app qmi8658b read 5` | 必须连续打印 `[0]` 至 `[4]` 五行数据，最后打印 `QMI8658B read: PASS`。静置时至少一轴加速度接近 `+/-1000 mg`，陀螺仪各轴应无持续大幅跳变。 |
| TC-05 | 缓慢转动和改变模块朝向，再执行 `app qmi8658b read 5` | 必须打印 `QMI8658B read: PASS`，且与 TC-04 相比，加速度和对应陀螺仪字段至少各有一个明显变化。 |
| TC-06 | 执行 `app qmi8658b raw 3` | 必须连续打印 `[0]` 至 `[2]` 三行原始计数，最后打印 `QMI8658B raw: PASS`。改变姿态后，至少一个 `acc` 原始值应变化。 |
| TC-07 | 执行 `app qmi8658b temp`、`app qmi8658b status`，再执行 `app qmi8658b timestamp` 两次 | 三类命令均必须打印 `PASS`；温度命令打印 `temp_x100=<有符号整数>`，状态命令打印 `STATUS0=0xXX`，两次时间戳均打印 `timestamp=<非负整数>`。时间戳为 24 位循环计数器，需按模 2^24 的差值判断递增，不能直接比较数值大小。 |

`read` 的单位为 mg、mdps 和 0.01 摄氏度。输出中的 `temp_x100` 为温度的 0.01 摄氏度数值；例如 `2534` 表示 25.34 摄氏度。

## 4. FIFO 测试

| 编号 | 操作步骤 | 预期结果 |
|---|---|---|
| TC-08 | 依次执行 `app qmi8658b init`、`app qmi8658b fifo start` | 两条命令均必须分别打印 `QMI8658B init: PASS` 与 `QMI8658B fifo_start: PASS`；FIFO 以六轴 Stream 模式、水印 32 工作。 |
| TC-09 | 等待约 0.5 秒，执行 `app qmi8658b fifo status` | 必须打印 `QMI8658B fifo_status: PASS` 和 `FIFO_STATUS=0xXX`；命令仅在非空位或水印位置位时通过，证明 FIFO 已开始采样。 |
| TC-10 | 执行 `app qmi8658b fifo read` | 必须打印非零的 `fifo_words_before=<N>`、`QMI8658B fifo_read: PASS` 和 `fifo_frames=<N>`，其中帧数应为 1 至 16；随后应有 N 行 `acc=... gyr=...` 数据。 |
| TC-11 | 在完成 TC-09 或 TC-10 后执行 `app qmi8658b fifo flush_read` | 必须打印 `fifo_words_before_flush=<N>` 且 N 大于 0、`QMI8658B fifo_flush: PASS`、`fifo_words_after_flush=<N>`、`QMI8658B fifo_read_after_flush: PASS`。125Hz 连续采样下，清空后最多允许新进入一个六轴样本，即 after 不大于 6 个字，读取帧数不大于 1。 |

## 5. 运动与敲击特性测试

| 编号 | 操作步骤 | 预期结果 |
|---|---|---|
| TC-12 | 依次执行 `app qmi8658b init`、`app qmi8658b motion config`、`app qmi8658b feature any on` | 必须依次打印 `init: PASS`、`motion_config: PASS`、`feature: PASS`。测试配置使用三轴 OR 逻辑和 1 样本窗口，便于验证任意运动。 |
| TC-13 | 在模块上做明显的快速移动，立即执行 `app qmi8658b motion status` | 必须打印 `QMI8658B motion_status: PASS` 和 `any=<0或1> no=<0或1> sig=<0或1> tap=<0或1>`。此用例仅验证状态寄存器读取；`any=0` 不能作为运动功能通过，实际触发判定必须执行 TC-31。 |
| TC-14 | 执行 `app qmi8658b feature any off` | 必须打印 `QMI8658B feature: PASS`。后续执行 `motion status` 时仍可成功读取，但不应因新的动作稳定出现 `any=1`。 |
| TC-15 | 先执行 `app qmi8658b feature any on` 与 `app qmi8658b feature no on`，再执行 `app qmi8658b feature sig on` | 三条均必须打印 `QMI8658B feature: PASS`。未启用 Any 与 No 时直接执行 `sig on`，预期打印 `QMI8658B feature: FAIL, ret=4`。 |
| TC-16 | 依次执行 `app qmi8658b tap config`、`app qmi8658b feature tap on` | `tap config` 必须先打印 `QMI8658B tap_odr_config: PASS`，表示已切换至 250Hz 采样；随后打印 `QMI8658B tap_config: PASS`，使能命令打印 `QMI8658B feature: PASS`。 |
| TC-17 | 轻敲模块外壳，立即执行 `app qmi8658b tap status` | 必须打印 `QMI8658B tap_status: PASS` 和 `number=<0/1/2> axis=<0/1/2/3> negative=<0/1>`；识别成功的通过值为 `number=1` 或 `2` 且 `axis=1`、`2` 或 `3`。仅当 `number` 非 0 时，`axis` 与 `negative` 才有判定意义。 |
| TC-18 | 分别执行 `app qmi8658b feature no off`、`app qmi8658b feature sig off`、`app qmi8658b feature tap off` | 每条必须打印 `QMI8658B feature: PASS`，完成特性测试后的恢复。 |

## 6. 同步采样、校准和自检

| 编号 | 操作步骤 | 预期结果 |
|---|---|---|
| TC-19 | 执行 `app qmi8658b sync on`，然后执行 `app qmi8658b read 3`，最后执行 `app qmi8658b sync off` | 依次必须打印 `QMI8658B sync: PASS`、三行数据及 `QMI8658B read: PASS`、`QMI8658B sync: PASS`。 |
| TC-20 | 将模块置于稳定、静止的水平平面，执行 `app qmi8658b cali` | 约 2.2 秒后必须打印 `QMI8658B calibration: PASS` 和 `gyro_gain=` 后 12 个十六进制字符。校准过程不可移动模块。 |
| TC-21 | 保持模块静止，执行 `app qmi8658b selftest all` | 必须打印 `QMI8658B selftest: PASS`、`acc_pass=1` 和 `gyr_pass=1`。任一为 0 或出现 FAIL 即不通过。 |

## 7. 配置、电源、寄存器和偏置测试

| 编号 | 操作步骤 | 预期结果与通过判定 |
|---|---|---|
| TC-22 | 执行 `app qmi8658b config normal`，再执行 `app qmi8658b read 1` | 第一条必须打印 `QMI8658B set_config: PASS`；第二条必须打印一行 `[0]` 数据和 `QMI8658B read: PASS`。 |
| TC-23 | 执行 `app qmi8658b config low` | 必须打印 `CTRL2=0x2D, CTRL3=0x36, CTRL7=0x01` 和 `QMI8658B set_config: PASS`，表示 8g/21Hz LP 加速度配置、预置陀螺仪 ODR 与仅加速度使能均已读回确认。该用例不替代外部电流测量。 |
| TC-24 | 在 TC-23 后依次执行 `app qmi8658b power normal`、`app qmi8658b config normal`、`app qmi8658b power snooze`、`app qmi8658b power suspend`、`app qmi8658b power normal` | `power normal` 在低功耗配置下应打印 `FAIL, ret=4`，这是预期的参数保护；其余命令在正确顺序下均必须打印 `QMI8658B power: PASS`。最终执行 `read 1` 必须成功，证明已恢复。 |
| TC-25 | 执行 `app qmi8658b reg ctrl1` | 必须打印 `QMI8658B reg_read_write: PASS` 及 `CTRL1=0xXX, test=0xXX, verify=0xXX`，其中 test 与 verify 必须相等，且与 CTRL1 的 FIFO_INT1 位相反。命令完成后自动恢复 CTRL1 原值。 |
| TC-26 | 执行 `app qmi8658b offset delta_zero`，再执行 `app qmi8658b read 1` | 必须打印 `QMI8658B offset_delta_zero: PASS`；随后读取必须打印 `QMI8658B read: PASS`。该命令提交零增量，不会消除先前通过 Delta Offset 累积的偏置。 |
| TC-27 | 模块静置，执行 `app qmi8658b cali`，再执行 `app qmi8658b init`，最后执行 `app qmi8658b gain apply` | 校准完成必须打印 `QMI8658B calibration: PASS` 和 12 个十六进制字符的 `gyro_gain`；重新初始化后应用命令必须打印 `QMI8658B gain_apply: PASS` 和与校准值完全一致的 `gyro_gain_verify`，验证保存值能够恢复。 |

## 8. INT1 中断完整链路测试

测试前确认 `gsensor_int` 对应 MCU 引脚连接到 QMI8658B INT1。GPIO 回调仅累加计数，因此不能在中断上下文中打印日志；以 `int_count` 增长作为硬件中断到达的判定。

| 编号 | 操作步骤 | 预期结果与通过判定 |
|---|---|---|
| TC-28 | 依次执行 `app qmi8658b init`、`app qmi8658b int callback`、`app qmi8658b int pin on` | 三条命令必须分别打印 `QMI8658B init: PASS`、`QMI8658B int_callback: PASS`、`QMI8658B int_pin: PASS`。`callback` 同时将计数清零。 |
| TC-29 | 执行 `app qmi8658b int map fifo`、`app qmi8658b int clear`、`app qmi8658b fifo start`，等待 0.5 秒，执行 `app qmi8658b int count`、`app qmi8658b fifo status`，再读取 FIFO 至低于水印并等待回填后再次读取 `int count` | 清零后首次 `int_count` 应为 1 或更大，`FIFO_STATUS` 应显示水印或非空状态。读 FIFO 至低于水印、等待回填后，第二次计数必须比第一次增加，证明 GPIO 能对两个独立水印周期响应。 |
| TC-30 | 执行 `app qmi8658b int pin off`，记录 `int count`；读取 FIFO 至低于水印，等待回填后再次执行 `int count` | `int_pin` 必须打印 `PASS`；关闭后的第二个计数必须与关闭前相同。该步骤必须产生新的水印周期，才能证明 INT1 输出关闭有效。 |
| TC-31 | 依次执行 `app qmi8658b init`、`app qmi8658b int callback`、`app qmi8658b int pin on`、`app qmi8658b motion config`、`app qmi8658b feature any on`、`app qmi8658b int map any`、`app qmi8658b int clear`；快速移动模块后读取 `int count` 与 `motion status` | 前置初始化会清除 FIFO 路由，所有配置命令必须打印 `PASS`；清零后的 `int_count` 必须大于 0，且 `motion_status` 必须打印 `PASS`。若状态位因读取时序已清除，可重复动作并立即读取；计数增长仍是中断到达的通过依据。 |
| TC-32 | 依次执行 `app qmi8658b init`、`app qmi8658b int callback`、`app qmi8658b int pin on`、`app qmi8658b tap config`、`app qmi8658b feature tap on`、`app qmi8658b int map tap`、`app qmi8658b int clear`；敲击模块后读取 `int count` 与 `tap status` | 前置初始化会清除 FIFO 路由，配置命令均必须打印 `PASS`；清零后的 `int_count` 必须大于 0。检测成功时 `tap_status` 打印 `number=1` 或 `number=2`、`axis=1/2/3`。由于机械结构影响，事件状态可能需要多次敲击确认，但 INT 计数增长是必须条件。 |

`imu_int_map` 当前公开接口仅允许映射到 INT1，不支持传入 `IMU_INT_NONE` 解除映射。测试完成后关闭 INT1 输出并重新初始化模块即可恢复默认路由。

## 9. 测试收尾

1. 执行 `app qmi8658b sync off`，并关闭已启用的运动和敲击特性。
2. 如产品业务需要特定运行配置，重新调用业务侧 IMU 初始化流程恢复配置。
3. 将 `QMI8658B_SHELL_TEST_ENABLE` 恢复为 `0` 后重新构建量产固件，确认 Shell 中不再出现 `qmi8658b` 命令。
