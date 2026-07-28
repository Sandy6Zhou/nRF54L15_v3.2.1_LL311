#include "my_comm.h"
#include "imu_api.h"

/* QMI8658B Shell测试开关：置1开启测试命令，量产前恢复为0 */
#define QMI8658B_SHELL_TEST_ENABLE    1

#define LOG_MODULE_NAME my_shell
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/********************************************************************
**函数名称:  cmd_system_info
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  输出系统信息（示例命令）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_system_info(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "=== System Information ===");
    shell_print(shell, "Device: nRF54L15");
    shell_print(shell, "Build Time: %s %s", __DATE__, __TIME__);
    shell_print(shell, "Uptime: %lld ms", k_uptime_get());
    return 0;
}

/********************************************************************
**函数名称:  cmd_ble_info
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  输出蓝牙状态信息（示例命令）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_ble_info(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "=== BLE Status ===");
    shell_print(shell, "Device Name: Harrison_UART_Service");
    shell_print(shell, "Advertising: Active");
    return 0;
}

/********************************************************************
**函数名称:  cmd_mem_stat
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  输出内存使用统计（示例命令）
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_mem_stat(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "=== Memory Statistics ===");
    shell_print(shell, "Heap Size: %d bytes", CONFIG_HEAP_MEM_POOL_SIZE);
    shell_print(shell, "Stack Usage: Check with 'kernel stacks'");
    return 0;
}

/********************************************************************
**函数名称:  cmd_reboot
**入口参数:  shell   ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  系统重启命令
**返 回 值:  0 表示成功
*********************************************************************/
static int cmd_reboot(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "System rebooting...");
    k_sleep(K_MSEC(500));
    sys_reboot(SYS_REBOOT_WARM);
    return 0;
}

/********************************************************************
**函数名称:  cmd_set_time
**入口参数:  sh      ---        Shell 实例指针
**           argc    ---        参数数量
**           argv    ---        参数数组
**出口参数:  无
**函数功能:  设置系统时间（Unix 时间戳）
**返 回 值:  0 表示成功，负数表示失败
*********************************************************************/
static int cmd_set_time(const struct shell *sh, size_t argc, char **argv)
{
    int ret;

    if (argc != 2)
    {
        shell_error(sh, "Usage: app settime <epoch_sec>");
        shell_print(sh, "  <epoch_sec>: seconds since 1970-01-01 00:00:00 UTC");
        return -EINVAL;
    }

    errno = 0;
    unsigned long long epoch = strtoull(argv[1], NULL, 10);
    if (errno != 0)
    {
        shell_error(sh, "Invalid number: %s", argv[1]);
        return -EINVAL;
    }
    extern int my_set_system_time(time_t _sec);
    ret = my_set_system_time((time_t)epoch);
    if (ret < 0)
    {
        shell_error(sh, "app_set_system_time failed, ret=%d", ret);
        return ret;
    }

    shell_print(sh, "System time set to epoch: %llu", epoch);
    return 0;
}

#if QMI8658B_SHELL_TEST_ENABLE
#define QMI8658B_TEST_FIFO_MAX_FRAMES     16U
#define QMI8658B_TEST_SENSOR_ACC           0x01U
#define QMI8658B_TEST_SENSOR_GYR           0x02U
#define QMI8658B_TEST_REG_CTRL1            0x02U
#define QMI8658B_TEST_REG_CTRL2            0x03U
#define QMI8658B_TEST_REG_CTRL3            0x04U
#define QMI8658B_TEST_REG_CTRL7            0x08U
#define QMI8658B_TEST_REG_CAL1_L           0x0BU
#define QMI8658B_TEST_REG_FIFO_COUNT       0x15U
#define QMI8658B_TEST_CTRL1_FIFO_INT1      0x04U
#define QMI8658B_TEST_FIFO_STATUS_NOT_EMPTY 0x10U
#define QMI8658B_TEST_FIFO_STATUS_WTM      0x40U

static imu_raw_data_t s_qmi8658b_fifo_frames[QMI8658B_TEST_FIFO_MAX_FRAMES];
static volatile uint32_t s_qmi8658b_int_count;
static uint8_t s_qmi8658b_gyro_gain[6];
static bool s_qmi8658b_gyro_gain_valid;

/********************************************************************
**函数名称:  qmi8658b_shell_int_callback
**入口参数:  无
**出口参数:  无
**函数功能:  记录QMI8658B INT1 GPIO中断次数
**返回值:    无
**注意事项:  运行于GPIO中断上下文，仅执行计数操作
*********************************************************************/
static void qmi8658b_shell_int_callback(void)
{
    s_qmi8658b_int_count++;
}

/********************************************************************
**函数名称:  qmi8658b_shell_print_result
**入口参数:  shell     ---        Shell实例指针（输入）
**           operation ---        操作名称（输入）
**           result    ---        IMU接口返回值（输入）
**出口参数:  无
**函数功能:  统一输出QMI8658B测试操作结果
**返回值:    无
*********************************************************************/
static void qmi8658b_shell_print_result(const struct shell *shell, const char *operation, imu_result_t result)
{
    if (result == IMU_SUCCESS)
    {
        shell_print(shell, "QMI8658B %s: PASS", operation);
    }
    else
    {
        shell_error(shell, "QMI8658B %s: FAIL, ret=%d", operation, result);
    }
}

/********************************************************************
**函数名称:  qmi8658b_shell_get_fifo_words
**入口参数:  无
**出口参数:  fifo_words ---      FIFO 当前字计数（输出）
**函数功能:  读取并解析 FIFO_COUNT 的 10 位字计数
**返回值:    IMU_SUCCESS 表示成功，其他表示错误码
*********************************************************************/
static imu_result_t qmi8658b_shell_get_fifo_words(uint16_t *fifo_words)
{
    uint8_t count[2];
    imu_result_t result;

    if (fifo_words == NULL)
    {
        return IMU_ERROR_PARAM;
    }

    result = imu_read_reg(QMI8658B_TEST_REG_FIFO_COUNT, count, sizeof(count));
    if (result == IMU_SUCCESS)
    {
        *fifo_words = (uint16_t)((uint16_t)count[0] | (((uint16_t)count[1] & 0x03U) << 8));
    }

    return result;
}

/********************************************************************
**函数名称:  qmi8658b_shell_parse_feature
**入口参数:  name    ---        特性名称字符串（输入）
**出口参数:  feature ---        解析后的特性类型
**函数功能:  解析Shell输入的嵌入式特性名称
**返回值:    0表示成功，-EINVAL表示参数无效
*********************************************************************/
static int qmi8658b_shell_parse_feature(const char *name, imu_feature_t *feature)
{
    if ((name == NULL) || (feature == NULL))
    {
        return -EINVAL;
    }

    if (strcmp(name, "any") == 0)
    {
        *feature = IMU_FEATURE_ANY_MOTION;
    }
    else if (strcmp(name, "no") == 0)
    {
        *feature = IMU_FEATURE_NO_MOTION;
    }
    else if (strcmp(name, "sig") == 0)
    {
        *feature = IMU_FEATURE_SIG_MOTION;
    }
    else if (strcmp(name, "tap") == 0)
    {
        *feature = IMU_FEATURE_TAP;
    }
    else
    {
        return -EINVAL;
    }

    return 0;
}

/********************************************************************
**函数名称:  qmi8658b_shell_parse_int_source
**入口参数:  name   ---        中断源名称字符串（输入）
**出口参数:  source ---        解析后的中断源
**函数功能:  解析Shell输入的QMI8658B中断源名称
**返回值:    0表示成功，-EINVAL表示参数无效
*********************************************************************/
static int qmi8658b_shell_parse_int_source(const char *name, imu_int_src_t *source)
{
    if ((name == NULL) || (source == NULL))
    {
        return -EINVAL;
    }

    if (strcmp(name, "fifo") == 0)
    {
        *source = IMU_INT_SRC_FIFO_WATERMARK;
    }
    else if (strcmp(name, "any") == 0)
    {
        *source = IMU_INT_SRC_ANY_MOTION;
    }
    else if (strcmp(name, "no") == 0)
    {
        *source = IMU_INT_SRC_NO_MOTION;
    }
    else if (strcmp(name, "sig") == 0)
    {
        *source = IMU_INT_SRC_SIG_MOTION;
    }
    else if (strcmp(name, "tap") == 0)
    {
        *source = IMU_INT_SRC_TAP;
    }
    else
    {
        return -EINVAL;
    }

    return 0;
}

/********************************************************************
**函数名称:  qmi8658b_shell_config_motion
**入口参数:  shell    ---        Shell实例指针（输入）
**出口参数:  无
**函数功能:  写入任意运动、静止和显著运动测试配置
**返回值:    0表示成功，负数表示失败
*********************************************************************/
static int qmi8658b_shell_config_motion(const struct shell *shell)
{
    imu_motion_config_t config;
    imu_result_t result;

    memset(&config, 0, sizeof(config));
    config.any_motion_threshold_x = 3U;
    config.any_motion_threshold_y = 3U;
    config.any_motion_threshold_z = 3U;
    config.no_motion_threshold_x = 3U;
    config.no_motion_threshold_y = 3U;
    config.no_motion_threshold_z = 3U;
    config.any_motion_window = 1U;
    config.no_motion_window = 10U;
    config.sig_motion_wait_window = 100U;
    config.sig_motion_confirm_window = 10U;
    config.mode_ctrl = 0x77U;    // Any/No-Motion均使能三轴并使用OR逻辑

    result = imu_set_motion_config(&config);
    qmi8658b_shell_print_result(shell, "motion_config", result);
    return (result == IMU_SUCCESS) ? 0 : -EIO;
}

/********************************************************************
**函数名称:  qmi8658b_shell_config_tap
**入口参数:  shell    ---        Shell实例指针（输入）
**出口参数:  无
**函数功能:  写入单击和双击检测测试配置
**返回值:    0表示成功，负数表示失败
*********************************************************************/
static int qmi8658b_shell_config_tap(const struct shell *shell)
{
    imu_tap_config_t config;
    imu_config_t sensor_config;
    imu_result_t result;

    memset(&sensor_config, 0, sizeof(sensor_config));
    sensor_config.acc_range = IMU_ACC_RANGE_8G;
    sensor_config.acc_odr = IMU_ODR_250HZ;    // 数据手册建议敲击检测加速度ODR高于200Hz
    sensor_config.gyr_range = IMU_GYR_RANGE_1024DPS;
    sensor_config.gyr_odr = IMU_ODR_250HZ;
    sensor_config.power_mode = IMU_POWER_NORMAL;
    sensor_config.lpf_enable = true;
    sensor_config.lpf_mode = IMU_LPF_MODE_0;
    result = imu_set_config(&sensor_config);
    qmi8658b_shell_print_result(shell, "tap_odr_config", result);
    if (result != IMU_SUCCESS)
    {
        return -EIO;
    }

    memset(&config, 0, sizeof(config));
    config.peak_window = 10U;
    config.priority = 0U;
    config.tap_window = 32U;
    config.double_tap_window = 64U;
    config.alpha = 64U;
    config.gamma = 64U;
    config.peak_magnitude_threshold = 256U;
    config.undefined_motion_threshold = 128U;

    result = imu_set_tap_config(&config);
    qmi8658b_shell_print_result(shell, "tap_config", result);
    return (result == IMU_SUCCESS) ? 0 : -EIO;
}

/********************************************************************
**函数名称:  qmi8658b_shell_cmd
**入口参数:  shell    ---        Shell实例指针（输入）
**           argc     ---        参数数量（输入）
**           argv     ---        参数数组（输入）
**出口参数:  无
**函数功能:  执行QMI8658B功能验证Shell测试命令
**返回值:    0表示成功，负数表示失败
*********************************************************************/
static int qmi8658b_shell_cmd(const struct shell *shell, size_t argc, char **argv)
{
    imu_data_t data;
    imu_raw_data_t raw;
    imu_chip_info_t chip_info;
    imu_self_test_result_t self_test;
    imu_motion_status_t motion_status;
    imu_tap_status_t tap_status;
    imu_fifo_config_t fifo_config;
    imu_config_t config;
    imu_feature_t feature;
    imu_int_src_t int_source;
    imu_result_t result;
    uint8_t chip_id;
    uint8_t status;
    uint8_t reg_value;
    uint8_t reg_verify;
    uint8_t reg_test;
    uint8_t gain_verify[6];
    uint32_t timestamp;
    int32_t temperature;
    int16_t offset[3];
    uint16_t frame_count;
    uint16_t fifo_words_before;
    uint16_t fifo_words_after;
    uint16_t index;
    uint8_t sensor_mask;
    bool enable;
    unsigned long count;
    char *end_ptr;

    if (argc < 2U)
    {
        shell_print(shell, "Usage: qmi8658b <init|config|power|id|info|read|raw|temp|status|reg|int|fifo|motion|tap|feature|sync|offset|cali|gain|selftest>");
        return -EINVAL;
    }

    if (strcmp(argv[1], "init") == 0)
    {
        result = imu_init(NULL);
        qmi8658b_shell_print_result(shell, "init", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "id") == 0)
    {
        result = imu_get_chip_id(&chip_id);
        qmi8658b_shell_print_result(shell, "chip_id", result);
        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "chip_id=0x%02X, expected=0x05", chip_id);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "info") == 0)
    {
        result = imu_get_chip_info(&chip_info);
        qmi8658b_shell_print_result(shell, "chip_info", result);
        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "firmware=%02X.%02X.%02X, usid=%02X%02X%02X%02X%02X%02X",
                        chip_info.firmware_version[0], chip_info.firmware_version[1], chip_info.firmware_version[2],
                        chip_info.usid[0], chip_info.usid[1], chip_info.usid[2],
                        chip_info.usid[3], chip_info.usid[4], chip_info.usid[5]);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "config") == 0)
    {
        if ((argc != 3U) || ((strcmp(argv[2], "normal") != 0) && (strcmp(argv[2], "low") != 0)))
        {
            shell_error(shell, "Usage: qmi8658b config <normal|low>");
            return -EINVAL;
        }

        config.acc_range = IMU_ACC_RANGE_8G;
        config.gyr_range = IMU_GYR_RANGE_1024DPS;
        config.gyr_odr = IMU_ODR_125HZ;
        config.lpf_enable = true;
        config.lpf_mode = IMU_LPF_MODE_0;
        if (strcmp(argv[2], "low") == 0)
        {
            config.acc_odr = IMU_ODR_21HZ_LP;
            config.power_mode = IMU_POWER_LOW_POWER;
        }
        else
        {
            config.acc_odr = IMU_ODR_125HZ;
            config.power_mode = IMU_POWER_NORMAL;
        }

        result = imu_set_config(&config);
        if (result == IMU_SUCCESS)
        {
            result = imu_read_reg(QMI8658B_TEST_REG_CTRL2, &reg_value, 1U);
        }
        if (result == IMU_SUCCESS)
        {
            result = imu_read_reg(QMI8658B_TEST_REG_CTRL3, &reg_verify, 1U);
        }
        if (result == IMU_SUCCESS)
        {
            result = imu_read_reg(QMI8658B_TEST_REG_CTRL7, &status, 1U);
        }

        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "CTRL2=0x%02X, CTRL3=0x%02X, CTRL7=0x%02X", reg_value, reg_verify, status);
            if ((strcmp(argv[2], "low") == 0) &&
                ((reg_value != 0x2DU) || (reg_verify != 0x36U) || (status != QMI8658B_TEST_SENSOR_ACC)))
            {
                result = IMU_ERROR_COMM;
            }
            else if ((strcmp(argv[2], "normal") == 0) &&
                     ((reg_value != 0x26U) || (reg_verify != 0x36U) ||
                      (status != (QMI8658B_TEST_SENSOR_ACC | QMI8658B_TEST_SENSOR_GYR))))
            {
                result = IMU_ERROR_COMM;
            }
        }

        qmi8658b_shell_print_result(shell, "set_config", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "power") == 0)
    {
        if (argc != 3U)
        {
            shell_error(shell, "Usage: qmi8658b power <normal|snooze|suspend|down>");
            return -EINVAL;
        }

        if (strcmp(argv[2], "normal") == 0)
        {
            result = imu_set_power_mode(IMU_POWER_NORMAL);
        }
        else if (strcmp(argv[2], "snooze") == 0)
        {
            result = imu_set_power_mode(IMU_POWER_GYRO_SNOOZE);
        }
        else if (strcmp(argv[2], "suspend") == 0)
        {
            result = imu_set_power_mode(IMU_POWER_SUSPEND);
        }
        else if (strcmp(argv[2], "down") == 0)
        {
            result = imu_set_power_mode(IMU_POWER_DOWN);
        }
        else
        {
            shell_error(shell, "Usage: qmi8658b power <normal|snooze|suspend|down>");
            return -EINVAL;
        }

        qmi8658b_shell_print_result(shell, "power", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "temp") == 0)
    {
        result = imu_read_temperature(&temperature);
        qmi8658b_shell_print_result(shell, "temperature", result);
        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "temp_x100=%ld", (long)temperature);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if ((strcmp(argv[1], "read") == 0) || (strcmp(argv[1], "raw") == 0))
    {
        count = 1U;
        if (argc == 3U)
        {
            count = strtoul(argv[2], &end_ptr, 10);
            if ((*end_ptr != '\0') || (count == 0U) || (count > 10U))
            {
                shell_error(shell, "count must be 1 to 10");
                return -EINVAL;
            }
        }

        for (index = 0U; index < count; index++)
        {
            if (strcmp(argv[1], "read") == 0)
            {
                result = imu_read(&data);
                if (result == IMU_SUCCESS)
                {
                    shell_print(shell, "[%u] acc(mg)=%ld,%ld,%ld gyr(mdps)=%ld,%ld,%ld temp_x100=%ld",
                                index, (long)data.acc_x, (long)data.acc_y, (long)data.acc_z,
                                (long)data.gyr_x, (long)data.gyr_y, (long)data.gyr_z,
                                (long)data.temperature);
                }
            }
            else
            {
                result = imu_read_raw(&raw);
                if (result == IMU_SUCCESS)
                {
                    shell_print(shell, "[%u] temp=%d acc=%d,%d,%d gyr=%d,%d,%d", index, raw.temperature,
                                raw.acc_x, raw.acc_y, raw.acc_z, raw.gyr_x, raw.gyr_y, raw.gyr_z);
                }
            }

            if (result != IMU_SUCCESS)
            {
                qmi8658b_shell_print_result(shell, argv[1], result);
                return -EIO;
            }
        }

        qmi8658b_shell_print_result(shell, argv[1], IMU_SUCCESS);
        return 0;
    }

    if (strcmp(argv[1], "status") == 0)
    {
        result = imu_read_status(&status);
        qmi8658b_shell_print_result(shell, "status", result);
        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "STATUS0=0x%02X, acc_drdy=%u, gyr_drdy=%u", status, status & 0x01U, (status >> 1) & 0x01U);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "reg") == 0)
    {
        if ((argc != 3U) || (strcmp(argv[2], "ctrl1") != 0))
        {
            shell_error(shell, "Usage: qmi8658b reg ctrl1");
            return -EINVAL;
        }

        result = imu_read_reg(QMI8658B_TEST_REG_CTRL1, &reg_value, 1U);
        if (result != IMU_SUCCESS)
        {
            qmi8658b_shell_print_result(shell, "reg_read", result);
            return -EIO;
        }

        reg_test = (uint8_t)(reg_value ^ QMI8658B_TEST_CTRL1_FIFO_INT1);
        result = imu_write_reg(QMI8658B_TEST_REG_CTRL1, &reg_test, 1U);
        if (result == IMU_SUCCESS)
        {
            result = imu_read_reg(QMI8658B_TEST_REG_CTRL1, &reg_verify, 1U);
        }
        if (imu_write_reg(QMI8658B_TEST_REG_CTRL1, &reg_value, 1U) != IMU_SUCCESS)
        {
            result = IMU_ERROR_COMM;
        }

        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "CTRL1=0x%02X, test=0x%02X, verify=0x%02X", reg_value, reg_test, reg_verify);
            if (reg_test != reg_verify)
            {
                shell_error(shell, "CTRL1 read-back mismatch");
                result = IMU_ERROR_COMM;
            }
        }
        qmi8658b_shell_print_result(shell, "reg_read_write", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "int") == 0)
    {
        if (argc < 3U)
        {
            shell_error(shell, "Usage: qmi8658b int <callback|clear|count|status|pin|map>");
            return -EINVAL;
        }

        if (strcmp(argv[2], "callback") == 0)
        {
            s_qmi8658b_int_count = 0U;
            result = imu_register_int_callback(qmi8658b_shell_int_callback);
            qmi8658b_shell_print_result(shell, "int_callback", result);
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if (strcmp(argv[2], "count") == 0)
        {
            shell_print(shell, "int_count=%lu", (unsigned long)s_qmi8658b_int_count);
            return 0;
        }

        if (strcmp(argv[2], "clear") == 0)
        {
            s_qmi8658b_int_count = 0U;
            shell_print(shell, "int_count=0");
            return 0;
        }

        if (strcmp(argv[2], "status") == 0)
        {
            result = imu_read_int_status(&status);
            qmi8658b_shell_print_result(shell, "int_status", result);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "STATUSINT=0x%02X", status);
            }
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if ((strcmp(argv[2], "pin") == 0) && (argc == 4U))
        {
            if (strcmp(argv[3], "on") == 0)
            {
                result = imu_int_pin_enable(IMU_INT_PIN1, true);
            }
            else if (strcmp(argv[3], "off") == 0)
            {
                result = imu_int_pin_enable(IMU_INT_PIN1, false);
            }
            else
            {
                shell_error(shell, "Usage: qmi8658b int pin <on|off>");
                return -EINVAL;
            }

            qmi8658b_shell_print_result(shell, "int_pin", result);
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if ((strcmp(argv[2], "map") == 0) && (argc == 4U))
        {
            if (qmi8658b_shell_parse_int_source(argv[3], &int_source) != 0)
            {
                shell_error(shell, "Usage: qmi8658b int map <fifo|any|no|sig|tap>");
                return -EINVAL;
            }

            result = imu_int_map(int_source, IMU_INT_PIN1);
            qmi8658b_shell_print_result(shell, "int_map", result);
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        shell_error(shell, "Usage: qmi8658b int <callback|clear|count|status|pin|map>");
        return -EINVAL;
    }

    if (strcmp(argv[1], "fifo") == 0)
    {
        if (argc != 3U)
        {
            shell_error(shell, "Usage: qmi8658b fifo <start|read|flush|flush_read|status>");
            return -EINVAL;
        }

        if (strcmp(argv[2], "start") == 0)
        {
            fifo_config.acc_enable = true;
            fifo_config.gyr_enable = true;
            fifo_config.mode = IMU_FIFO_STREAM;
            fifo_config.fifo_size = IMU_FIFO_SIZE_128;
            fifo_config.watermark = 32U;
            fifo_config.int_pin = IMU_INT_PIN1;
            result = imu_fifo_config(&fifo_config);
            qmi8658b_shell_print_result(shell, "fifo_start", result);
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if (strcmp(argv[2], "read") == 0)
        {
            result = qmi8658b_shell_get_fifo_words(&fifo_words_before);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "fifo_words_before=%u", fifo_words_before);
                if (fifo_words_before == 0U)
                {
                    result = IMU_ERROR_PARAM;
                }
            }
            if (result == IMU_SUCCESS)
            {
                result = imu_fifo_read(s_qmi8658b_fifo_frames, QMI8658B_TEST_FIFO_MAX_FRAMES, &frame_count);
            }
            if ((result == IMU_SUCCESS) && (frame_count == 0U))
            {
                result = IMU_ERROR_COMM;
            }
            qmi8658b_shell_print_result(shell, "fifo_read", result);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "fifo_frames=%u", frame_count);
                for (index = 0U; index < frame_count; index++)
                {
                    shell_print(shell, "[%u] acc=%d,%d,%d gyr=%d,%d,%d", index,
                                s_qmi8658b_fifo_frames[index].acc_x, s_qmi8658b_fifo_frames[index].acc_y,
                                s_qmi8658b_fifo_frames[index].acc_z, s_qmi8658b_fifo_frames[index].gyr_x,
                                s_qmi8658b_fifo_frames[index].gyr_y, s_qmi8658b_fifo_frames[index].gyr_z);
                }
            }
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if (strcmp(argv[2], "flush") == 0)
        {
            result = imu_fifo_flush();
            qmi8658b_shell_print_result(shell, "fifo_flush", result);
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if (strcmp(argv[2], "flush_read") == 0)
        {
            result = qmi8658b_shell_get_fifo_words(&fifo_words_before);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "fifo_words_before_flush=%u", fifo_words_before);
            }
            if (result == IMU_SUCCESS)
            {
                result = imu_fifo_flush();
            }
            qmi8658b_shell_print_result(shell, "fifo_flush", result);
            if (result != IMU_SUCCESS)
            {
                return -EIO;
            }

            result = qmi8658b_shell_get_fifo_words(&fifo_words_after);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "fifo_words_after_flush=%u", fifo_words_after);
                // 125Hz 连续采样下，CTRL9 清空命令执行期间可能新进入一个六轴样本（6个字）
                if ((fifo_words_before == 0U) || (fifo_words_after > 6U))
                {
                    result = IMU_ERROR_PARAM;
                }
            }
            if (result == IMU_SUCCESS)
            {
                result = imu_fifo_read(s_qmi8658b_fifo_frames, QMI8658B_TEST_FIFO_MAX_FRAMES, &frame_count);
            }
            qmi8658b_shell_print_result(shell, "fifo_read_after_flush", result);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "fifo_frames=%u", frame_count);
            }

            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if (strcmp(argv[2], "status") == 0)
        {
            result = imu_fifo_get_status(&status);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "FIFO_STATUS=0x%02X", status);
                if ((status & (QMI8658B_TEST_FIFO_STATUS_NOT_EMPTY | QMI8658B_TEST_FIFO_STATUS_WTM)) == 0U)
                {
                    result = IMU_ERROR_PARAM;
                }
            }
            qmi8658b_shell_print_result(shell, "fifo_status", result);
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        shell_error(shell, "Usage: qmi8658b fifo <start|read|flush|flush_read|status>");
        return -EINVAL;
    }

    if ((strcmp(argv[1], "motion") == 0) || (strcmp(argv[1], "tap") == 0))
    {
        if (argc != 3U)
        {
            shell_error(shell, "Usage: qmi8658b motion <config|status> or qmi8658b tap <config|status>");
            return -EINVAL;
        }

        if (strcmp(argv[2], "config") == 0)
        {
            return (strcmp(argv[1], "motion") == 0) ? qmi8658b_shell_config_motion(shell) : qmi8658b_shell_config_tap(shell);
        }

        if ((strcmp(argv[1], "motion") == 0) && (strcmp(argv[2], "status") == 0))
        {
            result = imu_get_motion_status(&motion_status);
            qmi8658b_shell_print_result(shell, "motion_status", result);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "any=%u no=%u sig=%u tap=%u", motion_status.any_motion, motion_status.no_motion,
                            motion_status.sig_motion, motion_status.tap);
            }
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        if ((strcmp(argv[1], "tap") == 0) && (strcmp(argv[2], "status") == 0))
        {
            result = imu_get_tap_status(&tap_status);
            qmi8658b_shell_print_result(shell, "tap_status", result);
            if (result == IMU_SUCCESS)
            {
                shell_print(shell, "number=%u axis=%u negative=%u", tap_status.tap_number, tap_status.axis,
                            tap_status.negative_polarity);
            }
            return (result == IMU_SUCCESS) ? 0 : -EIO;
        }

        shell_error(shell, "Usage: qmi8658b motion <config|status> or qmi8658b tap <config|status>");
        return -EINVAL;
    }

    if (strcmp(argv[1], "feature") == 0)
    {
        if ((argc != 4U) || (qmi8658b_shell_parse_feature(argv[2], &feature) != 0))
        {
            shell_error(shell, "Usage: qmi8658b feature <any|no|sig|tap> <on|off>");
            return -EINVAL;
        }

        if (strcmp(argv[3], "on") == 0)
        {
            enable = true;
        }
        else if (strcmp(argv[3], "off") == 0)
        {
            enable = false;
        }
        else
        {
            shell_error(shell, "feature state must be on or off");
            return -EINVAL;
        }

        result = imu_feature_enable(feature, enable);
        qmi8658b_shell_print_result(shell, "feature", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "sync") == 0)
    {
        if (argc != 3U)
        {
            shell_error(shell, "Usage: qmi8658b sync <on|off>");
            return -EINVAL;
        }

        enable = (strcmp(argv[2], "on") == 0);
        if (!enable && (strcmp(argv[2], "off") != 0))
        {
            shell_error(shell, "sync state must be on or off");
            return -EINVAL;
        }

        result = imu_set_sync_sample(enable);
        qmi8658b_shell_print_result(shell, "sync", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "cali") == 0)
    {
        result = imu_run_calibration(s_qmi8658b_gyro_gain);
        qmi8658b_shell_print_result(shell, "calibration", result);
        if (result == IMU_SUCCESS)
        {
            s_qmi8658b_gyro_gain_valid = true;
            shell_print(shell, "gyro_gain=%02X%02X%02X%02X%02X%02X", s_qmi8658b_gyro_gain[0], s_qmi8658b_gyro_gain[1],
                        s_qmi8658b_gyro_gain[2], s_qmi8658b_gyro_gain[3], s_qmi8658b_gyro_gain[4], s_qmi8658b_gyro_gain[5]);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "gain") == 0)
    {
        if ((argc != 3U) || (strcmp(argv[2], "apply") != 0))
        {
            shell_error(shell, "Usage: qmi8658b gain apply");
            return -EINVAL;
        }

        if (!s_qmi8658b_gyro_gain_valid)
        {
            shell_error(shell, "Run qmi8658b cali successfully before gain apply");
            return -EACCES;
        }

        result = imu_apply_gyro_gain(s_qmi8658b_gyro_gain);
        if (result == IMU_SUCCESS)
        {
            result = imu_read_reg(QMI8658B_TEST_REG_CAL1_L, gain_verify, sizeof(gain_verify));
        }
        if ((result == IMU_SUCCESS) && (memcmp(s_qmi8658b_gyro_gain, gain_verify, sizeof(gain_verify)) != 0))
        {
            result = IMU_ERROR_COMM;
        }
        qmi8658b_shell_print_result(shell, "gain_apply", result);
        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "gyro_gain_verify=%02X%02X%02X%02X%02X%02X", gain_verify[0], gain_verify[1], gain_verify[2],
                        gain_verify[3], gain_verify[4], gain_verify[5]);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "offset") == 0)
    {
        if ((argc != 3U) || (strcmp(argv[2], "delta_zero") != 0))
        {
            shell_error(shell, "Usage: qmi8658b offset delta_zero");
            return -EINVAL;
        }

        offset[0] = 0;
        offset[1] = 0;
        offset[2] = 0;
        result = imu_set_acc_offset(offset);
        if (result == IMU_SUCCESS)
        {
            result = imu_set_gyr_offset(offset);
        }

        qmi8658b_shell_print_result(shell, "offset_delta_zero", result);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "selftest") == 0)
    {
        if (argc != 3U)
        {
            shell_error(shell, "Usage: qmi8658b selftest <acc|gyr|all>");
            return -EINVAL;
        }

        if (strcmp(argv[2], "acc") == 0)
        {
            sensor_mask = QMI8658B_TEST_SENSOR_ACC;
        }
        else if (strcmp(argv[2], "gyr") == 0)
        {
            sensor_mask = QMI8658B_TEST_SENSOR_GYR;
        }
        else if (strcmp(argv[2], "all") == 0)
        {
            sensor_mask = QMI8658B_TEST_SENSOR_ACC | QMI8658B_TEST_SENSOR_GYR;
        }
        else
        {
            shell_error(shell, "selftest target must be acc, gyr or all");
            return -EINVAL;
        }

        memset(&self_test, 0, sizeof(self_test));
        result = imu_run_self_test(sensor_mask, &self_test);
        qmi8658b_shell_print_result(shell, "selftest", result);
        shell_print(shell, "acc_pass=%u, acc_mg=%ld,%ld,%ld", self_test.acc_pass, (long)self_test.acc_x_mg,
                    (long)self_test.acc_y_mg, (long)self_test.acc_z_mg);
        shell_print(shell, "gyr_pass=%u, gyr_mdps=%ld,%ld,%ld", self_test.gyr_pass, (long)self_test.gyr_x_mdps,
                    (long)self_test.gyr_y_mdps, (long)self_test.gyr_z_mdps);
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    if (strcmp(argv[1], "timestamp") == 0)
    {
        result = imu_read_timestamp(&timestamp);
        qmi8658b_shell_print_result(shell, "timestamp", result);
        if (result == IMU_SUCCESS)
        {
            shell_print(shell, "timestamp=%lu", (unsigned long)timestamp);
        }
        return (result == IMU_SUCCESS) ? 0 : -EIO;
    }

    shell_error(shell, "Unknown command: %s", argv[1]);
    return -EINVAL;
}
#endif /* QMI8658B_SHELL_TEST_ENABLE */

/* 注册自定义命令到 Shell 子系统 */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_app,
    SHELL_CMD(sysinfo, NULL, "Display system information", cmd_system_info),
    SHELL_CMD(bleinfo, NULL, "Display BLE status", cmd_ble_info),
    SHELL_CMD(memstat, NULL, "Display memory statistics", cmd_mem_stat),
    SHELL_CMD(reboot, NULL, "Reboot system", cmd_reboot),
#if QMI8658B_SHELL_TEST_ENABLE
    SHELL_CMD(qmi8658b, NULL, "QMI8658B test: qmi8658b <init|config|power|id|info|read|raw|temp|status|reg|int|fifo|motion|tap|feature|sync|offset|cali|gain|selftest|timestamp>", qmi8658b_shell_cmd),
#endif
    SHELL_SUBCMD_SET_END
);
/* Zephyr Shell 子系统提供的宏，随 nRF Connect SDK一起提供，用来在 Shell里注册一个“根命令”
 * 这个宏在头文件zephyr/shell/shell.h里定义，是Zephyr的Shell API的一部分
 */
SHELL_CMD_REGISTER(app, &sub_app, "Application commands", NULL);