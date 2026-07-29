#include "my_comm.h"

/* 注册 G-Sensor 模块日志 */
LOG_MODULE_REGISTER(my_gsensor, LOG_LEVEL_INF);

#define GSENSOR_PWR_NODE DT_ALIAS(gsensor_pwr_ctrl)
static const struct gpio_dt_spec gsensor_pwr_gpio = GPIO_DT_SPEC_GET(GSENSOR_PWR_NODE, gpios);

#define MY_GSENSOR_TASK_PRIORITY 5
#define MY_GSENSOR_TASK_STACK_SIZE 4 * 1024 // 先改为4K，未来开发过程中不够再调整

/* 消息结构体定义 */
typedef struct
{
    uint32_t msgID;
    void *pData;
    uint32_t DataLen;
} msg_t;

/* G-Sensor 单功能测试消息队列与线程资源 */
K_MSGQ_DEFINE(my_gsensor_msgq, sizeof(msg_t), 10, 4);
K_THREAD_STACK_DEFINE(my_gsensor_task_stack, MY_GSENSOR_TASK_STACK_SIZE);

static struct k_thread s_my_gsensor_task_data;
static struct k_timer s_gsensor_sample_timer;

/********************************************************************
**函数名称:  gsensor_sample_timer_cb
**入口参数:  param     ---        定时器参数
**出口参数:  无
**函数功能:  GSENSOR 周期采样定时器回调，仅向线程发送读取消息
**返 回 值:  无
*********************************************************************/
static void gsensor_sample_timer_cb(void *param)
{
    msg_t msg =
    {
        .msgID = MY_MSG_GSENSOR_SAMPLE,
        .pData = NULL,
        .DataLen = 0,
    };

    ARG_UNUSED(param);

    /* 定时器回调中仅投递消息，不执行传感器读取。 */
    (void)k_msgq_put(&my_gsensor_msgq, &msg, K_NO_WAIT);
}

/********************************************************************
**函数名称:  my_gsensor_task
**入口参数:  p1       ---        线程参数1（未使用）
**           p2       ---        线程参数2（未使用）
**           p3       ---        线程参数3（未使用）
**出口参数:  无
**函数功能:  G-Sensor 单功能测试线程，阻塞等待消息队列数据
**返回值:    无
**注意事项:  PM管理与消息 ID 业务处理暂不启用，供算法开发阶段扩展
*********************************************************************/
static void my_gsensor_task(void *p1, void *p2, void *p3)
{
    msg_t msg;
    imu_result_t imu_result;
    imu_data_t data;
    imu_raw_data_t raw;
    uint32_t sample_index = 0;

    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    imu_result = imu_init(NULL);
    if (imu_result != IMU_SUCCESS)
    {
        LOG_ERR("QMI8658B initialization failed (err %d)", imu_result);
        return;
    }
#if 1
    // 5s后循环按照5s间隔采集数据
    k_timer_start(&s_gsensor_sample_timer, K_MSEC(5000), K_MSEC(5000));
#endif
    for (;;)
    {
        k_msgq_get(&my_gsensor_msgq, &msg, K_FOREVER);

        switch (msg.msgID)
        {
            case MY_MSG_GSENSOR_SAMPLE:
                sample_index++;

                imu_result = imu_read(&data);
                if (imu_result == IMU_SUCCESS)
                {
                    LOG_INF("[%u] acc(mg)=%ld,%ld,%ld gyr(mdps)=%ld,%ld,%ld temp_x100=%ld",
                                (unsigned int)sample_index, (long)data.acc_x, (long)data.acc_y, (long)data.acc_z,
                                (long)data.gyr_x, (long)data.gyr_y, (long)data.gyr_z,
                                (long)data.temperature);
                }
                else
                {
                    LOG_ERR("QMI8658B converted data read failed (err %d)", imu_result);
                }

                imu_result = imu_read_raw(&raw);
                if (imu_result == IMU_SUCCESS)
                {
                    LOG_INF("[%u] temp=%d acc=%d,%d,%d gyr=%d,%d,%d", (unsigned int)sample_index, raw.temperature,
                                raw.acc_x, raw.acc_y, raw.acc_z, raw.gyr_x, raw.gyr_y, raw.gyr_z);
                }
                else
                {
                    LOG_ERR("QMI8658B raw data read failed (err %d)", imu_result);
                }
                break;

            default:
                break;

        }
    }
}

int my_gsensor_pwr_on(bool on)
{
    int err;
    static bool s_gsensor_power_state = false;  // false=关闭, true=开启

    /* 检查当前电源状态，避免重复操作 */
    if (s_gsensor_power_state == on)
    {
        /* 状态相同，无需操作 */
        LOG_INF("GSENSOR Power: already %s", on ? "ON" : "OFF");
        return 0;
    }

    /* 执行电源控制操作 */
    err = gpio_pin_set_dt(&gsensor_pwr_gpio, on ? 1 : 0);
    if (err == 0)
    {
        /* 操作成功，更新状态 */
        s_gsensor_power_state = on;
        LOG_INF("GSENSOR Power Control: %s", on ? "Power ON" : "Power OFF");
    }
    else
    {
        LOG_ERR("GSENSOR Power Control failed (err %d)", err);
    }

    return err;
}

int my_gsensor_init(k_tid_t *tid)
{
    int err;

    if (tid == NULL)
    {
        return -EINVAL;
    }

    /* 检查电源控制 GPIO 是否就绪 */
    if (!device_is_ready(gsensor_pwr_gpio.port))
    {
        LOG_ERR("GSENSOR Power GPIO device not ready");
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&gsensor_pwr_gpio))
    {
        LOG_ERR("GSENSOR Power GPIO not ready");
        return -ENODEV;
    }

    /* 配置电源引脚并开启传感器电源 */
    err = gpio_pin_configure_dt(&gsensor_pwr_gpio, GPIO_OUTPUT_ACTIVE);
    if (err)
    {
        LOG_ERR("Failed to configure GSENSOR Power GPIO (err %d)", err);
        return err;
    }
    /* 同步更新 G-Sensor 电源状态 */
    err = my_gsensor_pwr_on(true);
    if (err != 0)
    {
        return err;
    }

    /* 等待传感器电源稳定。 */
    k_sleep(K_MSEC(20));
    k_timer_init(&s_gsensor_sample_timer, gsensor_sample_timer_cb, NULL);

    *tid = k_thread_create(&s_my_gsensor_task_data, my_gsensor_task_stack,
                           K_THREAD_STACK_SIZEOF(my_gsensor_task_stack),
                           my_gsensor_task, NULL, NULL, NULL,
                           MY_GSENSOR_TASK_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(*tid, "MY_GSENSOR");
    LOG_INF("G-Sensor thread started");

    return 0;
}
