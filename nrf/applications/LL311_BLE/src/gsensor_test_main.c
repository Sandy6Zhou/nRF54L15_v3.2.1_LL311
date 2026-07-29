#include "my_comm.h"

LOG_MODULE_REGISTER(gsensor_test_main, LOG_LEVEL_INF);

static k_tid_t s_gsensor_task_id;

/*********************************************************************
**函数名称:  my_set_system_time
**入口参数:  _sec  --  自 1970-01-01 00:00:00 UTC 起的秒数
**出口参数:  无
**函数功能:  设置系统时间（RTC实时时钟）
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_set_system_time(time_t _sec)
{
    struct timespec ts;
    int ret;

    if (_sec < 1770000000)
    {
        LOG_INF("set system time failed, _sec=%ld", _sec);
        return -1;
    }

    ts.tv_sec  = _sec;
    ts.tv_nsec = 0;

    ret = sys_clock_settime(SYS_CLOCK_REALTIME, &ts);
    if (ret < 0)
    {
        LOG_INF("sys_clock_settime failed, ret=%d", ret);
        return ret;
    }

    return 0;
}

/*********************************************************************
**函数名称:  my_get_system_time_sec
**入口参数:  无
**出口参数:  无
**函数功能:  获取当前系统时间（RTC实时时钟）
**返 回 值:  成功返回 >= 0 的 time_t（自1970-01-01 00:00:00 UTC起的秒数）
**           失败返回 (time_t)-1
*********************************************************************/
time_t my_get_system_time_sec(void)
{
    struct timespec ts;
    int ret;

    ret = sys_clock_gettime(SYS_CLOCK_REALTIME, &ts);
    if (ret < 0)
    {
        LOG_INF("clock_gettime failed, ret=%d", ret);
        return (time_t)-1;
    }

    return ts.tv_sec;
}

/********************************************************************
**函数名称:  my_clock_get_time
**入口参数:  t - 指向 tm 结构体的指针，用于存储转换后的时间信息
**出口参数:  无
**功能描述:  获取当前系统时间并转换为 tm 结构体格式
**返 回 值:  无
*********************************************************************/
void my_clock_get_time(struct tm *t)
{
    time_t unix_time;  // 存储 Unix 时间戳

    // 获取当前系统时间的 Unix 时间戳（秒）
    unix_time = my_get_system_time_sec();

    // 将 Unix 时间戳转换为 UTC 时间的 tm 结构体格式
    gmtime_r(&unix_time, t);
}

/********************************************************************
**函数名称:  custom_timestamp_formatter
**入口参数:  output 日志输出结构体指针
**           timestamp 原始时间戳值（未使用，保留为兼容接口）
**           printer 时间戳打印函数指针
**出口参数:  无
**函数功能:  通过调用 my_clock_get_time() 获取当前系统时间，
**           然后将其格式化为 "[YYYY-MM-DD HH:MM:SS] " 格式的字符串，
**           最后通过传入的 printer 函数将格式化后的时间戳输出到日志系统。
**返 回 值:  打印的字符数
*********************************************************************/
int custom_timestamp_formatter(const struct log_output *output,
                               const log_timestamp_t timestamp,
                               const log_timestamp_printer_t printer)
{
    struct tm t;  // 存储时间信息的结构体

    // 获取当前系统时间并转换为 tm 结构体格式
    my_clock_get_time(&t);

    // 格式化时间戳为 "[YYYY-MM-DD HH:MM:SS] " 格式并打印
    // 注意：tm_year 需要加上 1900，tm_mon 需要加上 1（因为 tm_mon 范围是 0-11）
    return printer(output, "[%04d-%02d-%02d %02d:%02d:%02d] ",
                  t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                  t.tm_hour, t.tm_min, t.tm_sec);
}

/********************************************************************
**函数名称:  main
**入口参数:  无
**出口参数:  无
**函数功能:  初始化 G-Sensor 电源 和 QMI8658B 六轴传感器测试接口
**返回值:    0 表示初始化成功，负值表示初始化失败
**注意事项:  Shell 子系统由 Zephyr 自动启动，本函数仅完成测试环境初始化
*********************************************************************/
int main(void)
{
    int err;

    // 设置自定义日志时间戳格式化函数
    log_custom_timestamp_set(custom_timestamp_formatter);

    err = my_gsensor_init(&s_gsensor_task_id);
    if (err != 0)
    {
        LOG_ERR("GSENSOR power initialization failed (err %d)", err);
        return err;
    }

    LOG_INF("QMI8658B shell test is ready");

    for (;;)
    {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
