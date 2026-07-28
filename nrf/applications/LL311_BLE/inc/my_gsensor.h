#ifndef _MY_GSENSOR_H_
#define _MY_GSENSOR_H_

/********************************************************************
**函数名称:  my_gsensor_init
**入口参数:  tid      ---        指向线程 ID 变量的指针
**出口参数:  tid      ---        存储启动后的线程 ID
**函数功能:  初始化 G-Sensor 相关的 I2C 设备与 GPIO 引脚，并启动 G-Sensor 线程
**返 回 值:  0 表示成功，负值表示失败
*********************************************************************/
int my_gsensor_init(k_tid_t *tid);

#endif /* _MY_GSENSOR_H_ */
