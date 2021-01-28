## 📕简要描述

**致在打造一个基于HAL库的适配全系列stm32的模块驱动库，目前进度如下**（*现阶段仅适配了F1和F4系列*）：

| 更新时间  |                           更新内容                           |
| --------- | :----------------------------------------------------------: |
| 2021.1.20 | 更新基于HAL库的模拟IIC库，OLED驱动库，精准微妙延时库，以及AT24C02库 |
| 2021.1.21 |       优化了模拟IIC（即只用初始化一次，后续函数通用）        |
| 2021.1.24 | 更新了MPU6050模拟IIC驱动以及DMP欧拉角解算（解决了硬件IIC偶尔启动失败等BUG） |
| 2021.1.25 | 重定义了HAL_Delay函数（解决了毫秒延时函数冲突导致程序卡死等BUG） |
| 2021.1.26 | 修复并验证了硬件IIC驱动OLED（解决了未使能硬件IIC时程序编译错误等BUG），重定向Printf |
| 2021.1.28 |            更新了硬件SPI读写函数以及片选函数控制             |

## 🔨使用方法

①引入**YTGLib_Dev**文件夹放入工程文件中（❗注：一定要引入到工程的最外层文件里，即工程根目录中）

> 输入：git submodule add https://github.com/name-longming/YTGLib_Dev

②在头文件**YTGLib_Dev.h**中定义你需要的使用到的模块

```c
/*********************选择你要开启的模块***********************/

#define OLED_INIT_IIC1		//初始化OLED,并选中为IIC1驱动
#define DELAY_US  			//初始化微妙延时
#define MYIIC				//初始化模拟IIC
#define MPU6050				//初始化MPU6050
#define PRINTF				//初始化重定向Printf
#define MYSPI				//初始化SPI读写函数

/**************************************************************/
```

③在任何需要使用的.c文件中引入**#include "../YTGLib_Dev/YTGLib_Dev.h"**即可调用相应模块函数

```c
#include "../YTGLib_Dev/YTGLib_Dev.h"
```
