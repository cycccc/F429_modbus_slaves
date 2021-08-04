# F429_modbus_slaves
基于STM32F429的一个简单的modbus从站

## TCP_MODE

### 描述
    南京沁恒以太网芯片 CH395Q 和 STM32F429 用SPI进行简单通讯
### 创建日期
    2021/7/3
### 详细信息
    CH395Q芯片SPI的操作方式和W25Qxx系列flash相似
    目前 CH395Q 操作库中只有SPI操作这一种方式
#### 修改历史
    2021/7/23
        目前只能是单socket使用，多socket没有实验成功，因该是参数，函数的问题
        尝试添加modbusTCP协议
        modbusTCP测试成功，但是有个问题，ch395询问引脚(INT#)不能放在中断触发中，否则只能打开socket一次，第二次无法打开，不知道为什么
    2021/7/24
        BUFF_MAX 大小最小要设1024 太小会通讯中断，不知道为什么
        还有，文件还有些小问题，读写寄存器没有问题，但是读取线圈不不对劲，没有修复，可以参考RTU_MODBUS

## RTU_MODE

### 描述
    使用ST芯片自带的串口和MAX485芯片进行485的收发
### 创建日期
    2021/8/1
### 详细信息
    与TCP_MODBUS可以共同使用
#### 修改历史
    2021/8/4
        已经修复TCP_MODBUS中线圈出现的问题