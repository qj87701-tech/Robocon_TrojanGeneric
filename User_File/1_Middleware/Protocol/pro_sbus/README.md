# NRF24L01遥控接收协议

## 1. 本次目标

接收NRF24L01发送的31字节遥控数据包，解析16路SBUS通道和串口屏自定义数据，并保存为后续电机、底盘和机构模块可读取的数据。

本模块本次只负责接收协议，不直接控制达妙电机，也不使用RTOS、队列或SUBS。

## 2. 模块位置

协议代码放置在：

```text
User_File/1_Middleware/Protocol/pro_sbus/
├── pro_sbus.h
└── pro_sbus.cpp
```

NRF24L01仍只负责收取31字节原始数据，`Class_SBUS_Receive`负责校验、解包和失联处理。

## 3. 数据包格式

NRF24L01固定载荷长度为31字节。

| 字节 | 内容 |
| --- | --- |
| `data[0]` | SBUS帧头，固定为`0x0F` |
| `data[1..22]` | 16路SBUS通道，每路11 bit |
| `data[23]` | 状态标志，固定为`0xFF` |
| `data[24]` | SBUS帧尾，固定为`0xAF` |
| `data[25]` | bit0~1为模式，bit2为电机使能 |
| `data[26]` | 功能按钮1~8位图 |
| `data[27]` | 功能按钮9~16位图 |
| `data[28]` | 自定义数据标志，固定为`0x88` |
| `data[29..30]` | 保留字节，当前不参与校验 |

模式定义：

- `0`：无模式
- `1`：DEBUG
- `2`：MANUAL
- `3`：AUTOMATIC

## 4. 接收数据

`Class_SBUS_Receive`公开保存以下数据：

```cpp
typedef struct
{
    uint16_t Channel[16];
    uint8_t Mode;
    uint8_t Motor_Enable;
    uint8_t Function[16];
    uint8_t Online;
} Struct_SBUS_Data;
```

- `Channel[0..15]`保存16路SBUS原始值。
- `Function[0..15]`依次对应串口屏功能按钮1~16。
- `Online`表示最近100 ms内是否收到过有效数据包。

## 5. 数据处理

收到数据后依次检查：

```text
data[0]  == 0x0F
data[23] == 0xFF
data[24] == 0xAF
data[28] == 0x88
```

全部通过后，沿用R1工程的SBUS位运算解包方式解析16路通道，再解析模式、电机使能和16个功能按钮，并刷新在线状态。

任意一项校验失败时丢弃整帧，不修改上一次有效数据，也不刷新失联计时。

## 6. 初始化和失联处理

初始化或连续100 ms没有收到有效数据包时：

- `Channel[0..15]`全部恢复为中值`1000`。
- `Mode`清零。
- `Motor_Enable`清零。
- `Function[0..15]`全部清零。
- `Online`清零。

每收到一帧有效数据，失联计时重新开始，`Online`置1。

## 7. 裸机任务接入

- `Task_Init()`初始化`Class_SBUS_Receive`和`Class_NRF24L01`。
- `Task_Loop()`检查低电平的`NRF_IRQ`，接收原始数据并交给SBUS模块解析。
- `Task10ms_Callback()`调用SBUS的10 ms存活检测函数。
- 现有达妙电机和按键控制代码保持不变。
- `CMakeLists.txt`只增加`pro_sbus.cpp`源文件。

