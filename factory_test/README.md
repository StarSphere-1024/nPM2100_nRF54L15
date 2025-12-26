# factory_test

工厂测试固件（基于 Zephyr/NCS）。

## 串口

- Grove 口 UART：P1.12 = TX，P1.11 = RX
- UART 用于 **Shell 命令交互** 与 **日志输出**

## 模式

- **工作模式（默认）**：LED0 以1Hz频率闪烁
- **测试模式**：LED0 常亮，通过 shell 命令进入并启动各测试项

切换命令：

```sh
factory mode work
factory mode test
```

> 切回 work 会自动停止 NFC/扫描/广播。

## 测试项命令

```sh
factory nfc start
factory nfc stop

factory bt_scan start
factory bt_scan stop

factory bt_adv start
factory bt_adv stop

factory shipmode
factory system_off

factory pmic_gpio0 high
factory pmic_gpio0 low

factory pmic_gpio_int_test
factory lfxo_test

i2c scan i2c30
i2c read i2c22 0x6A 0x0F
i2c write i2c22 0x6A 0x10 0x00

gpio conf gpio0 0 ou1
gpio get gpio0 0
gpio set gpio0 0 1
gpio toggle gpio0 0
gpio devices
gpio blink gpio0 0
gpio info gpio0
```

`factory nfc start|stop` 启动或停止NFC标签仿真，启动时LED闪一下后熄灭，检测到NFC阅读器时LED亮起，停止后LED恢复常亮。启动后外部NFC阅读器可读取标签内容，停止后标签不可读取。

`factory bt_scan start|stop` 启动或停止BLE设备扫描，启动后每5秒在串口打印发现的BLE设备信息（地址、RSSI、类型），显示最近10个设备。

`factory bt_adv start|stop` 启动或停止BLE广播，启动后本设备作为BLE外围设备可被其他设备发现和连接。

`factory shipmode` 进入船运模式，系统断电并切断电源，需按船运键2s恢复。

`factory system_off` 进入系统关闭模式，进入前LED关闭，系统深度睡眠5秒后自动唤醒并重启。

`factory pmic_gpio0 high|low` 设置PMIC GPIO0输出为高电平或低电平，用于外部电路测试。

`factory pmic_gpio_int_test` 配置PMIC GPIO0为输入模式，当GPIO0电平发生变化时，在串口日志中显示上升沿或下降沿事件。

`factory lfxo_test` 测试低频晶振精度，运行10秒后在串口打印时钟偏差和误差值（ppb）。

`i2c scan <设备>` 扫描I2C总线上的设备。

`i2c read <设备> <地址> <寄存器> [<数据长度>]` 从I2C设备读取数据。

`i2c write <设备> <地址> <寄存器> [<数据>]` 向I2C设备写入数据。

`gpio conf <设备> <引脚> <配置>` 配置GPIO引脚（i|o 输入输出，u|d 上拉下拉，h|l 高低有效，0|1 初始化电平）。

`gpio get <设备> <引脚>` 读取GPIO引脚状态。

`gpio set <设备> <引脚> <电平>` 设置GPIO引脚输出。

`gpio toggle <设备> <引脚>` 切换GPIO引脚状态。

`gpio devices` 列出所有GPIO设备。

`gpio blink <设备> <引脚>` 使GPIO引脚闪烁。

`gpio info [设备]` 显示GPIO设备信息。

## 按键

- **PMIC 船运按键**：已在 devicetree 中启用 shiphold 行为（长按进入 ship）
- **用户按键**：按下触发冷启动复位
