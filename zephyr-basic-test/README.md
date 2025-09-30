# XIAO nRF54L15 Zephyr测试项目

打开串口终端(波特率115200)进行测试。

## 可用命令

### I2C命令

- `i2c scan <设备>` - 扫描I2C总线上的设备。

  ```shell
  i2c scan i2c22
  ```

- `i2c read <设备> <地址> <寄存器> [<数据长度>]` - 从I2C设备读取数据。

  ```shell
  i2c read i2c22 0x6A 0x0F
  ```

- `i2c write <设备> <地址> <寄存器> [<数据>]` - 向I2C设备写入数据。

  ```shell
  i2c write i2c22 0x6A 0x10 0x00
  ```

### SPI命令

- `spi conf <设备> <频率> [<设置>]` - 配置SPI设备参数。

  ```shell
  spi conf spi00 1000000 ol
  ```

  设置选项:
  - `o` - SPI_MODE_CPOL（时钟极性）
  - `h` - SPI_MODE_CPHA（时钟相位）
  - `l` - SPI_TRANSFER_LSB（最低位优先）
  - `T` - SPI_FRAME_FORMAT_TI（TI帧格式）

- `spi transceive <发送字节1> [<发送字节2> ...]` - 在SPI设备上收发数据。

  ```shell
  spi transceive 0x55 0xAA
  ```

### GPIO命令

- `gpio conf <设备> <引脚> <配置>` - 配置GPIO引脚

  ```shell
  gpio conf gpio0 0 ou1  # 配置为输出，上拉，初始电平为1
  ```

  配置选项:
  - `i|o` - 输入/输出模式
  - `u|d` - 上拉/下拉（可选）
  - `h|l` - 高电平有效/低电平有效（可选，默认高电平有效）
  - `0|1` - 初始化电平（可选，默认为0）

- `gpio get <设备> <引脚>` - 读取GPIO引脚状态

  ```shell
  gpio get gpio0 0
  ```

- `gpio set <设备> <引脚> <电平>` - 设置GPIO引脚输出

  ```shell
  gpio set gpio0 0 1
  ```

- `gpio toggle <设备> <引脚>` - 切换GPIO引脚状态

  ```shell
  gpio toggle gpio0 0
  ```

- `gpio devices` - 列出所有GPIO设备

  ```shell
  gpio devices
  ```

- `gpio blink <设备> <引脚>` - 使GPIO引脚闪烁

  ```shell
  gpio blink gpio0 0
  ```

- `gpio info [设备]` - 显示GPIO设备信息

  ```shell
  gpio info gpio0
  ```

### 批量GPIO命令

- `gpios init` - 配置所有预设GPIO为输出模式。
- `gpios high` - 设置所有预设GPIO为高电平。
- `gpios low` - 设置所有预设GPIO为低电平。
- `gpios toggle` - 翻转所有预设GPIO的电平。

## 硬件要求

- XIAO nRF54L15开发板
- 可选：NFC天线（用于NFC功能）
- 可选：电池（用于电源管理功能）
- 可选：I2C/SPI外设（用于通信测试）
