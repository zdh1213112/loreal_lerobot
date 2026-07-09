# USB 串口设备固定为 `/dev/ttyDHRight`

## 1. 背景

设备连接到电脑后，系统会识别为类似：

```bash
/dev/ttyUSB0
```

但是 `/dev/ttyUSB0` 这个编号并不固定。  
如果插拔设备、增加其他 USB 串口设备，编号可能变成：

```bash
/dev/ttyUSB1
/dev/ttyUSB2
```

因此，推荐使用 `udev` 规则，为设备创建一个固定名称：

```bash
/dev/ttyDHRight
```

程序中以后只需要连接：

```bash
/dev/ttyDHRight
```

---

## 2. 查看当前 USB 串口信息

插入设备后，执行：

```bash
udevadm info -q property -n /dev/ttyUSB0
```

本设备的关键信息如下：

```text
ID_VENDOR_ID=1a86
ID_MODEL_ID=7523
ID_USB_DRIVER=ch341
ID_PATH=pci-0000:00:14.0-usb-0:3:1.0
```

该设备是 CH340 / CH341 USB 转串口设备。

由于该设备没有唯一的 `ID_SERIAL_SHORT`，所以这里使用 USB 物理端口路径 `ID_PATH` 来固定设备名。

---

## 3. 创建 udev 规则

创建规则文件：

```bash
sudo nano /etc/udev/rules.d/99-dh-right.rules
```

写入以下内容：

```udev
SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:3:1.0", SYMLINK+="ttyDHRight", MODE="0666", GROUP="dialout", ENV{ID_MM_DEVICE_IGNORE}="1"
```

保存并退出。

---

## 4. 重新加载 udev 规则

执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后拔掉设备并重新插入。

---

## 5. 检查是否生效

执行：

```bash
ls -l /dev/ttyDHRight
```

如果配置成功，会看到类似结果：

```bash
/dev/ttyDHRight -> ttyUSB0
```

也可以使用：

```bash
readlink -f /dev/ttyDHRight
```

输出应该类似：

```bash
/dev/ttyUSB0
```

---

## 6. 程序中使用固定端口

之后程序中不要再使用：

```bash
/dev/ttyUSB0
```

而是使用：

```bash
/dev/ttyDHRight
```

例如：

```python
serial_port = "/dev/ttyDHRight"
baudrate = 115200
```

---

## 7. 权限问题处理

如果程序打开串口时报权限错误，可以将当前用户加入 `dialout` 用户组：

```bash
sudo usermod -aG dialout $USER
```

然后注销并重新登录。

检查当前用户组：

```bash
groups
```

如果输出中包含：

```text
dialout
```

说明权限配置成功。

---

## 8. 注意事项

该规则是根据 USB 物理接口路径 `ID_PATH` 固定的。

也就是说，设备必须插在当前这个 USB 接口上：

```text
pci-0000:00:14.0-usb-0:3:1.0
```

如果设备换到其他 USB 口，`/dev/ttyDHRight` 可能不会生成。

如果更换 USB 口，需要重新查看新的 `ID_PATH`：

```bash
udevadm info -q property -n /dev/ttyUSB0 | grep ID_PATH
```

然后修改规则文件中的：

```udev
ENV{ID_PATH}=="..."
```

---

## 9. 常用检查命令

查看串口设备：

```bash
ls -l /dev/ttyUSB*
```

查看固定软链接：

```bash
ls -l /dev/ttyDHRight
```

查看 USB 串口详细信息：

```bash
udevadm info -q property -n /dev/ttyUSB0
```

查看稳定设备路径：

```bash
ls -l /dev/serial/by-path/
ls -l /dev/serial/by-id/
```

重新加载 udev 规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 10. 总结

最终目标是将不稳定的串口设备名：

```bash
/dev/ttyUSB0
```

固定为：

```bash
/dev/ttyDHRight
```

这样程序中可以长期使用固定路径，避免因为 USB 设备编号变化导致连接失败。