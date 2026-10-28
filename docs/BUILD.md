# 构建与测试

源快照来自开发仓库提交 `3afd23f8d3c752da9baa468c33a10c98a1d251e2`。FPGA、共享算法和 UART RTL 保持原样，参赛整理仅调整文档、入口和控制台作品名称。

## 工具

- Gowin V1.9.11.03 Education x64；器件 GW2A-LV18PG256C8/I7（工具名称 GW2A-18C）。安装及许可按厂商要求，不随代码包分发工具或许可证。
- PowerShell 7；尤其 Flash 脚本使用当前 .NET 的进程树终止接口。
- Python 3.10+ 与 Tkinter、pySerial 3.5。
- Icarus Verilog（Windows 提供 `iverilog.exe` 与 `vvp.exe`）；指定自己的安装目录。源码和构建目录采用英文路径。

## 复现 V8d 发布配置

```powershell
pwsh -File scripts/build-release.ps1 -GowinHome 'D:\Gowin\Gowin_V1.9.11.03_Education_x64' -BuildDirectory 'C:\fpga_work\pantilt_v8d'
python scripts/inspect-camera-build.py --help
```

包装脚本等价于 `build.ps1 -CameraUart -ShapeDiagnostics`，在构建副本中开启串口联动与形状诊断，默认仍关闭 PWM。不要把通用 `build.ps1` 不带参数的结果当作发布版。输出在构建目录 `impl/pnr/OV5640_HDMI720P.fs`；复现源码配置不代表不同工具／路径下位流必定逐字节一致。

`.gprj` 除 `fpga/` 外还引用 `algorithm_sim/rtl/` 和 `servo_uart/src/`。即使发布版只显示原图，也要保留这些源文件和全部 Gowin `.v/.ipc`。`fpga/impl/` 两个过程配置是构建输入，不是可删缓存。

当前发布包是已经上板验收的位流，重新编译不会自动替换它，也不会自动下载到开发板。

## 不接硬件的检查

```powershell
python scripts/verify-submission.py
python -m pip install -r desktop_control/requirements.txt
python -m unittest desktop_control.test_control desktop_control.test_ports desktop_control.test_app -v
pwsh -File scripts/test-red-ball-shape.ps1 -IverilogDirectory 'C:\iverilog\bin' -Only shape,contour,contour3,minrun,merge,merge3,detector64,detector160,multicolor
pwsh -File scripts/test-camera-servo-uart.ps1 -IverilogDirectory 'C:\iverilog\bin'
pwsh -File scripts/test-camera-servo-pixels.ps1 -IverilogDirectory 'C:\iverilog\bin'
pwsh -File scripts/test-red-ball-shape.ps1 -IverilogDirectory 'C:\iverilog\bin' -Smoke720p -Only merge720p_smoke,pipeline720p_smoke
```

长尺寸视频仿真耗时较长。测试输出在 `reports/runtime/`，不纳入提交代码包。驱动特有的 `0xC0000005` 异常与实际仿真结果分开记录，脚本只有在新 VVP 完整时才继续执行，并核验仿真退出码、PASS 标志和源码哈希。

## 板上验证

按 `QUICKSTART.md` 操作。不要在未核对文件、工具、接线和独立舵机供电状态时执行下载脚本。

只读通信检查（先关闭控制台，端口按实际枚举选择）：

```powershell
python scripts/test_servo_uart_soak.py --port COM12 --duration 10 --output reports/runtime/uart_check.json
```

先创建输出目录。该脚本只发 HELLO、GET_LIMITS、GET_STATUS，不发送移动或启用。成功通信不代表舵机已到达指令角度。
