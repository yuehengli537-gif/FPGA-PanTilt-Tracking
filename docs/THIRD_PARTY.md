# 第三方来源与版权

视频基础工程来自 Sipeed 官方 [TangPrimer-20K-example](https://github.com/sipeed/TangPrimer-20K-example) 的 `Cam2HDMI/OV5640_HDMI720P_DDR3` 示例。原始下载 ZIP 的 SHA256 记录为 `4C51629A1B064E93C345DD98CCB2F0289AE8F566737FC9DA1202AA9B177109A9`。

摄像头配置、时钟、DDR3、帧缓存、HDMI及相关驱动以该示例为基础。Gowin 生成的加密 IP、`.ipc` 参数、原始版权头和其他第三方许可文字全部保留。第三方代码适用各自条款，本仓库不替它们重新指定 MIT 等通用许可证；没有将厂商工具、许可文件或运行环境打包。

项目后续开发包括采集／时序适配、红球候选合并与轮廓判定、目标状态处理、云台与串口控制、控制台及测试。完整开发历史见[原仓库](https://github.com/JIANJIAN06215/Tang-Primer-20k-camera/tree/feature/b-red-ball-shape)。当前导入提交与文件指纹见 `SOURCE_PROVENANCE.json`。

本项目直接依赖 Python 标准库与 pySerial 3.5；依赖通过安装工具获取，不把第三方运行库复制为项目原创代码。公开源代码不自动授予所有文件统一的再许可权。
