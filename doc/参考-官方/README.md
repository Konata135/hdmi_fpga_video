# 官方样例里"留下来有用"的那一部分

原 `官方示例/` 目录（68 MB）已经移出工程，放在 **`F:\TD\官方示例_仅参考\官方示例\`**。
这里只留了真正会被反复引用、而且体量很小的东西（约 197 KB）：

| 内容 | 为什么留 |
|---|---|
| `设计参考例程文档.md`、`代码说明.md` | 厂商对样例行为的说明（4 张图、SDRAM 双缓冲、key1/key2 行为、卡格式要求），本工程的功能基线就是对齐它 |
| `pin.adc`、`timing.sdc` | **板上事实的来源**：HDMI 引脚属性（LOCATION/IOSTANDARD/PULLTYPE）与本工程 `constraints/` 里那条 `clkc[2]↔clkc[3]` 例外，都是照它抄的 |
| `HX4S20开发板手册-HDL版2410.pdf` | **开发板手册**（2.4 MB）：第 15 节 4 个乒乓开关 SW1-SW4 = C8/C7/C6/C5（拨上为 1），第 16 节 4 个按键 KEY1-KEY4 = A2/B2/B1/C1（按下为低）。本工程的复位/切图/轮播开关引脚就出自这里 |
| `hdl/*.v` | 样例里**未加密**的 SD 扫描 / BMP 读 / 帧 FIFO / 彩条 / 数码管 / 按键消抖 / 音频与 I2S 源码，用于交叉核对（本工程 RTL 全部自研，不复制它们） |
| `convert_images_to_bmp.py`、`sync_to_sd.py`、`convert/README.md` | 厂商的图片转换与写卡脚本，本工程 `tools/prepare_images.py` / `write_card.py` 的思路来源 |

## 移出去的那 68 MB 里还有什么

在 `F:\TD\官方示例_仅参考\官方示例\` 下，需要时再取：

* `doc/APUG092_HDMI1.4b_Transmitter_V1.0.docx`（5.3 MB）—— HDMI 1.4b 发送器 IP 手册，**M7 做音频数据岛 / TERC4 / BCH / ACR 时会用到**；
* `src/td_project/.../best_result/*.bit`（630 KB）—— **官方样例的 bit 文件**，可以用来做 A/B：把官方 bit 烧进去看它的 TF/SDRAM 通路在本板上是否正常；
* `doc/TF卡图片/*.bmp`（4.6 MB）—— 厂商实测可显示的 640×480 BMP，配合上面那个 bit 做对比；
* 其余（加密 IP 源码、IP 生成的 afifo、音频 ROM/MIF、图片转换的中间产物、TD 构建产物）都是**不可用或可再生成**的。

## 纪律

这些是**参考资料，不是本工程的代码**。本工程所有 RTL 自研；需要抄"板级事实"（引脚、时序约束、器件几何）时，
以本工程 `constraints/` 和 `doc/` 里的记录为准，并在此处与厂商原文对照。
