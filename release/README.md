# 可直接烧录的固件

| 文件 | 来源 | 大小 |
| --- | --- | --- |
| `C1.hex` | Keil 工程编译（`C1_Keil/Project/OBJ/C1.hex`） | 约 22.7KB 代码 |
| `C1_gcc.hex` | GCC 工程编译（`C1_GCC/build/C1.hex`） | 约 21.5KB 代码 |

两者功能完全相同，任选一个烧录即可。真正的产物以对应工具的编译输出为准，
这里只是方便没装工具链的队员直接拿去烧：

```
STM32CubeProgrammer / DAP 下载器 / Keil Download 都可以直接烧 .hex
```

烧录前请确认：

1. 调试口用 **SWD**（不能用 JTAG，PA15/PB3 已用作编码器输入）
2. **先不接电机电源**
3. LCD/触摸屏拔掉（EN 用的 PA5/PF11 是触摸屏引脚）
