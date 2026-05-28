# zmk_config_zitaotech_sofle

ZitaoTech Sofle 分体键盘的 ZMK 配置。

## 当前层结构

当前只保留 4 个有效层，空的 `RESERVE` 层已经删除。

| 层编号 | 显示名 | 节点名 | 主要入口 |
| --- | --- | --- | --- |
| 0 | QWERTY | `default_layer` | 默认层 |
| 1 | FUNC | `lower_layer` | QWERTY 拇指键 `&lt 1 SPACE` |
| 2 | NUM | `raise_layer` | Caps 双击 `&to NUM` |
| 3 | MOUSE | `MOUSE_layer` | FUNC 里的 `&mo 3`，以及 `zip_temp_layer 3 600` |

重要约束：

- `#define NUM 2` 让 Caps 双击跳到第 2 层。
- `MOUSE_layer` 必须继续是第 3 层。指点设备驱动和临时鼠标层处理器都依赖层号 3。
- `zip_temp_layer 3 600` 必须继续指向第 3 层。
- 原来的空 `RESERVE` 层没有入口、没有绑定、没有运行价值，所以已删除。

## 指点设备逻辑

三个指点设备共用同一套层编号假设：

- `config/boards/shields/right_trackpoint/custom_driver_right/trackpoint_0x15.c`
- `config/boards/shields/left_bbtrackpad/custom_driver_left/a320.c`
- `config/boards/shields/left_bbtrackball/custom_driver_left/bbtrackball_input_handler.c`

注意：右手 `right_trackpoint` 是 split peripheral，不能直接查询 central 的 keymap 状态。它只上报指点输入；`config/boards/arm/zitaotech_sofle/custom_driver_left/trackpoint_scroll_processor.c` 在左手 central 侧负责判断 `MOUSE_layer` 和物理位 61，并把 TrackPoint X/Y 转成滚轮。

物理位监听：

| 物理位 | 用途 |
| --- | --- |
| 34 | 方向键模式监听 |
| 36 | 慢速/精细模式监听，取决于具体设备支持 |
| 61 | 滚动监听；右手 TrackPoint 在 central 侧确认最高层是 `MOUSE_layer` 后才生效 |

物理位 60 不再参与滚动监听。在当前 MOUSE 层里，60 只保留普通鼠标键行为。

## 键位图

以下图片是 Word 布局迁移后的维护参考图。以后如果继续改键位，需要同步更新两份 keymap 和这里的图片。

### QWERTY

![QWERTY layer](docs/keymap-images/qwerty.png)

### FUNC

![FUNC layer](docs/keymap-images/func.png)

### NUM

![NUM layer](docs/keymap-images/num.png)

### MOUSE

![MOUSE layer](docs/keymap-images/mouse.png)

## 双份 keymap 同步要求

这两份 keymap 必须保持同步：

- `config/zitaotech_sofle.keymap`
- `config/boards/arm/zitaotech_sofle/zitaotech_sofle.keymap`

提交布局改动前请检查：

- 两份 keymap 内容一致；
- `QWERTY`、`FUNC`、`NUM`、`MOUSE` 每层都是 66 个 bindings；
- `MOUSE_layer` 仍是第 4 个声明的层，也就是层编号 3；
- 不要重新引入 `RES_layer`、`#define RES`、`&to 4`、`&mo 4` 或 `&lt 4`；
- `MOUSE_LAYER_ID 3` 仍只指向 `MOUSE_layer`；右手 TrackPoint 的层判断必须留在 central 侧；
- 滚动模式只监听物理位 61，不监听 60。

## 建议验收

建议先编译固件，再上板检查：

- Caps 双击进入 `NUM`，且 `NUM` 层可以回到 QWERTY。
- QWERTY 拇指 `&lt 1 SPACE` 按住进 `FUNC`，轻敲输出 Space。
- FUNC 里的 `&mo 3` 能进入 `MOUSE`。
- 指点设备移动仍能通过 `zip_temp_layer 3 600` 临时触发第 3 层。
- 在 MOUSE 层，物理位 61 启用滚动；物理位 60 不启用滚动。
- 物理位 34 和 36 仍按预期作为方向/精细模式监听。
