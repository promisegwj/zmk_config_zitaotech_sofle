# J键滚动模式速度优化方案

## 问题描述

**现象：** 长按J键进入滚动模式后，小红点推动页面时太敏感，轻轻碰一下就滚动很远。

---

## 当前滚动速度计算逻辑

**代码位置：** `trackpoint_0x15.c:167-172`

```c
#define SCROLL_THRESHOLD 8  // 滚动阈值

if (j_key_pressed) {
    // J键按住：滚动模式
    int16_t scaled_dx = -(int16_t)dx * 3 / 2 * tp_factor;
    int16_t scaled_dy = -(int16_t)dy * 3 / 2 * tp_factor;

    scroll_accumulator_x += scaled_dx;
    scroll_accumulator_y += scaled_dy;

    if (abs(scroll_accumulator_y) >= SCROLL_THRESHOLD) {
        scroll_y = scroll_accumulator_y / SCROLL_THRESHOLD;
        scroll_accumulator_y = scroll_accumulator_y % SCROLL_THRESHOLD;
    }
    // 发送滚动事件
    input_report_rel(dev, INPUT_REL_WHEEL, scroll_y, true, K_FOREVER);
}
```

**参数说明：**
- `dx, dy` - 小红点原始位移值（-127 ~ 127）
- `3 / 2` - 基础速度倍率 = 1.5倍
- `tp_factor` - 根据LED亮度动态调整的系数（0.4 ~ 0.9）
- `SCROLL_THRESHOLD` - 滚动阈值8，累计超过8才发送1格滚动

**当前问题：**
- 滚动模式和鼠标移动使用**相同的速度倍率**（1.5 * tp_factor）
- 但滚轮的敏感度天生比鼠标移动高，需要更低的速度

---

## 优化方案对比

### 方案1：增加滚动专用速度倍率（推荐）

**思路：** 给滚动模式单独设置一个更低的倍率

**修改位置：** `trackpoint_0x15.c:51`

```c
/* ========= 滚动模式状态 ========= */
#define SCROLL_THRESHOLD 8
#define SCROLL_SPEED_DIVISOR 2  // 新增：滚动速度除数
```

**修改位置：** `trackpoint_0x15.c:167-168`

```c
// 修改前：
int16_t scaled_dx = -(int16_t)dx * 3 / 2 * tp_factor;
int16_t scaled_dy = -(int16_t)dy * 3 / 2 * tp_factor;

// 修改后：
int16_t scaled_dx = -(int16_t)dx * 3 / 2 / SCROLL_SPEED_DIVISOR * tp_factor;
int16_t scaled_dy = -(int16_t)dy * 3 / 2 / SCROLL_SPEED_DIVISOR * tp_factor;
```

**效果：**
- 滚动速度降低为原来的 **1/2**（50%）
- 例如：原来碰一下滚动10格 → 现在滚动5格
- 鼠标移动速度保持不变

**优点：**
- ✅ 精确控制滚动速度
- ✅ 不影响鼠标移动速度
- ✅ 参数可调（可以改成3、4进一步降低）

**缺点：**
- ⚠️ 需要测试确定最佳除数值

---

### 方案2：提高滚动阈值

**思路：** 增加累计阈值，降低滚动触发频率

**修改位置：** `trackpoint_0x15.c:51`

```c
// 修改前：
#define SCROLL_THRESHOLD 8

// 修改后：
#define SCROLL_THRESHOLD 16  // 或 20、24
```

**效果：**
- 需要累计更多的位移才发送滚动事件
- 滚动速度降低为原来的 **1/2**（如果改为16）

**优点：**
- ✅ 实现简单，只改一个数字
- ✅ 保持速度平滑度

**缺点：**
- ⚠️ 可能导致小幅度滚动无法触发
- ⚠️ 影响滚动精度（每格滚动的间隔变大）

---

### 方案3：增加防抖时间

**思路：** 限制滚动事件的发送频率

**修改位置：** `trackpoint_0x15.c` 新增防抖逻辑

```c
#define SCROLL_DEBOUNCE_MS 50  // 滚动防抖：50ms内不重复发送
static uint32_t last_scroll_time = 0;

// 发送滚动前检查：
uint32_t now = k_uptime_get_32();
if (scroll_x != 0 || scroll_y != 0) {
    if (now - last_scroll_time < SCROLL_DEBOUNCE_MS) {
        // 防抖：跳过此次滚动
        return;
    }
    input_report_rel(dev, INPUT_REL_WHEEL, scroll_y, true, K_FOREVER);
    last_scroll_time = now;
}
```

**效果：**
- 限制滚动频率为 **20次/秒**（1000ms / 50ms）
- 快速推动时滚动次数减少

**优点：**
- ✅ 保持滚动精度
- ✅ 有效降低快速操作时的敏感度

**缺点：**
- ⚠️ 慢速滚动时可能感觉卡顿
- ⚠️ 不改变单次滚动的距离

---

### 方案4：组合优化（最佳方案）

**思路：** 结合方案1和方案2的优点

**修改：**
```c
#define SCROLL_THRESHOLD 12        // 从8增加到12（提高33%）
#define SCROLL_SPEED_DIVISOR 2     // 速度除数2（降低50%）

// 滚动计算：
int16_t scaled_dx = -(int16_t)dx * 3 / 2 / SCROLL_SPEED_DIVISOR * tp_factor;
int16_t scaled_dy = -(int16_t)dy * 3 / 2 / SCROLL_SPEED_DIVISOR * tp_factor;
```

**综合效果：**
- 速度降低：`1 / 2 = 50%`
- 阈值提高：`12 / 8 = 150%`
- **总速度降低：** `50% × (8/12) ≈ 33%`（降低为原来的1/3）

**优点：**
- ✅ 平衡速度和精度
- ✅ 不会过度影响小幅度滚动
- ✅ 用户感知更自然

---

## 推荐方案

**我推荐方案1（滚动专用速度倍率）**，理由：

1. **实现简单** - 只需添加一个除数参数
2. **效果直观** - 速度降低比例明确
3. **不影响其他功能** - 鼠标移动速度保持不变
4. **可调性强** - 用户反馈后可快速调整

**建议初始值：**
```c
#define SCROLL_SPEED_DIVISOR 2  // 先尝试降低到50%
```

如果还觉得快，可以逐步调整：
- `SCROLL_SPEED_DIVISOR = 3` → 速度降至33%
- `SCROLL_SPEED_DIVISOR = 4` → 速度降至25%

---

## 鼠标移动 vs 滚动对比

| 操作模式 | 当前速度倍率 | 修改后速度倍率 |
|---------|------------|--------------|
| 鼠标移动 | `dx * 3/2 * tp_factor` | `dx * 3/2 * tp_factor` |
| J键滚动 | `dx * 3/2 * tp_factor` | `dx * 3/2 / 2 * tp_factor` |

**说明：**
- 鼠标移动速度：保持不变（用户习惯）
- J键滚动速度：降低50%（解决敏感问题）

---

## 测试建议

修改后建议测试场景：
1. **轻微推动** - 原来滚动10格，现在应该滚动5格左右
2. **快速推动** - 原来一滚到底，现在应该可控
3. **慢速推动** - 应该能精准控制滚动1-2格
4. **鼠标移动** - 确认J键未按时鼠标速度不变

---

## 已执行方案

**选择方案1：滚动专用速度除数**

**执行时间：** 2026-02-25
**提交哈希：** e0e3277

**修改内容：**
```c
// 文件：trackpoint_0x15.c

// 第52行：添加速度除数定义
#define SCROLL_SPEED_DIVISOR 2  // 滚动速度降低到50%

// 第168-169行：修改滚动计算公式
int16_t scaled_dx = -(int16_t)dx * 3 / 2 / SCROLL_SPEED_DIVISOR * tp_factor;
int16_t scaled_dy = -(int16_t)dy * 3 / 2 / SCROLL_SPEED_DIVISOR * tp_factor;
```

**预期效果：**
- J键滚动速度降低50%（原来滚动10格 → 现在滚动5格）
- 鼠标移动速度保持不变
- 如需进一步调整，修改 `SCROLL_SPEED_DIVISOR` 为 3（33%）或 4（25%）

**后续调整建议：**
- 如果还觉得快：改为 `SCROLL_SPEED_DIVISOR 3`
- 如果觉得太慢：改为 `SCROLL_SPEED_DIVISOR 1`（相当于恢复原速的50%）

---

**文档创建时间：** 2026-02-25
**创建者：** 贾维斯
**状态：** ✅ 已执行（等待测试反馈）
