# -*- coding: utf-8 -*-
import sensor, image, lcd, time, gc
import math
import KPU as kpu
from machine import Timer

from Maix import GPIO
from fpioa_manager import fm

# --- 1. 全局配置常量 ---

# LED/GPIO 设置
IO_LED_RED = 13
LED_ACTIVE_LOW = True  # LED 是否为低电平点亮

# 模型与检测设置
MODEL_ADDR = 0x800000
CLASSES = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']
ANCHOR = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)
CONF_THRESHOLD = 0.3   # 置信度阈值
NMS_THRESHOLD = 0.3    # NMS 阈值

TARGET_CLASSID = 14    # 'person'

# 闪烁逻辑参数
GLOBAL_PERIOD_MS = 2000    # 大周期 2秒
SUB_PERIOD_MS = 200        # 小周期 200ms (100亮+100灭)
SUB_ON_MS = 100            # 小周期中亮的时长

# 升级打断参数
# - 周期开始后至少过 1s 才允许“升级打断”
# - 只允许 new_blink_count > current_blink_count 打断
# - 打断发生时从打断那刻重启 2s 周期计时
INTERRUPT_MIN_MS = 1000

# 面积阈值比例配置
AREA_RATIO_MIN = 0.005     # 0.5% - 最小阈值
AREA_RATIO_13 = 0.02       # 2% - 中等阈值
AREA_RATIO_80 = 0.50       # 20% - 极近阈值

# 防抖参数
CONFIRM_FRAMES = 3       # 连续确认帧数

# 方向检测参数
DIRECTION_HISTORY_SIZE = 5  # 用于比较的历史帧数（计算平均值）
MAX_MISSED_FRAMES = 3       # 允许连续丢失的最大帧数，超过此值才清空历史数据

# 自适应平滑（EWMA）参数（按“折中：近距离更稳、远距离更跟手”）
# EWMA: smooth = alpha * raw + (1 - alpha) * prev
# - alpha 越小：平滑更强（更稳），但响应更慢
# - alpha 越大：响应更快（更跟手），但抖动抑制更弱
ALPHA_NEAR = 0.20  # 近距离（面积大、抖动大）→ 更小alpha → 更强平滑
ALPHA_FAR = 0.70   # 远距离（面积小、抖动小）→ 更大alpha → 更快响应
SMOOTHED_AREA_INIT = None  # 平滑面积初始值

# “靠近”判定的自适应阈值（percent）
# 远距离：浮动较小，阈值可以小一些；近距离：浮动较大，阈值要更大避免误判
APPROACH_EPS_FAR_PERCENT = 0.5
APPROACH_EPS_NEAR_PERCENT = 5.0
# --- 2. 辅助函数 ---

def calculate_area_thresholds(total_pixels):
    """根据总像素数计算面积阈值"""
    return {
        'min': int(total_pixels * AREA_RATIO_MIN),
        'mid': int(total_pixels * AREA_RATIO_13),
        'max': int(total_pixels * AREA_RATIO_80)
    }

# --- 3. 硬件初始化 ---

# LED 初始化
fm.register(IO_LED_RED, fm.fpioa.GPIO0)
led_b = GPIO(GPIO.GPIO0, GPIO.OUT)
LED_ON = 0 if LED_ACTIVE_LOW else 1
LED_OFF = 1 if LED_ACTIVE_LOW else 0
led_b.value(LED_OFF)

# LCD 初始化
lcd.init(type=2)
lcd.rotation(0)

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(1)
sensor.run(1)

# 获取实际分辨率并计算面积阈值
CAMERA_WIDTH = sensor.width()
CAMERA_HEIGHT = sensor.height()
TOTAL_PIXELS = CAMERA_WIDTH * CAMERA_HEIGHT
AREA_THRESH = calculate_area_thresholds(TOTAL_PIXELS)
AREA_THRESH_MIN = AREA_THRESH['min']
AREA_THRESH_13 = AREA_THRESH['mid']
AREA_THRESH_80 = AREA_THRESH['max']

print("摄像头分辨率: {}x{}, 总像素: {}".format(CAMERA_WIDTH, CAMERA_HEIGHT, TOTAL_PIXELS))
print("面积阈值: MIN={} (0.5%), MID={} (2%), MAX={} (20%)".format(
    AREA_THRESH_MIN, AREA_THRESH_13, AREA_THRESH_80))

# 时钟初始化
clock = time.clock()

# KPU 模型加载
task = kpu.load(MODEL_ADDR)
_ = kpu.init_yolo2(task, CONF_THRESHOLD, NMS_THRESHOLD, 5, ANCHOR)

# --- 4. LED 闪烁控制 ---

# LED 状态全局变量
g_period_start_time = 0
g_blink_duration = 0
g_cooldown_duration = 0
g_current_blink_count = 0  # 当前周期正在执行的闪烁次数（用于升级打断判断）

def is_period_ended():
    """检查当前周期是否已结束"""
    if g_period_start_time == 0:
        return True
    elapsed = time.ticks_diff(time.ticks_ms(), g_period_start_time)
    return elapsed >= g_cooldown_duration

def update_led_blink(blink_count):
    """LED闪烁控制函数"""
    global g_period_start_time, g_blink_duration, g_cooldown_duration, g_current_blink_count

    if blink_count == 0:
        if g_period_start_time != 0 and is_period_ended():
            g_period_start_time = 0
            g_blink_duration = 0
            g_cooldown_duration = 0
            g_current_blink_count = 0
            print("[LED] 周期结束，重置状态")
        return

    now_ms = time.ticks_ms()

    # 周期进行中：仅允许“升级打断”（new > current 且已过最小门限）
    if g_period_start_time != 0 and not is_period_ended():
        elapsed = time.ticks_diff(now_ms, g_period_start_time)
        if elapsed >= INTERRUPT_MIN_MS and blink_count > g_current_blink_count:
            blink_duration = blink_count * SUB_PERIOD_MS
            cooldown_duration = max(GLOBAL_PERIOD_MS, blink_duration)
            g_period_start_time = now_ms  # 打断：从现在起重启周期
            g_blink_duration = blink_duration
            g_cooldown_duration = cooldown_duration
            g_current_blink_count = blink_count
            print("[LED] 升级打断: 闪{}次 ({}ms), 冷却{}ms".format(
                blink_count, blink_duration, cooldown_duration))
        # 次数变少或未过门限：不打断，等待周期结束
        return

    # 周期已结束（或尚未开始）：开启新周期
    blink_duration = blink_count * SUB_PERIOD_MS
    cooldown_duration = max(GLOBAL_PERIOD_MS, blink_duration)
    g_period_start_time = now_ms
    g_blink_duration = blink_duration
    g_cooldown_duration = cooldown_duration
    g_current_blink_count = blink_count
    print("[LED] 新周期: 闪{}次 ({}ms), 冷却{}ms".format(
        blink_count, blink_duration, cooldown_duration))

def on_timer_time(timer, arg=None):
    """定时器回调：控制LED闪烁"""
    global g_period_start_time, g_blink_duration

    if g_period_start_time == 0:
        led_b.value(LED_OFF)
        return

    elapsed = time.ticks_diff(time.ticks_ms(), g_period_start_time)

    if elapsed < g_blink_duration:
        phase = elapsed % SUB_PERIOD_MS
        led_b.value(LED_ON if phase < SUB_ON_MS else LED_OFF)
    else:
        led_b.value(LED_OFF)

# 启动定时器
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC,
            period=20, unit=Timer.UNIT_MS, callback=on_timer_time,
            arg=None, start=True, priority=1, div=0)

def draw_ref_rect(img, area, color):
    """绘制4:3比例的参考框，居中显示"""
    h = int(math.sqrt(area * 3 / 4))
    w = int(h * 4 / 3)
    center_x = sensor.width() // 2
    center_y = sensor.height() // 2
    img.draw_rectangle(center_x - w // 2, center_y - h // 2, w, h, color=color)

def _clamp01(x):
    if x < 0:
        return 0.0
    if x > 1:
        return 1.0
    return x

def get_adaptive_alpha(area):
    """
    根据面积返回自适应 EWMA 的 alpha（反向关系：面积越大，alpha 越小）
    - 近距离：alpha ~= ALPHA_NEAR（更强平滑）
    - 远距离：alpha ~= ALPHA_FAR（更快响应）
    """
    if area <= AREA_THRESH_MIN:
        return ALPHA_FAR
    if area >= AREA_THRESH_80:
        return ALPHA_NEAR
    t = (area - AREA_THRESH_MIN) / (AREA_THRESH_80 - AREA_THRESH_MIN)
    t = _clamp01(t)
    # sqrt 让过渡更平滑：靠近阈值时变化更缓
    t = math.sqrt(t)
    return ALPHA_FAR - (ALPHA_FAR - ALPHA_NEAR) * t

def smooth_area_ewma(raw_area, prev_smoothed):
    """对面积做 EWMA 平滑（自适应 alpha）"""
    if prev_smoothed is None:
        return raw_area
    alpha = get_adaptive_alpha(raw_area)
    return alpha * raw_area + (1 - alpha) * prev_smoothed

def get_adaptive_approach_eps(area):
    """
    自适应“靠近”阈值 eps（比例），范围：远 0.5% → 近 5%
    返回值为比例（例如 0.005 表示 0.5%）
    """
    if area <= AREA_THRESH_MIN:
        return APPROACH_EPS_FAR_PERCENT / 100.0
    if area >= AREA_THRESH_80:
        return APPROACH_EPS_NEAR_PERCENT / 100.0
    t = (area - AREA_THRESH_MIN) / (AREA_THRESH_80 - AREA_THRESH_MIN)
    t = _clamp01(t)
    t = math.sqrt(t)
    eps_percent = APPROACH_EPS_FAR_PERCENT + (APPROACH_EPS_NEAR_PERCENT - APPROACH_EPS_FAR_PERCENT) * t
    return eps_percent / 100.0

def is_moving_closer(current_area, area_history, history_size):
    """判断目标是否在靠近（基于“平滑面积”的趋势判断 + 自适应阈值）"""
    # 定义：当前面积 > 历史平均面积 * (1 + eps)，eps 远 0.5% → 近 5%
    if len(area_history) < history_size:
        # 历史不足时无法判断“持续变大”，按“不靠近”处理
        return False
    avg_historical = sum(area_history[-history_size:]) / history_size
    eps = get_adaptive_approach_eps(current_area)
    return current_area > (avg_historical * (1.0 + eps))

def get_blink_count_by_area(area):
    """根据面积返回闪烁次数（旧逻辑，已停用，保留仅作参考）"""
    if area >= AREA_THRESH_80:
        return 5
    elif area > AREA_THRESH_13:
        return 2
    elif area > AREA_THRESH_MIN:
        return 1
    return 0

# --- 5. 主循环 ---

# 状态变量
last_blink_count = -1
confirm_counter = 0
area_history = []
lost_frame_counter = 0
smoothed_area = SMOOTHED_AREA_INIT

# 参考框配置
REF_BOXES = [
    (AREA_THRESH_MIN, (255, 255, 0)),  # 黄色 - 最小阈值
    (AREA_THRESH_13, (0, 0, 255)),     # 蓝色 - 中等阈值
    (AREA_THRESH_80, (255, 0, 0))      # 红色 - 极近阈值
]

try:
    while True:
        clock.tick()
        img = sensor.snapshot()
        code = kpu.run_yolo2(task, img)

        # 查找最大目标
        best_det = None
        max_area = 0
        if code:
            for det in code:
                if det.classid() == TARGET_CLASSID:
                    img.draw_rectangle(det.rect(), color=(0, 255, 0))
                    img.draw_string(det.x(), det.y(), CLASSES[det.classid()],
                                   color=(255, 0, 0), scale=2)
                    area = det.w() * det.h()
                    if area > max_area:
                        max_area = area
                        best_det = det

        # 决策逻辑
        suggested_blink_count = 0
        if best_det:
            lost_frame_counter = 0
            # 先平滑，再决策：近距离更稳、远距离更跟手
            smoothed_area = smooth_area_ewma(max_area, smoothed_area)
            area_for_decision = int(smoothed_area)

            raw_area_ratio = (max_area / TOTAL_PIXELS) * 100
            smooth_area_ratio = (area_for_decision / TOTAL_PIXELS) * 100
            current_eps = get_adaptive_approach_eps(area_for_decision) * 100  # 转换为百分比
            print("检测到目标，原始面积比例: {:.2f}%，平滑后: {:.2f}%，允许误差: {:.2f}%".format(
                raw_area_ratio, smooth_area_ratio, current_eps))

            # 新规则：
            # - area < MIN: 不闪
            # - area >= 80%阈值: 闪10下（等价持续闪）
            # - area < 80%阈值 且靠近: 闪3下
            # - area < 80%阈值 且不靠近(静止/远离): 闪1下
            if area_for_decision >= AREA_THRESH_80:
                suggested_blink_count = 10
                print("达到极近阈值(>=80%)，持续闪(10下)")
            elif area_for_decision >= AREA_THRESH_MIN:
                if is_moving_closer(area_for_decision, area_history, DIRECTION_HISTORY_SIZE):
                    suggested_blink_count = 3
                    print("目标在靠近(持续变大)，闪3下")
                else:
                    suggested_blink_count = 1
                    print("目标未靠近(静止/远离)，闪1下")
            else:
                suggested_blink_count = 0
                print("目标过小(<MIN)，不闪")

            # 历史记录也使用平滑后的面积，避免抖动导致误判
            area_history.append(area_for_decision)
            if len(area_history) > DIRECTION_HISTORY_SIZE:
                area_history = area_history[-DIRECTION_HISTORY_SIZE:]
        else:
            suggested_blink_count = 0
            lost_frame_counter += 1
            if lost_frame_counter >= MAX_MISSED_FRAMES:
                area_history = []
                smoothed_area = SMOOTHED_AREA_INIT
                lost_frame_counter = 0
                print("[方向检测] 连续丢失{}帧，清空历史数据".format(MAX_MISSED_FRAMES))

        # 防抖并更新LED
        if suggested_blink_count == last_blink_count:
            confirm_counter += 1
        else:
            confirm_counter = 0
            last_blink_count = suggested_blink_count

        if confirm_counter >= CONFIRM_FRAMES:
            update_led_blink(suggested_blink_count)

        # 绘制参考框
        #for area, color in REF_BOXES:
            #draw_ref_rect(img, area, color)

        lcd.display(img)
        gc.collect()

except Exception as e:
    print("Error:", e)
finally:
    tim.stop()
    kpu.deinit(task)
    fm.unregister(IO_LED_RED)
    print("Application stopped.")
