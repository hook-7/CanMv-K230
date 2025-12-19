# -*- coding: utf-8 -*-
import sensor, image, lcd, time, utime, gc
import math
import KPU as kpu
from machine import Timer

from Maix import GPIO
from board import board_info
from fpioa_manager import fm

# --- 1. 全局配置常量 ---

# LED/GPIO 设置
IO_LED_RED = 13
LED_ACTIVE_LOW = True  # LED 是否为低电平点亮

# 模型与检测设置
MODEL_ADDR = 0x800000
CLASSES = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']
ANCHOR = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)
CONF_THRESHOLD = 0.5   # 置信度阈值
NMS_THRESHOLD = 0.3    # NMS 阈值

TARGET_CLASSID = 14    # 'person'

# 闪烁逻辑参数 (双周期控制)
GLOBAL_PERIOD_MS = 2000    # 大周期 2秒 (2000ms)
SUB_PERIOD_MS = 200        # 小周期 200ms (100亮+100灭)
SUB_ON_MS = 100            # 小周期中亮的时长

# 面积阈值将在摄像头初始化后根据实际分辨率自动计算
# 初始值（会在初始化时被覆盖）
TOTAL_PIXELS = 320 * 240
AREA_THRESH_80 = int(TOTAL_PIXELS * 0.20)
AREA_THRESH_13 = int(TOTAL_PIXELS * 0.02)

# 防抖参数
CONFIRM_FRAMES = 2         # 连续确认帧数

# --- 2. 硬件初始化 ---

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

# 获取实际分辨率（自动适配）
CAMERA_WIDTH = sensor.width()
CAMERA_HEIGHT = sensor.height()
TOTAL_PIXELS = CAMERA_WIDTH * CAMERA_HEIGHT

# 重新计算面积阈值（基于实际分辨率）
AREA_THRESH_80 = int(TOTAL_PIXELS * 0.20)  # 20% 阈值
AREA_THRESH_13 = int(TOTAL_PIXELS * 0.02)  # 2% 阈值

print("摄像头分辨率: {}x{}, 总像素: {}".format(CAMERA_WIDTH, CAMERA_HEIGHT, TOTAL_PIXELS))

# 时钟初始化
clock = time.clock()

# KPU 模型加载
task = kpu.load(MODEL_ADDR)
_ = kpu.init_yolo2(task, CONF_THRESHOLD, NMS_THRESHOLD, 5, ANCHOR)

# --- 3. LED 闪烁控制函数和定时器（递减逻辑） ---

# 全局变量：使用绝对时间戳 (解决卡顿导致的时间拉长问题)
g_period_start_time = 0    # 周期开始的系统时间戳 (ms)
g_blink_duration = 0       # 预计闪烁持续时长 (ms)
g_cooldown_duration = 0    # 总周期/冷却时长 (ms)

def update_led_blink(blink_count):
    """
    LED闪烁控制函数 (基于系统时钟)
    """
    global g_period_start_time, g_blink_duration, g_cooldown_duration
    
    current_time = time.ticks_ms()
    
    # 计算当前周期是否已经结束
    # 如果从未开始过 (0) 或者 (当前时间 - 开始时间) 已经超过了 冷却时长
    elapsed = time.ticks_diff(current_time, g_period_start_time)
    
    if g_period_start_time == 0 or elapsed >= g_cooldown_duration:
        # 可以开始新周期
        
        # 计算参数
        blink_duration = blink_count * SUB_PERIOD_MS
        cooldown_duration = max(GLOBAL_PERIOD_MS, blink_duration)
        
        # 更新全局状态
        g_period_start_time = current_time
        g_blink_duration = blink_duration
        g_cooldown_duration = cooldown_duration
        
        if blink_count > 0:
            print("[LED] 新周期: 闪{}次 ({}ms), 冷却直到 +{}ms".format(
                blink_count, blink_duration, cooldown_duration))

def on_timer_time(timer, arg=None):
    """
    定时器回调：基于绝对时间控制 LED
    """
    global g_period_start_time, g_blink_duration, LED_ON, LED_OFF
    
    if g_period_start_time == 0:
        led_b.value(LED_OFF)
        return

    # 计算自周期开始以来经过的时间
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, g_period_start_time)
    
    # 1. 判断是否在闪烁期内
    if elapsed < g_blink_duration:
        # 在闪烁期内，计算相位
        phase = elapsed % SUB_PERIOD_MS
        
        # 前 SUB_ON_MS 亮，后半段灭
        if phase < SUB_ON_MS:
            led_b.value(LED_ON)
        else:
            led_b.value(LED_OFF)
    else:
        # 2. 超过闪烁期 (处于冷却期或空闲期)，强制灭
        led_b.value(LED_OFF)

# 启动定时器: TIMER0, CHANNEL0, 周期 20ms (50Hz)
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=20, unit=Timer.UNIT_MS, callback=on_timer_time, arg=None, start=True, priority=1, div=0)

# --- 辅助函数 ---

def draw_ref_rect(img, area, color):
    """
    根据面积自动画出一个 4:3 比例的参考框，并居中显示
    """
    # 屏幕比例 4:3 => W = 4/3 * H
    # Area = W * H = 4/3 * H^2
    # H^2 = Area * 3/4
    h = int(math.sqrt(area * 3 / 4))
    w = int(h * 4 / 3)

    # 获取实际分辨率并计算屏幕中心
    center_x = sensor.width() // 2
    center_y = sensor.height() // 2
    x = center_x - w // 2
    y = center_y - h // 2

    img.draw_rectangle(x, y, w, h, color=color)

# --- 主循环 ---

# 防抖状态变量
last_suggested_count = -1
confirm_counter = 0

try:
    while(True):
        clock.tick()

        # 图像采集与 AI 推理
        img = sensor.snapshot()
        code = kpu.run_yolo2(task, img)

        best_det = None
        max_area = 0

        # --- A. 处理检测结果 ---
        if code:
            for det in code:
                # 仅关注目标类别 (Person)
                if det.classid() == TARGET_CLASSID:
                    # 绘制检测框 (绿色) 与 标签 (红色)
                    img.draw_rectangle(det.rect(), color=(0, 255, 0))
                    img.draw_string(det.x(), det.y(), CLASSES[det.classid()], color=(255, 0, 0), scale=2)

                    # 寻找最大目标
                    area = det.w() * det.h()
                    if area > max_area:
                        max_area = area
                        best_det = det

        # --- B. 决策逻辑 (状态机) ---
        suggested_blink_count = 0

        if best_det:
            # 计算并打印面积比例
            area_ratio = (max_area / TOTAL_PIXELS) * 100
            print("检测到目标，面积比例: {:.2f}%".format(area_ratio))

            if max_area >= AREA_THRESH_80:
                suggested_blink_count = 5  # 极近: 闪5次
            elif max_area > AREA_THRESH_13:
                suggested_blink_count = 2  # 中等 (13%~80%): 闪2次
            else:
                suggested_blink_count = 1  # 较远 (<=13%): 闪1次
        else:
            suggested_blink_count = 0      # 未检测到: 灭

        # --- C. 防抖逻辑 (Hysteresis) ---
        if suggested_blink_count == last_suggested_count:
            confirm_counter += 1
        else:
            confirm_counter = 0
            last_suggested_count = suggested_blink_count

        # --- D. 更新LED闪烁 ---
        # 修正逻辑：只有当检测结果稳定 (超过确认帧数) 时，才允许发起新的闪烁请求
        target_blink_count = 0
        if confirm_counter >= CONFIRM_FRAMES:
            target_blink_count = suggested_blink_count

        # 尝试更新 LED 状态
        # update_led_blink 内部会检查当前是否空闲 (counter == 0)
        # 如果当前正在闪烁 (counter > 0)，请求会被忽略，直到下一次空闲
        update_led_blink(target_blink_count)

        # --- 绘制辅助参考框 (自动计算 4:3 比例) ---
        # 1. 13% 参考框 (蓝色)
        draw_ref_rect(img, AREA_THRESH_13, color=(0, 0, 255))

        # 2. 70% 参考框 (红色)
        draw_ref_rect(img, AREA_THRESH_80, color=(255, 0, 0))

        # --- E. 显示与垃圾回收 ---
        lcd.display(img)
        # print("FPS: %.2f" % clock.fps())
        gc.collect()

except Exception as e:
    print("Error:", e)
finally:
    # --- 5. 清理资源 ---
    tim.stop()
    kpu.deinit(task)
    fm.unregister(IO_LED_RED)
    print("Application stopped.")
