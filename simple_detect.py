# -*- coding: utf-8 -*-
import sensor, image, lcd, time, utime, gc
import math
import KPU as kpu
from machine import Timer

from Maix import GPIO
from board import board_info
from fpioa_manager import fm

# --- 全局配置常量 ---

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

# 新的闪烁逻辑参数
GLOBAL_PERIOD_MS = 2000    # 大周期 2秒
SUB_PERIOD_MS = 200        # 小周期 200ms (100亮+100灭)
SUB_ON_MS = 100            # 小周期中亮的时长

# 面积阈值 (基于 QVGA 320x240 = 76800 像素)
TOTAL_PIXELS = 320 * 240
AREA_THRESH_80 = int(TOTAL_PIXELS * 0.80)  # ~61440
AREA_THRESH_13 = int(TOTAL_PIXELS * 0.13)  # ~9984

# --- 初始化 ---

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

# 时钟初始化
clock = time.clock()

# KPU 模型加载
task = kpu.load(MODEL_ADDR)
_ = kpu.init_yolo2(task, CONF_THRESHOLD, NMS_THRESHOLD, 5, ANCHOR)

# --- 定时器与LED控制逻辑 (中断驱动) ---

# 全局变量，用于主循环和中断之间通信
g_target_blink_count = 0     # 主循环写入：期望的闪烁次数
g_locked_blink_count = 0     # 中断内部使用：当前周期锁定的次数
g_timer_start_ms = 0         # 中断内部使用：当前周期开始时间

def on_timer_blink(timer):
    """
    定时器回调函数：每 20ms 被触发一次
    负责精准控制 LED，不受主循环 FPS 影响
    """
    global g_target_blink_count, g_locked_blink_count, g_timer_start_ms, LED_ON, LED_OFF
    
    current_time = utime.ticks_ms()
    
    # 初始化
    if g_timer_start_ms == 0:
        g_timer_start_ms = current_time
        
    elapsed = utime.ticks_diff(current_time, g_timer_start_ms)
    
    # 1. 周期锁定逻辑：检查大周期是否结束
    if elapsed >= GLOBAL_PERIOD_MS:
        g_timer_start_ms = current_time
        elapsed = 0
        # 只有在周期开始时，才更新锁定的次数
        g_locked_blink_count = g_target_blink_count
        
    # 2. 执行闪烁逻辑
    active_duration = g_locked_blink_count * SUB_PERIOD_MS
    
    if elapsed < active_duration:
        # 处于活动期
        sub_phase = elapsed % SUB_PERIOD_MS
        if sub_phase < SUB_ON_MS:
            led_b.value(LED_ON)
        else:
            led_b.value(LED_OFF)
    else:
        # 处于休眠期
        led_b.value(LED_OFF)

# 启动定时器: TIMER0, CHANNEL0, 周期 20ms (50Hz)
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=20, callback=on_timer_blink)

# --- 辅助函数 ---

def _clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

# --- 主循环 ---

# 防抖状态变量
CONFIRM_FRAMES = 2
last_suggested_count = -1
confirm_counter = 0

while(True):
    clock.tick()
    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)

    best_det = None
    max_area = 0

    if code:
        for det in code:
            # 只处理目标类别 (人)
            if det.classid() == TARGET_CLASSID:
                # 绘制检测框
                _ = img.draw_rectangle(det.rect(), color=(0, 255, 0))
                # 修正：使用 img.draw_string 而不是 lcd.draw_string
                _ = img.draw_string(det.x(), det.y(), CLASSES[det.classid()], color=(255, 0, 0), scale=2)

                area = det.w() * det.h()
                if area > max_area:
                    max_area = area
                    best_det = det

    # --- 决定本帧建议的闪烁次数 ---
    suggested_blink_count = 0 
    
    if best_det:
        if max_area >= AREA_THRESH_80:
            suggested_blink_count = 5
        elif max_area >= (TOTAL_PIXELS * 0.40):
            suggested_blink_count = 2
        elif max_area >= AREA_THRESH_13:
            suggested_blink_count = 1
        else:
            suggested_blink_count = 0
            
    # --- 防抖逻辑 (Hysteresis) ---
    if suggested_blink_count == last_suggested_count:
        confirm_counter += 1
    else:
        confirm_counter = 0
        last_suggested_count = suggested_blink_count
        
    if confirm_counter >= CONFIRM_FRAMES:
        # 【关键】更新全局变量，通知定时器中断
        g_target_blink_count = suggested_blink_count
        # 为了防止溢出
        confirm_counter = CONFIRM_FRAMES 

    # --- 绘制辅助参考框 ---
    img.draw_rectangle(160-50, 120-50, 100, 100, color=(0, 0, 255))
    img.draw_rectangle(160-87, 120-87, 175, 175, color=(255, 255, 0))
    img.draw_rectangle(160-124, 120-124, 248, 248, color=(255, 0, 0))

    # --- 显示与输出 ---
    _ = lcd.display(img)

    # --- LED 控制已移交中断，主循环无需调用 ---
    
    gc.collect()

# --- 清理 ---
tim.stop() # 停止定时器
_ = kpu.deinit(task)
fm.unregister(IO_LED_RED)
print("Application stopped.")
