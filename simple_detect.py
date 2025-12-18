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

# 面积阈值 (基于 QVGA 320x240 = 76800 像素)
TOTAL_PIXELS = 320 * 240
AREA_THRESH_80 = int(TOTAL_PIXELS * 0.60)  # ~53760 (极近 70%)
AREA_THRESH_13 = int(TOTAL_PIXELS * 0.13)  # ~9984  (中等起步)

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

# 时钟初始化
clock = time.clock()

# KPU 模型加载
task = kpu.load(MODEL_ADDR)
_ = kpu.init_yolo2(task, CONF_THRESHOLD, NMS_THRESHOLD, 5, ANCHOR)

# --- 3. 定时器中断 (LED 控制核心) ---

# 全局变量：主循环与中断通信
g_target_blink_count = 0     # 主循环写入：期望的闪烁次数
g_locked_blink_count = 0     # 中断内部使用：当前周期锁定的次数
g_timer_start_ms = 0         # 中断内部使用：当前周期开始时间

def on_timer_blink(timer):
    """
    定时器回调：每 20ms 执行一次
    功能：实现双周期锁定与精准 LED 控制
    """
    global g_target_blink_count, g_locked_blink_count, g_timer_start_ms, LED_ON, LED_OFF
    
    current_time = utime.ticks_ms()
    
    # 首次运行初始化
    if g_timer_start_ms == 0:
        g_timer_start_ms = current_time
        
    elapsed = utime.ticks_diff(current_time, g_timer_start_ms)
    
    # [逻辑 A] 周期锁定：检查大周期是否结束
    if elapsed >= GLOBAL_PERIOD_MS:
        g_timer_start_ms = current_time
        elapsed = 0
        # 仅在周期开始瞬间，采纳新的目标次数 (实现 Period Locking)
        g_locked_blink_count = g_target_blink_count
        
    # [逻辑 B] 执行闪烁
    active_duration = g_locked_blink_count * SUB_PERIOD_MS
    
    if elapsed < active_duration:
        # 处于活动期 (Active Phase)
        sub_phase = elapsed % SUB_PERIOD_MS
        if sub_phase < SUB_ON_MS:
            led_b.value(LED_ON)
        else:
            led_b.value(LED_OFF)
    else:
        # 处于休眠期 (Rest Phase)
        led_b.value(LED_OFF)

# 启动定时器: TIMER0, CHANNEL0, 周期 20ms (50Hz)
tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=20, callback=on_timer_blink)

# --- 辅助函数 ---

def _clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

def draw_ref_rect(img, area, color):
    """
    根据面积自动画出一个 4:3 比例的参考框，并居中显示
    """
    # 屏幕比例 4:3 => W = 4/3 * H
    # Area = W * H = 4/3 * H^2
    # H^2 = Area * 3/4
    h = int(math.sqrt(area * 3 / 4))
    w = int(h * 4 / 3)
    
    # 屏幕中心 (160, 120)
    x = 160 - w // 2
    y = 120 - h // 2
    
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
            
        if confirm_counter >= CONFIRM_FRAMES:
            # 只有连续 N 帧一致，才更新给定时器
            g_target_blink_count = suggested_blink_count
            confirm_counter = CONFIRM_FRAMES # 防止溢出

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