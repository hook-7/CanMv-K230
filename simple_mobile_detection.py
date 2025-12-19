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
CONF_THRESHOLD = 0.5   # 置信度阈值
NMS_THRESHOLD = 0.3    # NMS 阈值

TARGET_CLASSID = 14    # 'person'

# 闪烁逻辑参数
GLOBAL_PERIOD_MS = 2000    # 大周期 2秒
SUB_PERIOD_MS = 200        # 小周期 200ms (100亮+100灭)
SUB_ON_MS = 100            # 小周期中亮的时长

# 面积阈值比例配置
AREA_RATIO_MIN = 0.005     # 0.5% - 最小阈值
AREA_RATIO_13 = 0.02       # 2% - 中等阈值
AREA_RATIO_80 = 0.20       # 20% - 极近阈值

# 防抖参数
CONFIRM_FRAMES = 1         # 连续确认帧数

# 方向检测参数
DIRECTION_HISTORY_SIZE = 3  # 用于比较的历史帧数（计算平均值）
MAX_MISSED_FRAMES = 3       # 允许连续丢失的最大帧数，超过此值才清空历史数据

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

def is_period_ended():
    """检查当前周期是否已结束"""
    if g_period_start_time == 0:
        return True
    elapsed = time.ticks_diff(time.ticks_ms(), g_period_start_time)
    return elapsed >= g_cooldown_duration

def update_led_blink(blink_count):
    """LED闪烁控制函数"""
    global g_period_start_time, g_blink_duration, g_cooldown_duration
    
    if blink_count == 0:
        if g_period_start_time != 0 and is_period_ended():
            g_period_start_time = 0
            g_blink_duration = 0
            g_cooldown_duration = 0
            print("[LED] 周期结束，重置状态")
        return
    
    if is_period_ended():
        blink_duration = blink_count * SUB_PERIOD_MS
        cooldown_duration = max(GLOBAL_PERIOD_MS, blink_duration)
        g_period_start_time = time.ticks_ms()
        g_blink_duration = blink_duration
        g_cooldown_duration = cooldown_duration
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

def is_moving_closer(current_area, area_history, history_size):
    """判断目标是否在靠近"""
    if len(area_history) < history_size:
        return True
    avg_historical = sum(area_history[-history_size:]) / history_size
    return current_area > avg_historical

def get_blink_count_by_area(area):
    """根据面积返回闪烁次数"""
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
            area_ratio = (max_area / TOTAL_PIXELS) * 100
            print("检测到目标，面积比例: {:.2f}%".format(area_ratio))

            suggested_blink_count = get_blink_count_by_area(max_area)
            
            if suggested_blink_count > 0:
                if not is_moving_closer(max_area, area_history, DIRECTION_HISTORY_SIZE):
                    suggested_blink_count = 0
                    print("目标远离或静止，禁止闪烁")
            
            area_history.append(max_area)
            if len(area_history) > DIRECTION_HISTORY_SIZE:
                area_history = area_history[-DIRECTION_HISTORY_SIZE:]
        else:
            suggested_blink_count = 0
            lost_frame_counter += 1
            if lost_frame_counter >= MAX_MISSED_FRAMES:
                area_history = []
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
        # for area, color in REF_BOXES:
        #     draw_ref_rect(img, area, color)

        lcd.display(img)
        gc.collect()

except Exception as e:
    print("Error:", e)
finally:
    tim.stop()
    kpu.deinit(task)
    fm.unregister(IO_LED_RED)
    print("Application stopped.")
