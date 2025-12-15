# -*- coding: utf-8 -*-
import sensor,image,lcd,time,utime
import math
import KPU as kpu

from Maix import GPIO
from board import board_info
from fpioa_manager import fm

# --- 全局配置常量 (Global Configuration Constants) ---

# LED/GPIO 设置
IO_LED_RED = 13
LED_ACTIVE_LOW = True  # LED 是否为低电平点亮 (Is the LED active low?)

# 模型与检测设置
MODEL_ADDR = 0x800000
CLASSES = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']
ANCHOR = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)
CONF_THRESHOLD = 0.5
NMS_THRESHOLD = 0.3

# 追踪算法参数 (Tracking Algorithm Parameters)
TARGET_CLASSID = 14          # 追踪目标类别 ID, 'person' (None a追踪所有) (Class ID to track, 'person')
IOU_THRESH = 0.3             # IOU 匹配阈值 (IOU matching threshold)
MAX_MISSED_FRAMES = 5        # 最大丢失帧数 (Max frames to keep a tracker without detection)
AREA_EMA_ALPHA = 0.7         # 面积平滑系数 (Area smoothing factor)
V_EMA_ALPHA = 0.8            # 速度平滑系数 (Velocity smoothing factor)
VREL_DEADBAND = 0.01         # 速度死区，用于忽略抖动 (Velocity deadband to ignore jitter)
APPROACH_VREL_THRESH = 0.03  # 判断为“正在靠近”的速度阈值 (Velocity threshold to be considered "approaching")
APPROACH_CONFIRM_FRAMES = 2  # 连续判断为“靠近”的帧数 (Frames in a row to confirm "approaching")

# 闪烁逻辑参数 (Blinking Logic Parameters)
# 根据您的要求修改：最快500ms，最慢3秒
LED_PERIOD_MIN_MS = 500      # (要求) 最快闪烁周期 (ms) (Fastest blink period)
LED_PERIOD_MAX_MS = 3000     # (要求) 最慢闪烁周期 (ms) (Slowest blink period)
LED_PULSE_MS = 50            # LED每次点亮的脉冲宽度 (ms) (Pulse width for each blink)
AREA_NEAR = 20000            # "近"的参考面积 (Area considered "near")
AREA_FAR = 1500              # "远"的参考面积 (Area considered "far")

# 日志设置 (Logging Settings)
LOG_INTERVAL_MS = 1000       # 日志打印频率 (ms) (Log printing interval)

# --- 初始化 (Initialization) ---

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

# KPU 模型加载
task = kpu.load(MODEL_ADDR)
_ = kpu.init_yolo2(task, CONF_THRESHOLD, NMS_THRESHOLD, 5, ANCHOR)

# --- 核心逻辑类与函数 (Core Logic Classes & Functions) ---

class TrackedTarget:
    """保存单个被追踪对象的状态"""
    def __init__(self, target_id, rect):
        self.id = target_id
        self.rect = rect
        self.area_s = None
        self.d_rel_prev = None
        self.v_rel_s = 0.0
        self.approach_cnt = 0
        self.last_seen = 0
        self.age = 0

def _clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))

def _rect_iou(a, b):
    # ... (省略未修改的函数体)
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ax2, ay2 = ax + aw, ay + ah; bx2, by2 = bx + bw, by + bh
    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = aw * ah + bw * bh - inter
    return inter / ua if ua > 0 else 0.0

def _det_rect(det):
    return (det.x(), det.y(), det.w(), det.h())

def _filter_dets_by_class(dets):
    # ... (省略未修改的函数体)
    if TARGET_CLASSID is None: return dets
    if isinstance(TARGET_CLASSID, (list, tuple, set)):
        return [d for d in dets if d.classid() in TARGET_CLASSID]
    return [d for d in dets if d.classid() == TARGET_CLASSID]

# ... (省略其他未修改的追踪辅助函数)
# Note: For brevity, assuming _match_targets, _update_tracked_target, _create_new_target are the same
# 为简洁起见, 假设 _match_targets, _update_tracked_target, _create_new_target 函数与之前相同

def handle_led_blink(led_pin, led_freq_ms, pulse_ms):
    """非阻塞地处理LED脉冲闪烁"""
    global led_phase_start_ms, LED_ON, LED_OFF

    if led_freq_ms <= 0:
        led_pin.value(LED_OFF)
        return

    current_time = utime.ticks_ms()
    elapsed = utime.ticks_diff(current_time, led_phase_start_ms)

    if elapsed >= led_freq_ms:
        led_phase_start_ms = current_time
        led_pin.value(LED_ON)
    elif elapsed < pulse_ms:
        led_pin.value(LED_ON)
    else:
        led_pin.value(LED_OFF)


# --- 主循环 (Main Loop) ---
tracked_targets = {}
next_target_id = 1
last_log_ms = 0
led_phase_start_ms = 0
led_freq = 0

while(True):
    clock.tick()
    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)
    now_ms = utime.ticks_ms()

    # --- 追踪逻辑 (Tracking Logic) ---
    dt = 1.0 / max(clock.fps(), 1.0)
    # (此处省略目标匹配与更新的详细代码, 它们与之前版本相同)
    # (Tracker matching and update code omitted for brevity, same as before)
    if code:
        matched_pairs, unmatched_dets, unmatched_target_ids = _match_targets(code, tracked_targets, now_ms)
        for det, target_id in matched_pairs: _update_tracked_target(target_id, det, dt, now_ms)
        det_to_target_id = {id(det): tid for det, tid in matched_pairs}
        for det in unmatched_dets:
            new_id = _create_new_target(det, now_ms)
            det_to_target_id[id(det)] = new_id
        for target_id in unmatched_target_ids:
            if now_ms - tracked_targets[target_id].last_seen > MAX_MISSED_FRAMES * (1000.0 / max(clock.fps(), 1.0)):
                del tracked_targets[target_id]
    else:
        # 清理丢失的目标 (Cleanup lost targets)
        targets_to_remove = [tid for tid, t in tracked_targets.items() if now_ms - t.last_seen > MAX_MISSED_FRAMES * (1000.0 / max(clock.fps(), 1.0))]
        for tid in targets_to_remove: del tracked_targets[tid]

    # --- LED 频率计算 (LED Frequency Calculation) ---
    led_freq = 0 # 默认为0 (不闪烁)
    best_approaching_target = None
    best_approaching_score = -1

    for target in tracked_targets.values():
        if target.approach_cnt >= APPROACH_CONFIRM_FRAMES:
            score = (target.area_s or 0) + max(0, target.v_rel_s) * 1000
            if score > best_approaching_score:
                best_approaching_score = score
                best_approaching_target = target

    if best_approaching_target:
        s_area = math.sqrt(best_approaching_target.area_s or 0)
        s_far = math.sqrt(AREA_FAR)
        s_near = math.sqrt(AREA_NEAR)
        
        # 计算进度 (0=远, 1=近)
        p = (s_area - s_far) / (s_near - s_far) if s_near > s_far else 0
        p = _clamp(p, 0.0, 1.0)
        
        # 将进度映射到闪烁周期
        led_freq = int(LED_PERIOD_MAX_MS - p * (LED_PERIOD_MAX_MS - LED_PERIOD_MIN_MS))

    # --- 绘图与显示 (Drawing & Display) ---
    if code:
        for det in code:
            # 在一次循环中完成所有绘制
            _ = img.draw_rectangle(det.rect())
            det_id = det_to_target_id.get(id(det))
            if det_id is not None and det_id in tracked_targets:
                target = tracked_targets[det_id]
                info = 'ID:%d v:%.2f' % (det_id, target.v_rel_s)
                _ = lcd.draw_string(det.x(), det.y(), info, lcd.RED, lcd.WHITE)
                _ = lcd.draw_string(det.x(), det.y() + 12, '%s:%.2f' % (CLASSES[det.classid()], det.value()), lcd.RED, lcd.WHITE)
            else:
                _ = lcd.draw_string(det.x(), det.y(), '%s:%.2f' % (CLASSES[det.classid()], det.value()), lcd.RED, lcd.WHITE)

    _ = lcd.display(img)

    # --- LED 控制 (LED Control) ---
    handle_led_blink(led_b, led_freq, LED_PULSE_MS)

    # --- 限频日志 (Throttled Logging) ---
    if utime.ticks_diff(now_ms, last_log_ms) >= LOG_INTERVAL_MS:
        last_log_ms = now_ms
        print("FPS: %.2f" % clock.fps())
        # print("LED Freq: %d ms" % led_freq) # 可选: 在此处打印频率用于调试

# --- 清理 (Cleanup) ---
_ = kpu.deinit(task)
fm.unregister(IO_LED_RED)
print("Application stopped.")

# --- (以下是为保持完整性而需要粘贴的、未修改的函数) ---
# Paste the following unmodified functions here to maintain integrity
def _update_tracked_target(target_id, det, dt, now_ms):
    """更新已匹配目标的追踪状态"""
    global tracked_targets
    if target_id not in tracked_targets: return
    target = tracked_targets[target_id]
    target.rect = _det_rect(det)
    target.last_seen = now_ms
    target.age += 1
    area_now = det.w() * det.h()
    target.area_s = area_now if target.area_s is None else (AREA_EMA_ALPHA * target.area_s + (1.0 - AREA_EMA_ALPHA) * area_now)
    d_rel = 1.0 / (math.sqrt(target.area_s) + 1e-6)
    if target.d_rel_prev is not None and dt is not None:
        v_rel = (target.d_rel_prev - d_rel) / dt
        target.v_rel_s = V_EMA_ALPHA * target.v_rel_s + (1.0 - V_EMA_ALPHA) * v_rel
    target.d_rel_prev = d_rel
    v_rel_eff = target.v_rel_s if abs(target.v_rel_s) > VREL_DEADBAND else 0.0
    target.approach_cnt = target.approach_cnt + 1 if v_rel_eff >= APPROACH_VREL_THRESH else 0

def _create_new_target(det, now_ms):
    """为新检测到的目标创建追踪对象"""
    global tracked_targets, next_target_id
    target_id = next_target_id; next_target_id += 1
    target = TrackedTarget(target_id, _det_rect(det))
    target.last_seen = now_ms; target.age = 1
    area_now = det.w() * det.h()
    target.area_s = area_now
    target.d_rel_prev = 1.0 / (math.sqrt(area_now) + 1e-6)
    tracked_targets[target_id] = target
    return target_id

def _match_targets(dets, tracked_targets_dict, now_ms):
    """多目标匹配"""
    global next_target_id
    dets = _filter_dets_by_class(dets)
    if not dets: return [], [], list(tracked_targets_dict.keys())
    matched_pairs, matched_det_indices, matched_target_ids = [], set(), set()
    for i, det in enumerate(dets):
        det_rect, best_target_id, best_iou = _det_rect(det), None, IOU_THRESH
        for target_id, target in tracked_targets_dict.items():
            if target_id in matched_target_ids: continue
            iou = _rect_iou(target.rect, det_rect)
            if iou > best_iou: best_iou, best_target_id = iou, target_id
        if best_target_id is not None:
            matched_pairs.append((det, best_target_id))
            matched_det_indices.add(i)
            matched_target_ids.add(best_target_id)
    unmatched_dets = [dets[i] for i in range(len(dets)) if i not in matched_det_indices]
    unmatched_target_ids = [tid for tid in tracked_targets_dict.keys() if tid not in matched_target_ids]
    return matched_pairs, unmatched_dets, unmatched_target_ids