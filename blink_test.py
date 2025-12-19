# -*- coding: utf-8 -*-
"""
LED闪烁节奏测试文件
用于测试闪烁逻辑是否正确工作
"""
import utime
from machine import Timer
from Maix import GPIO
from board import board_info
from fpioa_manager import fm

# --- 1. 配置参数 ---

# LED/GPIO 设置
IO_LED_RED = 13
LED_ACTIVE_LOW = True  # LED 是否为低电平点亮

# 闪烁逻辑参数 (双周期控制)
GLOBAL_PERIOD_MS = 2000    # 大周期 2秒 (2000ms)
SUB_PERIOD_MS = 200        # 小周期 200ms (100亮+100灭)
SUB_ON_MS = 100            # 小周期中亮的时长

# 测试闪烁次数（可以修改这个值来测试不同的闪烁次数）
TEST_BLINK_COUNT = 5       # 测试：闪烁3次

# --- 2. 硬件初始化 ---

# LED 初始化
fm.register(IO_LED_RED, fm.fpioa.GPIO0)
led_b = GPIO(GPIO.GPIO0, GPIO.OUT)
LED_ON = 0 if LED_ACTIVE_LOW else 1
LED_OFF = 1 if LED_ACTIVE_LOW else 0
led_b.value(LED_OFF)

print("LED闪烁测试程序启动")
print("配置参数:")
print("  - 大周期: {}ms".format(GLOBAL_PERIOD_MS))
print("  - 小周期: {}ms (亮{}ms + 灭{}ms)".format(SUB_PERIOD_MS, SUB_ON_MS, SUB_PERIOD_MS - SUB_ON_MS))
print("  - 测试闪烁次数: {}".format(TEST_BLINK_COUNT))
print("  - 活动期总时长: {}ms".format(TEST_BLINK_COUNT * SUB_PERIOD_MS))
print("  - 休眠期时长: {}ms".format(GLOBAL_PERIOD_MS - TEST_BLINK_COUNT * SUB_PERIOD_MS))
print("")

# --- 3. 定时器中断 (LED 控制核心) ---

# 全局变量
g_target_blink_count = TEST_BLINK_COUNT  # 目标闪烁次数（可以动态修改）
g_locked_blink_count = 0                 # 当前周期锁定的次数
g_timer_start_ms = 0                     # 当前周期开始时间
g_cycle_count = 0                         # 周期计数器（用于调试）

def on_timer_blink(timer):
    """
    定时器回调：每 20ms 执行一次
    功能：实现双周期锁定与精准 LED 控制
    """
    global g_target_blink_count, g_locked_blink_count, g_timer_start_ms, LED_ON, LED_OFF, g_cycle_count

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
        g_cycle_count += 1
        print("周期 #{} 开始，闪烁次数: {}".format(g_cycle_count, g_locked_blink_count))

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

print("定时器已启动，LED开始闪烁...")
print("按 Ctrl+C 停止程序")
print("")

# --- 4. 主循环（可选：动态修改闪烁次数） ---

try:
    # 可以在这里添加逻辑来动态修改 g_target_blink_count
    # 例如：根据时间或其他条件改变闪烁次数
    while True:
        #utime.sleep_ms(1000)
        # 示例：每5秒改变一次闪烁次数（可以注释掉）
        # if g_cycle_count % 5 == 0 and g_cycle_count > 0:
        #     g_target_blink_count = (g_target_blink_count % 5) + 1
        #     print("修改目标闪烁次数为: {}".format(g_target_blink_count))
        pass

except KeyboardInterrupt:
    print("\n程序被用户中断")
except Exception as e:
    print("错误:", e)
finally:
    # 清理资源
    tim.stop()
    led_b.value(LED_OFF)
    fm.unregister(IO_LED_RED)
    print("程序已停止，LED已关闭")

