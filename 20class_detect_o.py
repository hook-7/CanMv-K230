import sensor,image,lcd,time,utime
import KPU as kpu

from Maix import GPIO
from board import board_info
from fpioa_manager import fm

io_led_green = 12
io_led_red   = 13
io_led_blue  = 14


fm.register(io_led_red,fm.fpioa.GPIO0)
led_b = GPIO(GPIO.GPIO0,GPIO.OUT)
led_b.value(1)

# 初始化 LCD 屏（可选，用于显示检测结果）
lcd.init(type=2)
lcd.rotation(0) # 根据屏幕安装方向调整旋转角度（0~3）

sensor.reset()   # 重置摄像头
sensor.set_pixformat(sensor.RGB565)  # 设置图像格式为 RGB565（KPU 支持的格式）
sensor.set_framesize(sensor.QVGA)    # 设置图像分辨率为 QVGA (320×240)
#sensor.set_vflip(1) #flip camera; maix go use sensor.set_hmirror(0)
#sensor.set_hmirror(1)   # 设置摄像头水平镜像
sensor.set_vflip(1)     # 设置摄像头垂直翻转
sensor.run(1)           # 图像捕捉功能控制
clock = time.clock()

# 获取类别名称（需与模型训练的类别顺序对应，此处以 Pascal VOC 20 类为例）
classes = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']

# 1. 加载 YOLOv2 .kmodel 模型
task = kpu.load(0x800000) # 创建 KPU 任务句柄（task 参数）
# 锚框列表：格式为 [w1, h1, w2, h2, ..., w5, h5]（共 5×2=10 个数值）
anchor = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)

last_print_time = 0  # 记录上一次打印“间隔1秒”的时间戳

led_en = True
last_area = 0;
led_freq = 500

# 调用 kpu.init_yolo2() 配置检测规则
# 参数含义：task（KPU任务）、置信度阈值0.5、NMS阈值0.3、锚框数量5、锚框列表anchor
a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)
while(True):
    clock.tick()
     # 1. 采集一帧图像
    img = sensor.snapshot()

    # 2. 执行 YOLOv2 推理（返回检测结果列表）
    # code 是检测到的目标列表，每个目标包含：classid（类别ID）、confidence（置信度）、x/y/w/h（边界框）
    code = kpu.run_yolo2(task, img)
    print(clock.fps())
    # 3. 处理检测结果（在图像上绘制边界框和标签）
    if code:# 若检测到目标
#        a = lcd.display(img)
        area = 0;
        for i in code:
            # 在图像上绘制矩形框（颜色：红色，线宽：2）
            a=img.draw_rectangle(i.rect())
            a = lcd.display(img)
            print(i)
            if i.classid() == 14 and i.value() > 0.6:
 #               if i.value() > 0.9:
 #                   led_freq = 300;
 #               elif i.value() > 0.8:
 #                   led_freq = 400;
 #               elif i.value() > 0.7:
 #                   led_freq = 700;
 #               elif i.value() > 0.6:
 #                   led_freq = 1000;
 #               else:
 #                   led_freq = 0;
                area_buf = i.w()*i.h()
                if(area_buf > area):
                    area = area_buf;

                for i in code:
                    lcd.draw_string(i.x(), i.y(), classes[i.classid()], lcd.RED, lcd.WHITE)
                    lcd.draw_string(i.x(), i.y()+12, '%f1.3'%i.value(), lcd.RED, lcd.WHITE)

#            else:
#                led_freq = 0;



        if  area > 30000:
            led_freq = 300;
        elif  area > 25000:
            led_freq = 500;
        elif  area > 20000:
            led_freq = 700;
        elif  area > 1000:
            led_freq = 900;
        else:
            led_freq = 0;
        print(area)
        print(led_freq)
#            print("captur: %d,%f ",i,i.value())
            # 4. 在 LCD 屏显示处理后的图像
#            a = lcd.display(img)

#                lcd.draw_string(i.x(), i.y()+12, '%.3f'%i.value(), lcd.RED, lcd.WHITE)
    else:
        a = lcd.display(img)
        led_freq = 0

    current_time = utime.ticks_ms()  # 获取当前毫秒级时间戳

    # 非阻塞判断：是否已间隔 1000 毫秒
    time_diff = utime.ticks_diff(current_time, last_print_time)
#    if led_freq == 0:
#        led_en = 1
#        led_b.value(led_en)
#        last_print_time = current_time  # 更新时间戳
#    else:
    if time_diff >= led_freq:
            print("间隔 x 秒执行")
#            led_en = not led_en
            led_en = True
            led_b.value(led_en)
            last_print_time = current_time  # 更新时间戳
    elif time_diff >= 200:
            print("ledoff")
            led_en = True
            led_b.value(led_en)
    elif time_diff >= 3:
            print("start")
            led_en = False
            led_b.value(led_en)


a = kpu.deinit(task)
fm.unregister(board_info.LED_R)








