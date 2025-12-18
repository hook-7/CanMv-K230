'''
实验名称：物体检测（基于yolov8n）- 双周期闪烁逻辑移植版
实验平台：01Studio CanMV K230
说明：移植了 simple_detect.py 的双周期闪烁、防抖、面积判定逻辑。
'''

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *
from media.sensor import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import sys
import aidemo
import math
from machine import Pin, Timer 

# --- 1. 全局配置与状态变量 ---

# 闪烁逻辑参数 (双周期)
GLOBAL_PERIOD_MS = 2000    # 大周期 2秒
SUB_PERIOD_MS = 200        # 小周期 200ms
SUB_ON_MS = 100            # 亮 100ms

# 全局变量：主循环与中断通信
g_target_blink_count = 0     # 目标闪烁次数
g_locked_blink_count = 0     # 锁定次数
g_timer_start_ms = 0         # 周期开始时间

# 自定义YOLOv8检测类
class ObjectDetectionApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,max_boxes_num,confidence_threshold=0.5,nms_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.labels=labels
        self.model_input_size=model_input_size
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.max_boxes_num=max_boxes_num
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        self.x_factor = float(self.rgb888p_size[0])/self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1])/self.model_input_size[1]
        self.ai2d=Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

        # 面积阈值计算 (基于显示分辨率)
        self.total_pixels = self.display_size[0] * self.display_size[1]
        self.area_thresh_80 = int(self.total_pixels * 0.70) 
        self.area_thresh_13 = int(self.total_pixels * 0.13)

    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            result=results[0]
            result = result.reshape((result.shape[0] * result.shape[1], result.shape[2]))
            output_data = result.transpose()
            boxes_ori = output_data[:,0:4]
            scores_ori = output_data[:,4:]
            confs_ori = np.max(scores_ori,axis=-1)
            inds_ori = np.argmax(scores_ori,axis=-1)
            boxes,scores,inds = [],[],[]
            for i in range(len(boxes_ori)):
                if confs_ori[i] > self.confidence_threshold:
                    scores.append(confs_ori[i])
                    inds.append(inds_ori[i])
                    x = boxes_ori[i,0]
                    y = boxes_ori[i,1]
                    w = boxes_ori[i,2]
                    h = boxes_ori[i,3]
                    left = int((x - 0.5 * w) * self.x_factor)
                    top = int((y - 0.5 * h) * self.y_factor)
                    right = int((x + 0.5 * w) * self.x_factor)
                    bottom = int((y + 0.5 * h) * self.y_factor)
                    boxes.append([left,top,right,bottom])
            if len(boxes)==0:
                return []
            boxes = np.array(boxes)
            scores = np.array(scores)
            inds = np.array(inds)
            keep = self.nms(boxes,scores,self.nms_threshold)
            dets = np.concatenate((boxes, scores.reshape((len(boxes),1)), inds.reshape((len(boxes),1))), axis=1)
            dets_out = []
            for keep_i in keep:
                dets_out.append(dets[keep_i])
            dets_out = np.array(dets_out)
            dets_out = dets_out[:self.max_boxes_num, :]
            return dets_out

    # 绘制结果 & 返回最大 'person' 面积
    def draw_result(self,pl,dets):
        max_person_area = 0

        with ScopedTiming("display_draw",self.debug_mode >0):
            pl.osd_img.clear()

            # 1. 绘制辅助参考框 (居中, 4:3 比例)
            cx, cy = self.display_size[0] // 2, self.display_size[1] // 2

            # 蓝色 (13%)
            self._draw_ref_box(pl, self.area_thresh_13, (255, 0, 0, 255), cx, cy)
            # 红色 (70%)
            self._draw_ref_box(pl, self.area_thresh_80, (255, 255, 0, 0), cx, cy)

            if dets is not None:
                for det in dets:
                    class_id = int(det[5])
                    if class_id == 0:
                        x1, y1, x2, y2 = map(lambda x: int(round(x, 0)), det[:4])
                        x= x1*self.display_size[0] // self.rgb888p_size[0]
                        y= y1*self.display_size[1] // self.rgb888p_size[1]
                        w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                        h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]

                        # 绘制绿色检测框
                        pl.osd_img.draw_rectangle(x,y, w, h, color=(255, 0, 255, 0), thickness=4)
                        # 显示标签和置信度
                        pl.osd_img.draw_string_advanced(x, y-50, 32, " " + self.labels[class_id] + " " + str(round(det[4],2)), color=(255, 0, 255, 0))

                        # 计算面积
                        area = w * h
                        if area > max_person_area:
                            max_person_area = area

        return max_person_area

    def _draw_ref_box(self, pl, area, color, cx, cy):
        h = int(math.sqrt(area * 3 / 4))
        w = int(h * 4 / 3)
        x = cx - w // 2
        y = cy - h // 2
        pl.osd_img.draw_rectangle(x, y, w, h, color=color, thickness=2)

    def nms(self,boxes,scores,thresh):
        x1,y1,x2,y2 = boxes[:, 0],boxes[:, 1],boxes[:, 2],boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = np.argsort(scores,axis = 0)[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            new_x1,new_y1,new_x2,new_y2,new_areas = [],[],[],[],[]
            for order_i in order:
                new_x1.append(x1[order_i])
                new_x2.append(x2[order_i])
                new_y1.append(y1[order_i])
                new_y2.append(y2[order_i])
                new_areas.append(areas[order_i])
            new_x1 = np.array(new_x1)
            new_x2 = np.array(new_x2)
            new_y1 = np.array(new_y1)
            new_y2 = np.array(new_y2)
            xx1 = np.maximum(x1[i], new_x1)
            yy1 = np.maximum(y1[i], new_y1)
            xx2 = np.minimum(x2[i], new_x2)
            yy2 = np.minimum(y2[i], new_y2)
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            new_areas = np.array(new_areas)
            ovr = inter / (areas[i] + new_areas - inter)
            new_order = []
            for ovr_i,ind in enumerate(ovr):
                if ind < thresh:
                    new_order.append(order[ovr_i])
            order = np.array(new_order,dtype=np.uint8)
        return keep


if __name__=="__main__":

    display="hdmi" # 默认改为 hdmi 进行测试

    if display=="hdmi":
        display_mode='hdmi'
        display_size=[1280,720] # Sensor最大支持1280，改为720P输出以匹配Sensor
    elif display=="lcd3_5":
        display_mode= 'st7701'
        display_size=[800,480]
    elif display=="lcd2_4":
        display_mode= 'st7701'
        display_size=[640,480]

    rgb888p_size=[320,320]
    kmodel_path="/sdcard/examples/kmodel/yolov8n_320.kmodel"
    labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

    confidence_threshold = 0.5   
    nms_threshold = 0.2
    max_boxes_num = 20           

    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)

    # 实例化 Sensor 对象
    if display =="lcd2_4":
        s = Sensor(width=640, height=480)
    elif display =="lcd3_5":
        s = Sensor(width=1280, height=720) 
    else:  # hdmi
        s = Sensor(width=1280, height=720) # 降级 Sensor 到 720P 以节省内存并提高稳定性，显示端仍输出 1080P

    pl.create(s)

    print("Waiting for sensor initialization...")
    time.sleep_ms(1000) 

    print(f"PipeLine initialized: display_mode={display_mode}, display_size={display_size}")

    ob_det=ObjectDetectionApp(kmodel_path,labels=labels,model_input_size=[320,320],max_boxes_num=max_boxes_num,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    ob_det.config_preprocess()

    clock = time.clock()

    # 防抖变量
    CONFIRM_FRAMES = 2
    last_suggested = -1
    confirm_count = 0
    frame_count = 0

    # --- 虚拟 LED 状态变量 ---
    v_timer_start_ms = 0
    v_locked_blink_count = 0
    v_led_on = False

    try:
        while True:
            frame_count += 1

            # --- 虚拟 LED 闪烁逻辑更新 ---
            current_time = time.ticks_ms()
            if v_timer_start_ms == 0:
                v_timer_start_ms = current_time

            elapsed = time.ticks_diff(current_time, v_timer_start_ms)
            if elapsed >= GLOBAL_PERIOD_MS:
                v_timer_start_ms = current_time
                elapsed = 0
                v_locked_blink_count = g_target_blink_count 

            active_duration = v_locked_blink_count * SUB_PERIOD_MS
            if elapsed < active_duration:
                sub_phase = elapsed % SUB_PERIOD_MS
                v_led_on = (sub_phase < SUB_ON_MS)
            else:
                v_led_on = False

            # 获取帧数据
            try:
                img=pl.get_frame()
            except Exception as e:
                print(f"Failed to get frame: {e}")
                time.sleep_ms(100)  
                continue

            res=ob_det.run(img)
            max_area = ob_det.draw_result(pl, res)

            # --- 绘制虚拟 LED ---
            led_x = display_size[0] - 40
            led_y = 40

            led_color = (255, 128, 128, 128) # 默认灰色 (灭)
            if v_led_on:
                if v_locked_blink_count >= 5:
                    led_color = (255, 255, 0, 0) # 红 (极近)
                elif v_locked_blink_count >= 2:
                    led_color = (255, 255, 255, 0) # 黄 (中等)
                elif v_locked_blink_count >= 1:
                    led_color = (255, 0, 255, 0) # 绿 (远)

            # 绘制虚拟 LED (外框 + 填充感矩形)
            pl.osd_img.draw_rectangle(led_x - 25, led_y - 25, 50, 50, color=(255, 255, 255, 255), thickness=2)
            if v_led_on:
                pl.osd_img.draw_rectangle(led_x - 20, led_y - 20, 40, 40, color=led_color, thickness=20)
                pl.osd_img.draw_string_advanced(led_x - 10, led_y - 20, 40, "O", color=(255, 255, 255, 255))
            else:
                pl.osd_img.draw_rectangle(led_x - 5, led_y - 5, 10, 10, color=(255, 50, 50, 50), thickness=5)

            # --- 决策逻辑 ---
            suggested = 0
            if max_area > 0:
                if max_area >= ob_det.area_thresh_80:
                    suggested = 5 # 极近
                elif max_area > ob_det.area_thresh_13:
                    suggested = 2 # 中等
                else:
                    suggested = 1 # 较远/微小
            else:
                suggested = 0 # 灭

            # --- 防抖逻辑 ---
            if suggested == last_suggested:
                confirm_count += 1
            else:
                confirm_count = 0
                last_suggested = suggested

            if confirm_count >= CONFIRM_FRAMES:
                g_target_blink_count = suggested
                confirm_count = CONFIRM_FRAMES

            pl.show_image()

            if frame_count % 60 == 0:
                gc.collect()

            if frame_count % 30 == 0:
                print(f"Frame: {frame_count}, Area: {max_area}, Blink: {g_target_blink_count}")

    except Exception as e:
        sys.print_exception(e)
    finally:
        pl.destroy()
        print("Stopped")