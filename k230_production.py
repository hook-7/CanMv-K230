'''
实验名称：物体检测（基于yolov8n）
实验平台：01Studio CanMV K230
教程：wiki.01studio.cc
说明：可以通过display="xxx"参数选择"hdmi"、"lcd3_5"(3.5寸mipi屏)或"lcd2_4"(2.4寸mipi屏)显示方式
'''

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
import math
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

# 自定义YOLOv8检测类
class ObjectDetectionApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,max_boxes_num,confidence_threshold=0.5,nms_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.labels=labels
        # 模型输入分辨率
        self.model_input_size=model_input_size
        # 阈值设置
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.max_boxes_num=max_boxes_num
        # sensor给到AI的图像分辨率
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 显示分辨率
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        # 检测框预置颜色值
        self.color_four=[(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
                         (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
                         (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
                         (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
                         (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157)]
        # 宽高缩放比例
        self.x_factor = float(self.rgb888p_size[0])/self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1])/self.model_input_size[1]
        # Ai2d实例，用于实现模型预处理
        self.ai2d=Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

        # 移动侦测/闪烁逻辑变量
        self.area_history = []
        self.smoothed_area = None
        self.direction_history_size = 5
        self.total_pixels = self.rgb888p_size[0] * self.rgb888p_size[1]
        
        # 面积阈值比例
        self.area_ratio_min = 0.005
        self.area_ratio_80 = 0.50
        
        # 面积阈值 (像素)
        self.area_thresh_min = int(self.total_pixels * self.area_ratio_min)
        self.area_thresh_80 = int(self.total_pixels * self.area_ratio_80)
        
        # 平滑参数
        self.alpha_near = 0.20
        self.alpha_far = 0.70
        
        # 靠近判定阈值
        self.approach_eps_far_percent = 0.5
        self.approach_eps_near_percent = 5.0

        # 闪烁状态
        self.blink_interval_ms = 100
        self.last_blink_toggle_time = 0
        self.is_blink_on = False

        # 警告状态
        self.warning_active = False
        self.warning_start_time = 0
        self.warning_duration_ms = 3000

    # 配置预处理操作，这里使用了resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/libs/AI2D.py查看
    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理
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
                if confs_ori[i] > confidence_threshold:
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
            # NMS过程
            keep = self.nms(boxes,scores,nms_threshold)
            dets = np.concatenate((boxes, scores.reshape((len(boxes),1)), inds.reshape((len(boxes),1))), axis=1)
            dets_out = []
            for keep_i in keep:
                dets_out.append(dets[keep_i])
            dets_out = np.array(dets_out)
            dets_out = dets_out[:self.max_boxes_num, :]
            return dets_out

    def _clamp01(self, x):
        if x < 0:
            return 0.0
        if x > 1:
            return 1.0
        return x

    def get_adaptive_alpha(self, area):
        if area <= self.area_thresh_min:
            return self.alpha_far
        if area >= self.area_thresh_80:
            return self.alpha_near
        t = (area - self.area_thresh_min) / (self.area_thresh_80 - self.area_thresh_min)
        t = self._clamp01(t)
        t = math.sqrt(t)
        return self.alpha_far - (self.alpha_far - self.alpha_near) * t

    def smooth_area_ewma(self, raw_area, prev_smoothed):
        if prev_smoothed is None:
            return raw_area
        alpha = self.get_adaptive_alpha(raw_area)
        return alpha * raw_area + (1 - alpha) * prev_smoothed

    def get_adaptive_approach_eps(self, area):
        if area <= self.area_thresh_min:
            return self.approach_eps_far_percent / 100.0
        if area >= self.area_thresh_80:
            return self.approach_eps_near_percent / 100.0
        t = (area - self.area_thresh_min) / (self.area_thresh_80 - self.area_thresh_min)
        t = self._clamp01(t)
        t = math.sqrt(t)
        eps_percent = self.approach_eps_far_percent + (self.approach_eps_near_percent - self.approach_eps_far_percent) * t
        return eps_percent / 100.0

    def is_moving_closer(self, current_area, area_history, history_size):
        if len(area_history) < history_size:
            return False
        avg_historical = sum(area_history[-history_size:]) / history_size
        eps = self.get_adaptive_approach_eps(current_area)
        return current_area > (avg_historical * (1.0 + eps))

    # 绘制结果
    def draw_result(self,pl,dets):
        with ScopedTiming("display_draw",self.debug_mode >0):
            try:
                current_time = time.ticks_ms()
            except:
                current_time = int(time.time() * 1000)

            if dets is not None and len(dets) > 0:
                pl.osd_img.clear()
                
                # 1. 寻找最大目标 (用于判定是否靠近)
                max_area = 0
                best_det_idx = -1
                for i in range(len(dets)):
                    w_det = dets[i][2] - dets[i][0]
                    h_det = dets[i][3] - dets[i][1]
                    area = w_det * h_det
                    if area > max_area:
                        max_area = area
                        best_det_idx = i
                
                # 2. 更新状态与判断
                is_approaching = False
                if best_det_idx != -1:
                    # 平滑面积
                    self.smoothed_area = self.smooth_area_ewma(max_area, self.smoothed_area)
                    area_for_decision = self.smoothed_area
                    
                    # 判断趋势
                    if self.is_moving_closer(area_for_decision, self.area_history, self.direction_history_size):
                        is_approaching = True
                        # 触发警告
                        self.warning_active = True
                        self.warning_start_time = current_time
                    
                    # 更新历史
                    self.area_history.append(area_for_decision)
                    if len(self.area_history) > self.direction_history_size:
                        self.area_history.pop(0)

                # 3. 检查警告持续时间
                if self.warning_active:
                    if time.ticks_diff(current_time, self.warning_start_time) > self.warning_duration_ms:
                        self.warning_active = False

                # 4. 绘制框
                for i in range(len(dets)):
                    det = dets[i]
                    x1, y1, x2, y2 = map(lambda x: int(round(x, 0)), det[:4])
                    x= x1*self.display_size[0] // self.rgb888p_size[0]
                    y= y1*self.display_size[1] // self.rgb888p_size[1]
                    w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    
                    base_color = self.get_color(int(det[5]))
                    # base_color is (A, R, G, B) based on self.color_four
                    # Extract RGB
                    c_a, c_r, c_g, c_b = base_color
                    
                    if self.warning_active and i == best_det_idx:
                        # 警告效果：加粗线框（10倍粗），内透明外不透明过度
                        max_thick = 80 
                        for t in range(0, max_thick, 4):
                            # Alpha 从内(0)到外(255)过度
                            # 内框：t=0 -> alpha=0 (透明)
                            # 外框：t=max_thick -> alpha=255 (不透明)
                            alpha = int(255 * (t / max_thick))
                            if alpha > 255: alpha = 255
                            
                            draw_color = (alpha, c_r, c_g, c_b)
                            
                            # 向外扩展绘制，使用 thickness=4 步进以在保证性能的同时提供极大厚度
                            pl.osd_img.draw_rectangle(x - t, y - t, w + 2*t, h + 2*t, color=draw_color, thickness=4)
                        
                        # 绘制标签 (使用不透明颜色)
                        pl.osd_img.draw_string_advanced( x , y-50, 32, "WARNING", color=(255, c_r, c_g, c_b))
                        
                    else:
                        # 正常绘制
                        pl.osd_img.draw_rectangle(x,y, w, h, color=base_color, thickness=4)
                        pl.osd_img.draw_string_advanced( x , y-50,32," " + self.labels[int(det[5])] + " " + str(round(det[4],2)) , color=base_color)
            else:
                pl.osd_img.clear()
                if len(self.area_history) > 0:
                    self.area_history = []
                    self.smoothed_area = None


    # 多目标检测 非最大值抑制方法实现
    def nms(self,boxes,scores,thresh):
        """Pure Python NMS baseline."""
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

    # 根据当前类别索引获取框的颜色
    def get_color(self, x):
        idx=x%len(self.color_four)
        return self.color_four[idx]


if __name__=="__main__":

    # 显示模式，可以选择"hdmi"、"lcd3_5"(3.5寸mipi屏)和"lcd2_4"(2.4寸mipi屏)

    display="hdmi"

    if display=="hdmi":
        display_mode='hdmi'
        display_size=[1920,1080]

    elif display=="lcd3_5":
        display_mode= 'st7701'
        display_size=[800,480]

    elif display=="lcd2_4":
        display_mode= 'st7701'
        display_size=[640,480]

    rgb888p_size=[320,320] #特殊尺寸定义

    # 模型路径
    kmodel_path="/sdcard/examples/kmodel/yolov8n_320.kmodel"
    labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
    #labels = ["人", "自行车", "汽车", "摩托车", "飞机", "公共汽车", "火车", "卡车", "船", "交通灯", "消防栓", "停车标志", "停车收费表", "长凳", "鸟", "猫", "狗", "马", "羊", "牛", "大象", "熊", "斑马", "长颈鹿", "背包", "雨伞", "手提包", "领带", "手提箱", "飞盘", "滑雪板", "滑雪板", "运动球", "风筝", "棒球棒", "棒球手套", "滑板", "冲浪板", "网球拍", "瓶子", "酒杯", "杯子", "叉子", "刀子", "勺子", "碗", "香蕉", "苹果", "三明治", "橙子", "西兰花", "胡萝卜", "热狗", "披萨", "甜甜圈", "蛋糕", "椅子", "沙发", "盆栽", "床", "餐桌", "马桶", "电视", "笔记本电脑", "鼠标", "遥控器", "键盘", "手机", "微波炉", "烤箱", "烤面包机", "水槽", "冰箱", "书", "钟表", "花瓶", "剪刀", "泰迪熊", "吹风机", "牙刷"]
    # 其它参数设置
    confidence_threshold = 0.2
    nms_threshold = 0.2
    max_boxes_num = 50

    # 初始化PipeLine
    pl=PipeLine(rgb888p_size=rgb888p_size,display_size=display_size,display_mode=display_mode)

    if display =="lcd2_4":
        pl.create(Sensor(width=1280, height=960))  # 创建PipeLine实例，画面4:3

    else:
        sensor = Sensor(width=1920, height=1080)
        sensor.reset()
        sensor.set_vflip(1)
        pl.create(sensor)  # 创建PipeLine实例

    # 初始化自定义目标检测实例
    ob_det=ObjectDetectionApp(kmodel_path,labels=labels,model_input_size=[320,320],max_boxes_num=max_boxes_num,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    ob_det.config_preprocess()

    clock = time.clock()

    while True:

        clock.tick()

        img=pl.get_frame() # 获取当前帧数据
        res=ob_det.run(img) # 推理当前帧
        ob_det.draw_result(pl,res) # 绘制结果到PipeLine的osd图像
        # print(res)  # 打印当前结果
        pl.show_image() # 显示当前的绘制结果
        gc.collect()

        print(clock.fps()) #打印帧率
