'''
实验名称：YOLO目标检测（完整双标志位版）
硬件平台：01Studio CanMV K230
依赖库：media_lib_v2.1.0+, nncase_runtime2.4.0+
'''

from machine import UART, FPIOA, Pin
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import os
import ujson
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time,_thread
import utime
import image
import random
import gc
import sys
import aicube

# 硬件配置
DISPLAY_WIDTH = 800
DISPLAY_HEIGHT = 480
UART_BAUDRATE = 115200
# 类别配置（与原系统对应）
IRR_MAP = {
     0: ('红球', (0,0,0)),
     1: ('黄球', (255,255,0)),
     2: ('黑球', (255,0,0)),
     3: ('蓝球', (0,0,255)), 
     4: ('红方安全区', (64,64,64)),
     5: ('黑方安全区', (255,128,128)),
 }
CLASS_MAP = {  
     0: ('黑方安全区', (255,128,128)),
     1: ('红方安全区', (255,255,0)),
     2: ('黑球', (0,0,0)),
     3: ('蓝球', (255,0,0)),
     4: ('黄球', (0,0,255)), 
     5: ('红球', (64,64,64)),
     
 }
START_MAP = {  
     0: ('红方安全区', (255,128,128)),
     1: ('黑方安全区', (255,255,0)),   
     2: ('红方出发区', (255,128,128)),
     3: ('蓝方出发区', (255,255,0)),   
}
ST_MAP = ["","","红方出发区","蓝方出发区"]
Sr_x = 0
Sr_y = 0
Sb_x = 0
Sb_y = 0 # 出发区坐标
BALL_MAP = ["红球","黄球","黑球","蓝球"]
Q_MAP = ["红方安全区","","","黑方安全区"]
QY_MIN = 230
color = 0 # 决定阵营
mode = color # 取0,1,2,3,表球真实色彩
mode = 1
comeback = 0 # 模型切换
start = 0 # 出发区标志
precs = 0
#面积约束
ball_max = 260
gate_min = 260
#串口标志
r_flag = b'\x03'
y_flag = b'\x01'
B_flag = b'\x02'
b_flag = b'\x04'
start_flag =b'\x05'
stop_flag = b'\x07'
safe_flag = b'\x08'
send_flag = 0
det_boxes = 0
clock = time.clock() #定义时钟

class YOLODetector(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size=[640,640],anchors=[10.13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326],model_type="AnchorBaseDet",confidence_threshold=0.5,nms_threshold=0.25,nms_option=False,strides=[8,16,32],rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        # kmodel路径
        self.kmodel_path=kmodel_path
        # 类别标签
        self.labels=labels
        # 模型输入分辨率
        self.model_input_size=model_input_size
        # 检测任务的锚框
        self.anchors=anchors
        # 模型类型，支持"AnchorBaseDet","AnchorFreeDet","GFLDet"三种模型
        self.model_type=model_type
        # 检测框类别置信度阈值
        self.confidence_threshold=confidence_threshold
        # 检测框NMS筛选阈值
        self.nms_threshold=nms_threshold
        # NMS选项，如果为True做类间NMS,如果为False做类内NMS
        self.nms_option=nms_option
        # 输出特征图的降采样倍数
        self.strides=strides
        # sensor给到AI的图像分辨率，宽16字节对齐
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 视频输出VO分辨率，宽16字节对齐
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        # 调试模式
        self.debug_mode=debug_mode
        # 检测框预置颜色值
        self.color_four=[(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
                         (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
                         (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
                         (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
                         (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157)]
        # Ai2d实例，用于实现模型预处理
        self.ai2d=Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            # 计算padding参数
            top,bottom,left,right=self.get_padding_param()
            # 配置padding预处理
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [114,114,114])
            # 配置resize预处理
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            # build预处理过程，参数为输入tensor的shape和输出tensor的shape
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理,这里调用了aicube模块的后处理接口
    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            # AnchorBaseDet模型的后处理
            if self.model_type == "AnchorBaseDet":
                det_boxes = aicube.anchorbasedet_post_process( results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.anchors, self.nms_option)
            # GFLDet模型的后处理
            elif self.model_type == "GFLDet":
                det_boxes = aicube.gfldet_post_process( results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.nms_option)
            # AnchorFreeDet模型的后处理
            elif self.model_type=="AnchorFreeDet":
                det_boxes = aicube.anchorfreedet_post_process( results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.nms_option)
            else:
                det_boxes=None
            return det_boxes

    # 将结果绘制到屏幕上
    def draw_result(self,pl,det_boxes):
        with ScopedTiming("draw osd",self.debug_mode > 0):
            if det_boxes:
                pl.osd_img.clear()
                for det_boxe in det_boxes:
                    # 获取每一个检测框的坐标，并将其从原图分辨率坐标转换到屏幕分辨率坐标，将框和类别信息绘制在屏幕上
                    x1, y1, x2, y2 = det_boxe[2],det_boxe[3],det_boxe[4],det_boxe[5]
                    sx=int(x1 * self.display_size[0] // self.rgb888p_size[0])
                    sy=int(y1 * self.display_size[1] // self.rgb888p_size[1])
                    w = int(float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0])
                    h = int(float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1])
                    pl.osd_img.draw_rectangle(sx , sy , w , h , color=self.get_color(det_boxe[0]))
                    label = self.labels[det_boxe[0]]
                    score = str(round(det_boxe[1],2))
                    pl.osd_img.draw_string_advanced(sx, sy-50,32, label + " " + score , color=self.get_color(det_boxe[0]))
            else:
                pl.osd_img.clear()
                pl.osd_img.draw_rectangle(0, 0, 128, 128, color=(0,0,0,0))

    # 计算padding参数
    def get_padding_param(self):
        ratiow = float(self.model_input_size[0]) / self.rgb888p_size[0];
        ratioh = float(self.model_input_size[1]) / self.rgb888p_size[1];
        ratio = min(ratiow, ratioh)
        new_w = int(ratio * self.rgb888p_size[0])
        new_h = int(ratio * self.rgb888p_size[1])
        dw = float(self.model_input_size[0]- new_w) / 2
        dh = float(self.model_input_size[1] - new_h) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw - 0.1))
        return top,bottom,left,right
    # 根据当前类别索引获取框的颜色
    def get_color(self, x):
        idx=x%len(self.color_four)
        return self.color_four[idx]

def hardware_init():
    # 串口初始化
    fpioa = FPIOA()
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    return UART(UART.UART1, UART_BAUDRATE)

def single_send(x):
    num = x
    high_byte = (num >> 8) & 0xFF
    low_byte = num & 0xFF
    uart.write(bytes([high_byte, low_byte])) #{b.cx()}
def send_data(x1,y1,x2,y2,x3,y3,ins1 = 0,ins2 = 0):
    uart.write("a")
    uart.write("a")
    Num = [x1,y1,x2,y2,x3,y3]
    for j in range(6):
        single_send(Num[j])
    single_send(ins1)
    single_send(ins2)
    

def handle_category(category):
    global mode,start,comeback
    if category == r_flag:
        mode = 0
        comeback = 1 
    elif category == y_flag:
        mode = 1
        comeback = 0
    elif category == B_flag:#黑球
        mode = 2
        comeback = 0 
    elif category == b_flag:
        mode = 3
        comeback = 1 
    elif category == start_flag:
        start = 1
    elif category == stop_flag:
        start = 0
    elif category == safe_flag:
        comeback = 1   
    else:
        pass
def ball_check(x,y):
    if(320<=x<=520 and 300<=y<=430):#球在爪子内
        return True
    return False
def detect_start():
    q = {}
    m = {}
    qx = 0
    qy = 0
    mx = 0
    my = 0
    if det_boxes2:
        pl.osd_img.clear()
        for b in det_boxes2: #画矩形、箭头和字符表示
            #img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
            # area_str = f"长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
            # img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
            x1, y1, x2, y2 = map(int, b[2:6])
            x1, y1, x2, y2 = map(int, b[2:6])
            x1=int(x1 * yolo_start.display_size[0] // yolo_start.rgb888p_size[0])
            y1=int(y1 * yolo_start.display_size[1] // yolo_start.rgb888p_size[1])
            x2=int(x2 * yolo_start.display_size[0] // yolo_start.rgb888p_size[0])
            y2=int(y2 * yolo_start.display_size[1] // yolo_start.rgb888p_size[1])
            cls_name, color = START_MAP[b[0]]
            b_cx = (x1 + x2) // 2
            b_cy = (y1 + y2) // 2
            if cls_name == ST_MAP[2]:
                q[b_cy] = b_cx
                pl.osd_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=color, thickness=3)
                pl.osd_img.draw_string_advanced(x1, y1-35,32, f"{cls_name} {b[1]:.2f}", color=color)
            if cls_name == ST_MAP[3]:
                m[b_cy] = b_cx
                pl.osd_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=color, thickness=3)
                pl.osd_img.draw_string_advanced(x1, y1-35,32, f"{cls_name} {b[1]:.2f}", color=color)
        if len(q):
            qy = int(sum(q.keys())/len(q)+0.5)
            qx = int(sum(q.values())/len(q)+0.5)
        if len(m):
            my = int(sum(m.keys())/len(m)+0.5)
            mx = int(sum(m.values())/len(m)+0.5)
        return qx,qy,mx,my
    else:
        pl.osd_img.clear()
    return 0,0,0,0
def detect(cs = 0,cl = 0,start = 0):#cs只能取0或3，表红色和蓝色,cl表球真实色彩取0,1,2,3
    global precs,color,send_flag,comeback
    send_flag = 0
    p = {}
    q = {}
    m = {}
    ins = 0
    bx = 0
    by = 0
    qx = 0
    qy = 0
    mx = 0
    my = 0
    not_normal_cnt = 0
    not_danger_cnt = 0
    if cl == 0 or cl == 3:
        cs = cl
    else:
        cs = precs
    precs = cs
    if(start==1):
        send_data(bx,by,Sr_x,Sr_y,Sb_x,Sb_y,ins1 = ins,ins2 = send_flag)
        return
    # 绘制检测框
    if det_boxes and comeback == 1:
        pl.osd_img.clear()
        for b in det_boxes: #画矩形、箭头和字符表示
            #img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
            # area_str = f"长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
            # img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
            x1, y1, x2, y2 = map(int, b[2:6])
            x1, y1, x2, y2 = map(int, b[2:6])
            x1=int(x1 * yolo_main.display_size[0] // yolo_main.rgb888p_size[0])
            y1=int(y1 * yolo_main.display_size[1] // yolo_main.rgb888p_size[1])
            x2=int(x2 * yolo_main.display_size[0] // yolo_main.rgb888p_size[0])
            y2=int(y2 * yolo_main.display_size[1] // yolo_main.rgb888p_size[1])
            cls_name, color = CLASS_MAP[b[0]]
            b_cx = (x1 + x2) // 2
            b_cy = (y1 + y2) // 2
            if cls_name == BALL_MAP[cs]:
                p[b_cy] = b_cx
                pl.osd_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=color, thickness=3)
                pl.osd_img.draw_string_advanced(x1, y1-35,32, f"{cls_name} {b[1]:.2f}", color=color)
            else:
                if(ball_check(b_cx,b_cy) and cls_name != Q_MAP[0] and cls_name != Q_MAP[3]):
                    not_normal_cnt += 1
            if cls_name == Q_MAP[0]:
                q[b_cy] = b_cx
                pl.osd_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=color, thickness=3)
                pl.osd_img.draw_string_advanced(x1, y1-35,32, f"{cls_name} {b[1]:.2f}", color=color)
            if cls_name == Q_MAP[3]:
                m[b_cy] = b_cx
                pl.osd_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=color, thickness=3)
                pl.osd_img.draw_string_advanced(x1, y1-35,32, f"{cls_name} {b[1]:.2f}", color=color)
        if len(p):
            by, bx = min(p.items(), key=lambda item: (item[1] - 420)**2 + (item[0] - 480)**2)
        if len(q):
            qy = int(sum(q.keys())/len(q)+0.5)
            qx = int(sum(q.values())/len(q)+0.5)
            if(qy>QY_MIN and cs == 0):
                ins = 1
        if len(m):
            my = int(sum(m.keys())/len(m)+0.5)
            mx = int(sum(m.values())/len(m)+0.5)
            if(my>QY_MIN and cs == 3):
                ins = 1
    else:
        pl.osd_img.clear()
    if (cl == 1 or cl == 2) and comeback == 0:
        p = {}
        bx = 0
        by = 0
        if det_boxes:
            for b in det_boxes: #画矩形、箭头和字符表示
                #img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
                # area_str = f"长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
                # img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
                x1, y1, x2, y2 = map(int, b[2:6])
                x1=int(x1 * yolo_main.display_size[0] // yolo_main.rgb888p_size[0])
                y1=int(y1 * yolo_main.display_size[1] // yolo_main.rgb888p_size[1])
                x2=int(x2 * yolo_main.display_size[0] // yolo_main.rgb888p_size[0])
                y2=int(y2 * yolo_main.display_size[1] // yolo_main.rgb888p_size[1])
                cls_name, color = IRR_MAP[b[0]]
                b_cx = (x1 + x2) // 2
                b_cy = (y1 + y2) // 2
                if cls_name == BALL_MAP[cl]:
                    p[b_cy] = b_cx
                    pl.osd_img.draw_rectangle(x1, y1, x2-x1, y2-y1, color=color, thickness=3)
                    pl.osd_img.draw_string_advanced(x1, y1-35,32, f"{cls_name} {b[1]:.2f}", color=color)
                else:
                    if(ball_check(b_cx,b_cy) and cls_name != Q_MAP[0] and cls_name != Q_MAP[3]):
                        not_danger_cnt += 1       
        else:
            pl.osd_img.clear()
        if len(p):
            by, bx = min(p.items(), key=lambda item: (item[1] - 420)**2 + (item[0] - 480)**2)
    if((cl==cs and not_normal_cnt) or not_danger_cnt):
        send_flag = 1
    send_data(bx,by,qx,qy,mx,my,ins1 = ins,ins2 = send_flag)

# 初始化按键
fpioa = FPIOA()
fpioa.set_function(21, FPIOA.GPIO21)
button = Pin(21, Pin.IN, Pin.PULL_UP)
def btn():

    if button.value() == 0:  # 按键按下
            time.sleep_ms(10)  # 消抖
            if button.value() == 0:
                if(mode == 1 and comeback == 0):
                    mode = 0# 模型切换
                    comeback = 1
                else:
                    mode = 1# 模型切换
                    comeback = 0
                #print(f"图片已保存为 /sdcard/img_{number}.jpg") 
            while button.value() == 0:
                pass 
def read_uart():
    while True:
        text = uart.read(1)
#        print("thread on")
        if text:
            handle_category(text)
            #print(text)
        btn()
if __name__ == "__main__":
    display_mode="lcd"
    if display_mode=="hdmi":
        display_size=[800,480]
    else:
        display_size=[800,480]
    kmodel_path="/sdcard/app/best_AnchorBaseDet_can2_5_n_20250424182116.kmodel"
    kmodel_path2="/sdcard/app/chufaqv2.kmodel" #出发区
    kmodel_path3 = "sdcard/app/irr.kmodel"
    #kmodel_path3 = "sdcard/app/best_AnchorBaseDet_can2_5_n_20250424141042.kmodel" #不规则2

    #best_AnchorBaseDet_can2_5_n_20250421144931.kmodel #广角最优模型
    #best_AnchorBaseDet_can2_5_n_20250424182116.kmodel #4.24广角模型
    #best_AnchorBaseDet_can2_5_n_20250424233956.kmodel #出发区模型
    #sdcard/app/best_AnchorBaseDet_can2_5_n_20250422195651.kmodel #不规则1
    label=["红方安全区","黑球","黄球","蓝球","红球","黑方安全区"]
    # labels2=["红方","蓝方"]
    labels2=["红安","蓝安","红出","蓝出"]
    labels3=["红球","黄球","黑球","蓝球","红方安全区","黑方安全区"]
    confidence_threshold=0.5
    nms_threshold = 0.5
    anchors=[30,23,21,33,29,43,44,29,41,39,41,68,71,43,59,61,71,72]
    pl=PipeLine(rgb888p_size=[1280,720],display_size=display_size,display_mode=display_mode)
    pl.create()
    uart = hardware_init()
    yolo_start = YOLODetector(kmodel_path2,labels2,model_input_size=[640,640],anchors=anchors,rgb888p_size=[1280,720],display_size=display_size,debug_mode=0)
    yolo_start.config_preprocess()
    yolo_main = YOLODetector(kmodel_path,label,model_input_size=[640,640],anchors=anchors,rgb888p_size=[1280,720],display_size=display_size,debug_mode=0)
    yolo_main.config_preprocess()
    yolo_irr = YOLODetector(kmodel_path3,labels3,model_input_size=[640,640],anchors=anchors,rgb888p_size=[1280,720],display_size=display_size,debug_mode=0)
    yolo_irr.config_preprocess()
    _thread.start_new_thread(read_uart,()) #开启线程1,读取uart
    try:
        while True:
            clock.tick()
            os.exitpoint()
            with ScopedTiming("total",1):
                img=pl.get_frame()
                if(start==1):
                    det_boxes2 = yolo_start.run(img)
                    Sr_x,Sr_y,Sb_x,Sb_y = detect_start()
                else:
                    if(mode == 1 or mode == 2):
                        det_boxes = yolo_irr.run(img)
                    else:
                        det_boxes = yolo_main.run(img)
                        pass
                detect(cs = color,cl = mode,start=start)
                #yolo.draw_result(pl,det_boxes)
                pl.osd_img.draw_string_advanced(0, 0, 30, 'FPS: '+str("%.3f"%(clock.fps())), color = (255, 255, 255))
                pl.show_image()
                gc.collect()  # 手动释放内存
                
    except BaseException as e:
        sys.print_exception(e)
    finally:
        yolo_main.deinit()
        yolo_start.deinit()
        pl.destroy()
