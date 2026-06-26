'''
实验名称：颜色识别
实验平台：01Studio CanMV K230
教程：wiki.01studio.cc
'''

import time, os, sys
from media.sensor import * #导入sensor模块，使用摄像头相关接口
from media.display import * #导入display模块，使用display相关接口
from media.media import * #导入media模块，使用meida相关接口
from machine import UART #导入串口模块
from machine import FPIOA

# 颜色识别阈值 (L Min, L Max, A Min, A Max, B Min, B Max) LAB模型
# 下面的阈值元组是用来识别 红、绿、蓝三种颜色，当然你也可以调整让识别变得更好。
thresholds = [(0, 100, 41, 127, 10, 127), # 红色阈值 (30, 100, 15, 127, 15, 127)
              (30, 100, -64, -8, 50, 70), # 绿色阈值 
              (9, 100, -26, 127, -128, -17)] # 蓝色阈值(9, 100, -26, 127, -128, -17)

colors1 = [(255,0,0), (0,255,0), (0,0,255)]
colors2 = ['RED', 'GREEN', 'BLUE']
#面积约束
ball_max = 30000
gate_min = 20000
#串口标志
ball_flag = b'\xaa'
gate_flag = b'\xbb'
mode = 0
#初始化串口
fpioa = FPIOA()
# UART1代码
fpioa.set_function(3,FPIOA.UART1_TXD)
fpioa.set_function(4,FPIOA.UART1_RXD)
uart=UART(UART.UART1,115200) #设置串口号1和波特率

sensor = Sensor() #构建摄像头对象
sensor.reset() #复位和初始化摄像头
sensor.set_framesize(width=800, height=480) #设置帧大小为LCD分辨率(800x480)，默认通道0
sensor.set_pixformat(Sensor.RGB565) #设置输出图像格式，默认通道0

Display.init(Display.ST7701, to_ide=True) #同时使用3.5寸mipi屏和IDE缓冲区显示图像，800x480分辨率
#Display.init(Display.VIRT, sensor.width(), sensor.height()) #只使用IDE缓冲区显示图像

MediaManager.init() #初始化media资源管理器

sensor.run() #启动sensor
clock = time.clock() #定义时钟



def single_send(x):
    num = x
    high_byte = (num >> 8) & 0xFF
    low_byte = num & 0xFF
    uart.write(bytes([high_byte, low_byte])) #{b.cx()}

def send(x,y,area = 0):
    uart.write("a")
    Num = [x,y]
    for j in range(2):
        num = Num[j]
        high_byte = (num >> 8) & 0xFF
        low_byte = num & 0xFF
        uart.write(bytes([high_byte, low_byte])) #{b.cx()}
    single_send(area)
    uart.write("b")
def detect(mode = 0,cs = 0):
    if(mode == 0):
        blobs = img.find_blobs([thresholds[cs]],area_threshold=200,pixels_threshold=10,merge=True, margin=1) # 0,1,2分别表示红，绿，蓝色。
        p = {}
        if blobs:
            for b in blobs: #画矩形、箭头和字符表示
                tmp=img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
                #tmp=img.draw_cross(b[5], b[6], thickness = 2)
                #tmp=img.draw_string_advanced(b[0], b[1]-35, 30, colors2[i],color = colors1[i])
                area_str = f"area:{b.area()},ratio:{b.area()/384000}"
                img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
                print("坐标:",b.cx(),b.cy(),"面积:",b.area())
                if b.area() < ball_max:
                    p[b.cy()] = b.cx()
            if len(p):
                y = max(p.keys())
                x = p[y]
                send(x,y,b.area())
    if(mode == 1):
        blobs = img.find_blobs([thresholds[cs]],area_threshold=200,pixels_threshold=10,merge=True, margin=1) # 0,1,2分别表示红，绿，蓝色。
        p = {}
        if blobs:
            for b in blobs: #画矩形、箭头和字符表示
                img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
                #tmp=img.draw_cross(b[5], b[6], thickness = 2)
                area_str = f"area:{b.area()},ratio:{b.area()/384000}"
                img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
                print("坐标:",b.cx(),b.cy(),"面积:",b.area())
                if b.area() > gate_min:
                    p[b.cy()] = b.cx()
            if len(p):
                y = max(p.keys())
                x = p[y]
                send(x,y,b.area())
while True:
    clock.tick()
    img = sensor.snapshot() #拍摄一张图片
    text = uart.read(1) #接收128个字符
    print(text)
    if text == gate_flag:
        mode = 1
    if text == ball_flag:
        mode = 0
    detect(mode = mode)
    img.draw_string_advanced(0, 0, 30, 'FPS: '+str("%.3f"%(clock.fps())), color = (255, 255, 255))
    Display.show_image(img) #显示图片
   # print(clock.fps()) #打印FPS


