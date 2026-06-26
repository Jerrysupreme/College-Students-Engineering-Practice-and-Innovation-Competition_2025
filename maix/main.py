#导入必须的功能包
from maix import camera, display, image, nn, app,time,uart,pwm,pinmap,gpio
import threading
#导入yolo11训练转换好的模型
detector = nn.YOLOv5(model="/root/models/model_231677.mud", dual_buff = True)
#寻找串口设备
device = "/dev/ttyS0"
#设定摄像头的输入图像尺寸
input_width=800
input_height=480
cam = camera.Camera(input_width, input_height, detector.input_format())
cam.skip_frames(30) #跳过开头的30帧
disp = display.Display()
#设定波特率
serial = uart.UART(device, 115200)
#设定球颜色的顺序
labels = ["blue square","red square","black ball","blue ball","yellow ball","red ball"]
# #照明
pinmap.set_pin_function("B3", "GPIOB3")
led = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
led.value(1)

# 全局变量
# 类别配置（与原系统对应）
img = image.Image(input_width, input_height, detector.input_format())
BALL_MAP = ["red ball","yellow ball","black ball","blue ball"]
Q_MAP = ["red square","","","blue square"]
QY_MIN = 230
color = 0 # 决定阵营
mode = color # 取0,1,2,3,表球真实色彩
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

def single_send(x):
    num = x
    high_byte = (num >> 8) & 0xFF
    low_byte = num & 0xFF
    serial.write(bytes([high_byte, low_byte])) #{b.cx()}
def send_data(x1,y1,x2,y2,x3,y3,ins1 = 0,ins2 = 0):
    serial.write_str("a")
    serial.write_str("a")
    Num = [x1,y1,x2,y2,x3,y3]
    for j in range(6):
        single_send(Num[j])
    single_send(ins1)
    single_send(ins2)

def handle_category(category):
    global mode
    if category == r_flag:
        mode = 0
    elif category == y_flag:
        mode = 1
    elif category == B_flag:#黑球
        mode = 2
    elif category == b_flag:
        mode = 3
    else:
        pass
def ball_check(x,y):
    if(330<=x<=510 and 390<=y<=480):#球在爪子内
        return True
    return False

#串口接收线程
def uart_read_thread():
    while not app.need_exit():
        text = serial.read(1)
        if text:
            handle_category(text)
        time.sleep_ms(10)


def detect(cs = 0,cl = 0):#cs只能取0或3，表红色和蓝色,cl表球真实色彩取0,1,2,3
    global precs,color,send_flag,img
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
    img = cam.read()
    det_boxes = detector.detect(img, conf_th=0.5, iou_th=0.45)
    # 绘制检测框
    if det_boxes:
        for obj in det_boxes: #画矩形、箭头和字符表示
            #img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
            # area_str = f"长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
            # img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
            x1, y1, x2, y2 = obj.x, obj.y, obj.x+obj.w, obj.y+obj.h 
            cls_name = labels[obj.class_id]
            b_cx = (x1 + x2) // 2
            b_cy = (y1 + y2) // 2
            if cls_name == BALL_MAP[cs]:
                p[b_cy] = b_cx
                img.draw_rect(x1, y1, x2-x1, y2-y1, color=image.COLOR_RED, thickness=3)
                img.draw_string(x1, y1-35, f"{cls_name} {obj.score:.2f}", color=image.COLOR_RED,scale = 2)            
            else:
                if(ball_check(b_cx,b_cy) and cls_name != Q_MAP[0] and cls_name != Q_MAP[3]):
                    not_normal_cnt += 1
            if cls_name == Q_MAP[0]:
                q[b_cy] = b_cx
                img.draw_rect(x1, y1, x2-x1, y2-y1, color=image.COLOR_RED, thickness=3)
                img.draw_string(x1, y1-35, f"{cls_name} {obj.score:.2f}", color=image.COLOR_RED,scale = 2)
            if cls_name == Q_MAP[3]:
                m[b_cy] = b_cx
                img.draw_rect(x1, y1, x2-x1, y2-y1, color=image.COLOR_RED, thickness=3)
                img.draw_string(x1, y1-35, f"{cls_name} {obj.score:.2f}", color=image.COLOR_RED,scale = 2) 
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
    if cl == 1 or cl == 2:
        p = {}
        bx = 0
        by = 0
        if det_boxes:
            for obj in det_boxes: #画矩形、箭头和字符表示
                x1, y1, x2, y2 = obj.x, obj.y, obj.x+obj.w, obj.y+obj.h 
                cls_name = labels[obj.class_id]
                b_cx = (x1 + x2) // 2
                b_cy = (y1 + y2) // 2
                if cls_name == BALL_MAP[cl]:
                    p[b_cy] = b_cx
                    img.draw_rect(x1, y1, x2-x1, y2-y1, color=image.COLOR_RED, thickness=3)
                    img.draw_string(x1, y1-35, f"{cls_name} {obj.score:.2f}", color=image.COLOR_RED,scale = 2)
                else:
                    if(ball_check(b_cx,b_cy) and cls_name != Q_MAP[0] and cls_name != Q_MAP[3]):
                        not_danger_cnt += 1       
        if len(p):
            by, bx = min(p.items(), key=lambda item: (item[1] - 420)**2 + (item[0] - 480)**2)
    if((cl==cs and not_normal_cnt) or not_danger_cnt):
        send_flag = 1
    print("ball:",bx,by)
    send_data(bx,by,qx,qy,mx,my,ins1 = ins,ins2 = send_flag)

thresholds = [[32, 36, -10, 11, -4, 18]]      # red
def find_kuai():
    while not app.need_exit():
        img = cam.read()
        blobs = img.find_blobs(thresholds, pixels_threshold=500)
        p = {}
        b_cx,b_cy = 0, 0
        by,bx = 0,0
        for blob in blobs:
            img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_GREEN)
            b_cx,b_cy = blob[0]+blob[2]/2,blob[1]+blob[3]/2
            p[b_cy] = b_cx
        if len(p):
            by, bx = min(p.items(), key=lambda item: (item[1] - 420)**2 + (item[0] - 480)**2)
        qx,qy,mx,my = 0,0,0,0
        print("ball:",bx,by)
        bx,by = int(bx),int(by)
        send_data(bx,by,qx,qy,mx,my)
        fps=time.fps()
        img.draw_string(10, 10,f'fps:{fps:.2f}', color=image.COLOR_RED,scale=2)#画框并且标注信息
        disp.show(img)
def single_detect():
    while not app.need_exit():
        img = cam.read()
        det_boxes = detector.detect(img, conf_th=0.5, iou_th=0.45)
        p = {}
        b_cx,b_cy = 0, 0
        by,bx = 0,0
        # 绘制检测框
        if det_boxes:
            for obj in det_boxes: #画矩形、箭头和字符表示
                #img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
                # area_str = f"长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
                # img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
                x1, y1, x2, y2 = obj.x, obj.y, obj.x+obj.w, obj.y+obj.h
                b_cx = (x1 + x2) // 2
                b_cy = (y1 + y2) // 2
                p[b_cy] = b_cx
                img.draw_rect(x1, y1, x2-x1, y2-y1, color=image.COLOR_RED, thickness=3)
                #img.draw_string(x1, y1-35, f"{cls_name} {obj.score:.2f}", color=image.COLOR_RED,scale = 2)            
        if len(p):
            by, bx = min(p.items(), key=lambda item: (item[1] - 420)**2 + (item[0] - 480)**2)
        qx,qy,mx,my = 0,0,0,0
        bx,by = int(bx),int(by)
        send_data(bx,by,qx,qy,mx,my)
        fps=time.fps()
        print("ball:",bx,by,fps)
        #img.draw_string(10, 10,f'fps:{fps:.2f}', color=image.COLOR_RED,scale=2)#画框并且标注信息
        #disp.show(img)
def main_test():
    while not app.need_exit():
        #find_target_ball()
        detect(cs = color,cl = 1)
        fps=time.fps()
        img.draw_string(10, 10,f'fps:{fps:.2f}', color=image.COLOR_RED,scale=2)#画框并且标注信息
        #isp.show(img)
   
#threading.Thread(target=main_test,daemon=True).start()
threading.Thread(target=single_detect,daemon=True).start()
threading.Thread(target=uart_read_thread, daemon=True).start()
# threading.Thread(target=LED, daemon=True).start()
# 主循环
while not app.need_exit():
    time.sleep(10)