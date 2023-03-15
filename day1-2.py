import rospy
import cv2 as cv
from clover import srv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import numpy as np
import math
#  инициализируем библиотеки

rospy.init_node('computer_vision_sample')
bridge = CvBridge()

# Объявление прокси:
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

image_pub = rospy.Publisher('~debug', Image, queue_size=1)
# создаём топик для видео /\

 #Функция для полета в точку и ожидание окончания полета:
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
        
        
#функция автономного полёта:

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    w,h,ch = cv_image.shape
    

    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    fire = cv.inRange(hsv, (30//2, 100, 100), (46//2, 255, 255))
    contours, __ = cv.findContours(fire, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE) #находим контуры огня
    x , y = 0, 0
    fl = False
    if len(contours)>0:
        contours.sort(key=cv.minAreaRect)
        cnt = contours[0]
        
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        x,y , w,h  = box

        box = np.int0(box)
        cv.drawContours(cv_image,[box],-1,(0,0,255),1)

    wall = cv.inRange(hsv, (70//2,20,100), (97//2, 255, 255)) # определение края стены

    m1 = wall[340-100:][240-80:]
    
    m2 = wall[:100][240-80:]

    m3 = wall[240-80:][80:240-80]

    if cv.countNonZero(m1)>350:
        a = True
    else:
        a = False

    if cv.countNonZero(m1)>750:
        c = True
    else:
        c = False

    if cv.countNonZero(m2)>350:
        b = True
    else:
        b = False

    if cv.countNonZero(m3)>350:
        f = True
    else:
        f = False
    # определение стены на изображении
    if fl :
        c = get_telemetry(frame_id="aruco_map")
        if a and not b:# определение начала стены
            teln = c 
        if b and not a:# определение угла, лететь направо
            tel = c
            dlina = max((tel.x-teln.x), (tel.y-teln.y))-0.1
            navigate_wait(yaw = -90,frame_id="body")
            c = get_telemetry(frame_id="aruco_map") 
            teln = c
        if a and f: # определение угла, лететь налево
            tel = c
            dlina = max((tel.x-teln.x), (tel.y-teln.y))-0.1
            navigate_wait(yaw = 90,frame_id="body")
            c = get_telemetry(frame_id="aruco_map")
            teln = c
        if c.x>6.5: # опредедление конца зоны разведки
            navigate_wait(x = 6.5, y = 0, frame_id = "aruco_map") 
        if x!=0 and y!=0:# определение огня
            if w//2-30<x<w//2+30:
                if h//2-30<y<h//2+30:
                    print("fire" + str(c.x)+ str(c.y))
                if h//2-30 > y:
                    print("fire" + str(c.x-0.1)+ str(c.y-0.1))
                if h//2+30 <y:
                    print("fire" + str(c.x+0.1)+ str(c.y+0.1))
        navigate_wait(x=0.05,y=0,z=0,frame_id="body")
    
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))# вывод изображения в топик

    

    

navigate_wait(x = 0, y = 0, z = 0.75,speed=0.25, auto_arm=True, frame_id='body') #взлёт
start_cords= get_telemetry(frame_id='aruco_map') #запоминаем координаты взлёта


navigate_wait(x = 0.5, y = 1.5, z = 0.75,yaw = 0,speed=0.25, frame_id='aruco_map') # летим к стене
if fl == 1:
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback) # обработка изображения

    rospy.spin()
navigate_wait(x = start_cords.x, y = start_cords.y, z = start_cords.z,speed=0.25, frame_id='aruco_map') #подлёт к зоне H

land() #посадка

