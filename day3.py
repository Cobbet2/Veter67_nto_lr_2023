import rospy
import cv2 as cv
from clover import srv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import numpy as np
import math
#  

rospy.init_node('computer_vision_sample')
bridge = CvBridge()




get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

image_pub = rospy.Publisher('~debug', Image, queue_size=10)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def image_callback(data):
    global u
    global firess
    global people
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    wall = cv.inRange(hsv, (75 // 2, 20, 100), (100 // 2, 255, 255))
    
    contours , __ = cv.findContours(wall, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    x, y = 0, 0
    c = get_telemetry(frame_id="aruco_map")
    cv.drawContours(cv_image, contours, -1, (0, 0, 255), 1)
    m1 = wall.copy()[80:200,320-80:]

    fire = cv.inRange(hsv, (39 // 2, 20, 100), (53 // 2, 255, 255))
    
    contours , __ = cv.findContours(fire, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(cv_image, contours, -1, (0, 255, 255), 1)
    
    fires = cv.countNonZero(fire[120:200,:])

    fire = cv.inRange(hsv, (39 // 2, 20, 100), (53 // 2, 255, 255))
    
    contours , __ = cv.findContours(fire, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(cv_image, contours, -1, (0, 255, 255), 1)

    fireboomcentery = cv.countNonZero(fire[:,120:220])
    fireboomcenterx = cv.countNonZero(fire[120:200,:])

    head = cv.inRange(hsv, (190 // 2, 20, 100), (200 // 2, 255, 255))
    
    contours , __ = cv.findContours(head, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(cv_image, contours, -1, (255, 0, 255), 1)
    heady = cv.countNonZero(head[:,120:220])
    headx = cv.countNonZero(head[120:200,:])


    if cv.countNonZero(m1)>1000:
        if fires >1500:
            firess.append([c.x,c.y-0.2])
        navigate_wait(x=c.x+0.35, y = 4, z =1.25, frame_id="aruco_map")
    else:
        if fireboomcenterx>1500 and fireboomcentery>1500:
            firess.append([c.x,c.y])
            navigate_wait(x=c.x, y = c.y-0.1, z=1.25,frame_id="aruco_map")
        if headx>1500 and heady>1500:
            people.append([c.x,c.y])
            navigate_wait(x=c.x, y = c.y-0.1,z=1.25, frame_id="aruco_map")
        navigate_wait(x=c.x, y = c.y-0.1,z=1.25, frame_id="aruco_map")
    if c.x>6.5:
        navigate_wait(x = 0.5,y = 3.5,z=1.25, frame_id="aruco_map")
        
        u = False
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

firess = []
people = []
u = True
navigate_wait(x = 0, y = 0, z = 1.25,speed=0.25, auto_arm=True, frame_id='body')
start_cords= get_telemetry(frame_id='aruco_map')
navigate_wait(x = 0.5, y = 1, z = 1.25, frame_id="aruco_map")
navigate_wait(x = 0.5, y = 4, z = 1.25, frame_id="aruco_map")
navigate_wait(x = 1, y = 4, z = 1.25,yaw = 0 ,speed=0.5, frame_id='aruco_map')

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
while True:
    rospy.sleep(0.1)
    if not(u):
        break
navigate_wait(start_cords.x, start_cords.y, start_cords.z, frame_id="aruco_map")
land()
print("Fires: "+str(len(firess)))
for x in range(len(firess)):
    print("Fire " + str(x)+": "+ str(x[0]) +" "+ str(x[1]))
print("Injured: " + str(len(people)))
for x in range(len(people)):
    print("Injured "+str(x)+": "+str(x[0]) +" "+ str(x[1]))
for x in range(1,11):
    print("Wall "+ str(x) + ": " + "___" + "___")
rospy.speen()
