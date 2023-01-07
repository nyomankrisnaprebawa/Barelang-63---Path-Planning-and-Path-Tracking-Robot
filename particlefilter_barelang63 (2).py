#!/usr/bin/env python3

from scipy.spatial import distance
from numpy.random import uniform,normal
from std_msgs.msg import String
from std_msgs.msg import UInt16
from os import system, name
import time
import socket
import sys
import numpy as np
import scipy.stats
import math
import cv2
import webbrowser
import os
import rospy
import random
from os import system
from barelang63.msg import pathdata
from std_msgs.msg import Float32MultiArray  
from barelang63.msg import SensorFusion
sys.path.append("/home/asus/catkin_ws/PythonRobotics/PathPlanning/CubicSpline/")
sys.path.append("/home/asus/catkin_ws/src/barelang63/src/")

#Import Path planning to Localization
sys.path.append(os.path.dirname(os.path.abspath(__file__))) 
try:
    from rrt import RRT
    import cubic_spline_planner

except ImportError:
    raise

''' Variabel yang digunakan untuk path planning '''
dataForPath = ""
dataPath  = np.zeros((100))
goalPos = np.zeros((10))


activePath = False
pathMode = 1
obstacleA = []
obstacleB = []
cx = []
cy = []
xStartLast = 0
xEndLast = 0
yStartLast = 0
yEndLast = 0
xStart = 0
yStart = 0
xEnd = 0
yEnd = 0
scaleObstacle = 50
pointXCoor = []
pointYCoor = []
intpathX = []


lineXStart = 0
lineYStart = 0
lineXEnd = 0
lineYEnd = 0
lineXDrawS = 0
lineYDrawS = 0
lineXDrawE = 0
lineYDrawE = 0

''' Variabel yang digunakan untuk path planning '''

#Variable
robotID = 1
headingFromIMu = True
#Obstacle
totalLawan = 4
ObstaclePosition = np.zeros((totalLawan,2))
jumlahObstacle = np.zeros((5))
obstacleAngle = np.zeros((5))
obstacleDistance = np.zeros((5))                                              
obstacleFromKinematik = np.zeros((9))

#Particle Filter Configuration
headingFromIMU = True
totalPartikel = 300
totalLandmark = 6
deltaTime = 1

#Position
robotGlobalPosition = np.zeros((3))
robotInitialPosition = np.zeros((3))
estimatePosition = np.zeros((3))
estimateLocalPosition = np.zeros((3))
ballEstimatePosition = np.zeros((2))
opponentEstimatePosition = np.zeros((2))
opponentHeading = np.zeros((4))

#Landmark 
jarakRobotLandmark = np.zeros((totalLandmark))
jarakPartikelLandmark = np.zeros((totalPartikel, totalLandmark))
landmarkPosition = np.zeros((totalLandmark, 2))

#Partikel
particleGlobalPosition = np.zeros((totalPartikel,3))
particleLocalPosition = np.zeros((totalPartikel,3))
particleInitialPosition = np.zeros((totalPartikel,3))
particleWeight = np.zeros((totalPartikel))

#Data robot
dataFromMain = ""
dataMain= np.zeros((100))
globalFromKinematic = np.zeros((3))
speedFromKinematic = np.zeros((3))
goalPosFromMain = np.zeros((3))
landmarkFromKinematic = np.zeros((6))
imuFromKinematic = np.zeros((2))
imuInitHeading = 0
imuCurentHeading = 0
ballDistance = 0
ballAngle = 0
jumlahLandmark = 0



#Lapangan 
modeLapangan = 1
if modeLapangan == 1:
    panjangLapangan = 1200
    lebarLapangan = 800
    mapImage = np.zeros((1000,1400,3), np.uint8)


elif modeLapangan == 2:
    panjangLapangan = 900
    lebarLapangan = 600
    mapImage = np.zeros((800,1100,3), np.uint8)
    
else:
    pass

def worldCoorToImageCoor(x,y):
    if modeLapangan == 1: ## Konversi Koordinat global ke gambar 12x8
        x = x + 100
        # y = 1000 - (y + 100)
        y = y + 100
        return x,y

    elif modeLapangan == 2: ## Konversi Koordinat Global ke gambar 9x6
        x = x + 100
        y = y + 100
        # y = 800 - (y + 100)
        return x,y

def obstaclePosCalc():
    # Global Variabel
    global ObstaclePosition
    global obstacleAngle
    global obstacleDistance
    
    for i in range(totalLawan):
        ObstaclePosition[i,0] = obstacleDistance[i]
        ObstaclePosition[i,1] = 0
        

        opponentHeading[i] = imuFromKinematic[1] + obstacleAngle[i]
        if opponentHeading[i] >= 360:
            opponentHeading[i] = opponentHeading[i] - 360
        if opponentHeading[i] < 0:
            opponentHeading[i] = 360 + opponentHeading[i]
        
        theta = np.radians(opponentHeading[i])
        c,s = np.cos(theta),np.sin(theta)
        R = np.array(((c,-s),(s,c)))
        npOutMatmul = np.matmul(R,ObstaclePosition[i,:2])
        ObstaclePosition[i,0] = npOutMatmul[0] + estimatePosition[0] # X Lawan
        ObstaclePosition[i,1] = npOutMatmul[1] + estimatePosition[1] # Y Lawan
        

    for i in range (totalLawan):       
        x, y = worldCoorToImageCoor(int(ObstaclePosition[i,0]), int(ObstaclePosition[i,1]))
        cv2.circle(mapImage,(x, y), 25, (204,102,0), -1)

    # print "Posisi Obstacle 1",ObstaclePosition[0,:]
    # print "Posisi Obstacle 2",ObstaclePosition[1,:]
    # print "Posisi Obstacle 3",ObstaclePosition[2,:]
    # print "Posisi Obstacle 4",ObstaclePosition[3,:]
    
        
    

def ballPosCalc():
    
    global ballEstimatePosition
    global ballAngle
    global ballDistance  
    #Ini masih dalam koordinan lokal
    ballEstimatePosition[0] = ballDistance
    ballEstimatePosition[1] = ballAngle
    #Ditambahkan dengan posisi angle
    ballHeading = imuFromKinematic[1] + ballAngle
    if ballHeading >= 360:
        ballHeading = ballHeading - 360
    if ballHeading < 0:
        ballHeading = 360 + ballHeading
    
    theta = np.radians(ballHeading)
    c,s = np.cos(theta),np.sin(theta)
    R = np.array(((c,-s),(s,c)))
    npOutMatmul = np.matmul(R,ballEstimatePosition[:2])
    ballEstimatePosition[0] = npOutMatmul[0] + estimatePosition[0]
    ballEstimatePosition[1] = npOutMatmul[1] + estimatePosition[1]
    
    drawBall = True
    if (drawBall == True):
        x, y = worldCoorToImageCoor(int(ballEstimatePosition[0]), int(ballEstimatePosition[1]))
        cv2.circle(mapImage,(x, y), 14, (0,128,255), -1)
    # print"Ball X Position:", ballEstimatePosition[0]
    # print"Ball Y position:", ballEstimatePosition[1]




def drawLapangan():
    if modeLapangan == 1:
        
        mapImage[:] = (0, 153, 0)
        cv2.rectangle(mapImage, (100,100), (1300,900), (255,255,255), 3) #Garis Luar
        cv2.rectangle(mapImage, (20,600), (100,400), (255,255,255), 3) #Garis Luar Gawang Kiri
        cv2.rectangle(mapImage, (1300,600), (1380,400), (255,255,255), 3) #Garis Luar Gawang Kanan
        cv2.rectangle(mapImage, (100,650), (150,350), (255,255,255), 3) #Garis Gawang Kiri
        cv2.rectangle(mapImage, (1300,650), (1250,350), (255,255,255), 3) #Garis Gawang Kanan
        cv2.rectangle(mapImage, (100,750), (280,250), (255,255,255), 3) #Garis Pinalti Kiri
        cv2.rectangle(mapImage, (1300,750), (1120,250), (255,255,255), 3) #Garis Pinalti Kanan
        cv2.line(mapImage, (700,100), (700,900), (255,255,255), 3) #Garis Tengah
        cv2.circle(mapImage, (700,500), 130, (255,255,255), 3) # Lingkaran Tengah                    cv2.circle(mapImage, (700,500), 10, (255,255,255), 11) # Titik Lingkaran Tengah
        cv2.circle(mapImage, (700,300), 6, (0,0,0), 10) #Titik Lingkaran Atas
        cv2.circle(mapImage, (700,700), 6, (0,0,0), 10) #Titik Lingkaran bawah
        cv2.circle(mapImage, (320,500), 6, (255,255,255), 10) # Titik tengah Kiri
        cv2.circle(mapImage, (320,700), 6, (0,0,0), 10) # Titik Bawah Kiri
        cv2.circle(mapImage, (320,300), 6, (0,0,0), 10) # Titik Atas Kiri
        cv2.circle(mapImage, (1080,500), 6, (255,255,255), 10) # Titik tengah Kanan
        cv2.circle(mapImage, (1080,700), 6, (0,0,0), 10) # Titik bawah Kanan
        cv2.circle(mapImage, (1080,300), 6, (0,0,0), 10) # Titik bawah Kanan
        textLine = "(0,0)"
        x, y = worldCoorToImageCoor(0,0)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        textLine = "(0,800)"
        x, y = worldCoorToImageCoor(0,800)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        textLine = "(1200,800)"
        x, y = worldCoorToImageCoor(1200,800)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        textLine = "(1200,0)"
        x, y = worldCoorToImageCoor(1200,0)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        drawLandmark = True
        if drawLandmark == True:
            for i in range(totalLandmark):
                x,y = worldCoorToImageCoor(int(landmarkPosition[i,0]),int(landmarkPosition[i,1]))
                cv2.circle(mapImage,(x,y),15,(0,255,255), -1)
        
    
    elif modeLapangan == 2:
        mapImage[:] = (0,150,0)
        cv2.rectangle(mapImage, (100,100), (1000,700), (255,255,255), 3) #Garis Luar
        cv2.rectangle(mapImage, (10,500), (100,300), (255,255,255), 3) #Garis Luar Gawang Kiri
        cv2.rectangle(mapImage, (1000,500), (1090,300), (255,255,255), 3) #Garis Luar Gawang Kanan
        cv2.rectangle(mapImage, (100,550), (200,250), (255,255,255), 3) #Garis Pinalti Kiri
        cv2.rectangle(mapImage, (900,550), (1000,250), (255,255,255), 3) #Garis Pinalti Kanan
        cv2.line(mapImage, (550,100), (550,700), (255,255,255), 3) #Garis Tengah
        cv2.circle(mapImage, (550,400), 100, (255,255,255), 3) # Lingkaran Tengah
        cv2.circle(mapImage, (550,400), 6, (0,255,255), 10) # Lingkaran Tengah
        cv2.circle(mapImage, (840,550), 6, (255,0,0), 10) # Titik Obstacle kanan 3
        cv2.circle(mapImage, (840,250), 6, (255,0,0), 10) # Titik Obstacle kiri 3
        cv2.circle(mapImage, (840,400), 6, (255,0,0), 10) # Titik Obstacle tengah 3
        cv2.circle(mapImage, (740,550), 6, (255,0,0), 10) # Titik Obstacle kanan 2
        cv2.circle(mapImage, (740,250), 6, (255,0,0), 10) # Titik Obstacle kiri 2
        cv2.circle(mapImage, (740,400), 6, (255,0,0), 10) # Titik Obstacle tengah 2
        cv2.circle(mapImage, (640,550), 6, (255,0,0), 10) # Titik Obstacle kanan 1
        cv2.circle(mapImage, (640,250), 6, (255,0,0), 10) # Titik Obstacle kiri 1
        cv2.rectangle(mapImage, (713,100),(767,154),(127,0,255),3) #Posisi Robot Magenta
        cv2.rectangle(mapImage, (713,700),(767,646),(255,255,0),3) #Posisi Robot Cyan

        textLine = "(0,0)"
        x, y = worldCoorToImageCoor(0,0)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        textLine = "(0,600)"
        x, y = worldCoorToImageCoor(0,600)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        textLine = "(900,600)"
        x, y = worldCoorToImageCoor(900,600)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        textLine = "(900,0)"
        x, y = worldCoorToImageCoor(900,0)
        cv2.putText(mapImage, textLine, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 127), 1, cv2.LINE_AA)

        drawLandmark = False
        if drawLandmark == True:
            for i in range(totalLandmark):
                x,y = worldCoorToImageCoor(int(landmarkPosition[i,0]),int(landmarkPosition[i,1]))
                cv2.circle(mapImage,(x,y),15,(0,255,255), -1)
    

def dataRRTCb(dataIn):
    global activePath
    global dataForPath
    global goalPos
    dataForPath = dataIn.data

    dataPath = dataForPath.split("/")
    activePath = bool(dataPath[0])
    nilaiX = int(float(dataPath[1]))
    nilaiY = int(float(dataPath[2]))
    goalPos = [nilaiX,nilaiY]

    # print"Data Masuk Adalah : {} / {} ".format(activePath,goalPos)


def dataMasukCb(dataIn):
    global dataFromMain
    global ballDistance
    global ballAngle
    global jumlahLandmark
    global obstacleDistance
    global obstacleAngle
    global imuFromKinematic

    
    dataFromMain = dataIn.data
    dataMain = dataFromMain.split("/")
    # print"DataMasuk : ", dataFromMain
    globalFromKinematic[0] = format(float(dataMain[0]) + robotInitialPosition[0],'.2f')     #Data Ke-1 Posisi X
    globalFromKinematic[1] = format(float(dataMain[1]) + robotInitialPosition[1], '.2f')    #Data Ke-2 Posisi Y 
    globalFromKinematic[2] = format(float(dataMain[2]) + robotInitialPosition[2], '.2f')    #Data Ke-3 Hadap Robot
    
    speedFromKinematic[0] = format(float(dataMain[3]) ,'.2f')                               #Data Ke-4 Speed robot X
    speedFromKinematic[1] = format(float(dataMain[4]), '.2f')                               #Data Ke-5 Speed robot Y
    imuFromKinematic[1] = format(float(dataMain[5]),'.2f')                                  #Data Ke-6 Curent IMU (Sudut Headingn sekarang)
    ballDistance = int(dataMain[6])                                                         #Data Ke-7 Jarak Bola
    ballAngle = int(dataMain[7])                                                            #Data Ke-8 Sudut Bola
    # landmarkFromKinematic[0] = float(dataMain[8])                                           #Data Ke-9 Landmark 1
    # landmarkFromKinematic[1] = float(dataMain[9])                                           #Data Ke-10 Landmark 2
    # landmarkFromKinematic[2] = float(dataMain[10])                                          #Data Ke-11 Landmark 3
    # landmarkFromKinematic[3] = float(dataMain[11])                                          #Data Ke-12 Landmark 4
    # landmarkFromKinematic[4] = float(dataMain[12])                                          #Data Ke-13 Landmark 5
    # landmarkFromKinematic[5] = float(dataMain[13])                                          #Data Ke-14 Landmark 6
    # jumlahLandmark = int(dataMain[14])
    jumlahObstacle = int(dataMain[8])                                                      #Data Ke-16 Jumlah Obstacle yang terdeteksi
    obstacleFromKinematik = dataMain[9:len(dataMain)]                                      #Urutan data dari 16 sampai data terakhir
    obstacleAngle = np.zeros(int(jumlahObstacle))                                           #inisial Sudut Obstacle
    obstacleDistance = np.zeros(int(jumlahObstacle))                                        #inisial jarak Obstacle

    # try:
    #     for i  in range (len(obstacleFromKinematik)):
    #         if i%2 == 0:
    #             if i == 0:
    #                 obstacleDistance[0] = obstacleFromKinematik[i]
    #                 obstacleAngle[0] = obstacleFromKinematik[i+1]
    #             else:
    #                 obstacleDistance[i/2] = obstacleFromKinematik[i]
    #                 obstacleAngle[i/2] = obstacleFromKinematik[i+1]
    # except:
    #     pass




''' ROBOT PATH PLANNING BARELANG 63 WITH RRT* SMOOTHING'''

def get_path_length(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d

    return le

def get_target_point(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    partRatio = (le - targetL) / lastPairLen

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio

    return [x, y, ti]

def line_collision_check(first, second, obstacleList):
    # Line Equation

    x1 = first[0]
    y1 = first[1]
    x2 = second[0]
    y2 = second[1]

    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return False

    for (ox, oy, size) in obstacleList:
        d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
        if d <= size:
            return False

    return True  # OK

def path_smoothing(path, max_iter, obstacle_list):
    le = get_path_length(path)

    for i in range(max_iter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        first = get_target_point(path, pickPoints[0])
        second = get_target_point(path, pickPoints[1])

        if first[2] <= 0 or second[2] <= 0:
            continue

        if (second[2] + 1) > len(path):
            continue

        if second[2] == first[2]:
            continue

        # collision check
        if not line_collision_check(first, second, obstacle_list):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = get_path_length(path)

    return path

countPath = 0

def searchPathPlan(masukanStart, masukanGoal, masukanObstacle):
    global activePath
    global pointXCoor, pointYCoor,cx, cy,cyaw, intpathX
    global countPath
    ''' 
    Data yang diperlukan untuk start yaitu [x posisi sekarang , y posisi sekarang]
    dan data goal posisi merupakan titik yang ingin dituju berupa [x Tujuan , y Tujuan]
    sedangkan obstacle list merupakan posisi obstacle dan skala obstacle [x obstacle , y obstacle, skala obstacle]
    
    '''
    rrt = RRT(start=masukanStart, goal=masukanGoal,
    rand_area=[100, 900], obstacle_list=masukanObstacle)
    path = rrt.planning(animation=False)
    # Path smoothing
    maxIter = 1000
    smoothedPath = path_smoothing(path, maxIter, masukanObstacle)
    # print("Hasil Path : ",smoothedPath)

    ##Mereverse dan menyusun hasil dari path
    smoothedPath.reverse()

    
    mulusPath = []
    for i in smoothedPath:
        if i not in mulusPath:
            mulusPath.append(i)


    xPoint , yPoint = zip(*mulusPath)
    
    
    pointXCoordinate = np.around(xPoint)
    pointYCoordinate = np.around(yPoint)
    # print(intpathX)
    
    ##Deklarasi variabel baru sementara
    pointXCoor = []
    pointYCoor = []
    for i in range(len(pointXCoordinate)):
        if pointXCoordinate[i] not in pointXCoor or pointYCoordinate[i] not in pointYCoor :
            pointXCoor.append(int(pointXCoordinate[i]))
            pointYCoor.append(int(pointYCoordinate[i]))

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(pointXCoor, pointYCoor, ds=20)

    
    #Matikan Path Planning Jika sudah dapat Path
    countPath += 1
    if countPath > 5:
        if path is None:
            activePath = True
        else:
            print ("Found Path ....")
            activePath = False
            countPath = 0
        



def main():
    global ObstaclePosition
    global obstacleAngle
    global imuCurentHeading

    #Inisialisasi awalan robot
    robotInitialPosition[0] = 0
    robotInitialPosition[1] = 0
    robotInitialPosition[2] = 0
    
    #Inisial Posisi landmark
    if modeLapangan == 1:
        #Gawang Kiri
        landmarkPosition[0,0] = 1200    # Y Koordinat (Global)
        landmarkPosition[0,1] = 550     # X Koordinat (Global)
        #Gawang kanan
        landmarkPosition[1,0] = 1200    # Y Koordinat (Global)
        landmarkPosition[1,1] = 250     # X Koordinat (Global)
        #Penalti Kiri
        landmarkPosition[2,0] = 1020
        landmarkPosition[2,1] = 650
        #Penalti Kanan
        landmarkPosition[3,0] = 1020
        landmarkPosition[3,1] = 150
        #Lingkaran Kiri
        landmarkPosition[4,0] = 600
        landmarkPosition[4,1] = 530
        #Lingkaran Kanan
        landmarkPosition[5,0] = 600
        landmarkPosition[5,1] = 270
        


    elif modeLapangan == 2:     
        #Gawang Kiri
        landmarkPosition[0,0] = 900
        landmarkPosition[0,1] = 200
        #Gawang kanan
        landmarkPosition[1,0] = 900
        landmarkPosition[1,1] = 400

    speedFromKinematic[0] = 0
    speedFromKinematic[1] = 0
    imuInitHeading = 0
    imuCurentHeading = 0

    #Define Initial Position
    defineInitialPosition = True
    if defineInitialPosition == True:
        estimatePosition[0] = 0
        estimatePosition[1] = 0
        estimatePosition[2] = 0

        #Create Random Particle 90% dari posisi dan 10% random
        _10PercentParticle = int(totalPartikel * 0.1)
        for i in range(0,_10PercentParticle):
            particleInitialPosition[i,0] = uniform(0,panjangLapangan)
            particleInitialPosition[i,1] = uniform(0,lebarLapangan)
            particleInitialPosition[i,2] = uniform(0,360)
            particleGlobalPosition[i,:] = 0
            particleLocalPosition[i,:] = 0
        
        _90PercentParticle = totalPartikel - _10PercentParticle

        for i in range(_10PercentParticle + 1, _90PercentParticle):
            particleInitialPosition[i,0] = normal(estimatePosition[0],50)
            particleInitialPosition[i,1] = normal(estimatePosition[1],50)
            particleInitialPosition[i,2] = normal(estimatePosition[2],10)
            particleGlobalPosition[i,:] = 0
            particleLocalPosition[i,:] = 0

    particleGlobalPosition[:,:] = 0
    particleLocalPosition[:,:] = 0

    rospy.init_node('particle_filter', anonymous=False)
    rospy.Subscriber("sensorfusion", String, dataMasukCb)
    rospy.Subscriber("hua", String, dataRRTCb)
    
    path_pub =rospy.Publisher("path_pub", pathdata, queue_size=1    )
    rate = rospy.Rate(50)
    print("Done Init ROS..")
    

    # lastTime = 0
    # lastTime1 = 0

    while not rospy.is_shutdown():
        path = pathdata()
        # px = Float32MultiArray()
        # py = Float32MultiArray()
        
        # nowTime = time.clock()
        # timer = nowTime - lastTime
        # timer1 = nowTime - lastTime1
        # interval = 5
        # interval1 = 0.2
        
        drawLapangan()
        
        robotGlobalPosition[0] = globalFromKinematic[0]
        robotGlobalPosition[1] = globalFromKinematic[1]

        if headingFromIMU == True:
            imuInitHeading = 0
            imuCurentHeading = int(imuFromKinematic[1])
            robotGlobalPosition[2] = imuCurentHeading - imuInitHeading
        else:
            robotGlobalPosition[2] = globalFromKinematic[2]
        
        #Update Pergerakan Particle menggunakan Real velocity pada robot
        particleLocalPosition[:,0] += speedFromKinematic[0] * deltaTime
        particleLocalPosition[:,1] += speedFromKinematic[1] * deltaTime
        if headingFromIMU == True:
            imuInitHeading = 0
            imuCurentHeading = int(imuFromKinematic[1])
            particleLocalPosition[:,2] = imuCurentHeading - imuInitHeading

        else:
            particleLocalPosition[:,2] = globalFromKinematic[2]
        

        #Kalkulasi posisi dari particle di global koordinat
        updateParticleMovement = True
        if updateParticleMovement == True:

            for i in range(0,totalPartikel):
                if particleLocalPosition[i,2] >= 360:
                    particleLocalPosition[i,2] = particleLocalPosition[i,2] - 360
                if particleLocalPosition[i,2] < 0:
                    particleLocalPosition[i,2] = 360 + particleLocalPosition[i,2]
                #Kalau data imu dianggap tidak ada rotasi
                if headingFromIMU:
                    angle = particleLocalPosition[i,2]
                #kalau pakai yaw rate ditambahkan dengan inisialisasi posisi
                else:
                    angle = particleInitialPosition[i,2] + particleLocalPosition[i,2]

                #Cek Limit heading
                if angle >= 360:
                    angle = angle - 360
                if angle < 0:
                    angle = 360 + angle
                theta = np.radians(angle)
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c,-s), (s, c)))
                npOutMatmul1 = np.matmul(R, particleLocalPosition[i,:2]) 
                particleGlobalPosition[i,0] = npOutMatmul1[0] + particleInitialPosition[i,0]
                particleGlobalPosition[i,1] = npOutMatmul1[1] + particleInitialPosition[i,1]
                particleGlobalPosition[i,2] = angle
            #Jika partikel keluar lapangan generate partikel baru disekitar estimate position terakhir
            if particleGlobalPosition[i,0] < 0 or particleGlobalPosition[i,1] < 0 or particleGlobalPosition[i,0] >= panjangLapangan or particleGlobalPosition[i,1] >= lebarLapangan:
                if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                    particleInitialPosition[i,0] = uniform(0,panjangLapangan)
                    particleInitialPosition[i,1] = uniform(0,lebarLapangan)
                    particleInitialPosition[i,2] = uniform(0,360)
                    particleGlobalPosition[i,:] = 0
                    particleLocalPosition[i,:] = 0
                else:
                    particleInitialPosition[i,0] = normal(estimatePosition[0], 50)
                    particleInitialPosition[i,1] = normal(estimatePosition[1], 50)
                    particleInitialPosition[i,2] = normal(estimatePosition[2], 10)
                    particleGlobalPosition[i,:] = 0
                    particleLocalPosition[i,:] = 0
            

        jarakRobotLandmark[0] = landmarkFromKinematic[0] # Gawang Kiri
        jarakRobotLandmark[1] = landmarkFromKinematic[1] # Gawang Kanan
        jarakRobotLandmark[2] = landmarkFromKinematic[2] # Lingkaran Kiri
        jarakRobotLandmark[3] = landmarkFromKinematic[3] # Lingkaran Kanan
        jarakRobotLandmark[4] = landmarkFromKinematic[4] # Penalty Kiri
        jarakRobotLandmark[5] = landmarkFromKinematic[5] # Penalty Kanan
        
        # if timer > interval and jumlahLandmark >= 2: #Update dan reposisi Robot Setiap 5 Detik
        if jumlahLandmark > 2: 
            # lastTime = nowTime
            # print"ROBOT POSITION UPDATED !!!"
            resample = True
        else:
            resample = False

        #Mengukur jarak particle dan landmark
        for i in range(0,totalPartikel):
            for j in range(0,totalLandmark):
                if jarakRobotLandmark[j] > 0: 
                    jarakPartikelLandmark[i,j] = distance.euclidean(particleGlobalPosition[i,:2], [landmarkPosition[j]])    
                else:
                    jarakPartikelLandmark[i,j] = 0

        #Calculating weight
        #Inisial particle weight dengan 1.00
        particleWeight.fill(1.0)
        for i in range(0,totalPartikel):
            for j in range(0,totalLandmark):
                #stddev = 5
                particleWeight[i] *= scipy.stats.norm.pdf(jarakPartikelLandmark[i,j],jarakRobotLandmark[j],5)
        
        #Normalize weight
        totalWeight = sum(particleWeight)
        for i in range(0,totalPartikel):
            particleWeight[i] = particleWeight[i] / totalWeight

        #Kalkulasi estimasi posisi
        #Jika ada perintah resample
        if resample == True:
            estimatePosition[:] = np.average(particleGlobalPosition,weights=particleWeight,axis=0)
            estimateLocalPosition[:] = 0

        #Jika tidak update maka posisi estimasi sesuai dengan data kinematik robot
        # else:
        #     estimatePosition[0] = robotGlobalPosition[0]
        #     estimatePosition[1] = robotGlobalPosition[1]
        #     estimatePosition[2] = angle

        #Mark 888 jika result infinity atau nan
        if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
            estimatePosition[:] = -888
            ballEstimatePosition[:] = -888
            ObstaclePosition[:,:] = -888

        else:
            ballPosCalc()
            obstaclePosCalc()
            estimatePosition[0] = robotGlobalPosition[0]
            estimatePosition[1] = robotGlobalPosition[1]

        print("------------------------------||------------------------------")
        print("Barelang 63 - Particle Filter & Path Planning")
        print("Robot Global Position",robotGlobalPosition)
        print("Robot Estimate Position",estimatePosition)
        print("Ball Estimate Position",ballEstimatePosition)
        print("Obstacle 1 Estimate Position",ObstaclePosition[0,:])
        print("Obstacle 2 Estimate Position",ObstaclePosition[1,:])
        print("Obstacle 3 Estimate Position",ObstaclePosition[2,:])
        print("Obstacle 4 Estimate Position",ObstaclePosition[3,:])
        print("Goal Coordinate X : {}".format(pointXCoor))
        print("Goal Coordinate Y : {}".format(pointYCoor))
        print("Goal Coordinate adas : {}".format(intpathX))
        print ("Status Path Planning : {}".format(activePath))
        print ("Goal Position : {}".format(goalPos))
        


     

        path.x = pointXCoor
        path.y = pointYCoor
        # px.data = pointXCoor
        # py.data = pointYCoor
        path_pub.publish(path)
      


        '''ACTIVE PATH PLANNING'''
            
        startPos = [estimatePosition[0],estimatePosition[1]]
        obstacleList = [(100,100,25),(100,200,25), (100,300,25)]
        if activePath == True:
            searchPathPlan(startPos,goalPos,obstacleList)
        # x, y = worldCoorToImageCoor(100, 100)
        # cv2.circle(mapImage,(x, y), 25  , (255,255,255), -1) #Biru
        # x1, y1= worldCoorToImageCoor(100, 200)
        # cv2.circle(mapImage,(x1, y1), 25, (255,255,255), -1) #Biru
            
        # x2, y2= worldCoorToImageCoor(100, 300)
        # cv2.circle(mapImage,(x2, y2), 25, (255,255,255), -1) #Biru
        # reset_pub.publish(msgToOdometry)
        # print"MSG : ",msgToMain

        """DRAWING TO GUI"""
        drawParticle = False
        if drawParticle == True:
            for i in range(0,totalPartikel):
                x, y = worldCoorToImageCoor(int(particleGlobalPosition[i,0]), int(particleGlobalPosition[i,1]))
                cv2.circle(mapImage,(x, y), 7, (0,0,255), -1) #Merah

        drawSimRobot = True
        if drawSimRobot == True:
            x, y = worldCoorToImageCoor(int(robotGlobalPosition[0]), int(robotGlobalPosition[1]))
            cv2.circle(mapImage,(x, y), 25, (255,255,255), -1) #Biru
                
                
        drawEstimatePosition = True
        if drawEstimatePosition == True:
            x, y = worldCoorToImageCoor(int(estimatePosition[0]), int(estimatePosition[1]))
            cv2.circle(mapImage,(x, y), 15, (0,0,0), -1)

        drawPathPlanning = True
        if drawPathPlanning == True:
            x, y = worldCoorToImageCoor(int(goalPos[0]),int(goalPos[1]))
            cv2.circle(mapImage,(x,y),15,(200,50,250),-1)
            for z in range(len(pointXCoor)):
                if (z > 0):
                    xStartLast = int(pointXCoor[z-1])
                    yStartLast = int(pointYCoor[z-1])
                    xEndLast = int(pointXCoor[z])
                    yEndlast = int(pointYCoor[z])

                    xStart,yStart = worldCoorToImageCoor(xStartLast,yStartLast)
                    xEnd,yEnd     = worldCoorToImageCoor(xEndLast,yEndlast)

                    cv2.line(mapImage,(xStart,yStart),(xEnd,yEnd),(204,204,0),5) #Draw Garis path
        
        # drawRectangle = True
        # if drawRectangle == True:
        #     cv2.rectangle(mapImage,(384,0),(510,128),(0,255,0),3)
        
        # try:
        #     for i in range (totalLawan):       
        #         x, y = worldCoorToImageCoor(int(ObstaclePosition[i,0]), int(ObstaclePosition[i,1]))
        #         cv2.circle(mapImage,(x, y), 25, (255,0,127), -1)
        # except:
        #     pass

        """DRAWING TO GUI"""

        # textLine = "R{} Global Position : ({}, {}, {})".format(robotID, int(robotGlobalPosition[1]), int(robotGlobalPosition[0]), int(robotGlobalPosition[2]))
        # cv2.putText(mapImage, textLine, (100,930), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,127), 2, cv2.LINE_AA)

        # textLine = "R{} Estimate Position : ({}, {}, {})".format(robotID, int(estimatePosition[1]), int(estimatePosition[0]), int(estimatePosition[2]))
        # cv2.putText(mapImage, textLine, (100,950), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,127), 2, cv2.LINE_AA)

        # textLine = "Ball Estimate Position : ({}, {})".format( int(ballEstimatePosition[1]), int(ballEstimatePosition[0]))
        # cv2.putText(mapImage, textLine, (100,970), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,127), 2, cv2.LINE_AA)

        # textLine = "R{} X Velopcity : {}".format(robotID, float(speedFromKinematic[1]))
        # cv2.putText(mapImage, textLine, (540,930), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,127), 2, cv2.LINE_AA)

        # textLine = "R{} Y Velopcity : {}".format(robotID, float(speedFromKinematic[0]))
        # cv2.putText(mapImage, textLine, (540,950), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,127), 2, cv2.LINE_AA)
        # ### DRAW hasil particle, Estimasi, dan posisi Robot ###


        

        
        #Resample
        if resample == True:
            indexHighestWeight = np.argmax(particleWeight)
            xHighest = particleGlobalPosition[indexHighestWeight,0]
            yHighest = particleGlobalPosition[indexHighestWeight,1]
            thetaHighest = particleGlobalPosition[indexHighestWeight,2]

            _10PercentParticle = int(totalPartikel * 0.1)

            for i in range(0, _10PercentParticle):
                particleInitialPosition[i,0] = uniform(0,panjangLapangan)
                particleInitialPosition[i,1] = uniform(0,lebarLapangan)
                particleInitialPosition[i,2] = uniform(0,360)
                particleGlobalPosition[i,:] = 0
                particleLocalPosition[i,:] = 0

            _90PercentParticle = totalPartikel - _10PercentParticle

            for i in range(_10PercentParticle + 1, _90PercentParticle):
                if math.isnan(estimatePosition[0]) or math.isnan(estimatePosition[1]) or math.isnan(estimatePosition[2]) or math.isinf(estimatePosition[0]) or math.isinf(estimatePosition[1]) or math.isinf(estimatePosition[2]):
                    particleInitialPosition[i,0] = uniform(0, panjangLapangan)
                    particleInitialPosition[i,1] = uniform(0, lebarLapangan)
                    particleInitialPosition[i,2] = uniform(0, 360)
                    particleGlobalPosition[i,:] = 0
                    particleLocalPosition[i,:] = 0

                else:
                    particleInitialPosition[i,0] = normal(estimatePosition[0], 50)
                    particleInitialPosition[i,1] = normal(estimatePosition[1], 50)
                    particleInitialPosition[i,2] = normal(estimatePosition[2], 10)
                    particleGlobalPosition[i,:] = 0
                    particleLocalPosition[i,:] = 0
        
        showLine = True
        if showLine == True:
            for i in range(0,totalLandmark):
                if jarakRobotLandmark[i] > 0:
                    # print"Test Coba Draw"
                    lineXStart = int(estimatePosition[0])
                    lineYStart = int(estimatePosition[1])
                    lineXEnd = int(landmarkPosition[i-0,0])
                    lineYEnd = int(landmarkPosition[i-0,1])

                    lineXDrawS,lineYDrawS = worldCoorToImageCoor(lineXStart,lineYStart)
                    lineXDrawE,lineYDrawE = worldCoorToImageCoor(lineXEnd,lineYEnd)
                    cv2.line(mapImage,(lineXDrawS,lineYDrawS),(lineXDrawE,lineYDrawE),(255,0,0),3) #Draw Garis path
                    





        showGUI = True
        if showGUI:
            smallPic = cv2.resize(mapImage, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_CUBIC)
            cv2.imshow("Barelang 63 - Localization", smallPic)
            # print"SHOWING"
        if showGUI:
                cv2.waitKey(1)
        
        # rate.sleep()
        
        


if __name__ == '__main__':
    try:
        print ("Running Barelang 63 - Robot Localization")
        main()
    except :
        print ("Lokalisasi Selesai..")
        pass
