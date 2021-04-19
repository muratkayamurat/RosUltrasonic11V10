#!/usr/bin/env python

import time
import array
from math import sin, cos, atan2, pi, sqrt
from hiddriver import hiddriver
from sensor_msgs.msg import Range
import numpy as np
import rospy

TEST1 = 0
TEST2 = 1
MEASURING = 2

class manueldriver():
    def __init__(self):
    
        rospy.init_node('manueldriver')
        
        self.manhid = hiddriver("SeomakManualControl")
        
        if not (self.manhid.productFound):
            print("%s module not found"%("SeomakManualControl"))
        else:
            print("manuel driver started")
            
        self.distances = np.zeros(4)
        self.distanceses1= np.zeros(4)
        self.distances_id = np.zeros(4)
        self.adc = np.zeros(2)
        self.manuelcontrol = 0
        self.emergency = 0
        self.state = ""
        self.reading = True
        self.dataready = False
        self.stamp = time.time()  
        
        self.sonic_range_pub1 = rospy.Publisher('/manuel/ultrasonic1', Range, queue_size = 10)
        self.sonic_range_pub2 = rospy.Publisher('/manuel/ultrasonic2', Range, queue_size = 10)
        self.sonic_range_pub3 = rospy.Publisher('/manuel/ultrasonic3', Range, queue_size = 10)
        self.sonic_range_pub4 = rospy.Publisher('/manuel/ultrasonic4', Range, queue_size = 10)   
        
        

    def write_data(self, teststr):
        if(teststr == "T1"):
            self.manhid.write_device("T1")
            
        elif(teststr == "T2"):
            self.manhid.write_device("T2")
        else:
            self.manhid.write_device("T3")

    def error_control(self):
        pass

    def shutdown(self):
        pass

    def diagnostic_data(self, diagstr):
        diagcmd = diagstr.upper()[0]
        self.manhid.write_device(diagcmd)

    def read_data(self):
    # def read_data(self):
        # print("manuel driver reading started")
        # while(self.reading):
        #self.dataready=False
        dataf = np.zeros(12)
        locationStr = ""
        #self.distances = np.zeros(4)
        #self.distances_id = np.zeros(4)
        locationStr = self.manhid.read_device(64)
        if(locationStr == ""):
            self.dataready = False
            pass
            #self.errorCode = 16
        else:
            datas = locationStr.split(',')
            dataf = map(int, datas[0:12])
            self.distances[0] = dataf[0]
            self.distances[1] = dataf[2]
            self.distances[2] = dataf[4]
            self.distances[3] = dataf[6]
            #print(self.distances)
            self.distances_id[0] = dataf[1]
            self.distances_id[1] = dataf[3]
            self.distances_id[2] = dataf[5]
            self.distances_id[3] = dataf[7]
            self.adc = dataf[8:9]
            self.manuelcontrol = dataf[10]
            self.emergency = dataf[11]
            self.dataready = True
            self.stamp = time.time()        
        
        if(self.dataready):
            self.dataready = False
            distancesready = False
            ultraSonicRange1 = Range()
            ultraSonicRange2 = Range()
            ultraSonicRange3 = Range()
            ultraSonicRange4 = Range()
            self.distanceses = np.zeros(4)
            
            # if(self.desiredspeedleft<0 or self.desiredspeedright<0):
                # self.man.write_data("T3")
            # else:
            self.write_data("T1")
            # counter = 0
            # for i in self.man.distances_id:
                # self.distanceses[int(i)] = self.man.distances[counter]
                # counter = counter+1
                # distancesready = True
            distancesready = True    
            self.distanceses = self.distances
            #print(2.4*self.distanceses/9212.0)
            if(distancesready):
                ultraSonicRange1.header.stamp = rospy.Time.now()
                ultraSonicRange1.header.frame_id = "ultrasonic1"
                ultraSonicRange1.radiation_type = 0
                ultraSonicRange1.field_of_view = 0.52
                ultraSonicRange1.min_range = 0.2
                ultraSonicRange1.max_range = 2.4
                self.distanceses1[0] = 24*self.distanceses[0]/9212.0
                ultraSonicRange1.range = self.distanceses1[0]
                ultraSonicRange2.header.stamp = rospy.Time.now()
                ultraSonicRange2.header.frame_id = "ultrasonic2"
                ultraSonicRange2.radiation_type = 0
                ultraSonicRange2.field_of_view = 0.52
                ultraSonicRange2.min_range = 0.2
                ultraSonicRange2.max_range = 2.4
                self.distanceses1[1] = 24*self.distanceses[1]/9212.0
                ultraSonicRange2.range = self.distanceses1[1]
                ultraSonicRange3.header.stamp = rospy.Time.now()
                ultraSonicRange3.header.frame_id = "ultrasonic3"
                ultraSonicRange3.radiation_type = 0
                ultraSonicRange3.field_of_view = 0.52
                ultraSonicRange3.min_range = 0.2
                ultraSonicRange3.max_range = 2.4
                self.distanceses1[2] = 24*self.distanceses[2]/9212.0
                ultraSonicRange3.range = self.distanceses1[2]
                ultraSonicRange4.header.stamp = rospy.Time.now()
                ultraSonicRange4.header.frame_id = "ultrasonic4"
                ultraSonicRange4.radiation_type = 0
                ultraSonicRange4.field_of_view = 0.52
                ultraSonicRange4.min_range = 0.2
                ultraSonicRange4.max_range = 2.4
                self.distanceses1[3] = 24*self.distanceses[3]/9212.0
                ultraSonicRange4.range = self.distanceses1[3]
                
                
                if(self.distanceses1[0] > 2.4 ): 
                    self.distanceses1[0] = 2.4
                    ultraSonicRange1.range = self.distanceses1[0]
                if(self.distanceses1[1] > 2.4 ): 
                    self.distanceses1[1] = 2.4
                    ultraSonicRange2.range = self.distanceses1[1]
                if(self.distanceses1[2] > 2.4 ): 
                    self.distanceses1[2] = 2.4
                    ultraSonicRange3.range = self.distanceses1[2]
                if(self.distanceses1[3] > 2.4 ): 
                    self.distanceses1[3] = 2.4
                    ultraSonicRange4.range = self.distanceses1[3]
                #print(self.distanceses1)
                self.sonic_range_pub1.publish(ultraSonicRange1)
                self.sonic_range_pub2.publish(ultraSonicRange2)
                self.sonic_range_pub3.publish(ultraSonicRange3)
                self.sonic_range_pub4.publish(ultraSonicRange4)
                #print(self.distanceses1)

if __name__ == '__main__':
    mcd = manueldriver()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mcd.read_data()
        #print(mcd.ultrasonicDistances)
        #mcd.error_control()
        rate.sleep()
    mcd.shutdown() 
