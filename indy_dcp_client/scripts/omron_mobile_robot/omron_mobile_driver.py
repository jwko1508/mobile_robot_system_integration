#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import socket
import sys
import time

class Omron_mobile_robot:
    def __init__(self, host = "192.168.1.2"):
        """
        this constructure 
        """
        self.MAXBUFFER = 256
        self.PORT = 7171
        self.HOST = host
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print("[INFO] Connect to omron.")
        self.client_socket.connect((self.HOST, self.PORT))
        message_to_send = "admin\n"
    
        for _ in range(2):
            self.client_socket.send(message_to_send.encode())
            time.sleep(0.5)

        initialGuidance = ""
        while (initialGuidance.find("End of") == -1):
            initialGuidance = self.client_socket.recv(self.MAXBUFFER)
            initialGuidance = initialGuidance.decode()

    def __del__(self):
        print("[INFO] Disonnect to omron.")
        self.client_socket.close()

    def gotogoal(self, goalNumber = int()):
        """
        this function was wrote to move specific goal which user want to go.
        """
        goalNumber = int(goalNumber)

        print("[INFO] goto goal{}".format(goalNumber))
        if goalNumber < 0:
            print("[ERROR] please input the positive integer number.")
            return

        message_to_send = "goto goal" + str(goalNumber) + "\r\n"
        self.client_socket.send(message_to_send.encode())

        for_checking_arrive = ""
        arrival_message = "Arrived at Goal" + str(goalNumber)
        while (for_checking_arrive.find(arrival_message) == -1):
            for_checking_arrive = self.client_socket.recv(self.MAXBUFFER)
            for_checking_arrive = for_checking_arrive.decode()
            time.sleep(0.05)
    
    def get_location(self):
        """
        We can get omron mobile robot's location by using "onelinestatus" command.
        received message example :
        Status: Parking StateOfCharge: 95.3 Location: 5754 -3092 89 Temperature: 28
        So, I found the sub string which represents location by using string().find() function
        I found location index and temperature index. And 2
        """
        message_to_send = "onelinestatus\r\n"
        self.client_socket.send(message_to_send.encode())
        received_message = ""
        location_index = -1

        while True:
            received_message = self.client_socket.recv(self.MAXBUFFER)
            received_message = received_message.decode()
            location_index = received_message.find("Location")
            if(location_index >= 0):
                break

        temperature_index = received_message.find("Temperature")

        location_substr = received_message[location_index:temperature_index] # slicing string
        splited_location_substr = location_substr.split()
        x = float(splited_location_substr[1])
        y = float(splited_location_substr[2])
        heading = float(splited_location_substr[3])

        return x, y, heading

    def say(self, what = "안녕하세요."):
        message_to_send = '''say "''' + what + '''"\r\n'''
        self.client_socket.send(message_to_send.encode())

    def stop(self):
        message_to_send = "stop\r\n"
        self.client_socket.send(message_to_send.encode())

    # def 

