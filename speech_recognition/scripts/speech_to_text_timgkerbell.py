#!/usr/bin/env python
#-*- coding:utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic



import rospy
import speech_recognition as sr
import time

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

import sys

import sys, select, termios, tty

reload(sys)

sys.setdefaultencoding('utf-8')

def getKey():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key



def talker():

    # get audio from the microphone                                                                       
    r = sr.Recognizer()    

    pub = rospy.Publisher('dictation_text', String, queue_size=10)
    rospy.init_node('speech_to_text', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    r.dynamic_energy_threshold = False # asd
    while not rospy.is_shutdown():

    	key = getKey()
    	blank = 1

        YouSaid = ""
        with sr.Microphone() as source:                                                                       
            
        	#print(type(YouSaid))
            try:

            	print("Please speak")  
            	audio1 = r.listen(source, timeout=1,phrase_time_limit=3)
            	YouSaid1 = r.recognize_google(audio1, language='ko-kr')
            	#YouSaid = r.recognize_google(audio)
            	print("나 : " + YouSaid1)

            	if YouSaid1.find("팅커벨") != -1 :

            		num = 1

            		print("옴론 : " + "말씀하라. 주인.")
            		before = time.time()


            	# if key == 't':
            	# 	num = 1
            	else :
            		num = 0
            		print("옴론 : " + "명령하고싶으면 암호를 말하라.")

            	if num == 1:
            		
            		blank = 0

            		audio2 = r.listen(source, timeout=1,phrase_time_limit=5)
            		YouSaid2 = r.recognize_google(audio2, language='ko-kr')

            		print("나 : " + YouSaid2)
            		print(audio2.sample_segment)

            		blank = YouSaid2

            		YouSaid2 = ""

            		num = 0

                #YouSaid = r.recognize_google(audio)
                #print(type(YouSaid))

            except sr.WaitTimeoutError:
                print("listening timed out while waiting for phrase to start")
                # YouSaid = "listening timed out while waiting for phrase to start"
                continue
            except sr.UnknownValueError:

            	if blank != 1:
                	print("옴론 : " + "명령안할거면 부르지마라.")

                	blank = 1

                continue
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))

        if YouSaid == "":
            YouSaid = "Could not understand audio"

        # hello_str = "hello world %s" % YouSaid
        #if YouSaid == "coffee":
        # YouSaid = "Please give me a cup of coffee."
        # rospy.loginfo(YouSaid)
        pub.publish(YouSaid)
        rate.sleep()

if __name__ == '__main__':
    try:
    	settings = termios.tcgetattr(sys.stdin)

        talker()
    except rospy.ROSInterruptException:
        pass
