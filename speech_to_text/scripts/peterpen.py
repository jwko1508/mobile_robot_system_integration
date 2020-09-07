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



import speech_recognition as sr
import time
import os
import subprocess

from omron_msgs.srv import SetCmd
import rospy

from gtts import gTTS
from io import BytesIO

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

import sys

import sys, select, termios, tty

reload(sys)

sys.setdefaultencoding('utf-8')

cmdlists = [
            ["안녕","무슨일인가?",9],
            ["종료","퇴근이다",0],
            ["멈춰","멈춘다",8],
            ["가라","이동한다",2]]

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def tts_srv_client_cb(req):
    
    rospy.wait_for_service('tts_data')

    try:
        tts_data = rospy.ServiceProxy('tts_data',SetCmd)
        resp1 = tts_data(req)
        return resp1.success

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def talker():

    # get audio from the microphone                                                                       
    r = sr.Recognizer()    

    key = getKey()
    blank = 1
    num = 0

    rospy.init_node('speech_to_text', anonymous=True)

    # srv = rospy.Service('tts_data', tts , tts_srv_cb)
    # print("Ready to tts_srv.")
    r.dynamic_energy_threshold = False # asd


    # engine = pyttsx3.init() 
    # voices = engine.getProperty('voices')
    # engine.setProperty('voices', voices[1].id) # 여성

    while not rospy.is_shutdown():

        YouSaid = ""
        with sr.Microphone() as source:                                                                       
            
            #print(type(YouSaid))
            try:

                if num == 1:
                    
                    blank = 0

                    print("speak command")
                    audio = r.listen(source, timeout=1,phrase_time_limit=5)
                    YouSaid = r.recognize_google(audio, language='ko-kr')

                    for cmd in range(len(cmdlists)) :

                        if YouSaid == cmdlists[cmd][0]:

                            print("나 : " + YouSaid)
                            print("옴론 : " + cmdlists[cmd][1])
                            tts_cmd = gTTS(text=cmdlists[cmd][1], lang='ko')
                            tts_cmd.save("%s.mp3" % cmdlists[cmd][1])
                            os.system("mpg123 -q %s.mp3" % cmdlists[cmd][1])
                            os.remove("%s.mp3" % cmdlists[cmd][1])
                            cmdnum = cmdlists[cmd][2]
                            s = tts_srv_client_cb(cmdnum)
                            YouSaid = ""
                            num = 0

                if num == 0 :

                    blank = 1

                    # print("Please speak")  
                    audio = r.listen(source, timeout=1,phrase_time_limit=3)
                    YouSaid = r.recognize_google(audio, language='ko-kr')
                    #YouSaid = r.recognize_google(audio)
                    print("나 : " + YouSaid)


                if YouSaid.find("피터팬") != -1 :

                    blank = 1

                    num = 1
                    print("옴론 : " + "말씀해주세요. 주인님.")
                    # engine.setProperty('voice')
                    # engine.say("말씀하라. 주인.") 
                    # engine.runAndWait() 
                    tts = gTTS(text="말씀해주세요. 주인님.", lang='ko')
                    tts.save("호출.mp3")
                    os.system("mpg123 -q 호출.mp3")

                    # opener ="open" if sys.platform == "darwin" else "xdg-open"
                    # subprocess.call([opener, "호출.mp3"])
                    # print("Speaking.....")
                    # time.sleep(1)
                    # os.remove("호출.mp3")
                    # time.sleep(4)

                # if key == 't':
                #   num = 1
                # else :
                #   num = 0
                #   print("옴론 : " + "명령하고싶으면 암호를 말하라.")

                



                    # print("나 : " + YouSaid2)

                    # blank = YouSaid2

                    # YouSaid2 = ""

                    # num = 0

                #YouSaid = r.recognize_google(audio)
                #print(type(YouSaid))

            except sr.WaitTimeoutError:
                # print("listening timed out while waiting for phrase to start")
                # YouSaid = "listening timed out while waiting for phrase to start"
                continue
            except sr.UnknownValueError:

                if blank != 1:
                    print("옴론 : " + "명령안할거면 부르지마라.")
                    tts = gTTS(text="명령안할거면 부르지마라.", lang='ko')
                    tts.save("시간초과.mp3")
                    os.system("mpg123 -q 시간초과.mp3")
                    before = time.time()

                    num = 0

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
        rospy.spin()

if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)

        talker()
    except rospy.ROSInterruptException:
        pass
