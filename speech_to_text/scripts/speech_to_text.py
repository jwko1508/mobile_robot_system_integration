#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

from std_msgs.msg import String

def talker():

    # get audio from the microphone                                                                       
    r = sr.Recognizer()        

    pub = rospy.Publisher('dictation_text', String, queue_size=10)
    rospy.init_node('speech_to_text', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    r.dynamic_energy_threshold = False # asd
    while not rospy.is_shutdown():

        YouSaid = ""
        with sr.Microphone() as source:                                                                       
            print("Speak:")  
            try:
                audio = r.listen(source, timeout=1)
                # YouSaid = r.recognize_google(audio, language='ko-kr')
                YouSaid = r.recognize_google(audio)
                print("You said " + YouSaid)
                print(type(YouSaid))
            except sr.WaitTimeoutError:
                print("listening timed out while waiting for phrase to start")
                YouSaid = "listening timed out while waiting for phrase to start"
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))

        if YouSaid == "":
            YouSaid = "Could not understand audio"

        # hello_str = "hello world %s" % YouSaid
        #if YouSaid == "coffee":
        YouSaid = "Please give me a cup of coffee."
        rospy.loginfo(YouSaid)
        pub.publish(YouSaid)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
