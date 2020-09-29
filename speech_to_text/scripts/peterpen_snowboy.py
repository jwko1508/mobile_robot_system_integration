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

from command_srvs.srv import SetCommand

import rospy

from gtts import gTTS

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

import sys

import sys, select, termios, tty

import time
import threading
import signal
from triggerword import snowboy as sb
from triggerword import snowboydecoder

import playsound

reload(sys)

sys.setdefaultencoding('utf-8')

# num_list = [
# 	'gotogoal1',


# ]

cmdlists = [
			["안녕","무슨일인가요?",1],
			["종료","감사합니다",2],
			["멈춰","멈추겠습니다",3],
			["출발","이동하겠습니다",4]]


def restartSnowBoy_m():    
	snow_multy.start()

def restart_m(sec):
	print('again! after %d sec ' % sec)
	threading.Timer(sec, restartSnowBoy_m).start()

def callback_m():
	#stop snowboy
	# it means mic off    
	snow_multy.stop()    
	# print('Hi, I am here!')
	# restart after 5 sec
	restart(2)    
	# do not use other function here.
	# because snowboy has not yet stop.

def restartSnowBoy():    
	snow_peterpen.start()

def restart(sec):
	# print('again! after %d sec ' % sec)
	threading.Timer(sec, restartSnowBoy).start()

def callback():
	#stop snowboy
	# it means mic off    
	snow_peterpen.stop()    
	# print('Hi, I am here!')
	# restart after 5 sec
	restart(2)    
	# do not use other function here.
	# because snowboy has not yet stop.

def signal_handler(signal, frame):
    global interrupted
    interrupted = True


def tts_srv_client_cb(req):
	
	rospy.wait_for_service('tts_data')

	try:
		tts_data = rospy.ServiceProxy('tts_data',SetCommand)
		resp1 = tts_data(req)

		if resp1.success == True :
			print("옴론 : " + "실행 완료하였습니다.")
			# tts = gTTS(text="실행 완료하였습니다.", lang='ko')
			# tts.save("실행완료하였습니다.mp3")
			os.system("mpg123 -q 실행완료하였습니다.mp3")
			# playsound.playsound('/home/jwko/gra_ws/src/scripts/triggerword/mp3/실행완료하였습니다.mp3', True)
			# os.remove("실행완료하였습니다.mp3")

		else:
			print("옴론 : " + "실패했습니다.")
			# tts = gTTS(text="실패했습니다.", lang='ko')
			# tts.save("실패했습니다.mp3")
			os.system("mpg123 -q 실패했습니다.mp3")
			# playsound.playsound('/home/jwko/gra_ws/rc/speech_to_text/scripts/triggerword/mp3/실패했습니다.mp3', True)
			# os.remove("실패했습니다.mp3")

		return resp1.success

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

def myhook():
  print "shutdown time!"


# snow = sb.snowboy(callbackfunc = callback)
snow_peterpen = sb.snowboy_peterpen(callbackfunc = callback)
snow_multy = sb.snowboy_multy(callbackfunc = callback_m)


def talker():

	# get audio from the microphone                                                                       
	r = sr.Recognizer()    

	blank = 1
	num = 0
	trigger = 0

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
			
			try:
				# print("hi")
				if num == 1:
					
					blank = 0

					print("speak command")
					# audio = r.listen(source, timeout=3,phrase_time_limit=3)
					# YouSaid = r.recognize_google(audio, language='en-US')

					# for cmd in range(len(cmdlists)) :

					# 	if YouSaid == cmdlists[cmd][0]:

					# 		print("나 : " + YouSaid)
					# 		print("옴론 : " + cmdlists[cmd][1])
					# 		tts_cmd = gTTS(text=cmdlists[cmd][1], lang='ko')
					# 		tts_cmd.save("%s.mp3" % cmdlists[cmd][1])
					# 		os.system("mpg123 -q %s.mp3" % cmdlists[cmd][1])
					# 		os.remove("%s.mp3" % cmdlists[cmd][1])
					# 		cmdnum = cmdlists[cmd][2]
					# 		s = tts_srv_client_cb(cmdnum)
					# 		YouSaid = ""
					# 		num = 0

					snow_multy.start()

					count = 0
					listen = 0

					while True:         
						time.sleep(1)

						# print("main : %d" %snow_multy.ans_num)

						count = count + 1

						if snow_multy.ans_num > 0:

							for cmd in range(len(cmdlists)):
								if snow_multy.ans_num == cmdlists[cmd][2]:
									print("detection : %s %d" %(cmdlists[cmd][0],snow_multy.ans_num))
									print("옴론 : " + cmdlists[cmd][1])
									# tts_cmd = gTTS(text=cmdlists[cmd][1], lang='ko')
									# tts_cmd.save("%s.mp3" % cmdlists[cmd][1])
									os.system("mpg123 -q %s.mp3" % cmdlists[cmd][1])
									# os.remove("%s.mp3" % cmdlists[cmd][1])
									cmdnum = cmdlists[cmd][2]
									s = tts_srv_client_cb(cmdnum)

								if snow_multy.ans_num == 2:
									rospy.on_shutdown(myhook)

							listen = 1

						if listen == 1:
							snow_multy.stop()
							num = 0
							break

						if count == 11 :
							num = 0
							snow_multy.stop()
							print("옴론 : " + "무슨말인지 모르겠어요")
							# tts = gTTS(text="무슨말인지 모르겠어요.", lang='ko')
							# tts.save("시간초과.mp3")
							os.system("mpg123 -q 시간초과.mp3")
							# os.remove("시간초과.mp3")
							break

				if num == 0 :

					blank = 1

					print("Please speak")  
					# audio = r.listen(source, timeout=1,phrase_time_limit=2)
					# YouSaid = r.recognize_sphinx(audio, language='en-US')
					# start snowboy
					snow_peterpen.start()

					while True:         
						time.sleep(1)
						if snow_peterpen.isListening() == False:
							trigger = 1;

							snow_peterpen.stop()
							break

					# model = 'snow_peterpen/resources/models/peterpen.pmdl'

					# signal.signal(signal.SIGINT, signal_handler)

					# detector = snowboydecoder.HotwordDetector(model, sensitivity=0.5)
					# print('Listening... Press Ctrl+C to exit')

					# detector.start(detected_callback=snowboydecoder.play_audio_file,
					#                interrupt_check=interrupt_callback,
					#                sleep_time=0.03)

					# detector.terminate()


					# print('done')
					#YouSaid = r.recognize_google(audio)
					# print("나 : " + YouSaid)


				if trigger == 1 :

					blank = 1

					num = 1
					print("옴론 : " + "듣고있어요.")
					# engine.setProperty('voice')
					# engine.say("말씀하라. 주인.") 
					# engine.runAndWait() 
					# tts = gTTS(text="듣고있어요.", lang='ko')
					# tts.save("호출.mp3")
					os.system("mpg123 -q 호출.mp3")
					# os.remove("호출.mp3")

					triger = 0

			except sr.WaitTimeoutError:
				# print("werror")
				num = 0

				if blank != 1:
					print("옴론 : " + "무슨말인지 모르겠어요")
					# tts = gTTS(text="무슨말인지 모르겠어요.", lang='ko')
					# tts.save("시간초과.mp3")
					os.system("mpg123 -q 시간초과.mp3")
					# os.remove("시간초과.mp3")
					before = time.time()

					blank = 1
				# print("listening timed out while waiting for phrase to start")
				# YouSaid = "listening timed out while waiting for phrase to start"
				continue
			except sr.UnknownValueError:
				# print("uerror")
				num = 0

				if blank != 1:
					print("옴론 : " + "무슨말인지 모르겠어요.")
					# tts = gTTS(text="무슨말인지 모르겠어요.", lang='ko')
					# tts.save("시간초과.mp3")
					os.system("mpg123 -q 시간초과.mp3")
					# os.remove("시간초과.mp3")
					before = time.time()

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

if __name__ == '__main__':
	try:
		settings = termios.tcgetattr(sys.stdin)

		talker()
	except rospy.ROSInterruptException:
		pass
