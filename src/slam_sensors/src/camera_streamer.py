#!/usr/bin/env python
import threading
import io
import time
import picamera
import rospy
import sensor_msgs.msg as sensor_msgs

IMG_TOPIC = '/rpi_camera/image/compressed'
PUBLISH_RATE = 15
CAPTURE_RATE = 40

def main():
	rospy.init_node('camera_streamer')
	
	with picamera.PiCamera() as camera:
		#Initialize the camera
		stream = io.BytesIO()
		#camera.start_preview()
		camera.resolution = (640,480)
		camera.framerate = CAPTURE_RATE
		
		with PublisherThread(IMG_TOPIC) as publisher:
			try:
				for i, foo in enumerate(camera.capture_continuous(stream, format='jpeg', quality=10, use_video_port=True)):
					#print('Captured...')
					#print(i)
					stream.truncate()
					stream.seek(0)
					
					#Construct and publish the image message
					message = sensor_msgs.CompressedImage()
					message.format = 'jpeg'
					message.data = stream.getvalue()
					publisher.set_message(message)
					
					if rospy.is_shutdown():
						break
			finally:
				camera.stop_preview()

class PublisherThread():
	def __init__(self, topic_name, max_rate=PUBLISH_RATE):
		print
		#Create a publisher
		self.pub = rospy.Publisher(topic_name, sensor_msgs.CompressedImage)
		
		#Start a new thread to run the publisher
		self.current_msg = None
		self._stop = False
		self._timer = rospy.Rate(max_rate)
		self._thread = threading.Thread(target=self.run)
		self._thread.daemon = True
		self._thread.start()
	
	def run(self):
		while not self._stop:
			if self.current_msg:
				self.pub.publish(self.current_msg)
				#print('Published...')
			else:
				#print('Not Published...')
				pass
			self._timer.sleep()
	
	def set_message(self, message):
		self.current_msg = message
		
	def stop(self):
		self._stop = True
		self._thread.join()
	
	#Add methods to support use in a 'with...' block
	def __enter__(self):
		return self
	
	def __exit__(self, *args):
		self.stop()
		

if __name__ == '__main__':
	main()


		
