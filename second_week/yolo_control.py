import rclpy as rp 
from rclpy.node import Node 
from jetson_yolo_msg.msg import YoloDetection, YoloDetectionArray
from geometry_msgs.msg import Twist 


class PoseTopicSubscriber(Node):
	def __init__(self):
		super().__init__('yolo_subscriber')
		
		## yolo subscriber##
		self.subscription = self.create_subscription(YoloDetectionArray, '/Jetson_Yolo_Detection', self.callback, 10)
		
		self.num_of_object = 0
		self.object_center_x = 0
		self.object_center_y = 0
		self.privious_score = 0 
		self.bounding_box_total_x = 0
		self.bounding_box_total_y = 0
		
		##cmd_vel_publisher##
		self.msg = Twist()
		self.trigger = 0
		self.msg.linear.x = 0.0
		self.msg.linear.y = 0.0
		self.msg.angular.z = 0.0
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.timer_period = 0.1 
		self.timer = self.create_timer(self.timer_period, self.timer_callback)
		
	def timer_callback(self):
		self.publisher.publish(self.msg)
		
		
	def callback(self, msg):

		for detection in msg.detections:
			if (detection.class_name == "bottle"):
				self.num_of_object += 1
				if (self.privious_score < detection.confidence):
					self.privious_score = detection.confidence
					self.object_center_x = (detection.x2 + detection.x1) / 2
					self.object_center_y = (detection.y2 + detection.y1) / 2
					self.bounding_box_total_x = detection.x2 - detection.x1
					self.bounding_box_total_y = detection.y2 - detection.y1 
		
		print("총:", self.num_of_object)
		print("score:", self.privious_score)
		print("x:", self.object_center_x)
		print("y:", self.object_center_y)
		print("x_t:", self.bounding_box_total_x)
		print("y_t:", self.bounding_box_total_y)
		
		if(self.num_of_object >= 1):
			if(self.object_center_x > 300 and self.object_center_x < 340):
				self.msg.linear.y = 0.0
		##if(self.bounding_box_total_x > 600 and self.bounding_box_total_y > 470):
			if(self.bounding_box_total_y > 450):
				self.msg.linear.x = 0.0
			if(self.bounding_box_total_y < 450):
				self.msg.linear.x = 0.12
			if(self.object_center_x < 300):
				self.msg.linear.y = 0.12
			if(self.object_center_x > 340):
				self.msg.linear.y = -0.12
				
		if(self.num_of_object == 0):
			self.msg.linear.x = 0.0
			self.msg.linear.y = 0.0
			
					
		self.num_of_object = 0
		self.object_center_x = 0
		self.object_center_y = 0
		self.privious_score = 0
		self.bounding_box_total_x = 0
		self.bounding_box_total_y = 0
def main():
	rp.init()

	study_sub = PoseTopicSubscriber()
	rp.spin(study_sub)

	study_sub.destroy_node()
	rp.shutdown()

if __name__ == '__main__':
	main()


