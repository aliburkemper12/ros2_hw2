import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Walk(Node):
	def __init__(self):
		super().__init__('walk')
		self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
		self.timer_period = 0.5
		self.time = 0
		self.leftwhisker = 0
		self.rightwhisker = 0
		self.whisker = 0
		self.linear_speed = 0.0
		self.move_cmd = Twist()
		self.timer = self.create_timer(self.timer_period, self.timer_callback)
		self.subscription = self.create_subscription(
			LaserScan,
			'/base_scan',
			self.sensor_callback,
			10
		)
		
	def sensor_callback(self, msg):
		left_sensor = int((len(msg.ranges) / 6) * 5)
		middle_sensor = int(len(msg.ranges) / 2)
		right_sensor = int(len(msg.ranges) / 6)
		front = msg.ranges[middle_sensor]
		left = msg.ranges[left_sensor]
		right = msg.ranges[right_sensor]
		print("Sensor: " + str(front))
		print("Left Sensor: " + str(left))
		print("Right Sensor:" + str(right))
		self.leftwhisker = left
		self.whisker = front
		self.rightwhisker = right
		
		
	def forward(self):
		self.move_cmd.linear.x = self.linear_speed
		
	def timer_callback(self):
		self.move_cmd.linear.x = 0.0
		self.move_cmd.angular.z = 0.0

		if self.whisker < 0.5:
			# Too close — backup and turn
			self.move_cmd.linear.x = -0.2
			self.move_cmd.angular.z = 2.0
		# elif self.leftwhisker < 1.0:
		# 	# Obstacle on left — turn right
		# 	self.move_cmd.linear.x = 0.1
		# 	self.move_cmd.angular.z = -2.0
		# elif self.rightwhisker < 1.0:
		# 	# Obstacle on right — turn left
		# 	self.move_cmd.linear.x = 0.1
		# 	self.move_cmd.angular.z = 2.0
		# elif self.whisker < 2.0:
		# 	# Turn away from closer side
		# 	if self.leftwhisker > self.rightwhisker:
		# 		self.move_cmd.angular.z = 1.5
		# 	elif self.rightwhisker > self.leftwhisker:
		# 		self.move_cmd.angular.z = -1.5
		# 	self.move_cmd.linear.x = 0.3
		# else:
		# 	# All clear — go straight
		# 	self.move_cmd.linear.x = 0.1
		# 	self.move_cmd.angular.z = 0.0

		self.cmd_pub.publish(self.move_cmd)
		
def main(args=None):
	rclpy.init(args=args)
	turtle_controller = Walk()
	rclpy.spin(turtle_controller)
	turtle_controller.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
