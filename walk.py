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
		self.counter = 0
		self.leftwhisker = 0
		self.left_middlewhisker = 0
		self.rightwhisker = 0
		self.right_middlewhisker = 0
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
		left_middle_sensor = int((len(msg.ranges) / 3) * 2)
		middle_sensor = int(len(msg.ranges) / 2)
		right_middle_sensor = int(len(msg.ranges) / 3)
		right_sensor = int(len(msg.ranges) / 6)
		front = msg.ranges[middle_sensor]
		left_middle = msg.ranges[left_middle_sensor]
		left = msg.ranges[left_sensor]
		right_middle = msg.ranges[right_middle_sensor]
		right = msg.ranges[right_sensor]
		print("Sensor: " + str(front))
		print("Left Middle Sensor: " + str(left_middle))
		print("Left Sensor: " + str(left))
		print("Right Middle Sensor: " + str(right_middle))
		print("Right Sensor:" + str(right))
		self.leftwhisker = left
		self.left_middlewhisker = left_middle
		self.whisker = front
		self.right_middlewhisker = right_middle
		self.rightwhisker = right
		
	def timer_callback(self):
		self.move_cmd.linear.x = 0.0
		self.move_cmd.angular.z = 0.0

		# random movement counter
		self.counter += 1

		# random movement every 60 cycles
		if self.counter > 60:
			self.counter = 0
			# turn randomly but avoid a wall
			if self.leftwhisker > self.rightwhisker:
				self.move_cmd.angular.z = -2.0
			elif self.rightwhisker > self.leftwhisker:
				self.move_cmd.angular.z = 2.0
			self.cmd_pub.publish(self.move_cmd)

		if self.whisker < 0.5:
			# Too close — backup and turn
			if self.leftwhisker > self.rightwhisker:
				self.move_cmd.angular.z = -3.0
			elif self.rightwhisker > self.leftwhisker:
				self.move_cmd.angular.z = 3.0
			else:
				self.move_cmd.angular.z = 3.0
			self.move_cmd.linear.x = -1.0
		elif self.leftwhisker < 0.8 and (self.left_middlewhisker < self.leftwhisker):
			# Obstacle on left — turn right
			self.move_cmd.linear.x = 0.1
			self.move_cmd.angular.z = -0.3
		elif self.rightwhisker < 0.8 and (self.right_middlewhisker < self.rightwhisker):
			# Obstacle on right — turn left
			self.move_cmd.linear.x = 0.1
			self.move_cmd.angular.z = 0.3
		elif self.whisker < 1.0:
			# Turn away from closer side
			if self.leftwhisker > self.rightwhisker:
				self.move_cmd.angular.z = 2.0
			elif self.rightwhisker > self.leftwhisker:
				self.move_cmd.angular.z = -2.0
			else:
				self.move_cmd.angular.z = 2.0
			self.move_cmd.linear.x = 0.0
		else:
			# All clear — go straight
			self.move_cmd.linear.x = 0.8
			self.move_cmd.angular.z = 0.0

		self.cmd_pub.publish(self.move_cmd)
		
def main(args=None):
	rclpy.init(args=args)
	turtle_controller = Walk()
	rclpy.spin(turtle_controller)
	turtle_controller.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
