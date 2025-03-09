from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

import cv2

class line_follower(Node):

    def __init__(self):
        super().__init__('line_follower_node')
        self.camera_sub = self.create_subscription(Image,'/camera/image_raw',self.camera_cb,10)
        self.bridge=CvBridge() 
        self.vel_msg = Twist()
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)


    def camera_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        frame = frame[290:479,130:400]  
        edged = cv2.Canny(frame,60,100)  

        white_index =[]  
        mid_point_lines = 0
        for index,values in enumerate(edged[:][90]):
            if values == 255:
                white_index.append(index)
        print(white_index)

        if len(white_index) == 2:
            cv2.circle(img=edged,center= (white_index[0],90),radius=2,color=(255,0,0),thickness=1)
            cv2.circle(img=edged,center=(white_index[1],90),radius=2,color=(255,0,0),thickness=1)
            
            mid_point_lines = int((white_index[0] + white_index[1]) / 2)  
            cv2.circle(img=edged,center=(mid_point_lines,90),radius=3,color=(255,0,0),thickness=2)
            
        midpoint_robot = [133,90] 
        cv2.circle(img=edged,center=(midpoint_robot[0],midpoint_robot[1]),radius=3,color=(255,0,0),thickness=2)
        error = midpoint_robot[0] - mid_point_lines
        print(error)

        
        if error < 0:
            self.vel_msg.angular.z = -0.5
        else:
            self.vel_msg.angular.z = 0.5
        
        self.vel_msg.linear.x = 0.5
        self.cmd_vel_pub.publish(self.vel_msg)

        cv2.imshow('Frame',frame)
        cv2.imshow('Canny edge',edged)
        cv2.waitKey(1)

  
def main(args=None):
    rclpy.init(args=args)

    line_follower_sub = line_follower()

    rclpy.spin(line_follower_sub)
    line_follower_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()