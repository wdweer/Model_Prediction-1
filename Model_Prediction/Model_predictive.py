import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation
from tf.transformations import euler_from_quaternion

class Bicycle_Model_Predictive():
    def __init__(self):
        rospy.init_node("Bicycle_Model_Predictive")
        self.target_pose_sub = rospy.Subscriber('/target_pose', Pose, self.target_pose_callback)
        self.target_velocity_sub = rospy.Subscriber('/target_velocity', Twist, self.target_velocity_callback)
        self.target_angular_sub = rospy.Subscriber('/target_angular', Float64, self.target_angular_callback)
        self.model_predicted_num = 5
        self.dt = 0.1
        self.target_x = 0
        self.target_y = 0
        self.target_velocity = 0
        self.target_angular_z = 0
        rate = rospy.Rate(1000)
        file_path = '~/Desktop/car-racing/local_coordinates_tags.csv'
        df = pd.read_csv(file_path)
        self.cx = list(df['local_x'])
        self.cy = list(df['local_y'])
        self.model_prediction_x = []
        self.model_prediction_y = []
        self.Prediction_policy='heuristic'
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'bo-')
        self.ax.set_xlim(min(self.cx) - 1, max(self.cx) + 1)
        self.ax.set_ylim(min(self.cy) - 1, max(self.cy) + 1)
        self.track_line, = self.ax.plot(self.cx, self.cy, 'r--')
        self.animation = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        while not rospy.is_shutdown():
            plt.pause(0.001)
        
            rate.sleep()
    def init_plot(self):
        self.line.set_data([], [])
        self.track_line.set_data(self.cx, self.cy)
        return [self.line, self.track_line]
    def update_plot(self, frame):
        self.line.set_data(self.model_prediction_x, self.model_prediction_y)
        return [self.line, self.track_line]

    def target_pose_callback(self, data):
        self.target_x = data.position.x
        self.target_y = data.position.y
        self.target_z = data.position.z
        self.target_orientation_x=data.orientation.x 
        self.target_orientation_y=data.orientation.y 
        self.target_orientation_z=data.orientation.z 
        self.target_orientation_w=data.orientation.w 
        self.target_orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        if self.Prediction_policy=='heuristic':
           self.heuristic_model_predictive()# Signal to update plot
        if self.Prediction_policy=='MPC':
           self.MPC_model_predictive()
           

    def target_velocity_callback(self, data):
        self.target_velocity_x = data.linear.x
        self.target_velocity_y = data.linear.y
        self.target_velocity=((self.target_velocity_x)**2+(self.target_velocity_y)**2)**(1/2)
        self.target_yaw = math.atan2(self.target_velocity_y, self.target_velocity_x)
        print(self.target_yaw)

    def target_angular_callback(self, data):
        self.target_angular_z = data.data

    def get_yaw_from_orientation(self, x, y, z, w):
        euler = euler_from_quaternion([x, y, z, w])
        return euler[2] 

    def heuristic_model_predictive(self):
        model_prediction_x = self.target_x
        target_velocity_x = self.target_velocity_x
        target_yaw = self.target_yaw
        model_prediction_y = self.target_y
        target_velocity_y = self.target_velocity_y
        target_angular_z = self.target_angular_z
        self.model_prediction_x = []
        self.model_prediction_y = []
        for i in range(self.model_predicted_num):
            self.model_prediction_x.append(model_prediction_x)
            model_prediction_x += target_velocity_x
            target_yaw += target_angular_z
            target_velocity_x = self.target_velocity * math.cos(target_yaw)
        for j in range(self.model_predicted_num):
            self.model_prediction_y.append(model_prediction_y)
            model_prediction_y += target_velocity_y
            target_yaw += target_angular_z
            target_velocity_y = self.target_velocity * math.sin(target_yaw)

    def MPC_model_predictive(self):
        







if __name__ == '__main__':
    try:
        Bicycle_Model_Predictive()
    except rospy.ROSInterruptException:
        pass
