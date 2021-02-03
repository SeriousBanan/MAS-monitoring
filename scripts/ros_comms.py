#!/usr/bin/env python
from math import pi
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#debug mode
debug = 0

#Turning angle accuracy
turn_tolerance = 0.1
road_tolerance = 0.10
angle_tolerance = 0.05
correction_speed = 0.15


flag = False


#Turns left for 90 degrees
def left_turn(id):
    pub = rospy.Publisher('part2_cmr/cmd_vel_' + str(id), Twist, queue_size=1)
    

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.angular.z = 0.6
    #print(move_cmd.angular.z)

    # Set publish rate at 10 Hz
    rate = rospy.Rate(10)
    begin_yaw = get_yaw_angle('rosbots_' + str(id),'link')
    if debug:
        print("")
        print(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi))
        print((begin_yaw + pi/2) % (2*pi))


    #Command in progress
    for i in range(10):
        pub.publish(move_cmd) 
        rate.sleep() 

    while (get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) < ((begin_yaw + pi/2) % (2*pi) - turn_tolerance)) or (get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) > ((begin_yaw + pi/2)% (2*pi) + turn_tolerance)):
        pub.publish(move_cmd) 
        rate.sleep()  

    if debug:
        print("")
        print((get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) < ((begin_yaw + pi/2) % (2*pi) - turn_tolerance)) or (get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) > ((begin_yaw + pi/2)% (2*pi) + turn_tolerance)))
        print((begin_yaw - pi/2) % (2*pi) - turn_tolerance)
        print(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi))
        #print((begin_yaw + pi/2) % (2*pi))
        print((begin_yaw - pi/2) % (2*pi) + turn_tolerance)
        #print((get_yaw_angle('rosbots_' + str(id),'link') - begin_yaw) / (pi/2))
    #Stopping after command is completed
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()
    return

#Turns right for 90 degrees
def right_turn(id):
    pub = rospy.Publisher('part2_cmr/cmd_vel_'+ str(id), Twist, queue_size=1)
    rospy.init_node('command_node', anonymous=True)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.angular.z = -0.6
    #print(move_cmd.angular.z)

    # Set publish rate at 100 Hz
    rate = rospy.Rate(10)
    begin_yaw = get_yaw_angle('rosbots_' + str(id),'link')
    if debug:
        print("")
        print(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi))
        print((begin_yaw - pi/2) % (2*pi))

    #Command in progress
    for i in range(10):
        pub.publish(move_cmd) 
        rate.sleep()  
    while (get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) < ((begin_yaw - pi/2) % (2*pi) - turn_tolerance)) or (get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) > ((begin_yaw - pi/2)% (2*pi) + turn_tolerance)):
        pub.publish(move_cmd) 
        rate.sleep()  

    if debug:
        print("")
        print((get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) < ((begin_yaw - pi/2) % (2*pi) - turn_tolerance)) or (get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) > ((begin_yaw - pi/2)% (2*pi) + turn_tolerance)))
        #print((begin_yaw - pi/2) % (2*pi) - turn_tolerance)
        print(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi))
        print((begin_yaw + pi/2) % (2*pi))

        print(get_yaw_angle('rosbots_' + str(id),'link'))
        #print((begin_yaw - pi/2) % (2*pi) + turn_tolerance)
        #print((get_yaw_angle('rosbots_' + str(id),'link') - begin_yaw) / (pi/2))
    #Stopping after command is completed
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()
    return

def move_forward(id):
    pub = rospy.Publisher('part2_cmr/cmd_vel_'+ str(id), Twist, queue_size=1)
    rospy.init_node('command_node', anonymous=True)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()

    # Set publish rate at 100 Hz
    rate = rospy.Rate(10)

    start_coord = get_coords('rosbots_' + str(id))[0]
    direction = get_direction('rosbots_' + str(id))
    if direction == "up" or direction == "left":
        final_coord = start_coord + 1.9
    else:
        final_coord = start_coord - 1.9
    
    line_coords = round(get_coords('rosbots_' + str(id))[1], None)

    if debug:
        print("")
        print(start_coord)
        print(final_coord)
        print(direction)


    #Command in progress
    if direction == "up":
        while get_coords('rosbots_' + str(id))[0] < final_coord - 0.1:
            move_cmd.linear.x = 0.8
            
            if get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance or \
            get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance:
                if debug:
                    print("Correcting right.")
                    if(get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords + road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = -1 * correction_speed
            elif get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance or \
            get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance:
                if debug:
                    print("Correcting left.")
                    if(get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords - road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = correction_speed
            else:
                move_cmd.angular.z = 0.0
            
            pub.publish(move_cmd)
            rate.sleep() 
    elif direction == "left":
        while get_coords('rosbots_' + str(id))[0] < final_coord - 0.1:
            move_cmd.linear.x = 0.8

            if get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance or \
                get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance:
                if debug:
                    print("Correcting right.")
                    if(get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords - road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = -1 * correction_speed
            elif get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance or \
                get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance:
                if debug:
                    print("Correcting left.")
                    if(get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords + road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = correction_speed
            else:
                move_cmd.angular.z = 0.0
            
            pub.publish(move_cmd)
            rate.sleep() 

    elif direction == "down":
        while get_coords('rosbots_' + str(id))[0] > final_coord + 0.1:
            move_cmd.linear.x = 0.8

            if get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance or \
                get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance:
                if debug:
                    print("Correcting right.")
                    if(get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords - road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = -1 * correction_speed
            elif get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance or \
                get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance:
                if debug:
                    print("Correcting left.")
                    if(get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords + road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = correction_speed
            else:
                move_cmd.angular.z = 0.0
            
            pub.publish(move_cmd)
            rate.sleep()   

    elif direction == "right":
        while get_coords('rosbots_' + str(id))[0] > final_coord + 0.1:
            move_cmd.linear.x = 0.8

            if get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance or \
                get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance:
                if debug:
                    print("Correcting right.")
                    if(get_coords('rosbots_' + str(id))[1] > line_coords + road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords + road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) > angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = -1 * correction_speed
            elif get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance or \
                get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance:
                if debug:
                    print("Correcting left.")
                    if(get_coords('rosbots_' + str(id))[1] < line_coords - road_tolerance):
                        print(get_coords('rosbots_' + str(id))[1], line_coords - road_tolerance)
                    if(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None) < -1 * angle_tolerance):
                        print(get_yaw_angle('rosbots_' + str(id), 'link') % (2*pi) / (pi/2) - round(get_yaw_angle('rosbots_' + str(id),'link') % (2*pi) / (pi/2), None))
                move_cmd.angular.z = correction_speed
            else:
                move_cmd.angular.z = 0.0
            
            pub.publish(move_cmd)
            rate.sleep()  

    #Stopping after command is completed
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()
    return

def get_yaw_angle(name,relative_entity_name):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(str(name), str(relative_entity_name))
            orientation_list = [resp_coordinates.pose.orientation.x, resp_coordinates.pose.orientation.y, resp_coordinates.pose.orientation.z, resp_coordinates.pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            return yaw

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

#Returns x or y coord depending on the direction of movement
def get_coords(name):
    direction = get_direction(name)
    if direction == "left" or direction == "right":
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(str(name), 'link')
            return (resp_coordinates.pose.position.y, resp_coordinates.pose.position.x)

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
    
    else:
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(str(name), 'link')
            return (resp_coordinates.pose.position.x, resp_coordinates.pose.position.y)

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

#Returns direction of movement on the axis (positive direction of X is "top", and positive of Y is "left" )
def get_direction(name):

    if round(get_yaw_angle(name,'link') % (2*pi) / (pi/2), None) % 4 == 0:
        return "up"

    elif round(get_yaw_angle(name,'link') % (2*pi) / (pi/2), None) % 4 == 1: 
        return "left"

    elif round(get_yaw_angle(name,'link') % (2*pi) / (pi/2), None) % 4 == 2:
        return "down"

    elif round(get_yaw_angle(name,'link') % (2*pi) / (pi/2), None) % 4 == 3:
        return "right"


def callback_1(data):
    if data.data not in unique_messages[1]:
        unique_messages[1].append(data.data)

def callback_2(data):
    if data.data not in unique_messages[2]:
        unique_messages[2].append(data.data)

def callback_3(data):
    if data.data not in unique_messages[3]:
        unique_messages[3].append(data.data)

#Defines the sequence of commands based on given id and coordinates
def parse_command(id, x_start, y_start, x_end, y_end):
    x_diff = x_end - x_start
    y_diff = y_end - y_start

    if x_diff > 0 and y_diff == 0:
        cmd_direction = "up"
    elif x_diff < 0 and y_diff == 0:
        cmd_direction = "down"
    elif x_diff == 0 and y_diff > 0:
        cmd_direction = "left"
    else:
        cmd_direction = "right"

    direction = get_direction('rosbots_' + str(id))

    if direction == cmd_direction:
        move_forward(id)
    elif (direction == "top" and cmd_direction == "right") or (direction == "right" and cmd_direction == "down") or \
        (direction == "down" and cmd_direction == "left") or (direction == "left" and cmd_direction == "top"):
        right_turn(id)
        move_forward(id)
    elif (direction == "top" and cmd_direction == "left") or (direction == "left" and cmd_direction == "down") or \
        (direction == "down" and cmd_direction == "right") or (direction == "right" and cmd_direction == "top"):
        left_turn(id)
        move_forward(id)
    else:
        right_turn(id)
        right_turn(id)
        move_forward(id)

    return

def send_message(id, data):
    rate = rospy.Rate(100)
    for i in range (10):
        messengers[id].publish(data)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('command_node', anonymous=True)
    messengers = []
    unique_messages = {1: [], 2: [], 3: []}

    #creating publishers
    for i in range(3):
        messenger = rospy.Publisher('message_topic_' + str(i+1), String, queue_size=10)
        messengers.append(messenger)
    
    #creating listeners
    listener_1_1 = rospy.Subscriber('message_topic_1', String, callback_1)
    listener_1_2 = rospy.Subscriber('message_topic_2', String, callback_1)
    listener_1_3 = rospy.Subscriber('message_topic_3', String, callback_1)
    
    listener_2_1 = rospy.Subscriber('message_topic_1', String, callback_2)
    listener_2_2 = rospy.Subscriber('message_topic_2', String, callback_2)
    listener_2_3 = rospy.Subscriber('message_topic_3', String, callback_2)

    listener_3_1 = rospy.Subscriber('message_topic_1', String, callback_3)
    listener_3_2 = rospy.Subscriber('message_topic_2', String, callback_3)
    listener_3_3 = rospy.Subscriber('message_topic_3', String, callback_3)

