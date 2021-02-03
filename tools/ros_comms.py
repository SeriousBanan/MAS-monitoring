"Commands for communicate with ROS."
from math import pi
from collections import defaultdict
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
from tools.setup_loggers import logger

# Turning angle accuracy
_TURN_TOLERANCE = 0.1
_ROAD_TOLERANCE = 0.10
_ANGLE_TOLERANCE = 0.05
_CORRECTION_SPEED = 0.15


# Turns left for 90 degrees
def _left_turn(id_):
    "Left turn"
    pub = rospy.Publisher('part2_cmr/cmd_vel_' + str(id_), Twist, queue_size=1)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.angular.z = 0.6

    # Set publish rate at 10 Hz
    rate = rospy.Rate(10)
    begin_yaw = _get_yaw_angle('rosbots_' + str(id_), 'link')
    logger.debug(f"{begin_yaw % (2 * pi)}")
    logger.debug(f"{(begin_yaw + pi / 2) % (2 * pi)}")

    # Command in progress
    for _ in range(10):
        pub.publish(move_cmd)
        rate.sleep()

    while (_get_yaw_angle('rosbots_' + str(id_), 'link') %
           (2 * pi) < ((begin_yaw + pi / 2) %
                       (2 * pi) - _TURN_TOLERANCE)) or (_get_yaw_angle('rosbots_' + str(id_), 'link') %
                                                        (2 * pi) > ((begin_yaw + pi / 2) %
                                                                    (2 * pi) + _TURN_TOLERANCE)):
        pub.publish(move_cmd)
        rate.sleep()

    logger.debug((_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) < ((begin_yaw + pi / 2) % (2 * pi) - _TURN_TOLERANCE))
                 or (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) > ((begin_yaw + pi / 2) % (2 * pi) + _TURN_TOLERANCE)))
    logger.debug((begin_yaw - pi / 2) % (2 * pi) - _TURN_TOLERANCE)
    logger.debug(f"{_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi)}")
    logger.debug((begin_yaw - pi / 2) % (2 * pi) + _TURN_TOLERANCE)

    # Stopping after command is completed
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()

# Turns right for 90 degrees


def _right_turn(id_):
    "Right turn"
    pub = rospy.Publisher('part2_cmr/cmd_vel_' + str(id_), Twist, queue_size=1)
    rospy.init_node('command_node', anonymous=True)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.angular.z = -0.6

    # Set publish rate at 100 Hz
    rate = rospy.Rate(10)
    begin_yaw = _get_yaw_angle('rosbots_' + str(id_), 'link')

    logger.debug(f"{begin_yaw % (2 * pi)}")
    logger.debug(f"{(begin_yaw - pi / 2) % (2 * pi)}")

    # Command in progress
    for _ in range(10):
        pub.publish(move_cmd)
        rate.sleep()

    while (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) < ((begin_yaw - pi / 2) % (2 * pi) - _TURN_TOLERANCE) or
            _get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) > ((begin_yaw - pi / 2) % (2 * pi) + _TURN_TOLERANCE)):
        pub.publish(move_cmd)
        rate.sleep()

    logger.debug((_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) < ((begin_yaw - pi / 2) % (2 * pi) - _TURN_TOLERANCE))
                 or (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) > ((begin_yaw - pi / 2) % (2 * pi) + _TURN_TOLERANCE)))
    logger.debug(f"{_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi)}")
    logger.debug(f"{(begin_yaw + pi / 2) % (2 * pi)}")

    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link'))

    # Stopping after command is completed
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()


def _move_forward(id_):
    "_move_forward"
    pub = rospy.Publisher('part2_cmr/cmd_vel_' + str(id_), Twist, queue_size=1)
    rospy.init_node('command_node', anonymous=True)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()

    # Set publish rate at 100 Hz
    rate = rospy.Rate(10)

    start_coord = _get_coords('rosbots_' + str(id_))[0]
    direction = _get_direction('rosbots_' + str(id_))
    if direction == "up" or direction == "left":
        final_coord = start_coord + 1.9
    else:
        final_coord = start_coord - 1.9

    line_coords = round(_get_coords('rosbots_' + str(id_))[1], None)

    logger.debug(start_coord)
    logger.debug(final_coord)
    logger.debug(direction)

    # Command in progress
    if direction == "up":
        while _get_coords('rosbots_' + str(id_))[0] < final_coord - 0.1:
            move_cmd.linear.x = 0.8

            if _get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE or _get_yaw_angle('rosbots_' + str(id_), 'link') % (
                    2 * pi) / (pi / 2) - round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) > _ANGLE_TOLERANCE:
                move_cmd.angular.z = -1 * _CORRECTION_SPEED

                logger.debug("Correcting right.")
                if _get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords + _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') %
                    (2 * pi) / (pi / 2) - round(_get_yaw_angle('rosbots_' + str(id_), 'link') %
                                                (2 * pi) / (pi / 2), None) > _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None))
            elif (_get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE or
                  _get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                  round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) < -1 * _ANGLE_TOLERANCE):
                move_cmd.angular.z = _CORRECTION_SPEED

                logger.info("Correcting left.")
                if _get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords - _ROAD_TOLERANCE)
                if _get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) - \
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) < -1 * _ANGLE_TOLERANCE:
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None))
            else:
                move_cmd.angular.z = 0.0

            pub.publish(move_cmd)
            rate.sleep()
    elif direction == "left":
        while _get_coords('rosbots_' + str(id_))[0] < final_coord - 0.1:
            move_cmd.linear.x = 0.8

            if _get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE or _get_yaw_angle('rosbots_' + str(id_), 'link') % (
                    2 * pi) / (pi / 2) - round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) > _ANGLE_TOLERANCE:
                move_cmd.angular.z = -1 * _CORRECTION_SPEED

                logger.debug("Correcting right.")
                if _get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords - _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) > _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None))

            elif (_get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE or
                    _get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                    round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) < -1 * _ANGLE_TOLERANCE):
                move_cmd.angular.z = _CORRECTION_SPEED

                logger.debug("Correcting left.")
                if _get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords + _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) < -1 * _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None))

            else:
                move_cmd.angular.z = 0.0

            pub.publish(move_cmd)
            rate.sleep()

    elif direction == "down":
        while _get_coords('rosbots_' + str(id_))[0] > final_coord + 0.1:
            move_cmd.linear.x = 0.8

            if _get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE or _get_yaw_angle('rosbots_' + str(id_), 'link') % (
                    2 * pi) / (pi / 2) - round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) > _ANGLE_TOLERANCE:
                move_cmd.angular.z = -1 * _CORRECTION_SPEED

                logger.debug("Correcting right.")
                if _get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords - _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) > _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)))

            elif (_get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE or
                    _get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                    round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) < -1 * _ANGLE_TOLERANCE):
                move_cmd.angular.z = _CORRECTION_SPEED

                logger.debug("Correcting left.")
                if _get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords + _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) < -1 * _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)))

            else:
                move_cmd.angular.z = 0.0

            pub.publish(move_cmd)
            rate.sleep()

    elif direction == "right":
        while _get_coords('rosbots_' + str(id_))[0] > final_coord + 0.1:
            move_cmd.linear.x = 0.8

            if _get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE or _get_yaw_angle('rosbots_' + str(id_), 'link') % (
                    2 * pi) / (pi / 2) - round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None) > _ANGLE_TOLERANCE:
                move_cmd.angular.z = -1 * _CORRECTION_SPEED

                logger.debug("Correcting right.")
                if _get_coords('rosbots_' + str(id_))[1] > line_coords + _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords + _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) > _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2), None))

            elif (_get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE or
                    _get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                    round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) < -1 * _ANGLE_TOLERANCE):
                move_cmd.angular.z = _CORRECTION_SPEED

                logger.debug("Correcting left.")
                if _get_coords('rosbots_' + str(id_))[1] < line_coords - _ROAD_TOLERANCE:
                    logger.debug(_get_coords('rosbots_' + str(id_))[1], line_coords - _ROAD_TOLERANCE)
                if (_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                        round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)) < -1 * _ANGLE_TOLERANCE):
                    logger.debug(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2) -
                                 round(_get_yaw_angle('rosbots_' + str(id_), 'link') % (2 * pi) / (pi / 2)))

            else:
                move_cmd.angular.z = 0.0

            pub.publish(move_cmd)
            rate.sleep()

    # Stopping after command is completed
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    rate.sleep()


def _get_yaw_angle(name, relative_entity_name):
    "Get yaw angle"
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(str(name), str(relative_entity_name))
        orientation_list = [
            resp_coordinates.pose.orientation.x,
            resp_coordinates.pose.orientation.y,
            resp_coordinates.pose.orientation.z,
            resp_coordinates.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    except rospy.ServiceException as exception:
        rospy.loginfo("Get Model State service call failed:  {0}".format(exception))

# Returns x or y coord depending on the direction of movement


def _get_coords(name):
    "Get cords"
    direction = _get_direction(name)
    if direction in ["left", "right"]:
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(str(name), 'link')
            return (resp_coordinates.pose.position.y, resp_coordinates.pose.position.x)

        except rospy.ServiceException as exception:
            rospy.loginfo("Get Model State service call failed:  {0}".format(exception))

    else:
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(str(name), 'link')
            return (resp_coordinates.pose.position.x, resp_coordinates.pose.position.y)

        except rospy.ServiceException as exception:
            rospy.loginfo("Get Model State service call failed:  {0}".format(exception))

# Returns direction of movement on the axis (positive direction of X is "top", and positive of Y is "left" )


def _get_direction(name):
    "Get direction"

    if round(_get_yaw_angle(name, 'link') % (2 * pi) / (pi / 2)) % 4 == 0:
        return "up"

    elif round(_get_yaw_angle(name, 'link') % (2 * pi) / (pi / 2)) % 4 == 1:
        return "left"

    elif round(_get_yaw_angle(name, 'link') % (2 * pi) / (pi / 2)) % 4 == 2:
        return "down"

    elif round(_get_yaw_angle(name, 'link') % (2 * pi) / (pi / 2)) % 4 == 3:
        return "right"


def _get_callback(id_):
    "get callback"
    def callback(data):
        if data.data not in unique_messages[id_]:
            unique_messages[id_].append(data.data)

    return callback

# Defines the sequence of commands based on given id_ and coordinates


def move_agent(id_, x_start, y_start, x_end, y_end):
    "Move agent"
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

    direction = _get_direction('rosbots_' + str(id_))

    if direction == cmd_direction:
        _move_forward(id_)
    elif (direction == "top" and cmd_direction == "right") or (direction == "right" and cmd_direction == "down") or \
            (direction == "down" and cmd_direction == "left") or (direction == "left" and cmd_direction == "top"):
        _right_turn(id_)
        _move_forward(id_)
    elif (direction == "top" and cmd_direction == "left") or (direction == "left" and cmd_direction == "down") or \
            (direction == "down" and cmd_direction == "right") or (direction == "right" and cmd_direction == "top"):
        _left_turn(id_)
        _move_forward(id_)
    else:
        _right_turn(id_)
        _right_turn(id_)
        _move_forward(id_)


def send_message(id_, data):
    "Send message"
    rate = rospy.Rate(100)
    for _ in range(10):
        _messengers[id_].publish(data)
        rate.sleep()


def _initialize_messengers():
    "initialize messengers"

    messengers = []

    for id_ in range(3):
        messenger = rospy.Publisher(f'message_topic_{id_}', String, queue_size=10)
        messengers.append(messenger)

    return messengers


def _initialize_listeners():
    "initialize listeners"

    listeners = {}
    for publisher_id in range(3):
        for listener_id in range(3):
            subscriber = rospy.Subscriber(f"message_topic_{publisher_id}", String, _get_callback(listener_id))
            listeners[f"listener {listener_id} of {publisher_id}"] = subscriber

    return listeners


rospy.init_node('command_node', anonymous=True)

# creating publishers
_messengers = _initialize_messengers()
unique_messages: dict = defaultdict(list)

# creating listeners
_listeners = _initialize_listeners()
