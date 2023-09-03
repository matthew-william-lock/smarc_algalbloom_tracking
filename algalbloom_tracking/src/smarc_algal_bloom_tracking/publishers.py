import rospy

# SMaRC imports
from smarc_msgs.msg import AlgaeFrontGradient, GotoWaypoint
from geographic_msgs.msg import GeoPointStamped, GeoPoint
from std_msgs.msg import Bool

"""
PUBLISHERS
"""

def publish_gradient(lat,lon,x,y,gradient_pub):
    """ Publish gradient information """

    msg = AlgaeFrontGradient()
    msg.header.stamp = rospy.Time.now()
    msg.lat = lat
    msg.lon = lon
    msg.x = x
    msg.y = y

    gradient_pub.publish(msg)  


def publish_offset(lat,lon,pub):
    """ Publish lat_lon offset"""

    msg = GeoPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = -1

    pub.publish(msg)   


def publish_vp(lat,lon,vp_pub):
    """ Publish current vp """

    msg = GeoPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = -1

    vp_pub.publish(msg)   


def publish_waypoint(latlontoutm_service,next_waypoint,waypoint_pub,enable_waypoint_pub,travel_rpm,speed,waypoint_tolerance):    
    """ Publish waypoint to SAM"""

    # Convert lat,lon to UTM
    gp = GeoPoint()
    gp.latitude = next_waypoint[0, 1]
    gp.longitude = next_waypoint[0, 0]
    gp.altitude = -1
    utm_res = latlontoutm_service(gp)

    x = utm_res.utm_point.x
    y = utm_res.utm_point.y

    # Speed and z controls
    z_control_modes = [GotoWaypoint.Z_CONTROL_DEPTH]
    speed_control_mode = [GotoWaypoint.SPEED_CONTROL_RPM,GotoWaypoint.SPEED_CONTROL_SPEED]

    # Waypoint message
    msg = GotoWaypoint()
    msg.travel_depth = -1
    msg.goal_tolerance = waypoint_tolerance
    msg.lat = next_waypoint[0, 1]
    msg.lon = next_waypoint[0, 0]
    msg.z_control_mode = z_control_modes[0]
    msg.travel_rpm = travel_rpm
    msg.speed_control_mode = speed_control_mode[0]
    msg.travel_speed = speed
    msg.pose.header.frame_id = 'utm'
    msg.pose.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y

    # Enable waypoint following
    enable_waypoint_following = Bool()
    enable_waypoint_following.data = True
    enable_waypoint_pub.publish(enable_waypoint_following)

    # Publish waypoint
    waypoint_pub.publish(msg)
    # rospy.loginfo('Published waypoint : {},{}'.format(next_waypoint[0, 1],next_waypoint[0, 0]))
