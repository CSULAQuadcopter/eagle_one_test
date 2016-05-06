#! /usr/bin/env python
"""
This is where everything is going to live
"""

# We're using ROS here
import rospy

# ROS message
from std_msgs.msg import String, Empty

# Import all of the modes
from Land          import Landing
# from Follow        import Follow
from Takeoff       import Takeoff
from Emergency     import Emergency
from TakePicture   import TakePicture
from Reacquisition import Reacquisition

# For the state machine
from StateMachine import Smach

rospy.init_node('main')
rate = rospy.Rate(10)

state = 'nada'
transition = 'nada'

def state_cb(msg):
    global state
    state = msg.data

# pub_transition = rospy.Publisher('/smach/transition', String, queue_size=1)
sub_state = rospy.Subscriber('/smach/state', String, state_cb, queue_size=1000)
# Instantiate all of the modes
# Set all of the necessary parameters
# TODO instantiate these correctly i.e. correct parameters
# Land Mode Instantiation
land_speed = -1	 # m/s
land_min_alt = 1000  # mm
land_max_alt = 2000 # mm
land_height_diff = 0 #mm
land_timeout = 3 # seconds
land = Landing(land_speed, land_min_alt, land_height_diff, land_max_alt, land_timeout)

# Takeoff Mode Instantiation
takeoff_speed = 1 # percentage of max speed (kind of)
takeoff_max_alt = 2500  # mm
takeoff_timeout = 10 # seconds
takeoff_counter = 0 # times
takeoff_to_reac_timeout = 15 # seconds
takeoff = Takeoff(takeoff_speed, takeoff_max_alt, takeoff_timeout)

# Takeoff Mode Instantiation
emergency = Emergency()

# Take Picture Mode Instantiation
takepicture_time = 30 # seconds
# takepicture_max_time   = 3 # seconds
takepicture = TakePicture(takepicture_time)

# Reacquisition Mode Instantiation
reac_velocities = (0.3, 0.3, 0.3) 	 # m/s
reac_max_alt = 3000  # mm
reac_tag_timeout = 15 	 # seconds
reac_prev_state_timeout = 5 # seconds
reac = Reacquisition(reac_velocities, reac_max_alt, reac_tag_timeout, reac_prev_state_timeout)
# follow = Follow()

# Create the timers to transition to Reacquisition mode
# takeoff_to_reac_timer = rospy.Timer(rospy.Duration(takeoff_to_reac_timeout), goto_reacquisition)
# land_to_reac_timer = rospy.Timer(rospy.Duration(land_to_reac_timeout), goto_reacquisition)
# follow_to_reac_timer = rospy.Timer(rospy.Duration(follow_to_reac_timeout), goto_reacquisition)

# smach = Smach()

# TODO: Test all of this
# The start of it all
# def main():
takeoff.turn_off_timer(takeoff.timer)
land.turn_off_timer(land.timer)
reac.turn_off_timer(reac.prev_state_timer)
reac.turn_off_timer(reac.land_timer)
while not rospy.is_shutdown():
    print state

    # NOTE: There are some subtleties here. We want to be able to use more than
    # one mode for certain modes which is why follow mode is on it's own
    # Check if we're in takeoff mode
    if (state == 'takeoff'):
        print "Takeoff Mode"
        # NOTE: This was copy and pasted from the takeoff node
        # To get this guy to take off! For some reason just calling this
        # function once does not work. This value (50) was determined
        # experimentally
        # TODO: Turn on takeoff mode's timer and turn off every other mode's
        # timers EXCEPT take picture
        takeoff.turn_on_timer(takeoff.timer)
        land.turn_off_timer(land.timer)
        reac.turn_off_timer(reac.prev_state_timer)
        reac.turn_off_timer(reac.land_timer)
        while takeoff_counter < 50:
            takeoff.launch()
            takeoff_counter += 1
            rate.sleep()
        if(takeoff.altitude < takeoff.max_altitude):
            rospy.loginfo("Go up!")
            takeoff.change_altitude(takeoff_speed)
        elif(takeoff.altitude >= takeoff.max_altitude):
            takeoff_speed = 0
            rospy.loginfo("Stop!")
            # takeoff.change_altitude(takeoff_speed)
            # To change states, we publish the fact that we've
            # reached our takeoff altitude
            rospy.loginfo("Going to follow mode")
            transition = 'TAKEOFF_ALT_REACH'
            # takeoff.goto_follow()

    # Check if we're in land mode
    # elif (state == 'land'):
    #     # TODO: Turn on land mode's timer and turn off every other mode's
    #     # timers EXCEPT take picture
    #     takeoff.turn_off_timer(takeoff.timer)
    #     land.turn_on_timer(land.timer)
    #     reac.turn_off_timer(reac.prev_state_timer)
    #     reac.turn_off_timer(reac.land_timer)
    #
    #     land_height_diff = land.max_altitudeGoal - land.altitude
    #     print("%d" % land.altitude)
    #         # if(land.tag_acquired):
    #     if(land.altitude > land.min_altitude):
    #         print("Go down")
    #         if(land.height_diff > land.min_altitude):
    #             land.change_altitude(land_speed)
    #             print("Descending")
    #             # TODO: Check this. Is it nececessary? Seems odd.
    #             rospy.sleep(.5)
    #         # TODO: Check this. Is it nececessary? Seems odd.
    #         elif(land.height_diff <= land.min_altitude):
    #             pass
    #     elif(land.altitude < land.min_altitude):
    #         print("Eagle one has descended!")
    #         land.land()
    #
    # # Check if we're in take picture mode
    # elif (state == 'take_picture'):
    #     # NOTE: This timer should be left alone, but turn every other one off
    #     takeoff.turn_off_timer(takeoff.timer)
    #     land.turn_off_timer(land.timer)
    #     reac.turn_off_timer(reac.prev_state_timer)
    #     reac.turn_off_timer(reac.land_timer)
    #     print "Take Picture Mode"
    #     if(takepicture.finished()):
    #         print "Picture taken"
    #
    # # Check if we're in emergency mode
    # elif (state == 'emergency'):
    #     # NOTE: Turn off all timers
    #     takeoff.turn_off_timer(takeoff.timer)
    #     land.turn_off_timer(land.timer)
    #     reac.turn_off_timer(reac.prev_state_timer)
    #     reac.turn_off_timer(reac.land_timer)
    #     print "Emergency Mode"
    #     emergency.emergency_land()
    #
    # # Check if we're in reacquisition mode
    # elif (state == 'reacquisition'):
    #     # TODO: Turn on reacquisition mode's timer and turn off every other mode's
    #     # timers EXCEPT take picture
        # takeoff.turn_off_timer(takeoff.timer)
        # land.turn_off_timer(land.timer)
        # reac.turn_on_timer(reac.prev_state_timer)
        # reac.turn_on_timer(reac.land_timer)
    #
    #     print "Reacquisition Mode"
    #     reac.move()
    #
    # # Check if we're in secure mode and do nothing
    # elif (state == 'secure'):
    #     # Do nothing
    #     print "Secure Mode"
    #
    # # Check if we're in follow mode
    # # Follow mode is used in every mode except for secure and emergency
    # if (not ((smach.state == 'secure')) and (smach.state == 'emergency')):
    #     print "Follow Mode"

    # smach.change_state(transition)
    rate.sleep()
#
# if __name__ == '__main__':
#     main()
