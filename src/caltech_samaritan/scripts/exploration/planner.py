import sys
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from nav_msgs.msg import Odometry
from caltech_samaritan.msg import uav_pose 
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

array_of_goals = [(100, 100, 100), (110,110, 110)] # An array of goals with two arbitrary goals for checking the 13 meter distance parameter.

def in_bounds(p, r):
    return 0 <= p < r

def cell_entropy(p):
    if p < 0:
        return np.log(2)
    return -p * np.log(p) - (1 - p) * np.log(1 - p)


if len(sys.argv) < 2:
    print("enter correct uav name")
else:
    uav_prefix = sys.argv[1]
    no_of_uavs = sys.argv[2]
    uav_digit = ""
    for m in uav_prefix:
        if m.isdigit():
            uav_digit = uav_digit + m


class Planner:

    def __init__(self, core, datastore, movement_handler):
        self.core = core
        self.store = datastore
        self.movement = movement_handler

        if len(sys.argv) < 2:
            print("enter correct uav name")
        else:
            uav_prefix = sys.argv[1]
        
        rospy.Subscriber('/'+uav_prefix+'/planned_cmd_vel', Twist, self.planned_cmd_vel_callback)
        rospy.loginfo('Initialising a SimpleActionClient for move_base and waiting for connection...')
        self.sa_client = SimpleActionClient('/'+uav_prefix+'/move_base', MoveBaseAction)
        self.sa_client.wait_for_server()
        rospy.loginfo('Connected to move_base action server!')
        self.last_goal = None
        self.last_planned_cmd_vel_msg = None
        self.blacklisted_goals = []

        self.goal_of_uav = uav_pose()
        self.goal_of_uav_pub = rospy.Publisher('/goal_of_uav_topic', uav_pose, queue_size=15)
        rate = rospy.Rate(1)  # 1Hz   
        rospy.Subscriber('/goal_of_uav_topic', uav_pose, self.goal_of_uav_callback)

    def goal_of_uav_callback(self, data):
        self.a = data.x
        self.b = data.y
        self.c = data.z
        self.last_a = self.a[len(self.a)-1]
        self.last_b = self.b[len(self.b)-1]
        self.last_c = self.c[len(self.c)-1]
        self.coord = self.last_a, self.last_b, self.last_c
        array_of_goals.append(self.coord)
        rospy.loginfo('goal_of_uav_array =')
      
    def planned_cmd_vel_callback(self, twist_msg):
        self.last_planned_cmd_vel_msg = twist_msg

    def find_goal(self):
        frontiers_as_indices = self.find_frontiers_as_indices()

        if len(frontiers_as_indices) < 1:
            return None

        # Make sure frontiers are at least one metre apart and NOT in blacklisted regions
        f_candidates = []
        f_candidates_in_meters = []
        min_gap = 1.0 
        for f_ia, f_ib in frontiers_as_indices:
            too_close = False
            f_x, f_y = self.indices_to_meters(f_ia, f_ib)

            # Check that the current frontier is not in blacklisted regions
            blacklisted = False
            for bl_x, bl_y, _ in self.blacklisted_goals:
                distance = math.sqrt((f_x - bl_x) ** 2 + (f_y - bl_y) ** 2)
                if distance <= self.core.config['blacklisted_goal_radius']:
                    blacklisted = True
                    break
            if blacklisted:
                continue

            f_candidates_in_meters.append((f_x, f_y))
            f_candidates.append((f_ia, f_ib))

        # Iterate through all frontiers and find the one that gives maximises information gain
        max_eig = -1
        best_frontier = None

        eig_list = []
        eig_list_sorted = []
        for f_ia, f_ib in f_candidates:
            eig = self.goal_eig(f_ia, f_ib)
            eig_list.append(eig)
        eig_list.sort()

        mid, mid_index = self.div_array_by_no_of_uavs(eig_list, no_of_uavs)
        eig_list_trimed = self.trim_list(eig_list, mid_index+1)

        for q in range(0, mid_index):
            for f_ia, f_ib in f_candidates:
                eig = self.goal_eig(f_ia, f_ib)  
                if eig not in eig_list_trimed:
                    continue

                if eig == eig_list_trimed[mid_index - q]:

                    # Check if the goal with best EIG is possible to observer safely
                    goal_safe = self.find_safe_space_to_observe(f_ia, f_ib)
                    if goal_safe is None:
                        continue
                    is_it_safe = self.distance_goalsafe_to_other_goals(goal_safe)
                    if is_it_safe is None:
                        continue
                    best_frontier = goal_safe


        # Find a safe place to observe the best frontier
        if best_frontier is not None:
            goal_safe_ia, goal_safe_ib = best_frontier
            goal_x, goal_y = self.indices_to_meters(goal_safe_ia, goal_safe_ib)

            pos_x, pos_y, yaw = self.store.get_pose()
            angle_to_goal = math.atan2(goal_y - pos_y, goal_x - pos_x)

            self.goal_of_uav.x.append(round(goal_x, 3))
            self.goal_of_uav.y.append(round(goal_y, 3))
            self.goal_of_uav.z.append(round(int(uav_digit), 3))
            self.goal_of_uav_pub.publish(self.goal_of_uav)

            return round(goal_x, 3), round(goal_y, 3), round(angle_to_goal, 3)

        # Didn't find anything, return none
        return None

    
    def distance_goalsafe_to_other_goals(self, goal_safe): #to check if the new potential goal of a UAV is atleast 13meters away from other UAV's goals

        array_of_goals2 = array_of_goals # "array_of_goals" consists of an array containing other UAV's goals. eg [(1,1), (2,2), (3,3), (4,4), (5,5)]
        ix_goal_safe, iy_goal_safe = goal_safe #This UAV's new potential goal
        new_iax, new_iby = self.indices_to_meters(ix_goal_safe, iy_goal_safe) #converts new potential goal to coordinates
       
        z_list = []
        z_prev = 100
        for i, array_of_goals2 in enumerate(reversed(array_of_goals2)): #since other UAV's goals get "appended" at the end, we read the list in reverse. new list:[(5,5), (4,4)...] 
            if i == (2 * int(no_of_uavs)) - 1 : # eg for 3 UAVS, we will check the recent goals of other 2 UAVs
                break

            x = array_of_goals2[0] #stores x and y coordinate of other UAV's goal
            y = array_of_goals2[1]
            z = array_of_goals2[2]

            z_prev = z
            if z_prev in z_list:
                continue
            z_list.append(z)

            if z != int(uav_digit):
                dist_other_coordinates_to_findgoal = math.sqrt((x - new_iax) ** 2 + (y - new_iby) ** 2) #finds dist        
                if dist_other_coordinates_to_findgoal < 13: #checks condition of <13meters
                    return None

            if len(z_list) == int(no_of_uavs):
                break
                
        return 1

    def trim_list(self, l, n):
        return (l[:n])


    def div_array_by_no_of_uavs(self, eig_list, no_of_uavs):

        middle = float(len(eig_list))
        rospy.loginfo(middle)    
        mid = eig_list[int(middle) - 1]
        mid_index = (int(middle) - 1)
        return mid, mid_index
        

    def goal_eig(self, ia, ib):

        info = self.store.get_map_raw().info
        resolution = info.resolution

        eig_radius = 3 
        radius = int(eig_radius / resolution)

        m, n = self.store.get_map().shape
        eig_sum = 0
        total = 0
        position_x, position_y, position_yaw = self.store.get_pose()
        ix_in_meter, iy_in_meter = self.indices_to_meters(ia, ib)

        for a in range(ia - radius, ia + radius + 1):
            if not in_bounds(a, m):
                continue
            for b in range(ib - radius, ib + radius + 1):
                if not in_bounds(b, n):
                    continue
                distance = math.sqrt((a - ia) ** 2 + (b - ib) ** 2)
                if distance > (radius):
                    continue
                eig_sum  = (eig_sum +  self.cell_eig(a, b) ) 
                total += 1
      
        frac_5dp = (eig_sum / total) * (15 / self.dist_frontier_to_current_pos(ix_in_meter, iy_in_meter, position_x, position_y))
        return frac_5dp

    def dist_frontier_to_current_pos(self, x1, y1, x2, y2):
        dist_value = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        return dist_value
                

    def cell_eig(self, x, y):
        p_true = 0.9
        p = self.store.get_map()[x, y]
        if p < 0:
            return 0.5983
        elif p == 0 or p >= 1:
            return 4.259045e-6
        p_o = p_true * p + (1 - p_true) * (1 - p)
        p_n = p_true * (1 - p) + (1 - p_true) * p
        EH = -p_true * p * np.log(p_true * p / p_o) - p_true * (1 - p) * np.log(p_true * (1 - p) / p_n)
        return cell_entropy(p) - EH

    def find_safe_space_to_observe(self, ia, ib):

        costmap = self.store.get_global_costmap()
        m, n = costmap.shape

        candidates = [
            (3, 3),
            (3, 0),
            (3, -3),
            (0, -3),
            (-3, -3),
            (-3, 0),
            (-3, 3),
            (0, 3),
        ]

        best_score = -1
        best_ia = None
        best_ib = None
        best_iax = None
        best_iby = None


        for c_ia, c_ib in candidates:
            new_ia = ia + c_ia
            new_ib = ib + c_ib

            if not in_bounds(new_ia, m) \
                    or not in_bounds(new_ib, n) \
                    or costmap.mask[new_ia, new_ib] \
                    or costmap[new_ia, new_ib] != 0:
                continue

            score = 0
            for a in range(new_ia - 2, new_ia + 3):  
                for b in range(new_ib - 2, new_ib + 3):   
                    if not in_bounds(a, m) or not in_bounds(b, n):
                        continue

                    if not costmap.mask[a, b] and costmap[a, b] == 0:
                        score += 1

            if score > best_score:
                best_score = score
                best_ia = new_ia
                best_ib = new_ib
     
        if best_ia is not None and best_ib is not None:
            return best_ia, best_ib

        return None


    def find_frontiers_as_indices(self):
        costmap = self.store.get_global_costmap()
        if costmap is None:
            rospy.logwarn('No costmap is available - cannot find frontiers as indices, doing nothing.')
            return []

        m, n = costmap.shape
        safe_space_indices = np.where(costmap == 0)
        frontier_indices = []
        frontier_indices_in_meters = []

        candidates = [
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1),
            (-1, -1),
            (-1, 0),
        ]

        # We define a frontier as a cell that has at at least three free neighbours and and at least three unknown
        # neighbours. Remaining neighbours should either be free on unknown.

        for (ia, ib) in zip(*safe_space_indices):

            # Sanity check - skip masked inputs if they are accidentally included
            if costmap.mask[ia, ib]:
                continue

            free = 0
            known_occupied = 0
            for off_ia, off_ib in candidates:
                ia_new, ib_new = ia + off_ia, ib + off_ib

                if 0 <= ia_new < m and 0 <= ib_new < n:
                    value = costmap[ia_new, ib_new]
                    masked = costmap.mask[ia_new, ib_new]
                    if value == 0:
                        free += 1
                    elif not masked:
                        known_occupied += 1
                        break
                else:
                    # We're add the edge of the costmap, can consider this as an unknown
                    pass

            f_i_x, f_i_y = self.indices_to_meters(ia, ib)

            if known_occupied == 0 and 3 <= free <= 5:
                frontier_indices.append((ia, ib))
                frontier_indices_in_meters.append((f_i_x, f_i_y))

        rospy.loginfo('bro_front_indices=')      
        return frontier_indices

    def travel_to_goal(self):
        goal_status = self.sa_client.get_state()
        if goal_status == GoalStatus.PENDING:
            return False

        elif goal_status == GoalStatus.ACTIVE:
            if self.last_planned_cmd_vel_msg is None:
                return False

            msg = self.last_planned_cmd_vel_msg
            self.movement.request_velocity(
                lin_x=msg.linear.x,
                lin_y=msg.linear.y,
                lin_z=msg.linear.z,
                ang_x=msg.angular.x,
                ang_y=msg.angular.y,
                ang_z=msg.angular.z
            )
            return False

        elif goal_status in [GoalStatus.RECALLED, GoalStatus.PREEMPTED]:
            rospy.logwarn('Goal was recalled or preempted.')
            return True

        elif goal_status in [GoalStatus.REJECTED, GoalStatus.ABORTED]:
            rospy.logwarn('Goal was rejected or aborted - this goal was blacklisted.')
            self.blacklisted_goals.append(self.last_goal)
            self.last_goal = None
            return True

        elif goal_status == GoalStatus.SUCCEEDED:
            rospy.loginfo('Goal succeeded.')
            self.last_goal = None
            return True

        elif goal_status == GoalStatus.LOST:
            rospy.loginfo('Goal was lost!')
            self.last_goal = None
            return True

        rospy.logwarn('Unrecognised goal status was returned! Assuming goal reaching behaviour terminated.')
        return True

    def publish_goal(self, x_coord, y_coord, yaw):
        rospy.loginfo(" requesting to move to {}".format((x_coord, y_coord, yaw)))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_coord
        goal.target_pose.pose.position.y = y_coord
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.last_goal = (x_coord, y_coord, yaw)
        self.sa_client.send_goal(goal)

    def cancel_all_goals(self):
        self.sa_client.cancel_all_goals()

    def indices_to_meters(self, ia, ib):
        info = self.store.get_map_raw().info
        resolution = info.resolution 
        
        x = info.origin.position.x + ib * resolution + resolution / 2 
        y = info.origin.position.y + ia * resolution + resolution / 2  
        return x, y
            
    def meters_to_indices(self, x, y):
        info = self.store.get_map_raw().info
        resolution = info.resolution 
        ib = int(round((x - info.origin.position.x) / resolution))
        ia = int(round((y - info.origin.position.y) / resolution))
        return ia, ib
