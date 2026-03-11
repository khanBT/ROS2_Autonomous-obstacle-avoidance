import time
import threading
import traceback
import math
import statistics
# ros2 imports
import rclpy
from rclpy.executors import MultiThreadedExecutor
# module imports
from robot_interface import RobotInterface



class RobotControl:

    def __init__(self, robot_interface):
        self.ri=robot_interface
        print("[RobotControl] Initialized")
        return None
    
    def __del__(self):
        # class destructor
        # termination codes here if any
        return None
    
    def stop_robot(self):

      self.ri.linear_velocity=0.0
      self.ri.angular_velocity=0.0
    
    def get_current_velocities(self):

      linear=self.ri.linear_velocity
      angular=self.ri.angular_velocity

      return linear, angular

    def move_robot_front(self,speed):
      speed=abs(speed)
      self.ri.linear_velocity=speed
      self.ri.angular_velocity=0.0

    def move_robot_back(self,speed):
      speed=abs(speed)
      self.ri.linear_velocity=-speed
      self.ri.angular_velocity=0.0

    def move_robot_left(self,angular_speed):
      self.ri.linear_velocity=0.0
      self.ri.angular_velocity=+angular_speed
    

    def move_robot_right(self,angular_speed):
      self.ri.linear_velocity=0.0
      self.ri.angular_velocity=-angular_speed
      
    def timed_move_front(self,speed,seconds):
      self.move_robot_front(speed)
      time.sleep(seconds)
      self.stop_robot() 

    def timed_move_back(self,speed,seconds):
      self.move_robot_back(speed)
      time.sleep(seconds)
      self.stop_robot() 

    def timed_move_left(self,speed,seconds):
      self.move_robot_left(speed)
      time.sleep(seconds)
      self.stop_robot() 

    def timed_move_right(self,speed,seconds):
      self.move_robot_right(speed)
      time.sleep(seconds)
      self.stop_robot()

    def move_distance_front(self,speed,distance_m):
      speed=abs(speed)

      if speed==0:
       self.stop_robot()
       return
      else:
       time_needed=abs(distance_m)/speed
       self.timed_move_front(speed,time_needed)

    def move_distance_back(self,speed,distance_m):
      speed=abs(speed)

      if speed==0:
       self.stop_robot()
       return
      else:
       time_needed=abs(distance_m)/speed
       self.timed_move_back(speed,time_needed)

    def turn_robot_left(self,angular_speed,angle_rad):
      angular_speed=abs(angular_speed)
      if angular_speed==0:
       self.stop_robot()
       return
      else:
       time_needed=abs(angle_rad)/angular_speed
       self.timed_move_left(angular_speed,time_needed)

    def turn_robot_right(self,angular_speed,angle_rad):
      angular_speed=abs(angular_speed)
      if angular_speed==0:
       self.stop_robot()
       return
      else:
       time_needed=abs(angle_rad)/angular_speed
       self.timed_move_right(angular_speed,time_needed)

    def get_min_scan_angle(self):
      scan_angle=self.ri.scan_angle_min
      return scan_angle

    def get_max_scan_angle(self):
      scan_angle=self.ri.scan_angle_max
      return scan_angle

    def get_angle_increment(self):
      scan_angle=self.ri.scan_angle_increment
      return scan_angle

    def get_min_scan_range(self):
      scan_angle=self.ri.scan_range_min
      return scan_angle

    def get_max_scan_range(self):
      scan_angle=self.ri.scan_range_max
      return scan_angle

    def get_all_scan_ranges(self):
      scan_angle=self.ri.scan_ranges
      return scan_angle

    def get_scan_range_byindex(self,index):
      ranges=self.ri.scan_ranges
      if index<0 or len(ranges)==0:
       return None
      else:
       if index>=len(ranges):
        return None
       else:
        return ranges[index]

    def get_front_scan_range(self):
      angle_min=self.ri.scan_angle_min
      inc=self.ri.scan_angle_increment
      if inc==0:
       return None
      else:
       index=round(((0-angle_min)/inc))

       value=self.get_scan_range_byindex(index)
      return value

    def get_left_scan_range(self):
      angle_min=self.ri.scan_angle_min
      inc=self.ri.scan_angle_increment
      if inc==0:
       return None
      else:
       index=round(((math.pi/2-angle_min)/inc))

       value=self.get_scan_range_byindex(index)
      return value

    def get_right_scan_range(self):
      angle_min=self.ri.scan_angle_min
      inc=self.ri.scan_angle_increment
      if inc==0:
       return None
      else:
       index=round(((-math.pi/2-angle_min)/inc))

       value=self.get_scan_range_byindex(index)
      return value

    def get_back_scan_range(self):
      angle_min=self.ri.scan_angle_min
      inc=self.ri.scan_angle_increment
      if inc==0:
       return None
      else:
       index=round(((math.pi-angle_min)/inc))

       value=self.get_scan_range_byindex(index)
      return value

    def get_min_range_no_inf_with_index(self):
      ranges=self.ri.scan_ranges
      

      clean=[r for r in ranges if not math.isinf(r)]
      if len(clean)==0:
       return None, None
      else:
       min_val=min(clean)
       index=ranges.index(min_val)

      return (min_val,index)

    def get_max_range_no_inf_with_index(self):
      ranges=self.ri.scan_ranges
      

      clean=[r for r in ranges if not math.isinf(r)]
      if len(clean)==0:
       return None, None
      else:
       max_val=max(clean)
       index=ranges.index(max_val)

      return (max_val,index)

    def get_front_idx(self):
      angle_min=self.ri.scan_angle_min
      inc=self.ri.scan_angle_increment
      if inc==0:
       return None
      else:
       index=round(((0-angle_min)/inc))

      return index

    def obstacle_prediction(self,threshhold=0.3):
      ranges=self.get_all_scan_ranges()
      inc=self.ri.scan_angle_increment
      

      if len(ranges)==0 or inc==0:
        print("None")

        return "none"
      else:
       front_idx=self.get_front_idx()
       offset=round((math.pi/4)/inc)
       left_idx=front_idx+offset
       right_idx=front_idx-offset


       print(front_idx,offset,left_idx,right_idx,len(ranges))

       front_window=ranges[right_idx:left_idx+1]
       print(len(front_window))
       clean_window=[r for r in front_window if not math.isinf(r)]
       print(len(clean_window))
       
       if len(clean_window)==0:
        print("Obstacle prediction: None")
        return "none"
       
       min_val=min(clean_window)
       print("Min front distance:",min_val)

       if min_val>threshhold:
        print("Obstacle prediction: none")

        return "none"

       else:

        print("Something is close, need classification.")

        modes=statistics.multimode(clean_window)
        if len(modes)>1:

         print("Obstacle")

         return "obstacle"

        else:
      
         print("Wall")

         return "wall"

    def direction_tracking(self):
      yaw=self.ri.odom_orientation_y
      yaw_shift=yaw+math.pi
      slice_size=(2*math.pi)/16

      sector=int(yaw_shift/slice_size)


      if sector>=16:
       sector=15
       
      sector_shift=(sector-8)%16
      directions=["-N-", "NNW", "N-W", "WNW", "-W-", "WSW", "S-W", "SSW",
      "-S-", "SSE", "S-E", "ESE", "-E-", "ENE", "N-E", "NNE"]
      direction=directions[sector_shift]


      print(yaw,yaw_shift,slice_size,sector,sector_shift,direction)

      return direction
    
    def set_velocity(self,linear,angular):
      self.ri.linear_velocity=float(linear)
      self.ri.angular_velocity=float(angular)

    def naive_obstacle_avoider(self,threshold=0.35):
      ranges=self.get_all_scan_ranges()
      
      if len(ranges)==0:
       print(len(ranges))

       return 

      segment_size=len(ranges)//8

      segs=[]
      for i in range(8):
        start=i*segment_size
        end=(i+1)*segment_size
        segs.append(ranges[start:end])

      right_seg=segs[2]
      front_right_seg=segs[3]
      front_seg=segs[4]
      front_left_seg=segs[5]
      left_seg=segs[6]

      clean1=[r for r in right_seg if not math.isinf(r)]
      clean2=[r for r in front_right_seg if not math.isinf(r)]
      clean3=[r for r in front_seg if not math.isinf(r)]
      clean4=[r for r in front_left_seg if not math.isinf(r)]
      clean5=[r for r in left_seg if not math.isinf(r)]

      r_min=min(clean1) if clean1 else float('inf')
      fr_min=min(clean2) if clean2 else float('inf')
      f_min=min(clean3) if clean3 else float('inf')
      fl_min=min(clean4) if clean4 else float('inf')
      l_min=min(clean5) if clean5 else float('inf')

      print(f"mins: L={l_min:.2f} FL={fl_min:.2f} F={f_min:.2f} FR={fr_min:.2f} R={r_min:.2f}")

      if f_min>threshold and fl_min>threshold and fr_min>threshold:
        self.set_velocity(0.2,0)
        time.sleep(0.2)
        print("Move forward")

      elif fl_min<=threshold:
        self.set_velocity(0.08,-0.7)
        time.sleep(0.2)
        print("Turn right and move")
      elif fr_min<=threshold:
        self.set_velocity(0.08,0.7)
        time.sleep(0.2)
        print("Turn robot left")
      elif f_min<=threshold:
         if l_min>r_min:
          self.set_velocity(0.08,0.7)
          time.sleep(0.2)
          print("Move left")
         else:
          self.set_velocity(0.08,-0.7)
          time.sleep(0.2)
          print("Move right")
      else:
        self.set_velocity(0.2,0.0)
        print("move forward")    

    def get_current_position_xyz(self):
      return {
         
         "x": self.ri.odom_position_x,
         "y": self.ri.odom_position_y,
         "z": self.ri.odom_position_z
      
      }
    
    def get_current_orientation_rpy(self):
     return{
     
       "r":self.ri.odom_orientation_r,
       "p":self.ri.odom_orientation_p,
       "y": self.ri.odom_orientation_y
     
     }

    def get_distance_xy(self,pos1,pos2):

      dx=pos2["x"]-pos1["x"]
      dy=pos2["y"]-pos1["y"]

      return math.sqrt((dx*dx)+(dy*dy))

        


def spin_node():
    """
    make the robot interface program to run in a separate thread
    NOTE: THE ROBOT WILL NOT WORK IF THIS FUNCTION IS REMOVED !!!
    """
    global executor
    executor.spin()
    return None


if __name__ == "__main__":

    # initialize ros2 with python
    rclpy.init(args=None)
    # instantiate robot interface program module
    robot_interface = RobotInterface()
    # start robot interface program execution
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(robot_interface)
    # run robot interface program in a separate thread
    threading.Thread(target=spin_node).start()
    # wait for a few seconds for program to initialize
    print("Getting Ready in 5 Seconds...")
    time.sleep(5.0)
    print("READY !!!")

    try:

        robot=RobotControl(robot_interface)
        robot.stop_robot()
        print("Stop command sent")
        time.sleep(2)

        start_position=robot.get_current_position_xyz()
        ori=robot.get_current_orientation_rpy()
        print("Start pos:",start_position,"Orientation:",ori)

        prev_pos=start_position
        total_dist=0.0

        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)

       
        #robot.move_robot_front(0.2)
        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)
        #time.sleep(1.0)
        #robot.stop_robot()
        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)

        #robot.move_robot_back(0.2)
        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)
        #time.sleep(1.0)
        #robot.stop_robot()
        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)

        #robot.move_robot_left(0.9)
        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)
        #time.sleep(1)
        #robot.stop_robot()
       
       
        #robot.move_robot_right(0.9)
        #linear,angular=robot.get_current_velocities()
        #print(linear,angular)
        #time.sleep(1)
        #robot.stop_robot()

        # robot.timed_move_front(0.1,1)
        # time.sleep(1)
        # robot.timed_move_back(0.1,1)
        # time.sleep(1)
        # robot.timed_move_left(0.1,1)
        # time.sleep(1)
        # robot.timed_move_right(0.1,1)
        # time.sleep(1)

        # robot.move_distance_front(0.9,0.6)
        # time.sleep(1)
        # robot.move_distance_back(0.9,0.6)
        # time.sleep(1)

        # robot.turn_robot_left(0.5,1)
        # time.sleep(1)

        # robot.turn_robot_right(0.5,1)
        # time.sleep(1)

        # angle=robot.get_min_scan_angle()
        # print("min scan angle:",angle)

        # angle=robot.get_max_scan_angle()
        # print("max scan angle:",angle)

        # increment=robot.get_angle_increment()
        # print("angle increment:",increment)

        # min_range=robot.get_min_scan_range()
        # print("min scan range:",min_range)

        # max_range=robot.get_max_scan_range()
        # print("max scan range:",max_range)

        # all_range=robot.get_all_scan_ranges()
        # print("All scan ranges:",len(all_range))

        # index=robot.get_scan_range_byindex(0)
        # print(index)
        # index=robot.get_scan_range_byindex(10)
        # print(index)
        # index=robot.get_scan_range_byindex(-1)
        # print(index)

        # front=robot.get_front_scan_range()
        # print("front scan:",front)

        # left=robot.get_left_scan_range()
        # print("Left scan:",left)

        # right=robot.get_right_scan_range()
        # print("Right scan:",right)

        # back=robot.get_back_scan_range()
        # print("Back scan:",back)

        # min_val,index=robot.get_min_range_no_inf_with_index()
        # print("min finite:",min_val,"at index:", index)

        # max_val,index=robot.get_max_range_no_inf_with_index()
        # print("max finite:",max_val,"at index:",index)

        pred=robot.obstacle_prediction()
        print("Prediction",pred)

        direction=robot.direction_tracking()
        print("direction:",direction)

        start= time.time()
        while time.time()-start<40:
          robot.naive_obstacle_avoider()

          curr_pos=robot.get_current_position_xyz()
          step_dis=robot.get_distance_xy(prev_pos,curr_pos)
          total_dist+=step_dis

          print(f"distance travelled: {total_dist:.3f} m")

          linear,angular=robot.get_current_velocities()
          print(f"velocity->linear: {linear:.3f}, angular:{angular:.3f}")
          prev_pos=curr_pos

          
        robot.stop_robot()



    except Exception as error:
        # report exception
        print("~~~~~~~~~~~ ERROR: ~~~~~~~~~~~")
        print(traceback.print_exception(error))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # clean up before shutdown
        executor.shutdown()
        robot_interface.destroy_node()

    finally:
        # shutdown ros2
        robot.stop_robot()
        time.sleep(0.5)
        robot.stop_robot()
        rclpy.shutdown()

