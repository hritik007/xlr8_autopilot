import numpy as np

def check_if_obstacle_detected():
    #lidar must be tilted in order to detect obstacle

    #if lidar tells that path from (-40 degree to +40 degrees) (the front path of the rover ) is clear , then 
    # it returns ture , otherwise it returns false

    #take 8 strips of 10 degrees each in (-40 to +40) degrees
    #take average or varience of each 10 degree strip
    #if any one of the strips says about obstacle , then it will return true
    return False

def master_string_generator():
    print("not developed yet")
    #this is the last function to be developed in the file

def ranges_trimmer(lidar_ranges_360):
    ranges_320_to_360=lidar_ranges_360[320:360] #-40 to -1 degree (total 40 values) 
    ranges_0_to_40   =lidar_ranges_360[0:40]#0 to 39 degree (total 40 values)
    lidar_ranges_80=np.concatenate((ranges_320_to_360,ranges_0_to_40 ), axis=0)
    return lidar_ranges_80