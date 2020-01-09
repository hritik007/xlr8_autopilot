import numpy as np
import math

def ranges_trimmer(lidar_ranges_360, is_obstacle_detected):
    ranges_320_to_360=lidar_ranges_360[320:360] #-40 to -1 degree (total 40 values) 
    ranges_0_to_40   =lidar_ranges_360[0:40]#0 to 39 degree (total 40 values)
    lidar_ranges_minus_40_to_plus_39=np.concatenate((ranges_320_to_360,ranges_0_to_40 ), axis=0)
    #return lidar_ranges_80

    ###################normalizer function###############################
    lidar_ranges_minus_40_to_minus_1 = lidar_ranges_minus_40_to_plus_39[0:40]#-40 to -1 degree  (total 40 values)
    lidar_ranges_0_to_39 = lidar_ranges_minus_40_to_plus_39[40:80]#0 to 39 degree (total 40 values)

    base_minus_40_to_minus_1=np.zeros(40)
    base_0_to_40=np.zeros(40)
    
    for r in range(40):#0 to 39 == -40 to -1 in range
        r2=40-r#40 , 39 , 38 , ..... , 2, 1
        r2=r2*-1#-40 , -39 , -38 , ...... ,-2 , -1
        base_minus_40_to_minus_1[r]=math.cos(math.radians(r2))*lidar_ranges_minus_40_to_minus_1[r]

    for r in range(40):#0 to 39 == 0 to 39 in range
        base_0_to_40[r]=math.cos(math.radians(r))*lidar_ranges_0_to_39[r]

    normalized_ranges_minus_40_to_plus_39=np.concatenate((base_minus_40_to_minus_1,base_0_to_40 ), axis=0)# equals to 1.08
    ###########################################################################################
    """
    for i in range(80):
        print(str(i-40)+" : "+str(normalized_ranges_minus_40_to_plus_39[i]))
    print("")
    """
    ##########################10 degrees average taker###########################
    
    average_normalized_range=np.zeros(20)
    for i in range(20):
        for j in range(4):
            average_normalized_range[i]=average_normalized_range[i]+normalized_ranges_minus_40_to_plus_39[(i*4)+j]
        
        if(average_normalized_range[i]/4>6):
            average_normalized_range[i]=6
            #print(average_normalized_range[i])
        else:
            average_normalized_range[i]=average_normalized_range[i]/4
            #print(average_normalized_range[i])
    #print("")
    #############################################################################

    #############################################################################
    rover_height=0.45 #45 cm
    max_obstacle_height_to_stop=0.1 #10 cm
    max_scanning_range=1.274 #110 cm
    theta=math.asin(rover_height/max_scanning_range)
    least_scanning_range_after_obstacle=max_scanning_range-(max_obstacle_height_to_stop/math.sin(theta))

    ###################################################################33
    obstacle_angles=np.zeros(20)
    left_weight=0
    right_weight=0
    if(is_obstacle_detected==0):
        for i in range(20):
            if(average_normalized_range[i]<least_scanning_range_after_obstacle):
                is_obstacle_detected=1
                obstacle_angles[i]=1
                if(i<10):
                    left_weight=left_weight+1
                else:
                    right_weight=right_weight+1
            else:
                obstacle_angles[i]=0
            
            if(left_weight==0 and right_weight==0):
                is_obstacle_detected=0
            elif(left_weight==right_weight):
                is_obstacle_detected=-1
            elif(left_weight>right_weight):
                is_obstacle_detected=-1
            elif(right_weight>left_weight):
                is_obstacle_detected=1
    return right_weight,left_weight,is_obstacle_detected

def print_normalized_ranges(normalized_ranges_minus_40_to_plus_39):
    temp2=-40
    for rr in  normalized_ranges_minus_40_to_plus_39:
        print(str(temp2)+" degrees : range = "+str(rr)+" meters")
        temp2=temp2+1
    print("")

def print_20(ranges):
    i=-10
    for rr in ranges:
        print(str(i)+" : "+str(rr))
        i=i+1
    print("")