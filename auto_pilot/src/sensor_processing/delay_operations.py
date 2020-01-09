import time

def delay_for_time(type_of_delay):
    print("")
    if(type_of_delay=="avoid_affinity_error"):
        print("*Moving rover 1 second straight to avoid affinity error*")
        time.sleep(1)
    if(type_of_delay=="rotation_safety"):
        print("*Rotation safety delay for 1 second*")
        time.sleep(1)
    if(type_of_delay=="marker_forward_fashion"):
        print("*Moving forward for 2 seconds and for re-searching*")
        time.sleep(2)
    if(type_of_delay=="lidar_straight_move"):
        print("5 seconds straight (lidar case)")
        time.sleep(5)
    if(type_of_delay=="roll_long_turn"):
        print("Long turn delay")
        time.sleep(2.5)