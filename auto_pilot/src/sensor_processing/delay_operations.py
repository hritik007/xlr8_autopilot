import time

def delay_for_time(type_of_delay):
    print("")
    if(type_of_delay=="avoid_affinity_error"):
        print("*Moving rover 4 seconds straight to avoid affinity error*")
        time.sleep(4)
    if(type_of_delay=="rotation_safety"):
        print("*Rotation safety delay for 2 seconds*")
        time.sleep(2)
    if(type_of_delay=="marker_forward_fashion"):
        print("*Moving forward for 5 seconds and for re-searching*")
        time.sleep(5)