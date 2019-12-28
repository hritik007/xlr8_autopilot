import numpy as np

arr=np.zeros(360)

count=0
len=360
for i in range (0,360):
    arr[i]=arr[i]+count
    count=count+1

arr_330_to_360=arr[320:360]
arr_0_to_30   =arr[0:40]

lidar_ranges_80=np.concatenate((arr_330_to_360,arr_0_to_30 ), axis=0)
print(lidar_ranges_80)