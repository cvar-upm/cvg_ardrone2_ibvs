#time stamp
float64 time

# position of the target_frame
float32 x
float32 y
float32 z

#attitude of the target_frame
float32 yaw
float32 pitch
float32 roll 

# YPR_system:
#  [*] wYvPuR
#  [*] xYyPzR
#  [*] note that: wYvPuR equals to xRyPzY
#  [*] note that: xYyPzR equals to wRvPuY
#  TODO_JP TODO_JL make sure that this variable is assigned in every part of the code
string YPR_system
# This is, homogeneous transformation of the target_frame with respect to the reference_frame
string target_frame
string reference_frame
