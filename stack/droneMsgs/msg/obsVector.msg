#time stamp
float64 time

#vector of landmarks
Observation3D[] obs

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

