string grasp_id
string hand_id
string object_id
ManipulatorState pregrasp
ManipulatorState grasp
ManipulatorState postgrasp
# The torque to apply to each joint once the manipulator have reached the grasp posture
# Negative value would imply to have the specified torque value applied in the outward direction
TorqueIntensity torque_intensity
# The estimated probability of success for this grasp, or some other
# measure of how "good" it is.
float64 grasp_quality
