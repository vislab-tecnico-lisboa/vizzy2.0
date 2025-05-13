figure
% torso_tilt  - It's the same for both arms and the head
waist_angles	=	[-1.0]*pi/180;

left_shoulder_angles = [0.0     0.0     0.0]*pi/180;
left_arm_angles	=	[0.0     0.0     0.0     0.0     0.0]*pi/180;

cd waist_left_arm_fwd_kin;
[T_Root_0_, T_0n_leftArm, rpy_root_end_leftArm] = WaistLeftArmFwdKinVizzy(waist_angles,left_shoulder_angles,left_arm_angles, 1);
cd ..

right_shoulder_angles = [0.0     0.0     0.0]*pi/180;
right_arm_angles	=	[0.0     0.0     0.0     0.0     0.0]*pi/180;
cd 
cd waist_right_arm_fwd_kin;
[T_Root_0__, T_0n_rightArm, rpy_root_end_rightArm] = WaistRightArmFwdKinVizzy(waist_angles,right_shoulder_angles,right_arm_angles, 1);
cd ..
 
%% neck_pan   neck_tilt    eyes_tilt    version    vergence
neck_eyes_angles	=	[0.0 0.0 0.0 0.0 0.0]*pi/180;

cd waist_head_fwd_kin;
[T_Root_0, T_0n_leftEye, Tp_0n_rightEye, Tp_0n_inertialSensor, rpy_root_end_leftEye, rpy_root_end_rightEye] = WaistHeadFwdKinVizzy(waist_angles,neck_eyes_angles, 1);
cd ..

%% Right Hand

cd right_wrist_fingers_fwd_kin;
root_to_wrist_mat=T_Root_0__*T_0n_rightArm;
thumb_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristThumbFwdKinVizzy(thumb_angles,root_to_wrist_mat,1)
index_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristIndexFwdKinVizzy(index_angles,root_to_wrist_mat,1)
middle_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristMiddleFwdKinVizzy(middle_angles,root_to_wrist_mat,1)
ring_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristRingFwdKinVizzy(ring_angles,root_to_wrist_mat,1)
cd ..