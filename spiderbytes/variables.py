
dog_joints = [
    "hip_joint_fl","knee_joint_fl","leg_joint_fl",
    "hip_joint_fr","knee_joint_fr","leg_joint_fr",
    "hip_joint_rl","knee_joint_rl","leg_joint_rl",
    "hip_joint_rr","knee_joint_rr","leg_joint_rr",
]

spiderbytes_joints = [
    "joint_rotation_1","joint_limb_1","joint_leg_1",
    "joint_rotation_2","joint_limb_2","joint_leg_2",
    "joint_rotation_3","joint_limb_3","joint_leg_3",
    "joint_rotation_4","joint_limb_4","joint_leg_4",
]
# in dog robot, the rotation sign of each joint. for example -1 * radian on hip_fl makes it move forward otherwise moves backward
dog_joints_rotation_signs = {
    'hip_joint_fl': -1,
    'knee_joint_fl': -1,
    'leg_joint_fl': -1,
    'hip_joint_fr': +1,
    'knee_joint_fr': -1,
    'leg_joint_fr': -1,
    'hip_joint_rl': -1,
    'knee_joint_rl': -1,
    'leg_joint_rl': -1,
    'hip_joint_rr': +1,
    'knee_joint_rr': -1,
    'leg_joint_rr': -1,
}

# follow these signs on dog robot to convert it into spider pose. should apply these signs on top of above dog_joints_rotation_signs
spider_pose_joint_signs = {
    'hip_joint_fl': +1,
    'knee_joint_fl': +1,
    'leg_joint_fl': +1,
    'hip_joint_fr': +1,
    'knee_joint_fr': +1,
    'leg_joint_fr': +1,
    'hip_joint_rl': -1,
    'knee_joint_rl': +1,
    'leg_joint_rl': +1,
    'hip_joint_rr': -1,
    'knee_joint_rr': +1,
    'leg_joint_rr': +1,
}



spider_home_angle_from_dog_pose = {
        'hip': (3.14 / 2) / 2,
        'knee': 0.3, #(3.14 / 2) / 3,
        'leg': 0.0
    }
spider_crouch_position = {
    'hip': spider_home_angle_from_dog_pose['hip'],
    'knee': (3.14 / 2) / 4,
    'leg': spider_home_angle_from_dog_pose['leg']
}

joint_groups_spider = [
    
    [0, 1, 2],
    [9, 10, 11],
    [3, 4, 5],
    [6, 7, 8],
]
joint_groups_dog = [
    
    [0, 1, 2],
    
    [3, 4, 5],
    [6, 7, 8],
    [9, 10, 11],
]
hip_joints = {
    0: 'hip_joint_fl',
    3: 'hip_joint_fr',
    6: 'hip_joint_rl',
    9: 'hip_joint_rr',
}

knee_joints = {
    1: 'knee_joint_fl',
    4: 'knee_joint_fr',
    7: 'knee_joint_rl',
    10: 'knee_joint_rr',
}
leg_joints = {
    2: 'leg_joint_fl',
    5: 'leg_joint_fr',
    8: 'leg_joint_rl',
    11: 'leg_joint_rr',
}