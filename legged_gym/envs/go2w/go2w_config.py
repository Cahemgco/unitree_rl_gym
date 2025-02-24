from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class GO2WRoughCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 6000
        num_actions = 16
        num_observations = 77 + 187

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.5] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.0,   # [rad]
            'RL_hip_joint': 0.0,   # [rad]
            'FR_hip_joint': 0.0 ,  # [rad]
            'RR_hip_joint': 0.0,   # [rad]

            'FL_thigh_joint': 0.67,     # [rad]
            'RL_thigh_joint': 0.67,   # [rad]
            'FR_thigh_joint': 0.67,     # [rad]
            'RR_thigh_joint': 0.67,   # [rad]

            'FL_calf_joint': -1.3,   # [rad]
            'RL_calf_joint': -1.3,    # [rad]
            'FR_calf_joint': -1.3,  # [rad]
            'RR_calf_joint': -1.3,    # [rad]
            
            'FL_foot_joint':0.0,
            'RL_foot_joint':0.0,
            'FR_foot_joint':0.0,
            'RR_foot_joint':0.0,

        }
        init_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.0,   # [rad]
            'RL_hip_joint': 0.0,   # [rad]
            'FR_hip_joint': 0.0 ,  # [rad]
            'RR_hip_joint': 0.0,   # [rad]

            'FL_thigh_joint': 0.67,     # [rad]
            'RL_thigh_joint': 0.67,   # [rad]
            'FR_thigh_joint': 0.67,     # [rad]
            'RR_thigh_joint': 0.67,   # [rad]

            'FL_calf_joint': -1.3,   # [rad]
            'RL_calf_joint': -1.3,    # [rad]
            'FR_calf_joint': -1.3,  # [rad]
            'RR_calf_joint': -1.3,    # [rad]

            'FL_foot_joint':0.0,
            'RL_foot_joint':0.0,
            'FR_foot_joint':0.0,
            'RR_foot_joint':0.0,
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'hip_joint': 50.,'thigh_joint': 50.,'calf_joint': 50.,"foot_joint":20}  # [N*m/rad]
        damping = {'hip_joint': 1,'thigh_joint': 1,'calf_joint': 1,"foot_joint":0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        wheel_speed = 1
        
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2w/urdf/go2w.urdf'
        name = "go2w"
        foot_name = "foot"
        wheel_name =["foot"]
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = []
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter "base","calf","hip","thigh"
        replace_cylinder_with_capsule = False
        flip_visual_attachments = True
  
    class rewards( LeggedRobotCfg.rewards ):
        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.4 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 0.9 # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 1.
        base_height_target = 0.34
        max_contact_force = 100. # forces above this value are penalized
       
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = -0.8
            tracking_lin_vel = 3.0
            tracking_ang_vel = 1.5
            lin_vel_z = -0.1
            ang_vel_xy = -0.05
            orientation = -2
            torques = -0.0001
            dof_vel = -1e-7
            dof_acc = -1e-7
            base_height = -0.5
            feet_air_time =  0
            collision = -0.1
            feet_stumble = -0.1
            action_rate = -0.0002
            stand_still = -0.01
            dof_pos_limits = -0.9
            # arm_pos = -0.
            hip_action_l2 = -0.1

class GO2WRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.03
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_go2w'
        num_steps_per_env = 48 # per iteration
        max_iterations = 3000
        # load_run = "/home/zifanw/rl_robot/legged-robots-manipulation/loco_manipulation_gym/logs/Go2w/Jan21_02-18-32_"
        # checkpoint =1550
  
