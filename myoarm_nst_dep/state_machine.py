# ros services
import rospy
import copy
from simple_pid import PID

from gazebo_msgs.srv import SpawnEntity, SetLinkProperties
from std_srvs.srv import Trigger
from rospy_tutorials.msg import Floats
from roboy_cognition_msgs.srv import Talk

spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_entity", SpawnEntity)
detach_joint = rospy.ServiceProxy("/roboy/simulation/joint/detach", Trigger)
attach_joint = rospy.ServiceProxy("/roboy/simulation/joint/attach", Talk)
set_prop = rospy.ServiceProxy("/gazebo/set_link_properties", SetLinkProperties)

@nrp.MapVariable("init", initial_value=False)

@nrp.MapVariable("spawn_sdf", initial_value=spawn_model)
@nrp.MapVariable("throw_ball", initial_value=detach_joint)
@nrp.MapVariable("new_ball", initial_value=attach_joint)
@nrp.MapVariable("change_mass", initial_value=set_prop)
@nrp.MapVariable("ball_type", initial_value=0)

# state machine
@nrp.MapVariable("control_state", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapVariable("keep_state", initial_value=0, scope=nrp.GLOBAL)

# synapse modulation
@nrp.MapVariable("multiplier", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("multiplier_safe", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("vel_mult", initial_value=1.0, scope=nrp.GLOBAL)
@nrp.MapVariable("throw_mult", initial_value=1.0, scope=nrp.GLOBAL)

@nrp.MapVariable("speed", initial_value=False, scope=nrp.GLOBAL)
@nrp.MapRobotPublisher("ros_multiplier", Topic("/bf_myo_nst/multiplier", Floats))

# success
@nrp.MapVariable("goal_distance", initial_value=-1.0, scope=nrp.GLOBAL)
@nrp.MapRobotPublisher("ros_goal_distance", Topic("/bf_myo_nst/goal_distance", std_msgs.msg.Float64))
@nrp.MapVariable("success", initial_value=[10.0, 10.0, 10.0], scope=nrp.GLOBAL)
@nrp.MapRobotPublisher("ros_success", Topic("/bf_myo_nst/success", Floats))
@nrp.MapVariable("throw_count", initial_value=0, scope=nrp.GLOBAL)

@nrp.MapVariable("lift", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapVariable("throw", initial_value=0, scope=nrp.GLOBAL)

@nrp.MapVariable("pid", initial_value=None, scope=nrp.GLOBAL)

# robot state analyzer
@nrp.MapVariable("max_amp", initial_value=[0.0, 0.0, 0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("amp_history", initial_value=200*[10.0], scope=nrp.GLOBAL)
@nrp.MapVariable("max_amp_prev", initial_value=0.0, scope=nrp.GLOBAL)

@nrp.MapVariable("goal_amp", initial_value=0.0, scope=nrp.GLOBAL)
@nrp.MapRobotPublisher("ros_goal_amp", Topic("/bf_myo_nst/goal_amp", Floats))
@nrp.MapVariable("amp_history", initial_value=4*[0.0], scope=nrp.GLOBAL)

@nrp.MapVariable("periode_time_act", initial_value=[0.0, 0.0], scope=nrp.GLOBAL)


# sensing
@nrp.MapVariable("muscle_positions", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_velocities", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_efforts", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_velocities_history", initial_value=50*[10.0], scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("link_states", Topic('/gazebo/link_states', gazebo_msgs.msg.LinkStates))
@nrp.MapRobotSubscriber("joint_states", Topic('/joint_states', sensor_msgs.msg.JointState))

# actuation
@nrp.MapVariable("muscle_actuation", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)

# controller instance
@nrp.MapVariable("dep_controller", initial_value=None, scope=nrp.GLOBAL)


@nrp.Robot2Neuron()
def state_machine (t, init,
                   spawn_sdf, throw_ball, new_ball, change_mass, ball_type,
                   control_state, keep_state, 
                   multiplier, lift, ros_multiplier, vel_mult, multiplier_safe, speed, throw, throw_mult,
                   goal_distance, ros_goal_distance, success, ros_success, throw_count,
                   max_amp, max_amp_prev, amp_history, goal_amp, ros_goal_amp, periode_time_act, amp_history,
                   muscle_actuation, muscle_positions, muscle_velocities, muscle_efforts, muscle_velocities_history,
                   link_states, joint_states, pid,
                   dep_controller):

    ### ======================================
    ## DEFINE GOAL
    goal_distance.value = 0.0
    goal_amp.value = 0.5
    
    ball_type.value = 0
    ball_box = ['2000g', '200g','5000g']
    ball_name = 'robot::cricket_ball_' + ball_box[ball_type.value]
    

    initial_multiplier = [1.0,0.1,0.0,0.0]
    initial_vel_mult = 0.6
    ### ======================================
    
    ### ======================================
    # RUNTIME EXPERIMENTS
    # control_state.value = 
    # vel_mult.value = 0.6 
    # goal_distance.value = -1.
    # multiplier.value[2]= 0.0   0
    # clientLogger.info(multiplier.value + [vel_mult.value])
    # clientLogger.info(multiplier_safe.value)
    # clientLogger.info(success.value)
    # pid.value.tunings = 0.01,1.0,0.5
    # pid.value = PID(1.0, 0.1, 0.05)
    # pid.value.tunings = 5.0,10.5,0.5
    # multiplier.value[1] = 15.0
    # pid.value.tunings = 0.0,0.3,0.5
    # pid.value.output_limits = (0.0, 5.0)
    # vel_mult.value = 0.6
    # throw_ball.value = 10.0
    ### ======================================

    ### ======================================
    ## STATE MACHINE
    control_dict = {0: 'DEP antagonist learning',
                    1: 'Lift arm and swing',
                    2: 'throw ball',
                    3: 'stop arm',
                    4: 'sucess evaluation',
                    5: 'new ball'}
    clientLogger.advertise('-- Control State --\n', control_dict[control_state.value], '\n', keep_state.value)
    import copy
    from simple_pid import PID
    ### ======================================
    
    ### ======================================
    # INITIALIZATION
    if init.value == False:
        multiplier.value = initial_multiplier
        muscle_actuation.value = [0.0, 0.0, 0.0, 0.0]
        ball_state_index = link_states.value.name.index(ball_name)
        ball_dist = link_states.value.pose[ball_state_index].position.y - goal_distance.value
        success.value = 3 * [ball_dist]
        vel_mult.value = initial_vel_mult

        
        pid.value = PID(0.0005,0.015,0.0005)
        pid.value.tunings = 0.1,0.8,0.01
        pid.value.setpoint = goal_amp.value
        pid.value.output_limits = (0.001, 0.5)
        init.value = True
    ### ======================================

    elif t > 2.0:
        #multiplier.value = initial_multiplier
         # Logging / ROS publishing
        ros_multiplier.send_message(multiplier.value + [vel_mult.value])
        ros_goal_distance.send_message(goal_distance.value)
        ros_goal_amp.send_message([-goal_amp.value, goal_amp.value])
        
    ### ======================================
        if control_state.value == 0:
            # DEP Learning
            keep_state.value = keep_state.value + 1
            multiplier.value = [1.0,0.1,0.0,0.0]
            
            if keep_state.value < 50:
                # lift arm
                lift.value = 1
                clientLogger.info('lift arm')
                
            elif 50 < keep_state.value < 700:
                lift.value = 0
                clientLogger.info('learning')
                                 
            elif keep_state.value > 700:       
                # turn off dep learning
                clientLogger.info('turn off learning')
                dep_controller.value.set_urate(0.0)
                control_state.value = 3
                keep_state.value = 0
                multiplier.value = initial_multiplier
            
        
    ### ======================================
        elif control_state.value == 1:
            # swing up until amplitude
            lift.value = 0
            
            #clientLogger.info('update', vel_mult.value, muscle_positions.value[0], muscle_positions.value[1],  max_amp.value, goal_amp.value)
        
           
            if keep_state.value < 3:
                # trigger arm
                multiplier.value = initial_multiplier
                lift.value = 1
                max_amp.value[1] = t
                keep_state.value = keep_state.value + 1
                pid.value.set_auto_mode(True, last_output=0.5)
               
            
            else:
                if abs(joint_states.value.velocity[2]) < 0.05:
                    # save new max amplitude
                       
                    new_amp = abs(joint_states.value.position[2])
                    max_amp.value[0] = new_amp
                    max_amp.value[2] = t
                    clientLogger.info('set amplitude setpoint ', pid.value.setpoint, 'act ',max_amp.value[0])
                    multiplier.value[1] = pid.value(max_amp.value[0])
                    
                    amp_history.value = np.roll(amp_history.value, -1)
                    amp_history.value[-1] = max_amp.value[0]
            
                if abs(joint_states.value.position[2]) < 0.01:
                    # save new perriode time
                    new_periode = t - periode_time_act.value[1]
                    
                    periode_time_act.value[1] = new_periode
                    periode_time_act.value[0] = t
  

                    
            clientLogger.info('histor ', amp_history.value, np.sum(amp_history.value) / 4)
            if goal_amp.value - (0.01*goal_amp.value) < (np.sum(amp_history.value) / 4) < goal_amp.value + (0.01 *goal_amp.value):
                # max amplitude stable for some time
                clientLogger.info('goal amplitude reached', t, max_amp.value)

                control_state.value = 2
                keep_state.value = 0
                throw_count.value = throw_count.value + 1 
                multiplier_safe.value = None
                vel_mult.value = initial_vel_mult
                amp_history.value = 4 * [0.0]
                

        ### ======================================

        elif control_state.value == 2:
            # set goal velocity amplification and throw

            # clientLogger.info('diff is ',muscle_positions.value[0]-max_amp.value[0])
            
            
            if (joint_states.value.velocity[2]) > 0.001 \
                and joint_states.value.position[2] < 0.0 \
                and keep_state.value == 0:
                    keep_state.value = 1
                    #vel_mult.value = 0.0001
                    #multiplier.value[0] = 0.01
                    multiplier.value[1] = throw_mult.value
                    #dep_controller.value.set_synboost(2000.0)
            
            if (joint_states.value.position[2]) >  goal_amp.value * 0.8\
                and joint_states.value.position[2] > 0.0 \
                and keep_state.value == 1:
                # throw ball
                throw_ball.value()
                control_state.value = 3
       
                # reset velocity multiplier
                multiplier.value = initial_multiplier
                vel_mult.value = initial_vel_mult
                dep_controller.value.set_synboost(1.0)
    ### ======================================
  

        elif control_state.value == 3:
            # stop by inverting velocity input
            multiplier.value[0:4] = [0.0, 0.0, 1.0, 0.0]
            
            # track velocities
            muscle_velocities_history.value = np.roll(muscle_velocities_history.value, -1)
            muscle_velocities_history.value[-1] = np.sum(np.absolute(muscle_velocities.value))

            if np.sum(muscle_velocities_history.value) < 0.05:
                # motion stopped
                control_state.value = 4
                multiplier.value = initial_multiplier
                multiplier_safe.value = None
                
                
    ### ======================================
        elif control_state.value == 4:
            # success evaluation
          
            ball_state_index = link_states.value.name.index(ball_name)
            ball_vel = link_states.value.twist[ball_state_index].linear.y
            
            if abs(ball_vel) > 0.001:  # todo wait until ball does not move, or hits ground
                clientLogger.info('wait for ball to stop rolling')
                
            else:
                ball_pos = link_states.value.pose[ball_state_index].position
                actual_dist = ball_pos.y - goal_distance.value
                success.value[ball_type.value] = actual_dist
                clientLogger.info('ball pos y ', ball_pos.y, 'actual distance ', actual_dist, 'goal dist', goal_distance.value)

                if abs(actual_dist) < 0.1:
                    clientLogger.info('ball in box one -- success')
                elif actual_dist < 0.0:
                    clientLogger.info('ball too near')
                    throw_mult.value = min(1000, throw_mult.value \
                    + 10 * abs(actual_dist))
                elif actual_dist > 0.0:
                    clientLogger.info('ball too far')
                    throw_mult.value = max(1.0, throw_mult.value \
                    - 10 * abs(actual_dist))

                ros_success.send_message(success.value + [throw_count.value] + [0.0])
                clientLogger.info('success ', success.value)
                control_state.value = 5
                
                
    ### ======================================
        elif control_state.value == 5:
            # take up new ball
            new_ball.value(ball_name)

            control_state.value = 1
            keep_state.value = 0



