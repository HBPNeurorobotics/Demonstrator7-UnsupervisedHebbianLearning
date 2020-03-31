from sensor_msgs.msg import Image
import rospy
import math

# controller instance
@nrp.MapVariable("dep_controller", initial_value=None, scope=nrp.GLOBAL)

@nrp.MapVariable("throw", initial_value=0, scope=nrp.GLOBAL)

# sensing
@nrp.MapVariable("muscle_positions", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_velocities", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_efforts", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)

# actuation
@nrp.MapVariable("muscle_actuation", initial_value=[0.0,0.0], scope=nrp.GLOBAL)

# multiplier [position, velocity]
@nrp.MapVariable("multiplier", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("vel_mult", initial_value=None, scope=nrp.GLOBAL)

@nrp.Robot2Neuron()
def c_dep_controller (t,
                      dep_controller, throw,
                      muscle_positions, muscle_velocities, muscle_efforts,
                      muscle_actuation,
                      multiplier, vel_mult):

    ## ============================================== ##
    # USER PARAMETER #
    # ---------------------------- #
    motor_number = 2
    sensor_number = 2
    usedelaysensors = False
    max_force = 5000
    # ---------------------------- #
    urate = 0.01  
    synboost = 1.7              
    epsh = 0.00005          
    ## ============================================== ##

    
    
    # =======================================================================================
    ## import and initialize dep controller
    if dep_controller.value is None:
            # initialize global variables
            muscle_positions.value = [0.0] * 4
            muscle_velocities.value = [0.0] * 4
            muscle_efforts.value = [0.0] * 4
            muscle_actuation.value = [0.0] * motor_number

            # import DEP C++ controller library
            # load conflict if loaded directly from the resources path!
            sys.path.append('/home/bbpnrsoa/')
            import generalcontrolloop_wrapper

            # create and initialize new DEP controller instance
            dep_controller.value = generalcontrolloop_wrapper.PyGeneralControlLoop()
            dep_controller.value.init(sensor_number, motor_number, usedelaysensors)

            # initial parameter:
            dep_controller.value.set_initfeedbackstrength(0.0)    
            dep_controller.value.set_timedist(8)                  
            dep_controller.value.set_delay(50)                    

            # force settings:
            dep_controller.value.set_pretension(0.0)              
            dep_controller.value.set_maxforce(max_force)                
            dep_controller.value.set_springmultone(0.001)            

            dep_controller.value.set_regularization(10)         

            # runtime parameter
            dep_controller.value.set_urate(urate)                 
            dep_controller.value.set_synboost(synboost)           
            dep_controller.value.set_epsh(epsh)                   

            # add guiding to output values: guidetype 1 for sinewave on guideidx motor with guideampl and guidefreq
            dep_controller.value.set_guidetype(0)                
            dep_controller.value.set_guideidx(0)                  
            dep_controller.value.set_guideampl(0.3)               
            dep_controller.value.set_guidefreq(0.5)               


    # =======================================================================================


    if t > 2.0:       
        # =======================================================================================

        ## execute dep controller:
        if t % 3 < 0.02:
            clientLogger.info('=======Executing DEP controller======')

        dep_input = [ [multiplier.value[0] *  muscle_positions.value[1] \
                       + multiplier.value[1] * max(0,muscle_velocities.value[1])**vel_mult.value \
                       + multiplier.value[2] * max(0,muscle_velocities.value[0]) \
                       + multiplier.value[3] * muscle_actuation.value[1],\
                       multiplier.value[0] *  muscle_positions.value[0] \
                       + multiplier.value[1] *  max(0,muscle_velocities.value[0])**vel_mult.value  \
                       + multiplier.value[2] * max(0,muscle_velocities.value[1]) \
                       + multiplier.value[3]* max(0,muscle_actuation.value[0])],\
                       [0.0] * sensor_number]

        

        dep_output = dep_controller.value.run(dep_input[0], dep_input[1])

   
        # =======================================================================================
        ## actuation
        muscle_actuation.value[0:2] = [1000.0*x for x in dep_output]
       
        # ======================================================================================
