from roboy_simulation_msgs.msg import Tendon

@nrp.MapVariable("muscle_efforts", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_positions", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("muscle_velocities", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("initial_muscle_length", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("last_muscle_length", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)

@nrp.MapRobotSubscriber('muscle_states_msg', Topic('/tendon_states', Tendon))
@nrp.Robot2Neuron()
def a_muscle_feedback (t, muscle_states_msg, muscle_velocities, muscle_positions, muscle_efforts, initial_muscle_length, last_muscle_length):
    import math
    import numpy as np
       

    # get initial length
    if isinstance(initial_muscle_length.value, type(None)):
        initial_muscle_length.value = [muscle_states_msg.value.l[0], muscle_states_msg.value.l[1],  muscle_states_msg.value.l[2],  muscle_states_msg.value.l[3]]
        #clientLogger.info('save initial muscle length', initial_muscle_length.value)

    ## muscle length:
    muscle_positions.value = [value - initial_muscle_length.value[idx] for idx, value in enumerate([muscle_states_msg.value.l[0], muscle_states_msg.value.l[1],  muscle_states_msg.value.l[2],  muscle_states_msg.value.l[3]])]
    
    ## muscle velocity:
    muscle_velocities.value = np.subtract(muscle_positions.value, last_muscle_length.value) * 50 # per second
    last_muscle_length.value = muscle_positions.value
    


