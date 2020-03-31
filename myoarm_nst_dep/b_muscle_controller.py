from std_msgs.msg import Float64
from roboy_middleware_msgs.msg import MotorCommand

# Actuation Variable
@nrp.MapVariable("muscle_actuation", initial_value=[0.0,0.0,0.0,0.0], scope=nrp.GLOBAL)
@nrp.MapVariable("lift", initial_value=0, scope=nrp.GLOBAL)
        

# ROS subscriber
@nrp.MapRobotPublisher("muscles", Topic("/roboy/middleware/MotorCommand", MotorCommand))

@nrp.Robot2Neuron()
def b_muscle_controller (t, muscle_actuation, lift, muscles):
    from roboy_middleware_msgs.msg import MotorCommand
    
    if lift.value == 1:
           muscle_actuation.value = [0.0, 40.0, 3.0, 3.0]
           clientLogger.info('lift the arm')
    
    msg = MotorCommand()   
    msg.id = 0
    
    if t > 0.2:
        msg.motors = range(4)
            
        msg.set_points = muscle_actuation.value
        
        #clientLogger.info('send command')
        muscles.send_message(msg)

            

        
        

