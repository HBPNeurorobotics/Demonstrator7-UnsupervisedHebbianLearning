from rospy_tutorials.msg import Floats

@nrp.MapVariable("dep_controller", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotPublisher("dep_c_matrix_orig", Topic("/bf_myo_nst/dep_weights", Floats))
@nrp.Robot2Neuron()
def plot_weights (t, 
                  dep_controller, 
                  dep_c_matrix_orig):
    if dep_controller.value is not None:
        cc_curr = np.asarray(dep_controller.value.get_cmatrix())
        weights = [cc_curr[0][0], cc_curr[0][1], cc_curr[1][0], cc_curr[1][1]]
        #clientLogger.info('weights', weights)
        dep_c_matrix_orig.send_message(weights)