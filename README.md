# Demonstrator7-UnsupervisedHebbianLearning

This Repository contains the NRP experiment files for Demonstrator 7, ball mass estimation and throwing, utilizing unuspervised Hebbian Learning.


## Model Description
A one-layer fully connected feedforward neural
network learns the sensory-motor mapping between
muscle states (length and force) and muscle
commands with a variation of Differential Hebbian
Learning as proposed in [1]. After a fast self-
exploration phase, the network‘s behavior emerges to
a periodic attractor motion. Since the network input is
directly linked to the object's mass via a sensitivity to
muscle forces, a highly efficient periodic motion in
resonance with the overall system is generated (arm
and attached ball with variable mass).

After the initial learning phase the evolved senorimotor
network is exploited for a ball throwing task. Hereby,
only muscle afferent feedback is adapted in order to reach
a desired goal. The feedback strength is adapted by means
of a PID to controller to reach a predefined swinging amplitude.
The final velocity speedup that relates to the throwing 
frequency is then adapted according to successull/non-successfull
goal reaching until an optimal control setup is found.

[1] Der, R., & Martius, G. (2016). Self-organized control for musculo-
skeletal robots. 1–11. http://arxiv.org/abs/1602.02990


## User Documentation
This repository contains the relevant experiment files to run the 
experiment in the NRP. After installing the model and environment of 
the common Demonstrator 7 setup, located in https://gitlab.com/neurocomputing/hbp/sga2/c2561/model
this experiment folder may be copied into you .opt/nrpStorage and
after storage update the experiment can be run.
The experiment procedure is implemented as a State Machine, so that
initial Hebbian Learning, swing up phase, throwing, success evaluation and 
ball pick up are executed endlessly in a repeating manner. The desired ball 
and goal can be adapted in the beginning of TransferFunction "StateMachine"