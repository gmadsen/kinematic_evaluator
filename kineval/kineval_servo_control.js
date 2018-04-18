
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/


kineval.setpointDanceSequence = function execute_setpoints() {
    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return;

    var index = kineval.params.dance_pose_index;
    var error = 0
    // STENCIL: implement FSM to cycle through dance pose setpoints
    for (joint in robot.joints) {
          kineval.params.setpoint_target[joint] = kineval.setpoints[kineval.params.dance_sequence_index[index]][joint];  // current setpoint target
          error += kineval.params.setpoint_target[joint] - robot.joints[joint].angle
    }

    if(Math.abs(error) < .05) {
      kineval.params.dance_pose_index += 1;
     kineval.params.dance_pose_index = kineval.params.dance_pose_index % kineval.params.dance_sequence_index.length;
   }


}

kineval.setpointClockMovement = function execute_clock() {


    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return;

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return;
    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (joint in robot.joints) {
      var error =  kineval.params.setpoint_target[joint] - robot.joints[joint].angle
      robot.joints[joint].control = robot.joints[joint].servo.p_gain * error
    }
}
