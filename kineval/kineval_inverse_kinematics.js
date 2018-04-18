
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) {
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

    kineval.randomizeIKtrial = function randomIKtrial () {

   // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

   // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

   // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

   // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }

}


kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

     var jac = [];
     var endeffector_world = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local);
     var delx = vector_subtraction(endeffector_target_world.position,endeffector_world);
     var delorien = vector_subtraction(endeffector_target_world.orientation,
       rotation_euler_convert_eberly(robot.joints[endeffector_joint].xform))

    if (kineval.params.ik_orientation_included){
        delx.push(delorien[0],delorien[1],delorien[2])
    }
    else { delx.push(0,0,0)}

    kineval.computeJacobian(robot.endeffector.frame,jac)
    if (kineval.params.ik_pseudoinverse == true){
      if (jac.length >= jac[0].length){
      jac = matrix_multiply(jac,numeric.inv(matrix_multiply(matrix_transpose(jac),jac)))
    }
    else { jac = matrix_multiply(numeric.inv(matrix_multiply(jac,matrix_transpose(jac))),jac)}
    }
    var deltheta = matrix_multiply(jac,matrix_transpose(delx));
    deltheta = matrix_transpose(deltheta);
    kineval.computeJcontrols(robot.endeffector.frame,jac.length-1,deltheta)
    }

    kineval.computeJacobian = function compute_jacobian(joint,J) {
      // start with endeffector joint,
      // fills in J for each parent joint of endeff
      var h = generate_homogeneous_transform;
      var endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

      if (robot.joints[joint].parent !== robot.base) {
        var par_joint = robot.joints[robot.links[robot.joints[joint].parent].parent]
        var ja = par_joint.axis;
        var world_jointorigin = matrix_multiply(par_joint.xform, [[0],[0],[0],[1]])
        var world_jointaxis = matrix_multiply(par_joint.xform, [[ja[0]],[ja[1]],[ja[2]],[1]])
        var side1 = vector_subtraction(world_jointaxis, world_jointorigin);
        var side2 = vector_subtraction(endeffector_world, world_jointorigin);

         if (par_joint.type == "prismatic"){
           side1.push(0,0,0)
           J.unshift(side1)
         }
         else{
           var jlin = vector_cross(side1,side2);
           jlin.push(side1[0],side1[1],side1[2]);
         J.unshift(jlin)
       }
         compute_jacobian(par_joint.name, J);

      }
      else {return 0};
    }

kineval.computeJcontrols = function jacontrols(joint,cidx,deltheta) {
  //start with endeff, updates joint controles, based on deltheta
  // doesn't change deltheta
  if (cidx >= 0) {
    var par_joint = robot.joints[robot.links[robot.joints[joint].parent].parent]
    par_joint.control = deltheta[0][cidx]*kineval.params.ik_steplength;
    cidx = cidx - 1;
    jacontrols(par_joint.name,cidx,deltheta);
}
else {return 0};
}

kineval.testJacobian = function test_jacobian() {
  // test function for IK functionality
  var endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
  var delx = vector_subtraction([1,2,1],endeffector_world);

 kineval.computeJacobian(robot.endeffector.frame)
 var deltheta = matrix_multiply(J,matrix_transpose(delx))
 deltheta = matrix_transpose(deltheta);
 kineval.computeJcontrols(robot.endeffector.frame,J.length-1,deltheta);
 }
