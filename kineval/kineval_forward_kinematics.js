/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//Added last element array method
if (!Array.prototype.last) {
  Array.prototype.last = function() {
    return this[this.length - 1];
  };
};


/////////////////////////////////////////////////////////////////////////
/////////////////////FK Implementation
////////////////////////////////////////////////////////////////////////

kineval.robotForwardKinematics = function robotForwardKinematics() {

  kineval.buildFKTransforms = function notsure() {
    return;
  }

  // user interface needs the heading (z-axis) and lateral (x-axis) directions
  //   of robot base in world coordinates stored as 4x1 matrices in
  //   global variables "robot_heading" and "robot_lateral"

  //
  // if geometries are imported and using ROS coordinates (e.g., fetch),
  //   coordinate conversion is needed for kineval/threejs coordinates:
  //
  if (robot.links_geom_imported) {
    var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI / 2), generate_rotation_matrix_X(-Math.PI / 2));
  } else {
    offset_xform = generate_identity(4)
  }

  if (typeof kineval.buildFKTransforms === 'undefined') {
    textbar.innerHTML = "forward kinematics not implemented";
    return;
  }

  var h = generate_homogeneous_transform;
  var matrix_stack = generate_identity(4);
  traverseFKBase();


  //     traverseFKBase
  function traverseFKBase() {
    matrix_stack.push(matrix_multiply(h(robot.origin), offset_xform))

    robot.links[robot.base].xform = matrix_copy(matrix_stack.last());
    robot.links[robot.base].children.forEach(function(joint) {
      traverseFKJoint(robot.joints[joint]);
    });
    matrix_stack.pop()
  }

  //     traverseFKJoint
  function traverseFKJoint(joint) {
    //var axis = joint.axis
    var q = quaternion_from_axisangle(joint.axis, joint.angle)
    var q_unit = quaternion_normalize(q)
    var q_rot = quaternion_to_rotation_matrix(q_unit)

    if (joint.type == "prismatic") {
      var pris_tran = [0, 0, 0]
      pris_tran[0] = joint.axis[0] * joint.angle;
      pris_tran[1] = joint.axis[1] * joint.angle;
      pris_tran[2] = joint.axis[2] * joint.angle;
      q_rot = generate_translation_matrix(pris_tran)
    }

    var system_rot = matrix_multiply(matrix_stack.last(), h(joint.origin))
    //joint.xform = matrix_copy(system_rot);
    matrix_stack.push(matrix_multiply(system_rot, q_rot));
    joint.xform = matrix_copy(matrix_stack.last());
    traverseFKLink(robot.links[joint.child]);
  }

  //     traverseFKLink
  function traverseFKLink(link) {
    link.xform = matrix_copy(matrix_stack.last());
    if (link.children.length == 0) {
      matrix_stack.pop();
      return;
    } else {
      link.children.forEach(function(joint) {
        traverseFKJoint(robot.joints[joint]);
      });
    }
    matrix_stack.pop();
  }


  if (robot.links_geom_imported) {
    robot_heading = matrix_multiply(robot.links[robot.base].xform, [
      [1],
      [0],
      [1],
      [1]
    ]);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform, [
      [0],
      [1],
      [0],
      [1]
    ]);
  } else {
    robot_heading = matrix_multiply(robot.links[robot.base].xform, [
      [0],
      [0],
      [1],
      [1]
    ]);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform, [
      [1],
      [0],
      [0],
      [1]
    ]);
  }
}
