/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
  // test whether geometry of current configuration of robot is in collision with planning world

  // form configuration from base location and joint angles
  var q_robot_config = [
    robot.origin.xyz[0],
    robot.origin.xyz[1],
    robot.origin.xyz[2],
    robot.origin.rpy[0],
    robot.origin.rpy[1],
    robot.origin.rpy[2]
  ];

  for (i = 6; i < q_index.length; i++) {
    q_robot_config[i] = robot.joints[q_index[i]].angle;
  }

  // test for collision and change base color based on the result
  collision_result = kineval.poseIsCollision(q_robot_config);

  robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
  // perform collision test of robot geometry against planning world
  // OUTPUT: return either False, or STRING of link in collision

  // test base origin (not extents) against world boundary extents
  if ((q[0] < robot_boundary[0][0]) || (q[0] > robot_boundary[1][0]) || (q[2] < robot_boundary[0][2]) || (q[2] > robot_boundary[1][2])) {
    return robot.base;
  }

  // create collision FK
  collision_FK(q)
  // traverse robot kinematics to test each body for collision
  for (link in robot.links) {
    var linkage = robot.links[link];
    //return traverse_collision_forward_kinematics_link(robot.links[robot.base],robot.links[robot.base].xform,q)
    var coll = traverse_collision_link(linkage, linkage.xform_coll)
    if (coll != false) {
      return coll
    }
  }
  return false

}



function traverse_collision_link(link, mstack) {

  //TODO need proper matrix chain, since base moves

  // test collision by transforming obstacles in world to link space
  /*
      mstack_inv = matrix_invert_affine(mstack);
  */


  var mstack_inv = numeric.inv(mstack);

  var i;
  var j;

  // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
  for (j = 0; j < robot_obstacles.length; j++) {
    //for (j in robot_obstacles) {

    var obstacle_local = matrix_multiply(mstack_inv, robot_obstacles[j].location);

    // assume link is in collision as default
    var in_collision = true;

    // if obstacle lies outside the link extents along any dimension, no collision is detected
    if (
      (obstacle_local[0][0] < (link.bbox.min.x - robot_obstacles[j].radius)) ||
      (obstacle_local[0][0] > (link.bbox.max.x + robot_obstacles[j].radius))
    )
      in_collision = false;
    if (
      (obstacle_local[1][0] < (link.bbox.min.y - robot_obstacles[j].radius)) ||
      (obstacle_local[1][0] > (link.bbox.max.y + robot_obstacles[j].radius))
    )
      in_collision = false;
    if (
      (obstacle_local[2][0] < (link.bbox.min.z - robot_obstacles[j].radius)) ||
      (obstacle_local[2][0] > (link.bbox.max.z + robot_obstacles[j].radius))
    )
      in_collision = false;

    // if obstacle lies within link extents along all dimensions, a collision is detected and return true
    if (in_collision)
      return link.name;
  }
  return false
}

function collision_FK(q_config) {
  if (robot.links_geom_imported) {
    var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI / 2), generate_rotation_matrix_X(-Math.PI / 2));
  } else {
    var offset_xform = generate_identity(4)
  }

  var h = generate_homogeneous_transform;
  var matrix_stack = generate_identity(4);
  traverseFKBasecoll(matrix_stack, q_config);

  //     traverseFKBase
  function traverseFKBasecoll(matrix_stack, q_config) {
    var q_origin = {};
    q_origin.xyz = [q_config[0], q_config[1], q_config[2]];
    q_origin.rpy = [q_config[3], q_config[4], q_config[5]];
    matrix_stack.push(matrix_multiply(h(q_origin), offset_xform))

    robot.links[robot.base].xform_coll = matrix_copy(matrix_stack.last());
    robot.links[robot.base].children.forEach(function(joint) {
      traverseFKJointcoll(robot.joints[joint], matrix_stack, q_config);
    });
    matrix_stack.pop()
  }

  //     traverseFKJoint
  function traverseFKJointcoll(joint, matrix_stack, q_config) {
    //var axis = joint.axis
    var q = quaternion_from_axisangle(joint.axis, q_config[q_names[joint.name]]);
    var q_unit = quaternion_normalize(q);
    var q_rot = quaternion_to_rotation_matrix(q_unit);

    if (joint.type == "prismatic") {
      var pris_tran = [0, 0, 0]
      pris_tran[0] = joint.axis[0] * q_config[q_names[joint.name]];
      pris_tran[1] = joint.axis[1] * q_config[q_names[joint.name]];
      pris_tran[2] = joint.axis[2] * q_config[q_names[joint.name]];
      q_rot = generate_translation_matrix(pris_tran)
    }

    var system_rot = matrix_multiply(matrix_stack.last(), h(joint.origin))
    matrix_stack.push(matrix_multiply(system_rot, q_rot));
    joint.xform_coll = matrix_copy(matrix_stack.last());
    traverseFKLinkcoll(robot.links[joint.child], matrix_stack);
  }

  //     traverseFKLink
  function traverseFKLinkcoll(link, matrix_stack) {
    link.xform_coll = matrix_copy(matrix_stack.last());
    if (link.children.length == 0) {
      matrix_stack.pop();
      return;
    } else {
      link.children.forEach(function(joint) {
        traverseFKJointcoll(robot.joints[joint], matrix_stack, q_config);
      });
    }
    matrix_stack.pop();
  }
}
