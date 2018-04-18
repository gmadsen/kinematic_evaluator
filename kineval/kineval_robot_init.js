/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics
     in HTML5/JavaScript and threejs

     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

*/

kineval.initRobot = function initRobot() {

  // ASSUME: robot kinematics are described separate js file (eg., "robot_urdf_example.js")

  // initialize and create threejs mesh objects for robot links
  kineval.initRobotLinks();

  // initialize robot joints and create threejs mesh objects for robot joints and form kinematic hiearchy
  kineval.initRobotJoints();

  // initialize robot collision state
  robot.collision = false;

}

kineval.initRobotLinks = function initRobotLinks() {

  for (x in robot.links) {
    robot.links[x].name = x;
    robot.links[x].parent = "";
    robot.links[x].children = [];
  }

  // initialize controls for robot base link
  robot.control = {
    xyz: [0, 0, 0],
    rpy: [0, 0, 0]
  };
}

kineval.initRobotJoints = function initRobotJoints() {
  // build kinematic hierarchy by looping over each joint in the robot
  //   (object fields can be index through array-style indices, object[field] = property)
  //   and insert threejs scene graph (each joint and link are directly connect to scene root)
  // NOTE: kinematic hierarchy is maintained independently by this code, not threejs

  var x, tempmat;

  q_names = {}; // store mapping between joint names and q DOFs
  q_index = []; // store mapping between joint names and q DOFs
  var q_robot_config = [
    robot.origin.xyz[0],
    robot.origin.xyz[1],
    robot.origin.xyz[2],
    robot.origin.rpy[0],
    robot.origin.rpy[1],
    robot.origin.rpy[2]
  ];

  for (x in robot.joints) {

    // store mapping between joint names and q DOFs

    q_names[x] = q_robot_config.length;
    q_index[q_robot_config.length] = x;
    q_robot_config = q_robot_config.concat(robot.joints[x].angle);

    // give the joint its name as an id
    robot.joints[x].name = x;

    // initialize joint angle value and control input value
    robot.joints[x].angle = 0;
    robot.joints[x].control = 0;
    robot.joints[x].servo = {};
    // STENCIL: set appropriate servo gains for arm setpoint control
    robot.joints[x].servo.p_gain = .1;
    robot.joints[x].servo.p_desired = 0;
    robot.joints[x].servo.d_gain = 0;

    // STENCIL: complete kinematic hierarchy of robot for convenience.
    //   robot description only specifies parent and child links for joints.
    //   additionally specify parent and child joints for each link
    for (var y in robot.links) {
      if (robot.joints[x].parent == robot.links[y].name) {
        robot.links[y].children.push(robot.joints[x].name)
      }
      if (robot.joints[x].child == robot.links[y].name) {
        robot.links[y].parent = robot.joints[x].name
      }
    }

  }

}
