/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics
        in HTML5/JavaScript and threejs

    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT:
// compute motion plan and output into robot_path array
// elements of robot_path are vertices based on tree structure in tree_init()
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

  // exit function if RRT is not implemented
  // exit if more than one or no search_alg are chosen
  //   start by uncommenting kineval.robotRRTPlannerInit
  if (typeof kineval.robotRRTPlannerInit === 'undefined') return;
  if ( (!kineval.params.planner_rrt_star && !kineval.params.planner_rrt_connect) ||
   (kineval.params.planner_rrt_star && kineval.params.planner_rrt_connect) ) {return};

  if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
    kineval.robotRRTPlannerInit();
    kineval.params.generating_motion_plan = true;
    kineval.params.update_motion_plan = false;
    kineval.params.planner_state = "initializing";
  }
  if (kineval.params.generating_motion_plan) {
    if (kineval.params.planner_rrt_connect) {rrt_result = robot_rrt_planner_iterate()}
    else {rrt_result = robot_rrt_star_planner_iterate()}
    if (rrt_result === "reached") {
      kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
      kineval.params.generating_motion_plan = false;
      textbar.innerHTML = "planner execution complete";
      kineval.params.planner_state = "complete";
    } else kineval.params.planner_state = "searching";
  } else if (kineval.params.update_motion_plan_traversal || kineval.params.persist_motion_plan_traversal) {

    if (kineval.params.persist_motion_plan_traversal) {
      kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index + 1) % kineval.motion_plan.length;
      textbar.innerHTML = "traversing planned motion trajectory";
    } else
      kineval.params.update_motion_plan_traversal = false;

    // set robot pose from entry in planned robot path
    robot.origin.xyz = [
      kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
      kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
      kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
    ];

    robot.origin.rpy = [
      kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
      kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
      kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
    ];

    // KE 2 : need to move q_names into a global parameter
    for (x in robot.joints) {
      robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
    }

  }
}


// STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

  // form configuration from base location and joint angles
  var q_start_config = [
    robot.origin.xyz[0],
    robot.origin.xyz[1],
    robot.origin.xyz[2],
    robot.origin.rpy[0],
    robot.origin.rpy[1],
    robot.origin.rpy[2]
  ];

  for (i = 6; i < q_index.length; i++) {
    q_start_config[i] = robot.joints[q_index[i]].angle;
  }


  // set goal configuration as the zero configuration
  var i;
  q_goal_config = new Array(q_start_config.length);
  for (i = 0; i < q_goal_config.length; i++) q_goal_config[i] = 0;

  // flag to continue rrt iterations
  rrt_iterate = true;
  rrt_iter_count = 0;

  // make sure the rrt iterations are not running faster than animation update
  cur_time = Date.now();

  // Initialize trees
  T_a = tree_init(q_start_config);
  T_b = tree_init(q_goal_config);

  // initialize rrt flag
  rrt_flag = 0


}

function robot_rrt_star_planner_iterate(){
  var i;
  if (rrt_iterate && (Date.now() - cur_time > 10)) {
    cur_time = Date.now();



    var delta = kineval.params.plan_steplength
    var radius = 2*delta
    var successful = false;

    var tree_a = T_a;


    var extend_failure_counter = 0
    while (successful == false) {
      extend_failure_counter++
      var q_rand = random_config(q_index.length);
      var vertex_near = nearest_neighbor(q_rand,T_a);
      var q_new = new_config(vertex_near, q_rand, delta)

      if ( (kineval.poseIsCollision(q_new) == false) && (joint_limit_coll(q_new) == false) ){successful = true}
      if (extend_failure_counter > 100){return "iterating"}
    }
    var nearby_verts = near_vertices(T_a,q_new,radius)
    var q_min = choose_parent(nearby_verts,vertex_near,q_new)
    insertTreeVertex(T_a,q_new);
    T_a.vertices[T_a.newest].parent = q_min
    T_a.vertices[T_a.newest].distance = q_min.distance + ndiml2dist(T_a.vertices[T_a.newest].vertex,q_min.vertex)
    insertTreeEdge(T_a, T_a.vertices.indexOf(q_min),T_a.newest)

    rewire(T_a, nearby_verts, q_min, T_a.vertices[T_a.newest])
    console.log(ndiml2dist(T_a.vertices[T_a.newest].vertex,T_b.vertices[0].vertex))

    if (ndiml2dist(T_a.vertices[T_a.newest].vertex,T_b.vertices[0].vertex) < delta ) {


      var curr_vertex = T_a.vertices[T_a.newest]
      var i = 0
      while (curr_vertex.parent.length != 0) {
        kineval.motion_plan[i] = curr_vertex
        curr_vertex = curr_vertex.parent
        i++
      }
      kineval.motion_plan.push(T_a.vertices[0])
      kineval.motion_plan.reverse()
      kineval.motion_plan.push(T_b.vertices[0])

      for (var j=0; j < kineval.motion_plan.length; j++){
        kineval.motion_plan[j].geom.material.color = {r:1,g:0,b:0};
      }

      return "reached"

    }


    return "failed"

}
  }



function robot_rrt_planner_iterate() {

  var i;

  if (rrt_iterate && (Date.now() - cur_time > 10)) {
    cur_time = Date.now();

    rrt_flag += 1
    rrt_flag = rrt_flag % 2;


    var delta  = kineval.params.plan_steplength
    var successful = false;
    if (rrt_flag == 1) {
      var tree_a = T_a;
      var tree_b = T_b;
    } else {
      var tree_a = T_b;
      var tree_b = T_a;
    }

    var extend_failure_counter = 0
    while (successful == false) {
      extend_failure_counter++
      var q_rand = random_config(q_index.length);
      successful = rrt_extend(q_rand, tree_a, delta);
      if ( extend_failure_counter > 100){return "failed"}
    }
    if (rrt_connect(tree_a, tree_b, delta) == true) {
      var curr_vertex = T_b.vertices[T_b.newest]
      var i = 0
      while (curr_vertex.parent.length != 0) {
        kineval.motion_plan[i] = curr_vertex
        curr_vertex = curr_vertex.parent
        i++
      }
      curr_vertex = T_a.vertices[T_a.newest]
      kineval.motion_plan.reverse()
      //kineval.motion_plan.push(T_b.vertices[0])

      while (curr_vertex.parent.length != 0) {
        kineval.motion_plan[i] = curr_vertex
        curr_vertex = curr_vertex.parent
        i++
      }
      kineval.motion_plan.push(T_a.vertices[0])
      kineval.motion_plan.reverse()
      for (var j=0; j < kineval.motion_plan.length; j++){
        kineval.motion_plan[j].geom.material.color = {r:1,g:0,b:0};
      }


      return "reached"

    }


    return "failed"


    // STENCIL: implement single rrt iteration here. an asynch timing mechanism
    //   is used instead of a for loop to avoid blocking and non-responsiveness
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
  }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////



function tree_init(q) {

  // create tree object
  var tree = {};

  // initialize with vertex for given configuration
  tree.vertices = [];
  tree.vertices[0] = {};
  tree.vertices[0].vertex = q;
  tree.vertices[0].edges = [];
  tree.vertices[0].parent = [];
  tree.vertices[0].distance = 0;

  // create rendering geometry for base location of vertex configuration
  add_config_origin_indicator_geom(tree.vertices[0]);

  // maintain index of newest vertex added to tree
  tree.newest = 0;

  return tree;

}

function insertTreeVertex(tree, q) {


  // create new vertex object for tree with given configuration and no edges
  var new_vertex = {};
  new_vertex.edges = [];
  new_vertex.vertex = q;

  // create rendering geometry for base location of vertex configuration
  add_config_origin_indicator_geom(new_vertex);

  // maintain index of newest vertex added to tree
  tree.vertices.push(new_vertex);
  tree.newest = tree.vertices.length - 1;


}

function add_config_origin_indicator_geom(vertex) {

  // create a threejs rendering geometry for the base location of a configuration
  // assumes base origin location for configuration is first 3 elements
  // assumes vertex is from tree and includes vertex field with configuration

  temp_geom = new THREE.CubeGeometry(0.1, 0.1, 0.1);
  temp_material = new THREE.MeshLambertMaterial({
    color: 0xffff00,
    transparent: true,
    opacity: 0.7
  });
  temp_mesh = new THREE.Mesh(temp_geom, temp_material);
  temp_mesh.position.x = vertex.vertex[0];
  temp_mesh.position.y = vertex.vertex[1];
  temp_mesh.position.z = vertex.vertex[2];
  scene.add(temp_mesh);
  vertex.geom = temp_mesh;
}


function insertTreeEdge(tree, q1_idx, q2_idx) {

  // add edge to first vertex as pointer to second vertex
  tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

  // add edge to second vertex as pointer to first vertex
  tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

  // can draw edge here, but not doing so to save rendering computation
}



//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

function near_vertices(tree,q_new,delta){
  var near_vertices = [];
  for (var i=0; i < tree.vertices.length; i++){
    if(ndiml2dist(tree.vertices[i].vertex,q_new) < delta){
      //console.log(ndiml2dist(tree.vertices[i].vertex,q_new))
      near_vertices.push(tree.vertices[i]);
    }
  }
  return near_vertices;
}

function rewire(tree,near_vertices,q_min,q_new){
  //Inputs: tree object; array of vicinity vertices, vertex of min parent, vertex of new
  // Outputs: none
  // Modifies: tree edge connections

  for (var i = 0; i < near_vertices.length; i++){
    var c_new = q_new.distance + ndiml2dist(q_new.vertex, near_vertices[i].vertex)
    if ((c_new < near_vertices[i].distance) && (near_vertices[i] != q_min)){
      near_vertices[i].distance = c_new
      near_vertices[i].parent = q_new
      insertTreeEdge(tree, tree.vertices.indexOf(q_new),tree.vertices.indexOf(near_vertices[i]))
    }
  }
}



function choose_parent(near_vertices,q_nearest,q_new){
  //Inputs :
  //near_vertices = array of vertex objects from system tree
  // q_nearest = vertex object nearest
  // q_new = float array of proposed q_new
  //OUTPUTS:
  // q_min , vertex object which will produce lowest cost as a parent to q_new

  var q_min = q_nearest;
  var c_min = q_nearest.distance + ndiml2dist(q_nearest.vertex,q_new)

  for (var i = 0; i < near_vertices.length; i++){
    var c_temp = near_vertices[i].distance + ndiml2dist(near_vertices[i].vertex,q_new)
    if (c_temp < c_min){
      q_min = near_vertices[i];
      c_min = c_temp;
    }
  }
  return q_min
}


function rrt_extend(q_rand, tree, delta) {
  // Inputs: q_rand- point in C space, tree
  var vertex_near = nearest_neighbor(q_rand, tree);
  var q_new = new_config(vertex_near, q_rand, delta)
  //  console.log(kineval.poseIsCollision(q_new))
  //  console.log(joint_limit_coll(q_new))
  if ((kineval.poseIsCollision(q_new) == false) && (joint_limit_coll(q_new) == false)) {
    insertTreeVertex(tree, q_new);
    tree.vertices[tree.newest].parent = vertex_near
    tree.vertices[tree.newest].distance = vertex_near.distance + ndiml2dist(tree.vertices[tree.newest].vertex, vertex_near.vertex)
    insertTreeEdge(tree, tree.vertices.indexOf(vertex_near), tree.newest);
    return true
  }
  return false
}

function rrt_connect(tree1, tree2, delta) {
  var successful = true
  while (successful == true) {
    successful = rrt_extend(tree1.vertices[tree1.newest].vertex, tree2, delta);
    if (successful == true && ndiml2dist(tree1.vertices[tree1.newest].vertex, tree2.vertices[tree2.newest].vertex) < delta) {
      return true
    }
  }
  return false
}
//   rrt_connect

function new_config(vertex_near, q_rand, delta) {
  //Inputs: q_near is vertex object of tree, q_rand-point in C-space
  var q_near = vertex_near.vertex;
  var step_vector = vector_subtraction_ndim(q_rand, q_near);
  step_vector = vector_normalize(step_vector);
  step_vector = vector_scalar_mult(step_vector, delta);
  var q_new = vector_add(q_near, step_vector)
  return q_new
}


function random_config(q_length) {
  var coll = true;
  var delta = []
  var eps = .1
  while (coll == true) {

    var q_rand = []
    q_rand[0] = getRandomArbitrary(robot_boundary[0][0], robot_boundary[1][0]);
    delta[0] = (robot_boundary[1][0] - robot_boundary[0][0])* eps
    q_rand[1] = 0
    delta[1] = 0
    q_rand[2] = getRandomArbitrary(robot_boundary[0][2], robot_boundary[1][2]);
    delta[2] = (robot_boundary[1][2] - robot_boundary[0][2])*eps
    q_rand[3] = 0
    delta[3] = 0
    q_rand[4] = getRandomArbitrary(-6.3, 6.3);
    delta[4] = (6.3 + 6.3)*eps
    q_rand[5] = 0
    delta[5] = 0

    for (var i = 6; i < q_length; i++) {
      if (robot.joints[q_index[i]].type == "revolute") {
        q_rand[i] = getRandomArbitrary(robot.joints[q_index[i]].limit.lower, robot.joints[q_index[i]].limit.upper)
        delta[i] = (robot.joints[q_index[i]].limit.upper - robot.joints[q_index[i]].limit.lower)*eps;
      } else if (robot.joints[q_index[i]].type == "continuous") {
        q_rand[i] = getRandomArbitrary(-6.3, 6.3)
        delta[i] = (6.3 + 6.3)*eps
      } else if (robot.joints[q_index[i]].type == "prismatic") {
        q_rand[i] = getRandomArbitrary(robot.joints[q_index[i]].limit.lower, robot.joints[q_index[i]].limit.upper)
        delta[i] = (robot.joints[q_index[i]].limit.upper - robot.joints[q_index[i]].limit.lower)*eps;
      } else {
        q_rand[i] = 0
        delta[i] = 0
      }
    }

    coll = kineval.poseIsCollision(q_rand)
  }
  return q_rand
}


function joint_limit_coll(q) {
  var coll = false;
  for (var i = 6; i < q.length; i++) {
    if (robot.joints[q_index[i]].type == "revolute") {
      if (q[i] < robot.joints[q_index[i]].limit.lower || q[i] > robot.joints[q_index[i]].limit.upper) {
        return true
      }
    } else if (robot.joints[q_index[i]].type == "prismatic") {
      if (q[i] < robot.joints[q_index[i]].limit.lower || q[i] > robot.joints[q_index[i]].limit.upper) {
        return true
      }
    }
  }
  return false
}

function nearest_neighbor(q, tree) {
  //Input: C-space point, tree structure
  //Outputs: return nearest vertex in tree to q
  var min_dist = 10000000;
  for (var i = 0; i < tree.vertices.length; i++) {
    var vertex_dist = ndiml2dist(q, tree.vertices[i].vertex)
    if (vertex_dist < min_dist) {
      min_dist = vertex_dist;
      var nearest = tree.vertices[i]
    }
  }
  return nearest
}



function ndiml2dist(q1, q2) {
  // Inputs: arrays of equal n length
  // Outputs: new scalar l2 norm from q1 to q2
  if (q1.length != q2.length) {
    console.log('q1 and q2 are not same length, cannot find distance')
    return null
  }
  var dist = 0;
  for (var i = 0; i < q1.length; i++) {
    dist += Math.pow(q1[i] - q2[i], 2)
  }
  return Math.sqrt(dist)
}


function getRandomArbitrary(min, max) {
  return Math.random() * (max - min) + min;
}
