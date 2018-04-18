//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS
//////////////////////////////////////////////////

////////////// Matrix functions///////////////////

function matrix_copy(m1) {
  // returns 2D array that is a copy of m1

  var mat = [];
  var i, j;

  for (i = 0; i < m1.length; i++) { // for each row of m1
    mat[i] = [];
    for (j = 0; j < m1[0].length; j++) { // for each column of m1
      mat[i][j] = m1[i][j];
    }
  }
  return mat;
}

//   matrix_multiply, m1 and m2 must be [[]]
function matrix_multiply(m1, m2) {
  if (m1[0].length != m2.length) {
    console.log('matrix mult not possible')
    return undefined
  }
  var m3 = []
  for (var i = 0; i < m1.length; i++) {
    m3[i] = []
    for (var j = 0; j < m2[0].length; j++) {
      m3[i][j] = 0;
      for (var k = 0; k < m1[0].length; k++) {
        m3[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }
  return m3
}

//   matrix_transpose
function matrix_transpose(m) {
  if (m[0].length == undefined) {
    var mat = [m.slice()];
  } else {
    var mat = m.slice()
  }

  var mt = [];
  for (var j = 0; j < mat[0].length; j++) {
    mt[j] = []
    for (var i = 0; i < mat.length; i++) {
      mt[j][i] = mat[i][j];
    }
  }
  return mt
}

//   generate_identity
function generate_identity(dim) {
  var m = [];
  for (i = 0; i < dim; i++) {
    m[i] = [];
    for (j = 0; j < dim; j++) {
      if (i == j) {
        m[i][j] = 1;
      } else {
        m[i][j] = 0;
      }
    }
  }
  return m
}

//   generate_translation_matrix
function generate_translation_matrix(ref) {
  var x = ref[0];
  var y = ref[1];
  var z = ref[2];
  return [
    [1, 0, 0, x],
    [0, 1, 0, y],
    [0, 0, 1, z],
    [0, 0, 0, 1]
  ];
}
//   generate_rotation_matrix_X
function generate_rotation_matrix_X(rad) {
  var c = Math.cos;
  var s = Math.sin;
  return [
    [1, 0, 0, 0],
    [0, c(rad), -s(rad), 0],
    [0, s(rad), c(rad), 0],
    [0, 0, 0, 1]
  ];
}
//   generate_rotation_matrix_Y
function generate_rotation_matrix_Y(rad) {
  var c = Math.cos;
  var s = Math.sin;
  return [
    [c(rad), 0, s(rad), 0],
    [0, 1, 0, 0],
    [-s(rad), 0, c(rad), 0],
    [0, 0, 0, 1]
  ];
}
//   generate_rotation_matrix_Z
function generate_rotation_matrix_Z(rad) {
  var c = Math.cos;
  var s = Math.sin;
  return [
    [c(rad), -s(rad), 0, 0],
    [s(rad), c(rad), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
}

function generate_homogeneous_transform(origin) {
  var xyz = origin.xyz;
  var rpy = origin.rpy;
  var mult = matrix_multiply;
  var tran = generate_translation_matrix;
  var matx = generate_rotation_matrix_X;
  var maty = generate_rotation_matrix_Y;
  var matz = generate_rotation_matrix_Z;
  var t1 = mult(tran(xyz), matx(rpy[0]));
  var t2 = mult(maty(rpy[1]), matz(rpy[2]));
  var h = mult(t1, t2);
  return h
}

//////////// Vector Functions//////////////
// vector dot product nxn
function vector_dot(m1, m2) {
  var dot = matrix_multiply([m1], matrix_transpose(m2))
  return dot[0][0]
}
//   vector_cross dim = 3
function vector_cross(m1, m2) {

  var m2_tran = matrix_transpose(m2);
  var a1 = m1[0];
  var a2 = m1[1];
  var a3 = m1[2];
  var m1_skew = [
    [0, -a3, a2],
    [a3, 0, -a1],
    [-a2, a1, 0]
  ];
  return matrix_transpose(matrix_multiply(m1_skew, m2_tran))[0];
}

//   vector_normalize
function vector_normalize(m) {
  // Inputs:
  var norm = [];
  var den = 0;
  for (var i = 0; i < m.length; i++) {
    den += Math.pow(m[i], 2)
  }
  den = Math.sqrt(den);
  for (i = 0; i < m.length; i++) {
    norm[i] = m[i] / den;
  }
  return norm
}

function vector_scalar_mult(v, s) {
  // Inputs: v- ndim vector as array, s is a float
  //Outputs: new ndim vector scaled by s
  scale_v = [];
  for (i = 0; i < v.length; i++) {
    scale_v[i] = v[i] * s;
  }
  return scale_v
}

//   vector_normalize
function vector_subtraction(v1, v2) {
  //for IK only
  var v3 = [];
  v3[0] = v1[0] - v2[0];
  v3[1] = v1[1] - v2[1];
  v3[2] = v1[2] - v2[2];
  return v3
}

function vector_subtraction_ndim(v1, v2) {
  // subtract v2 from v1
  if (v1.length != v2.length) {
    console.log('v1 and v2 do not have matching dimensions')
    return undefined
  }
  var v3 = [];
  for (i = 0; i < v1.length; i++) {
    v3[i] = v1[i] - v2[i];
  }
  return v3
}

function vector_add(v1, v2) {
  // add v1 and v2
  if (v1.length != v2.length) {
    console.log('v1 and v2 do not have matching dimensions')
    return undefined
  }
  var v3 = [];
  for (i = 0; i < v1.length; i++) {
    v3[i] = v1[i] + v2[i];
  }
  return v3
}

function rotation_euler_convert(rot) {
  var theta = -Math.asin(rot[2][0])
  var psi = Math.atan2(rot[2][1] / Math.cos(theta), rot[2][2] / Math.cos(theta));
  var rho = Math.atan2(rot[1][0] / Math.cos(theta), rot[0][0] / Math.cos(theta));
  return [theta, psi, rho]
}

function rotation_euler_convert_eberly(rot) {
  var theta_y = Math.asin(rot[0][2]);
  var theta_x = Math.atan2(-rot[1][2], rot[2][2]);
  var theta_z = Math.atan2(-rot[0][1], rot[0][0]);
  return ([theta_x, theta_y, theta_z])
}

function angle_to_transform(joint, angle) {
  var q = quaternion_from_axisangle(joint.axis, angle)
  var q_unit = quaternion_normalize(q)
  var q_rot = quaternion_to_rotation_matrix(q_unit)

  if (joint.type == "prismatic") {
    var pris_tran = [0, 0, 0]
    pris_tran[0] = joint.axis[0] * angle;
    pris_tran[1] = joint.axis[1] * angle;
    pris_tran[2] = joint.axis[2] * angle;
    q_rot = generate_translation_matrix(pris_tran)
  }
  return q_rot
}

//Input nxn array of array
function lu_decomposition(m) {
  if (m[0].length != m.length) {
    console.log("m is not square")
    return undefined
  }
  var A_not = matrix_copy(m)
  var L = generate_identity(m.length)

  for (var j = 0; j < m.length - 1; j++) {
    var lsub = generate_identity(m.length)
    var lsub_inv = matrix_copy(lsub)
    for (var i = j + 1; i < m.length; i++) {
      lsub[i][j] = -A_not[i][j] / A_not[j][j]
      lsub_inv[i][j] = A_not[i][j] / A_not[j][j]
    }
    L = matrix_multiply(L, lsub_inv);
    A_not = matrix_multiply(lsub, A_not)
  }

  return {
    L: L,
    U: A_not
  }

}

//Input nxn array of array
function plu_decomposition(m) {
  if (m[0].length != m.length) {
    console.log("m is not square")
    return undefined
  }
  var A_not = matrix_copy(m)
  var L = []
  var P = []

  for (var j = 0; j < m.length - 1; j++) {
    var max_pivot_row = j
    var max_value = A_not[j][j]
    for (var k = j + 1; k < m.length; k++) {
      if (Math.abs(A_not[k][j]) > Math.abs(A_not[j][j])) {
        max_pivot_row = k;
        max_value = A_not[k][k]
      }
    }
    // check if singular
    //if (Math.abs(max_value) <= .01){
    //console.log("matrix is or near singular, cannot calculation decomposition")
    //return undefined
    //}

    var psub = generate_permutation(j, max_pivot_row, m.length)
    var lsub = generate_identity(m.length)
    var lsub_inv = matrix_copy(lsub)
    A_not = matrix_multiply(psub.perm, A_not)
    for (var i = j + 1; i < m.length; i++) {
      lsub[i][j] = -A_not[i][j] / A_not[j][j]
      lsub_inv[i][j] = A_not[i][j] / A_not[j][j]
    }
    L.push(lsub_inv);
    P.push(psub);
    A_not = matrix_multiply(lsub, A_not)
  }
  //partially commute all L matrices to separate L and P, and find total L matrix
  //compute total P matrix
  var total_l = generate_identity(m.length);
  var total_p = generate_identity(m.length);

  for (var i = 0; i < L.length; i++) {
    var current_l_commuted = matrix_copy(L[i])
    var total_p = matrix_multiply(total_p, P[P.length - 1 - i].perm)
    for (var j = 0; j < P.length; j++) {
      if (j > i) {
        current_l_commuted = generate_partial_permu_commute(current_l_commuted, P[j], i)
      }
    }
    total_l = matrix_multiply(total_l, current_l_commuted)
  }

  return {
    L: total_l,
    U: A_not,
    P: total_p
  }
}

//Input: nxn A matrix, b array
function linear_solved(A, b) {
  var plu = plu_decomposition(A);
  var d_array = matrix_multiply(plu.P, matrix_transpose(b));
  var d = []
  for (var i = 0; i < d_array.length; i++) {
    d.push(d_array[i][0])
  }
  var y = forward_substitution(plu.L, d)
  var x = backward_substitution(plu.U, y)
  return x
}

function matrix_inverse(A) {
  var plu = plu_decomposition(A);
  var ident = generate_identity(A.length)
  var inv = []
  for (var i = 0; i < A.length; i++) {
    inv.push(lin_solve_inv(plu, ident[i]))
  }
  return matrix_transpose(inv)
}

function lin_solve_inv(plu, b) {
  var d_array = matrix_multiply(plu.P, matrix_transpose(b));
  var d = []
  for (var i = 0; i < d_array.length; i++) {
    d.push(d_array[i][0])
  }
  var y = forward_substitution(plu.L, d)
  var x = backward_substitution(plu.U, y)
  return x
}

//Inputs: nxn lower triangle matrix, 1xn array d
//Outputs: 1xn array y that solves L*y = d
function forward_substitution(lower_tri_mat, d) {
  var y = [];
  y[0] = d[0] / lower_tri_mat[0][0];

  for (var i = 1; i < d.length; i++) {
    var cur_y = y.slice(0, i)
    var curr_weights = lower_tri_mat[i].slice(0, i)
    var total = matrix_multiply([curr_weights], matrix_transpose(cur_y))
    y[i] = (d[i] - total) / lower_tri_mat[i][i]
  }
  return y
}

//Inputs: nxn upper triangle matrix u, 1xn array y
//Outputs: 1xn array x that solves U*x = y
function backward_substitution(u, y) {
  var x = [];
  var end = y.length - 1
  x[end] = y[end] / u[end][end];
  for (var i = end - 1; i >= 0; i--) {
    var cur_x = x.slice(i + 1, end + 1)
    var curr_weights = u[i].slice(i + 1, end + 1)
    var total = matrix_multiply([curr_weights], matrix_transpose(cur_x))
    x[i] = (y[i] - total) / u[i][i]
  }
  return x
}

// generate l' such that pl = l'p
// Input permutation object p for row swap greater index than current pivot, atomic lower triangle matrix l, current pivot column
// Output l_prime lower triangle matrix
function generate_partial_permu_commute(l, p, pivot_col) {
  var l_prime = matrix_copy(l);
  var tempval = l_prime[p.row1][pivot_col]
  l_prime[p.row1][pivot_col] = l_prime[p.row2][pivot_col];
  l_prime[p.row2][pivot_col] = tempval;
  return l_prime
}

function generate_permutation(row1, row2, size) {
  var permutation = generate_identity(size)
  var temp_row = permutation[row1].slice()
  permutation[row1] = permutation[row2];
  permutation[row2] = temp_row
  return {
    perm: permutation,
    row1: row1,
    row2: row2
  }
}

// Array Methods
Array.prototype.diff = function(a) {
  return this.filter(function(i) {
    return a.indexOf(i) < 0;
  });
};
