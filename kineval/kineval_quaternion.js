//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    function quaternion_from_axisangle(axis,angle) {
      var q = [0,[0,0,0]]
      q[0] = Math.cos(angle/2);
      q[1][0] = Math.sin(angle/2*axis[0]);
      q[1][1] = Math.sin(angle/2*axis[1]);
      q[1][2] = Math.sin(angle/2*axis[2]);
      return q

    }
    //   quaternion_normalize
    function quaternion_normalize(q1) {
      var a = q1[0];
      var b = q1[1][0];
      var c = q1[1][1];
      var d = q1[1][2];
      var q_norm = Math.sqrt(Math.pow(a,2) + Math.pow(b,2) +
      Math.pow(c,2) +  Math.pow(d,2));
      return [a/q_norm,[b/q_norm,c/q_norm,d/q_norm]]

    }
    //   quaternion_to_rotation_matrix
    function quaternion_to_rotation_matrix(q) {
      var q0 = q[0];
      var q1 = q[1][0];
      var q2 = q[1][1];
      var q3 = q[1][2];
      var a11 = 1 - 2*(Math.pow(q2,2) + Math.pow(q3,2));
      var a12 = 2*(q1*q2 - q0*q3);
      var a13 = 2*(q0*q2 + q1*q3);
      var a21 = 2*(q1*q2 + q0*q3);
      var a22 = 1 - 2*(Math.pow(q1,2) + Math.pow(q3,2));
      var a23 = 2*(q2*q3 - q0*q1);
      var a31 = 2*(q1*q3 - q0*q2);
      var a32 = 2*(q0*q1 + q2*q3);
      var a33 = 1 - 2*(Math.pow(q1,2) + Math.pow(q2,2));
      return [[a11,a12,a13,0],[a21,a22,a23,0],[a31,a32,a33,0],[0,0,0,1]];

    }
    //   quaternion_multiply
    function quaternion_multiply(q1,q2) {
      var q3 = [0,[0,0,0]]
      var a = q1[0];
      var b = q1[1][0];
      var c = q1[1][1];
      var d = q1[1][2];
      var e = q2[0];
      var f = q2[1][0];
      var g = q2[1][1];
      var h = q2[1][2];

      q3[0] = a*e -b*f - c*g - d*h;
      q3[1][0] = a*f + b*e + c*h - d*g;
      q3[1][1] = a*g - b*h + c*e + d*f;
      q3[1][2] = a*h + b*g -c*f + d*e;
      return q3

    }
