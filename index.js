function toRadians(degrees) {
  return degrees * Math.PI / 180;
}

function toDegrees(radians) {
  return radians * 180 / Math.PI;
}

function distanceBetweenTwoPoints(lat1, lon1, lat2, lon2) {
  var R = 6371; // km
  var dLat = toRadians(lat2-lat1);
  var dLon = toRadians(lon2-lon1);
  var lat1 = toRadians(lat1);
  var lat2 = toRadians(lat2);

  var a = Math.sin(dLat/2) * Math.sin(dLat/2) +
          Math.sin(dLon/2) * Math.sin(dLon/2) * Math.cos(lat1) * Math.cos(lat2);
  var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  var d = R * c;
  return d;
}

const distance = distanceBetweenTwoPoints(51.5072, 0.1275, 48.8566, 2.3522);

function sortByFirstTwoLetters(a, b) {
  if (a[0] < b[0]) {
    return -1;
  }
  if (a[0] > b[0]) {
    return 1;
  }
  if (a[1] < b[1]) {
    return -1;
  }
  if (a[1] > b[1]) {
    return 1;
  }
  return 0;
}

// find all images without alternate text via an image tag foo
// and give them a red border
function process() {
  var images = document.getElementsByTagName('img');
  for (var i = 0; i < images.length; i++) {
    if (images[i].alt == '') {
      images[i].style.border = '2px solid red';
    }
  }
}

function inverseMatrix(matrix) {
  var det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
  return [[matrix[1][1] / det, -matrix[0][1] / det],
          [-matrix[1][0] / det, matrix[0][0] / det]];
}

// log inverse matrix of a 4 x 4 matrix
console.log(inverseMatrix([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]]));  


// inverse kinematics for a 6DOF robot arm where the input is the desired end effector position
// and euler angles and the output is the joint angles
function inverseKinematics(x, y, z, roll, pitch, yaw) {
  var l1 = 2.5;
  var l2 = 3;
  var l3 = 2.5;
  var l4 = 2.5;
  var l5 = 2.5;
  var l6 = 2;

  var theta1 = Math.atan2(y, x);
  var theta2 = Math.atan2(z, Math.sqrt(x * x + y * y)) - Math.atan2(l4, Math.sqrt(l3 * l3 + l2 * l2));
  var theta3 = Math.atan2(l3, l2) - Math.atan2(z, Math.sqrt(x * x + y * y));
  var theta4 = Math.atan2(l6, l5) + roll;
  var theta5 = pitch;
  var theta6 = yaw;
  return [theta1, theta2, theta3, theta4, theta5, theta6];
}

// log inverse kinematics for a 6DOF robot arm
console.log(inverseKinematics(0, 0, 12.5, 0, 0, 0));


function  matrixDotProduct(a, b) {
  var result = [];
  for (var i = 0; i < a.length; i++) {
    result[i] = [];
    for (var j = 0; j < b[0].length; j++) {
      var sum = 0;
      for (var k = 0; k < a[0].length; k++) {
        sum += a[i][k] * b[k][j];
      }
      result[i][j] = sum;
    }
  }
  return result;
}

// function that takes two matrices of strings and returns their dot product where 
// a product of two strings is the concatenation of the two strings with a * character in between
// and an addition of two strings is the concatenation of the two strings with a + character in between
function stringMatrixDotProduct(a, b) {
  var result = [];
  for (var i = 0; i < a.length; i++) {
    result[i] = [];
    for (var j = 0; j < b[0].length; j++) {
      var sum = '';
      for (var k = 0; k < a[0].length; k++) {
        if (sum != '') {
          sum += '+';
        }
        sum += '(' + a[i][k] + '*' + b[k][j] + ')';
      }
      result[i][j] = sum;
    }
  }
  return result;
}

// function that takes two matrices of strings and returns their dot product where 
// a product of two strings is the concatenation of the two strings with a * character in between
// and an addition of two strings is the concatenation of the two strings with a + character in between
// if the one of the parameters of the multiplication is a one, then the other parameter is returned

const Test_Matrix_1 = [
  ['a', 'b'],
  ['c', 'd'],
];

const Test_Matrix_2 = [
  ['e', 'f'],
  ['g', 'h'],
];

// log out matrix dot string of Test_Matrix_1 and Test_Matrix_2
console.log(stringMatrixDotProduct(Test_Matrix_1, Test_Matrix_2));

const Test_Matrix_3 = [
  ['1', '0'],
  ['0', '1'],
];

const Test_Matrix_4 = [
  ['0', '1'],
  ['1', '0'],
];

// log out matrix dot string of Test_Matrix_3 and Test_Matrix_4
console.log(stringMatrixDotProduct(Test_Matrix_3, Test_Matrix_4));

const buildHomogeneousDenavitMatrixRowString = (a, alpha, d, theta) => {
  return [
    'cos(' + theta + ')', '-sin(' + theta + ')*cos(' + alpha + ')', 'sin(' + theta + ')*sin(' + alpha + ')', a + '*cos(' + theta + ')',
    'sin(' + theta + ')', 'cos(' + theta + ')*cos(' + alpha + ')', '-cos(' + theta + ')*sin(' + alpha + ')', a + '*sin(' + theta + ')',
    '0', 'sin(' + alpha + ')', 'cos(' + alpha + ')', d,
    '0', '0', '0', '1'
  ];
}

// function that takes a row from denavit-hartenberg table and returns the homogeneous transformation matrix
// for that row as a 4 x 4 matrix
function buildHomogeneousDenavitMatrixRow(a, alpha, d, theta) { 
  return [
    [Math.cos(theta), -Math.sin(theta) * Math.cos(alpha), Math.sin(theta) * Math.sin(alpha), a * Math.cos(theta)],
    [Math.sin(theta), Math.cos(theta) * Math.cos(alpha), -Math.cos(theta) * Math.sin(alpha), a * Math.sin(theta)],
    [0, Math.sin(alpha), Math.cos(alpha), d],       
    [0, 0, 0, 1]    
  ];  
}

// using the buildHomogeneousDenavitMatrixRow function, build a homogeneous denavit matrix for a 6DOF robot arm
// where the input is the the PTaH parameters as a two dimensional array and the output is the homogeneous denavit matrix
// as a two dimensional array of numbers
function buildHomogeneousDenavitMatrix(ptah) {
  var result = buildHomogeneousDenavitMatrixRow(ptah[0][0], ptah[0][1], ptah[0][2], ptah[0][3])
  for (var i = 1; i < ptah.length; i++) {
    result = matrixDotProduct(result, buildHomogeneousDenavitMatrixRow(ptah[i][0], ptah[i][1], ptah[i][2], ptah[i][3]));  
  }   
  return result;
} 

const d90 = toRadians(90);

// a function forward that uses the buildHomogeneousDenavitMatrix function to perform forward kinematics for a 6DOF robot arm
// where the input is the 6 angles of the robot arm and the output is the position of the end effector as a 3D vector
function forward(theta1, theta2, theta3, theta4, theta5, theta6) {
  const a1 = 2.5;
  const a2 = 3;
  const a3 = 2.5;
  const a4 = 2.5;
  const a5 = 2.5;
  const a6 = 2;

  // Mine: theta, alpha, r, d
  // Theirs: r, alpha, d, theta
  const ptah = [    
    [0,   d90,  a1,       theta1],
    [a2,  0,    0,        theta2+d90],
    [0,   -d90, 0,        theta3-d90],
    [0,   d90,  a3 + a4,  theta4],
    [0,   -d90, 0,        theta5], 
    [0,   0,    a5+ a6,   theta6],
  ];

  const homogeneousDenavitMatrix = buildHomogeneousDenavitMatrix(ptah);

  const result = [homogeneousDenavitMatrix[0][3], homogeneousDenavitMatrix[1][3], homogeneousDenavitMatrix[2][3]];

  // return the result where each item in the result is rounded to 2 decimal places
  return result.map(x => Math.round(x * 100) / 100);
}


console.log(forward(0, 0, 0, 0, 0, 0));


// a function that takes a subset of a given matrix and returns the subset as a new matrix based on the number of rows and columns to take
function matrixSubset(matrix, rows, columns) {
  var result = [];
  for (var i = 0; i < rows; i++) {
    result[i] = [];
    for (var j = 0; j < columns; j++) {
      result[i][j] = matrix[i][j];
    }
  }
  return result;
}

// given a matrix remove all negative zeros and round all numbers to 2 decimal places
function cleanMatrix(matrix) {
  return matrix.map(row => row.map(x => x === -0 ? 0 : Math.round(x * 100) / 100));
}

// roll pitch and yaw are the euler angles  
// roll is the rotation around the x axis
// pitch is the rotation around the y axis
// yaw is the rotation around the z axis
// the output is the rotation matrix as a 3 x 3 matrix
// clean the matrix by removing all negative zeros and rounding all numbers to 2 decimal places
function buildRotationMatrix(roll, pitch, yaw) {
  const c1 = Math.cos(roll);
  const c2 = Math.cos(pitch);
  const c3 = Math.cos(yaw);
  const s1 = Math.sin(roll);
  const s2 = Math.sin(pitch);
  const s3 = Math.sin(yaw);
  return cleanMatrix([
    [c2 * c3, -c2 * s3, s2],
    [c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3, -c2 * s1],
    [s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3, c1 * c2]
  ]);
}

// zxz euler angles
// the output is the rotation matrix as a 3 x 3 matrix
// clean the matrix by removing all negative zeros and rounding all numbers to 2 decimal places
function buildRotationMatrixZXZ(z1, x, z2) {
  const c1 = Math.cos(z1);
  const c2 = Math.cos(x);
  const c3 = Math.cos(z2);
  const s1 = Math.sin(z1);
  const s2 = Math.sin(x);
  const s3 = Math.sin(z2);
  return cleanMatrix([
    [c1 * c3 - c2 * s1 * s3, -c3 * s1 - c1 * c2 * s3, s2 * s3],
    [c1 * s2 * s3 + c3 * s1, c1 * c2 * c3 - s1 * s3, -c3 * s2],
    [s1 * s2, c1 * s2, c2]
  ]);
}

// log out the rotation matrix for a given roll, pitch and yaw of zeros
console.log(buildRotationMatrix(0, 0, 0));

// log out the rotation matrix for a given roll, pitch and yaw of 90 degrees
console.log(buildRotationMatrix(toRadians(90), toRadians(90), toRadians(90)));

// rotateAroundXAxis takes a 3 x 3 matrix and a rotation angle in radians and returns a new 3 x 3 matrix
// use the matrixDotProduct function to perform the matrix multiplication
// no need to clean the matrix
function rotateAroundXAxis(matrix, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return matrixDotProduct(matrix, [
    [1, 0, 0],
    [0, c, -s],
    [0, s, c]
  ]);
}
