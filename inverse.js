// inverse kinematics for a 6DOF robot arm where the input is the desired end effector position, three euler angles and the output is the joint angles, 
// and a denavit hartenberg table
// the output is a list of 6 angles in radians
function inverse(x, y, z, roll = 0, pitch = 0, yaw = 0) {
  // the d1 value is the distance between the base of the robot arm and the first joint
  const d1 = 2.5;
  // the a2 value is the distance between the first and second joints
  const a2 = 3;
  // the a3 value is the distance between the second and third joints
  const a3 = 2.5;
  // the d4 value is the distance between the third and fourth joints
  const d4 = 2.5;
  // the d5 value is the distance between the fourth and fifth joints
  const d5 = 2.5;
  // the d6 value is the distance between the fifth and sixth joints
  const d6 = 2.5;
  // the d7 value is the distance between the sixth joint and the end effector
  const d7 = 2;

  // the rotation matrix is calculated using the buildRotationMatrix function
  // the roll, pitch and yaw are the euler angles
  // the roll is the rotation around the x axis
  // the pitch is the rotation around the y axis
  // the yaw is the rotation around the z axis
  const rotationMatrix = buildRotationMatrix(roll, pitch, yaw);

  // the rotation matrix is multiplied by the rotation matrix for the end effector
  // the end effector rotation matrix is calculated using the buildRotationMatrixZXZ function
  // the z1, x and z2 are the euler angles for the end effector
  // the z1 is the rotation around the z axis
  // the x is the rotation around the x axis
  // the z2 is the rotation around the z axis
  const rotationMatrixEndEffector = buildRotationMatrixZXZ(toRadians(-90), toRadians(0), toRadians(-90));
  const rotationMatrixEndEffectorInverse = buildRotationMatrixZXZ(toRadians(90), toRadians(0), toRadians(90));
  const rotationMatrixCombined = matrixDotProduct(rotationMatrix, rotationMatrixEndEffector);

  // the end effector position is calculated by multiplying the rotation matrix by the end effector position
  // the end effector position is the position of the end effector in the base frame
  const endEffectorPosition = matrixDotProduct(rotationMatrix, [0, 0, d7  + d6]);

  // the position of the end effector is calculated by adding the end effector position to the input position
  const position = [x, y, z].map((x, index) => x + endEffectorPosition[index]);

  // the theta1 angle is calculated using the atan2 function
  // the atan2 function takes the y and x position and returns the angle in radians
  const theta1 = Math.atan2(position[1], position[0]);

  // the theta3 angle is calculated using the atan2 function
  // the atan2 function takes the y and x position and returns the angle in radians
  const theta3 = Math.atan2(position[2] - d1, Math.sqrt(position[0] ** 2 + position[1] ** 2)) - Math.atan2(d4, Math.sqrt(a2 ** 2 + a3 ** 2));

  // the theta2 angle is calculated using the atan2 function
  // the atan2 function takes the y and x position and returns the angle in radians
  const theta2 = Math.atan2(Math.sqrt(position[0] ** 2 + position[1] ** 2) - a2 * Math.cos(theta3), position[2] - d1 - a2 * Math.sin(theta3)) - Math.atan2(a3, -d4);

  // the rotation matrix for the first three joints is calculated using the buildRotationMatrixZYX function
  // the z1, y and x are the euler angles for the first three joints
  // the z1 is the rotation around the z axis
  // the y is the rotation around the y axis
  // the x is the rotation around the x axis
  const rotationMatrixFirstThreeJoints = buildRotationMatrixZYX(theta1, theta2, theta3);

  // the rotation matrix for the last three joints is calculated by multiplying the rotation matrix for the first three joints by the rotation matrix for the end effector
  const rotationMatrixLastThreeJoints = matrixDotProduct(rotationMatrixFirstThreeJoints, rotationMatrixEndEffectorInverse);

  // theta4, theta5 and theta6 are calculated using the rotation matrix to euler angles function
  // the rotation matrix to euler angles function takes the rotation matrix and returns the euler angles
  const [theta4, theta5, theta6] = rotationMatrixToEulerAngles(rotationMatrixLastThreeJoints);

  // return the joint angles
  return [theta1, theta2, theta3, theta4, theta5, theta6];
}