import roslib
roslib.load_manifest('learning_tf')
import tf
explicit_quat = [0, 0, 0, 0]
euler = tf.transformations.euler_from_quaternion(explicit_quat)
print(euler)
