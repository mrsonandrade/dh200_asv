"""
``pyswarming.utils``
========================

The PySwarming utils is composed of different classes and functions,
for details, see the documentation available in two forms: docstrings provided
with the code, and a loose standing reference guide, available from
`the PySwarming homepage <https://github.com/mrsonandrade/pyswarming>`_..

Classes and functions present in pyswarming.utils are listed below.

Classes
---------

     

Functions
------------------

   swarm_centroid
   heading_error_degrees
   ensure_negative_180_to_180
   ensure_0_to_360
   ensure_negative_pi_to_pi
   ensure_0_to_2pi

"""

__all__ = ['swarm_centroid', 'heading_error_degrees',
           'ensure_negative_180_to_180', 'ensure_0_to_360',
           'ensure_negative_pi_to_pi', 'ensure_0_to_2pi']

import numpy as np

###################################################################
# Classes
###################################################################



###################################################################
# Functions
###################################################################

def swarm_centroid(r):
    """
    Calculates the centroid (x,y,z) of the swarm.

    Parameters
    ----------
    r : numpy.array
        array must have the robots positions in cartesian
        coordinates (i.e. np.asarray([[x1, y1, z1],
        [x2, y2, z2], ..., [xN, yN, zN]])).

    Returns
    -------
    c : numpy.array
        array containing the centroid
    """
    
    c = r.sum(axis=0)/r.shape[0]

    return c

def heading_error_degrees(current_angle, desired_angle):
    """
    Calculates the heading error between the robot
    current heading angle and the desired heading
    angle. The inputs are given in degrees, and the
    output is given in degrees within [-180, 180].

    Parameters
    ----------
    current_angle : float or int
        robot current heading angle in degrees.

    desired_angle : float or int
        robot desired heading angle in degrees.

    Returns
    -------
    error : float or int
        angular error in degrees within [-180, 180].
    """

    current_angle = current_angle % 360
    desired_angle = desired_angle % 360

    error = (desired_angle - current_angle + 180) % 360 - 180

    return error

def ensure_negative_180_to_180(angle_degrees):
    """
    Normalization, ensuring that the output
    angle will be within [-180, 180].

    Parameters
    ----------
    angle_degrees : float or int
        angle in degrees.

    Returns
    -------
    angle_output : float or int
        angle in degrees within [-180, 180].
    """

    angle_output = (angle_degrees + 180) % 360 - 180

    return angle_output

def ensure_0_to_360(angle_degrees):
    """
    Normalization, ensuring that the output
    angle will be within [0, 360].

    Parameters
    ----------
    angle_degrees : float or int
        angle in degrees.

    Returns
    -------
    angle_output : float or int
        angle in degrees within [0, 360].
    """

    angle_output = (angle_degrees + 360) % 360

    return angle_output

def ensure_negative_pi_to_pi(angle_radians):
    """
    Normalization, ensuring that the output
    angle will be within [-π, π].

    Parameters
    ----------
    angle_radians : float or int
        angle in radians.

    Returns
    -------
    angle_output : float or int
        angle in radians within [-π, π].
    """

    angle_output = (angle_radians + np.pi) % (2 * np.pi) - np.pi

    return angle_output

def ensure_0_to_2pi(angle_radians):
    """
    Normalization, ensuring that the output
    angle will be within [0, 2π].

    Parameters
    ----------
    angle_radians : float or int
        angle in radians.

    Returns
    -------
    angle_output : float or int
        angle in radians within [0, 2π].
    """

    angle_output = (angle_radians + 2 * np.pi) % (2 * np.pi)

    return angle_output

