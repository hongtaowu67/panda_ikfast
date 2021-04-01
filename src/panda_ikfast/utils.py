import numpy as np

def quat2rotm(quat):
    """
    Quaternion to rotation matrix.
    
    @type  quat: numpy array
    @param quat: quaternion (w, x, y, z)
    """
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    s = w * w + x * x + y * y + z * z

    rotm = np.array([[
        1 - 2 * (y * y + z * z) / s, 2 * (x * y - z * w) / s,
        2 * (x * z + y * w) / s
    ],
                     [
                         2 * (x * y + z * w) / s, 1 - 2 * (x * x + z * z) / s,
                         2 * (y * z - x * w) / s
                     ],
                     [
                         2 * (x * z - y * w) / s, 2 * (y * z + x * w) / s,
                         1 - 2 * (x * x + y * y) / s
                     ]])

    return rotm