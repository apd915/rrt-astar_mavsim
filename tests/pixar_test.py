import numpy as np


def branchless_onb(n):
    """
    Build an orthonormal basis from a single unit normal vector.
    
    Parameters:
    -----------
    n : numpy array, shape (3,)
        Unit normal vector
    
    Returns:
    --------
    b1, b2 : numpy arrays, shape (3,)
        Two orthonormal basis vectors perpendicular to n and each other
    """


    n = n / np.linalg.norm(n)
    # Extract the sign of n.z
    sign = np.copysign(1.0, n[2])  # +1 if n.z >= 0, -1 if n.z < 0
    
    # Compute scaling factor a (avoiding catastrophic cancellation)
    a = -1.0 / (sign + n[2])
    
    # Compute intermediate cross-term
    b = n[0] * n[1] * a
    
    # First basis vector
    b1 = np.array([
        1.0 + sign * n[0] * n[0] * a,  # x-component
        sign * b,                        # y-component
        -sign * n[0]                     # z-component
    ])
    
    # Second basis vector
    b2 = np.array([
        b,                               # x-component
        sign + n[1] * n[1] * a,         # y-component
        -n[1]                            # z-component
    ])
    
    return b1, b2



if __name__ == "__main__":

    n = np.array([0.0, 0.0, 0.0])
    n = n / np.linalg.norm(n)

    b1, b2 = branchless_onb(n=n)


    testPoint = 0

