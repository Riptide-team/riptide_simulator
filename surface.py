import matplotlib.pyplot as plt
import numpy as np
from scipy.special import jn, jn_zeros


def fun(x, y, slope = 0.3, depth = 10):
    a = np.sin(np.sqrt(x**2+y**2)) + slope * np.sqrt(x**2+y**2) - depth
    return np.minimum(np.zeros(a.shape), a)


# Allow calculations up to m = mmax
def displacement(n, k, r, theta, t, slope = 0.6, depth = 10):
    # Pick off the mth zero of Bessel function Jn
    zero = jn_zeros(n, k)[-1]
    a = 6 * jn(n, r) * np.sin(n*theta) * np.cos(t) + slope * r - depth
    return np.minimum(np.zeros(a.shape), a)


if __name__ == "__main__":

    # # First try
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # x = y = np.arange(-30, 30, 0.5)
    # X, Y = np.meshgrid(x, y)
    # zs = np.array(fun(np.ravel(X), np.ravel(Y)))
    # Z = zs.reshape(X.shape)
    # ax.plot_surface(X, Y, Z)
    
    # Second try
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    n, k = 3, 3
    r = np.linspace(0, 2*jn_zeros(n, k)[-1], 100)
    theta = np.linspace(0, 2 * np.pi, 100)

    # Create arrays of cartesian co-ordinates (x, y) ...
    x = np.array([rr*np.cos(theta) for rr in r])
    y = np.array([rr*np.sin(theta) for rr in r])
    z = np.array([displacement(n, k, rr, theta, 0) for rr in r])
    ax.plot_surface(x, y, z)

    plt.show()
