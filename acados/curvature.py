import numpy as np
import matplotlib.pyplot as plt
import BicycleModel

[_, xref, yref, _, kapparef] = BicycleModel.getTrack("LMS_Track.txt")

def Curvature(xref, yref):

    x_t = np.gradient(xref)
    y_t = np.gradient(yref)

    vel = np.array([ [x_t[i], y_t[i]] for i in range(x_t.size)])

    speed = np.sqrt(x_t * x_t + y_t * y_t)

    tangent = np.array([1/speed] * 2).transpose() * vel

    ss_t = np.gradient(speed)
    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)

    curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t)**1.5

    return curvature_val

def PJcurvature(xref,yref):
    """
    input  : the coordinate of the three point
    output : the curvature and norm direction
    refer to https://github.com/Pjer-zhang/PJCurvature for detail
    """
    curvature = []

    for i in range(len(xref[:-2])):
        x = xref[i:i+3]
        y = yref[i:i+3]

        t_a = np.linalg.norm([x[1]-x[0],y[1]-y[0]])
        t_b = np.linalg.norm([x[2]-x[1],y[2]-y[1]])
        
        M = np.array([
            [1, -t_a, t_a**2],
            [1, 0,    0     ],
            [1,  t_b, t_b**2]
        ])

        a = np.matmul(np.linalg.inv(M),x)
        b = np.matmul(np.linalg.inv(M),y)
        

        kappa = -2*(a[2]*b[1]-b[2]*a[1])/(a[1]**2.+b[1]**2.)**(1.5)
        curvature.append(kappa)
    print(a)
    print(b)
    
    return curvature

curvature = np.round(PJcurvature(xref, yref))

print(curvature)
#print(kapparef)


# curvature_val = np.round(Curvature(xref, yref))

# print(np.round(curvature_val))
# print("\n")
# print(kapparef)

# plt.plot(xref[0:], yref[0:])
# plt.show()

# print(np.abs(kapparef) == np.abs(curvature_val))