import numpy as np
import matplotlib.pyplot as plt
import BicycleModel

[sref, xref, yref, _, _] = BicycleModel.getTrack("LMS_Track.txt")


def trackLength(xref, yref):

    s = np.zeros(len(xref))

    for i in range(len(xref[:-1])):
        point1 = np.array((xref[i], yref[i]))
        point2 = np.array((xref[i+1], yref[i+1]))
        distance = np.linalg.norm(point2-point1)
        s[i + 1] = distance + s[i]


    
    return s

length = trackLength(xref, yref)

print(length)