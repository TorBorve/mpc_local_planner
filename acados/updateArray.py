import numpy as np
import curvature
import trackLength
import BicycleModel

#[s, xref, yref, _, kappa] = BicycleModel.getTrack("LMS_Track.txt")

s = 0
xref = 1
yref = 2
kappa = 3

def updateArray(s, xref, yref, kappa):
    array = np.array([s, xref, yref, kappa])
    for i in range(10):
        to_add = np.array([s, xref, yref, kappa])
        array = np.vstack((array ,to_add))
    
    
    

    return array

print(updateArray(s, xref, yref, kappa))

