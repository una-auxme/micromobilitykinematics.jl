# Calculation of the kinematic model
Within the kinematic model, the joint positions are primarily calculated, taking into account variable steering angle settings, either to transfer them to the design environment or to utilize them for optimization purposes.

## Initial Position of the Steering System
The selection of a suitable reference coordinate system is crucial for defining the initial position of key points. Such a coordinate system should easily integrate predefined lengths, widths, and distances and facilitate the calculation of positioning during rotational movements. A suitable point for this purpose is the intersection point of the rotational axes of the steering geometry. Figure illustrates this coordinate system, with the corresponding axes shown in black. Overall, the figure provides a functional representation of the adjustment capability of the steering geometry, with significant information highlighted through color visualization. For determining the initial position of the key reference points, only those vectors represented by a continuous line should be considered.

![](https://github.com/adribrune/micromobilitykinematics.jl/blob/main/docs/src/assets/Screenshot%202024-05-07%20102221.png)

<img src="https://github.com/adribrune/micromobilitykinematics.jl/blob/main/docs/src/assets/Screenshot%202024-05-07%20102221.png" alt="Bildbeschreibung" width="200" height="150">

##