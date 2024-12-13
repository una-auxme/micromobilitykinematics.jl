# Calculation of the kinematic model
Within the kinematic model, the joint positions are primarily calculated, taking into account variable steering angle settings, either to transfer them to the design environment or to utilize them for optimization purposes.

## Initial Position of the Steering System
The selection of a suitable reference coordinate system is crucial for defining the initial position of key points. Such a coordinate system should easily integrate predefined lengths, widths, and distances and facilitate the calculation of positioning during rotational movements. A suitable point for this purpose is the intersection point of the rotational axes of the steering geometry. Figure illustrates this coordinate system, with the corresponding axes shown in black. Overall, the figure provides a functional representation of the adjustment capability of the steering geometry, with significant information highlighted through color visualization. For determining the initial position of the key reference points, only those vectors represented by a continuous line should be considered.


![Figure 3.2: The functional representation of the setting option rotational component](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/rotational%20component.png)


The radius by which the rotational component can rotate around the x-axis lies in the rest position along the z-axis.
rest position along the z-axis and is shown in Figure 3.2 as a red continuous vector.
As there is no further angle of rotation in the rest position and there is a direct dependence of the position of the radii exists, the radius of rotation is represented by a vector (blue continuous vector) parallel to the x-axis. The coordinates of the vectors can be defined as follows:

```math
\mathbf{r}_{1x} = \begin{pmatrix} 0 \\ 0 \\ -XSteeringRadius \end{pmatrix} \tag{3.1}
```

```math
\mathbf{r}_{1z} = \begin{pmatrix} -ZSteeringRadius \\ 0 \\ 0 \end{pmatrix}
```

Taking into account the rotation around the vector $r^1_x$, the positioning of both joint centers of rotation $P_L$ and $P_R$ (yellow continuous vector) can be determined. By vector addition of the rotation vectors r1x, r1z and taking into accountthe component width b and the joint dimensions d (green continuous vector ) results in the following relationship:


```math 

p_1 = \mathbf{r_x} + \mathbf{r_z} + \begin{pmatrix} 0 \\ \pm(d + (b/2)) \\ 0 \end{pmatrix} 

```


## Positioning with influence of the steering angle

Using the reference values of the initial position, the same characteristic points of the rotational component can now be calculated as a function of the steering angle. The first step involves performing an elementary rotation around the x-axis of the coordinate system, with a predefined angle θx. To map the coordinates of a point in a rotated coordinate system, the elementary rotation matrix is applied to the vector:

```math

r^{(2)}_x = R_x(\theta_x) \cdot \mathbf{r}_x \tag{3.2}

```

The coordinates of the mapped vector represent a vector rotated by the angle θx in the original coordinate system.
All other starting points of the steering system experience an additional rotational movement in which the component can rotate around the vector $r^2_x$. To describe this rotation, a further rotation matrix can be applied to the initial positions. This rotation is not described by an elementary rotation around one of the coordinate axes, but is based on the rotation around a position vector n positioned in space.

```math

R_n(\theta_x) =
\begin{bmatrix}
n_2^2 \cdot (1 - \cos \theta_x) + \cos(\theta_x) & n_1 \cdot n_2 \cdot (1 - \cos(\theta_x)) - n_3 \sin(\theta_x) & n_1 \cdot n_3 \cdot (1 - \cos \theta_x) + n_2 \sin \theta_x \\
n_2 \cdot n_1 \cdot (1 - \cos \theta_x) + n_3 \sin \theta_x & (n_2^2) \cdot (1 - \cos \theta_x) + \cos \theta_x & n_2 \cdot n_3 \cdot (1 - \cos \theta_x) - n_1 \sin \theta_x \\
n_3 \cdot n_1 \cdot (1 - \cos \theta_x) - n_2 \sin \theta_x & n_3 \cdot n_2 \cdot (1 - \cos \theta_x) + n_1 \sin \theta_x & (n_3^2) \cdot (1 - \cos \theta_x) + \cos \theta_x
\end{bmatrix} \tag{3.3}

```

 For the positioning of both joint pivot center point vectors $p^1$:

```math
r^{(2)}_x = R_x(\theta_x) \cdot r^{(1)}_x \tag{3.4}
```

```math
p_2 = R_{r^{(2)}_x} (\theta_x) \cdot p_1
```


## Determination of the joint center of rotation of the track lever

To enable comprehensive modeling of the steering, the positions of the outer joint rotation centers are essential. They link the tie rod and the steering components, facilitating the complete integration of the steering kinematic chain. The initial position discussed previously is now expanded by the kinematic chain, as shown in the figure below. The yellow vector represents the tie rod, which pivots around a center on the blue axis of rotation of the wheel carrier. Due to the symmetric properties of the steering geometry at rest, a one-sided representation was chosen for clarity, as an identical procedure applies to the opposite side. In this illustration, the joints are represented by circles, with the red color indicating the sought component.

![Figure 3.3: The functional representation of the adjustment options for the steering geometry](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/rotation%20of%20track%20lever.png)



### Definition of an independent coordinate system
The joint positions depend on the dynamic movement of the wheel, which is connected through the track lever. The joints move in a circular path around the wheel's axis of rotation. When a steamer is compressed, the wheel suspension's position changes, resulting in an overall displacement of the lever and the required joint center of rotation.

![Figure 3.4: Consideration of a damped wheel suspension](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Consideration_of_a_damped_wheel_suspension.png)

Figure 3.4 illustrates these relationships, with the initial position in blue and displaced elements in green. The green cylinders represent the wishbone joints, allowing rotation around the dashed axes. Vectors \mathbf{ and $\mathbf{q^2}$​ simplify the wishbone positions, ending in joints that form the wheel's axis of rotation (dashed line in Fig. above). Vectors $\mathbf{l^1}$​ and $\mathbf{l^2}$​ indicate the steering arms, with center of rotation marked as $\mathbf{M}$​. The tie rod, shown as a pink dashed line, visualizes the displacement.

Given these considerations, selecting an independent coordinate system is crucial. This system must account for the kinematic chain of the steering geometry and allow for future modifications. A suitable coordinate system can be positioned at the rear lower wishbone bearing, where the dimensions are known, and the damper's dynamic movement can be integrated. In Figure 3.4, this coordinate system, Q-BKS, is represented with black basis vectors and aligned with the rotational component coordinate system, L-BKS, to facilitate the transfer of coordinates for essential points.

### Determining the centre of rotation of the steering lever

In order to ascertain the requisite joint position with due consideration of the effects of damped wheel suspension, it is necessary to exclude the influence of the latter. In an ideal scenario, a known point of wheel suspension would be selected, allowing for a straightforward calculation of the requisite position. Some of the points have already been determined through the calculation of the wheel suspension. These include the positions of the joints (depicted as circles in Fig. 3.4), which permit the wheels to rotate. The skilful choice of the Q-BKS allows for the straightforward tracking of these joints, rendering them an optimal reference point. Furthermore, it is established that the centre of rotation of the track lever is situated on the connecting line of both joints, providing a valuable reference for the z-coordinate axis of the reference coordinate system.

In order to calculate the requisite point, a reference coordinate system D-BKS is positioned in the lower wishbone joint (illustrated by circles in Fig. 3.4), with the direction of the z-axis unit vector coinciding with the axis of rotation of the wheel bracket. Figure 3.5 provides an overview of the orientation and positioning of the coordinate systems defined thus far. In combination with the previous Figure 3.4, it can be seen that the positions are readily comprehensible within the independent coordinate system Q-BKS.

The centre of rotation of the steering lever is situated at a specific distance, designated as $|\mathbf{^Do_M}| = d$, along the z-axis of the D-BKS. It is possible to transform a vector represented in the D-BKS into the Q-BKS by applying the following rotation matrix:

```math
\mathbf{Q}_v = \mathbf{Q}_{\mathbf{R}\mathbf{D}} \cdot \mathbf{D}_v \tag{3.5}
```

```math
=
\begin{bmatrix}
e_{D,x} \cdot e_{Q,x} & e_{D,y} \cdot e_{Q,x} & e_{D,z} \cdot e_{Q,x} \\
e_{D,x} \cdot e_{Q,y} & e_{D,y} \cdot e_{Q,y} & e_{D,z} \cdot e_{Q,y} \\
e_{D,x} \cdot e_{Q,z} & e_{D,y} \cdot e_{Q,z} & e_{D,z} \cdot e_{Q,z} \\
\end{bmatrix}
\cdot
\begin{bmatrix}
x_D \\
y_D \\
z_D \\
\end{bmatrix} \tag{3.6}
```

The rotation matrix $\mathbf{^QR_D}$ is constituted by the scalar products of the base vectors of the reference coordinate systems, reflecting their relationship to each other. It is thus necessary to determine the basis vectors of the D-BKS. The connecting vector of the wishbone joint forms the z-axis and can be determined by a simple vector subtraction of the position vectors. The following therefore applies to the unit vector of the z-axis:


```math
e_{D,z} = \frac{\mathbf{^Qo_{G_1}} - \mathbf{^Qo_{G_2}}}{|\mathbf{^Qo_{G_1}} - \mathbf{^Qo_{G_2}}|} \tag{3.7}
```

The graphical correlation is shown in Figure 3.6.

![Figure 3.6: Graphical derivation of a base vector of the coordinate system D-BKS](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Graphical_derivation_of_a_base_vector_of_the_coordinate_system_D-BKS.png)

The remaining two base vectors of the coordinate system can be determined through the following procedure. For illustrative purposes, please direct your attention to Figure 3.7, which depicts the individual steps in visual form. By employing the previously determined unit vector of the z-axis and the unit vector along the z-axis of the Q-BKS, an imaginary plane can be constructed (depicted in green in Figure 3.7) with the requisite perpendicularity to the axis being sought. To accomplish this, one must simply form the cross product of the two vectors in order to determine a vertical vector at the intersection of the two base vectors. This process is then repeated with the two previously calculated, known base vectors of the coordinate system.

```math
e_{Q,y} = e_{Q,x} \times e_{D,z} \tag{3.8}
```

```math
e_{Q,x} = e_{Q,x} \times e_{Q,y}  \tag{3.9}
```

![Figure 3.7: Method for determining the basis vectors](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Method_for_determining_the_basis_vectors.png)

Zur endgültigen Bestimmung des Drehpunktes wird dessen Positionsvektor $\mathbf{^Do_{M}}$ unter Anwendung von Gleichung 3.5 in das Q-BKS eingespeist. Durch eine zusätzliche Vektoraddition mit dem Positionsvektor des Ursprungs der D-BKS $\mathbf{^QR_{D}}$ ist nun die Lage des Drehpunktes $M_L$ innerhalb der Q-BKS bekannt.

```math
\mathbf{^Qo_{M}} = \mathbf{^QR_{D}} \cdot \mathbf{^Do_{M}} + \mathbf{^Qo_{G_1}} \tag{3.10}

```

### Calculation of the joint position

The preceding steps are essential preliminary procedures that facilitate the calculation of the final, missing component. In conclusion, the L-component was initially modelled by calculating the most significant points within its reference coordinate system. Given the dynamic properties of the wheel suspension and the uncertainty surrounding the influence of subsequent dimensioning on the L-BKS, an independent coordinate system was defined. This Q-BKS enables the tracing of both the position of the L-BKS and the movement of the control arms. In order to calculate the missing joint in an efficient manner, the D-BKS was defined. This involved calculating the centre of rotation of the steering lever and then transforming it into the independent coordinate system. The objective is to combine the two systems, namely the L-BKS and the D-BKS, into a coherent kinematic system with the aim of determining the joint position.

A comprehensive examination of the kinematic system reveals the existence of two additional components whose influence has not yet been incorporated into the existing calculations. Firstly, the rotation of the track lever, whose centre point has been determined thus far, and secondly, the tie rod, which connects both systems and translates the movement of the L-component to the wheel. In the absence of a connection to the track lever, the tie rod is capable of moving freely around the centre of the joint on the L-component, thereby forming a sphere of potential alignments. If the circular path of the track lever intersects the sphere of the tie rod, this describes the exact position of the joint on the track lever. The maximum number of intersection points that can be determined is limited to two possible positions of the joint, one of which must be selected. Figure 3.8 illustrates the relationship in which the component dimensioning enables two intersection points.

## Selection criterion for joint positions

In the preceding section, two potential positionings were identified; a criterion must therefore be formulated to select between them. To this end, the x coordinates of vectors $\mathbf{l^1}$​ and $\mathbf{l^2}$​are compared, as illustrated in Figure 3.8. Within the Q reference coordinate system, it is verified whether the x-coordinate of vector $\mathbf{l^2}$​ is genuinely smaller than that of vector $\mathbf{l^1}$​.

```math
x_{^Qo_{L_2}} - x_{^Qo_M} < x_{^Qo_{L_1}} -x_{^Qo_M} \tag{3.11}
```

```math
x_{l_2} < x_{l_1} \tag{3.12}
```

![Figure 3.8: Visualisation of the intersection points between the sphere and the circular path ](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Visualisation_of_the_intersection_points_between_the_sphere_and_the_circular_path.png)






