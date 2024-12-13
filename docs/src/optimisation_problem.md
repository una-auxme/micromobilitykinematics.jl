# Optimisation Problem 
Prior to the optimisation of the components, the formulation of the optimisation problem is a crucial preliminary step. Following the mathematical definition of the optimisation problem, the objective function with associated restrictions is then sought. In essence, the optimisation problem should be formulated in the following manner:

```math
\begin{array}{ccc}
    \hline
    \textbf{(MP)} & \textbf{min}  & f(x) \\
    \hline
     & \textbf{unter}& g_i(x) \leq 0 \\
                   & & \text{für } i = 1, \ldots, p \\
    \hline
                   &  &h_j(x) = 0 \\
                   &  &\text{für } j = 1, \ldots, m \\
    \hline
                   &  &x \in X \subseteq \mathbb{R}^n \\
    \hline
\end{array}
```

The following section will examine the potential objective functions, f(x), and conditions that are ultimately employed in the formulation of an optimisation problem.

## Derivation of the target function

The optimisation employs the ratio between the distance L of the intersection point C and the wheelbase w of the vehicle. It is assumed that the inner steering angle is identical for both the steering system in use and the Ackermann steering system.

The comparison of a real steering system with the Ackermann steering system depicted in Figure 3.9 yields three distinct scenarios. The optimal configuration is defined by the Ackermann operation, in which both wheel axles of the front wheels intersect on the rear wheel axle. The remaining two scenarios are distinguished by the presence of intersection points C, situated at a distance y from the rear wheel axle. It can be seen that the distance L can be both larger and smaller than the ideal value w. Therefore, the ratio is not a meaningful objective function for optimisation. It is not possible to approximate both sides to the value 1 for a minimisation or maximisation problem. To avoid this issue, the absolute difference of the distances w and L is chosen as the objective function.

The following sections present the derivation and calculation of the distance L, with reference to Figure 3.9.

![Figure 3.9: Graphical representation of the derivation using the Ackermann condition](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Graphical_representation_of_the_derivation_using_the_Ackermann_condition.png)

### Calculation of the steering angle

The calculation of the intersection points C of the wheel axles is essential for the continuous derivation, as these points depend on the steering angles of the inner δi and outer δo wheels. It is already established that the kinematic chain is influenced by the effect of the L-component, whereby characteristic points of the steering geometry are shifted. The steering lever mounted on the wheel is either pulled or pushed by the tie rod, depending on the steering setting, which in turn leads to a change in the wheel suspension. The kinematic model of the steering system already describes the positions of the components with individual steering adjustment options.

![Figure 3.10: Graphical determination of the steering angle δ](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Graphical_determination_of_the_steering_angle_δ.png)

In Figure 3.10, the angles designated as $\delta$ represent the angular displacement of the wheel in relation to the initial position of the steering geometry. Given the direct linkage between the steering lever l and the wheel mount M, the transition from position L1 to L2 can also be interpreted as an angular change between the steering levers $l_1$ and $l_2$. Consequently, the included angle of the steering lever vectors $l_1$ and $l_2$ is to be determined.

```math
\delta = \arccos\left(\frac{\mathbf{l}_1 \cdot \mathbf{l}_2}{|\mathbf{l}_1| \cdot |\mathbf{l}_2|}\right) \tag{3.13}
```

In accordance with the established criteria, the positions of the joints $L_1$ and $L_2$, as well as the position of the centre of rotation $M$, are known solely within the previously defined reference coordinate systems. Consequently, the vectors $l_1$ and $l_2$ must initially be determined.

```math
\mathbf{l}_1 = \mathbf{o}_M - \mathbf{o}_{L_1}
```

```math
\mathbf{l}_2 = \mathbf{o}_M - \mathbf{o}_{L_2}
```

### Positioning the reference coordinate system

The positioning of the reference coordinate system exerts a more pronounced influence on the determination of the values in the derivation. The dependence of the target function on the Ackermann operation plays a pivotal role in the decision-making process. Consequently, the comparison of distances and angles is of paramount importance, which is why the intersection point Ca of an ideal steering system was selected as the centre of the reference coordinate system. This can be observed graphically in Figure 3.9.


### Transferring the axes to the reference coordinate system

The axes of the wheels can be described in a coordinate system as a linear function of the following form:

```math
i(x) = m_i x + b_i \quad  o(x) = m_o x + b_o 
```

In this equation, the variable "i" is used to denote the inner wheel, while "o" is used to denote the outer wheel. The gradient m of the functions is derived from the right-angled triangle formed by the x-axis and the wheelbase w. Given the parallelism of the equations y = w and y-axis, there is an alternating angle at the intersection of the x-axis, which is composed of 90° + δ. It can therefore be concluded that the turning angle δo or δi is situated within the triangle of inclination. In Figure 3.9, the alternating angle is highlighted in green and orange. The following applies to the slope of the functions:


```
m_i = \frac{\Delta y_i}{\Delta x_i} = -\frac{w}{\frac{w}{\tan \delta_i}} = -\frac{1}{\tan \delta_i}, \quad
m_o = \frac{\Delta y_o}{\Delta x_o} = -\frac{w}{\frac{w}{\tan \delta_o}} = -\frac{1}{\tan \delta_o} \tag{3.14}
```

In order to transform the axes into linear functions within the reference coordinate system, it is necessary to ascertain the respective displacements along the y-axis. It should be noted that the intersection point of function $i(x)$ is at the origin, which eliminates the requirement for a shift in this instance. By utilising the known values of the gradient triangle, it is possible to determine the distance of the intersection points of the functions with the x-axis, which subsequently represents the x-coordinate of the position.

```math
x = \Delta x_o - (\Delta x_i + T) \tag{3.15}
```

Given that the y-value must be zero for an intersection of the x-axis and the positions are known, the constant can be determined as follows:

### Calculation of the intersection point

The representation of the wheel rotation axes by linear functions facilitates the search for the intersection point C. The determination procedure is based on the equality of the coordinates at this point, which allows for the establishment of the following equation.

```math
\begin{align*}
i(x) &= o(x) \\
m_i x &= m_o x + b_o \\
x (m_i - m_o) &= b_o \\
x &= \frac{b_o}{m_i - m_o} 
\end{align*} \tag{3.17}
```


The y-value is calculated by inserting the x-coordinate into one of the two linear functions, which are defined by the following equations:


```math
\begin{align*}
y &= m_o x + b_o \tag{3.18} \\
  &= -\frac{1}{\tan \delta_o} \cdot \frac{b_o}{m_i - m_o} + \left(\frac{1}{\tan \delta_o} \left( \Delta x_o - (\Delta x_i + T\right) \right)
\end{align*}
```


### Calculation of the distance L

The distance between the front axle and the point of intersection of the wheel axle is represented by the variable L. This variable is constituted by the ideal distance w and the y-coordinate of the aforementioned point of intersection.

```math
L = w + y \tag{3.19}
```


This function describes the absolute difference between the intersection points, with the ratio of which is ideally $1$ or $100\%$.


## Conditions of the optimisation problem

In order to identify the constraints of the optimisation problem, it is necessary to study critical scenarios and formulate constraints based on their mathematical relationship. Given the increasing complexity resulting from the addition of a further operational area with the use of both steering angles θx and θz, it is more intuitive to adopt a purely planar view of the steering geometry for the time being. Subsequently, the developed solution approaches are transferred to a three-dimensional problem.

### Limits of the optimisation parameters and constants

A cursory examination may lead to the erroneous conclusion that the angle range considered is too narrow. This is related to the prioritisation of the optimisation to ensure stable driving behaviour at higher speeds. Consequently, it is necessary to optimise the steering geometry for the angular range that occurs more frequently at higher speeds. Two constants of the optimisation are directly related to the angular range and together form the limits of the optimisation. The angle tuple (θx, θz) describes the angle setting of the L-component and are elements of the following angle set:

```math
\{ (\theta_x, \theta_z) \mid (\theta_x, \theta_z) \in \mathbb{Z} \times \mathbb{Z}, \ -10 \leq \theta_x \leq 10, \ -35 \leq \theta_z \leq 35 \}
```

The symmetrical configuration of the steering geometry components allows for the optimisation to be conducted with respect to a single steering direction. Consequently, an angular range can be defined for the optimisation.

```math
\{ (\theta_x, \theta_z) \mid (\theta_x, \theta_z) \in \mathbb{Z} \times \mathbb{Z}, \ 0 \leq \theta_x \leq 10, \ 0 \leq \theta_z \leq 35 \} \tag{3.21}
```

Another variable is the potential track circle radius (Rs) of a vehicle, which must be maintained throughout a steering movement. Vehicles with comparable dimensions have a track circle radius of between $2000 mm$ and $3000 mm$, which has also been adopted. 

In order to prevent potential oversizing of the components, restrictions are imposed on the optimisation parameters. A length range of $10mm$ to $260mm$ is specified for the tie rod in order to ensure that the tie rod does not exceed the track length of the vehicle. A tie rod that exceeds the track of the vehicle has the potential to come into contact with the wheel to be steered. Another parameter is the length of the steering lever, which may range from $40 mm to 200 mm$. This should prevent undersizing and ensure the existence of a steering lever. It should be noted that the maximum permissible value serves merely as a rough guideline that should not be exceeded. The two rotation radii of the L-component contribute significantly to the steering behaviour, which is why their lengths are also taken into account in optimisation. The limits are chosen to be similar to the steering lever and are between $50mm$ and $200mm$. It is possible that this has already been amended.


### Track circle condition

The term "track circle" is used to describe the circular path of the wheel that is furthest from the centre of rotation in a drive train. In the absence of a defined desired turning circle for the vehicle, the resulting steering geometry will only encompass a portion of the specified range of the turning angle spectrum. The relationship between the steering angle and the toe circle can be derived using the definition of Ackermann steering geometry. If Figure 3.9 depicts the maximum deflection of the front wheel, then the centre of rotation $(C_a)$ of an ideal Ackermann steering system is directly correlated with the smallest possible track circle. In an ideal scenario, the wheel axle intersects the rear wheel axle at the centre of rotation of the vehicle, with their connection corresponding to the track circle radius $(r_s)$. Consequently, the following condition can be formulated:

```math
\delta_o \geq \sin^{-1} \left( \frac{w}{r_s} \right) \tag{3.22}
```


It is essential that the maximum steering angle of the dimensioned steering geometry is greater than that of the ideal Ackermann steering geometry, which has been adapted to align with the dimensions of the micromobility vehicle.

### Conditional part of the Ackerman condition

The relationship between the inner δi and outer δo angles of incidence of the deflected wheel has already been expressed mathematically by the Ackermann equation (Equation 2.1), whose visual relationship is shown in Figure 2.1. It can be observed from the figure that the inner angle of incidence should be greater than the outer angle of incidence of the wheel at all times. This implies that the following condition must be met for the optimisation to be achieved:

```math
\delta_o \leq \delta_i \tag{3.23}
```


It is necessary that the current steering angle $(\delta_i)$ of the inner wheel be greater than the steering angle $ (\delta_o)$ of the outer wheel, with the same steering geometry dimensions.


### Interaction reliability of the components

Should the dimensions of the tie rod and steering lever prove insufficient, the kinematic chain may be subject to interruption. It is not feasible to ascertain an intersection point between the circular trajectory of the steering lever and the movement domain of the tie rod in this particular instance. It is recommended that Figure 3.8, which illustrates the graphical determination, be consulted once more in order to gain further insight into this connection. If either the radius of the drawbar sphere coloured in purple or the circular path of the steering lever are too short, an interaction of both movements is precluded. The same applies if the drawbar or the steering lever are oversized so that the circular path of the steering lever is enclosed by the drawbar sphere, or if the opposite situation arises. For an intuitive formulation of a restriction of this scenario, the problem is transferred to a two-dimensional system. Figures 3.11, 3.12 and 3.13 illustrate the plane on which the circular path of the steering lever is located, with the section containing the tie rod sphere depicted in purple. Figure 3.11 illustrates the scenarios that must be avoided in order to optimise the tie rod, with the known positions labelled. To ensure interaction, the shortest distance d between the centre of rotation M and the joint P must be smaller than the sum of the radii of both circles. Similarly, the radius of the sphere must not exceed the sum of the shortest distance d and the radius of the steering lever. Both scenarios are colour-coded in the figure, with blue representing the minimum dimensioning and green indicating the maximum dimensioning.

```math
\begin{align*}
|d| & \leq |l| + |z| \\
|d| - (|l| + |z|) & \leq 0  \tag{3.24}
\end{align*}
```

```math
\begin{align*}
|z| & \leq |d| + |l| \\
|z| - (|d| + |l|) & \leq 0 \tag{3.25}
\end{align*}
```

The dimensioning of the steering lever, whose scenarios are illustrated in Figure 3.13, should be considered in a similar manner. To guarantee that the specified circles can also interact, the aforementioned condition can be immediately implemented. The limitation of the maximum dimensioning is based on a comparable methodology. In the event that the intersection of the spheres is situated within the circle of the pull lever, no interaction is feasible.

```math
\begin{align*}
|d| & \leq |l| + |z| \\
|d| - (|l| + |z|) & \leq 0 \tag{3.26} \\
\end{align*}
```
```math
\begin{align*}
|l| & \leq |d| + |z| \\
|l| - (|d| + |z|) & \leq 0 \tag{3.27}
\end{align*}

```

In light of the subsequent assignment of value ranges to the individual components, the scenario should be disregarded as part of the optimisation process.

![Figure 3.11: Graphic display if the selected lengths are too small](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Graphic_display_if_the_selected_lengths_are_too_small.png)


![Figure 3.12: Graphical representation if the tie rod is too large z](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Graphical_representation_if_the_tie_rod_is_too_large_z.png)


![Figure 3.12: Graphic display if the steering lever is too large l](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Graphic_display_if_the_steering_lever_is_too_large_l.png)


### Avoiding the singularitys

Nevertheless, in the event that a component is significantly longer in comparison to the remaining components, a return effect is observed with regard to the steering lever. To elucidate this phenomenon, a visual illustration is provided in Figure 3.14. Let us assume that the dimensions of the steering lever $(l)$ are significantly longer than the radius of the z-rotation $(r_z)$ of the L-component. A stepwise rotation around the origin of the vector rz with the steering angle θz results in a displacement of the positions along their circular paths, which are represented by the green and orange lines in Figure 3.14. If the direction of rotation of the vector $r_z$ is clockwise, an identical rotation of the vector $l$ would be expected due to the kinematic dependency. However, if the angle of rotation is exceeded, the steering lever $l$ moves in the opposite direction to that of the original rotation. The direction of movement is divided into two phases, with the start, turning and end points shown more intensively in their respective colours and their direction of movement illustrated by arrows. For the steering angles of the wheels, this had the consequence that beyond a certain steering angle, the wheel would steer in the opposite direction, which would have serious consequences for the vehicle dynamics.

![Figure 3.13: Visualisation of the return effect with several steering settings](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Visualisation_of_the_return_effec_with_several_steering_settings.png)

To obviate this potential outcome, it is imperative to ascertain that the extant dimensioning configuration falls within the permissible range for all conceivable setting orientations. In order to define the singularity of the validity range, it is necessary to use a mathematical expression that will allow the establishment of a limit and, consequently, a comparison criterion. In the event that a setting exceeds the aforementioned singularity, it is evident that the dimensioning is not suitable. By comparing the start, turn and end settings of the example, it is possible to derive the following differentiation criterion. During the adjustment process, the steering angle of the wheel increases by δo until the singularity is exceeded. Consequently, the following condition is formulated.

```math
\delta_i < \delta_j, \quad i < j, \quad i, j \in \mathbb{Z} \tag{3.28}
```

The preceding analysis pertains solely to the examination of a steering geometry aspect, namely the variation of the angle of rotation, which allows the formulation of restrictions. It is also important to assess the opposite side in order to optimise the process. Consequently, the previously defined conditions are checked for their compatibility within the aforementioned scenario. The verification procedure is straightforward visually, as demonstrated by the illustration of the aforementioned scenario for the opposite side of the steering geometry (see Fig. 3.15). As the setting angle increases, the included angle δ of the steering lever l also increases, necessitating the fulfilment of condition 3.28 for the right-hand side of the geometry.

![Figure 3.14: Visualisation of the singulatiry with several steering settings on the opposite side of the geometry](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/Visualisation_of_the_sigularity__with_several_steering_settings_on_the_opposite_side_of_the_geometry.png)

All previous scenarios of this condition have been represented exclusively in two-dimensional space. Consequently, any differences in position and their effects on the movement of the angle must be subjected to analysis. In order to achieve this, the principal elements of the scenario depicted in Figure 3.14 are projected onto the y-z plane. As a result of the inclined plane of rotation of the steering lever, there is a height differential between the joints; however, this has no impact on the vectorial determination of the angle $δi$ and $δo$.

![Figure 3.14: Projection of the main points of the scenario onto the y-z plane](/src/assets/Projection_of_the_main_points_of_the_scenario_onto_the_y-z_plane.png)