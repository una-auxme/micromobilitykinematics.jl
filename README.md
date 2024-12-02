# micromobilitykinematics


[![Documentation](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml/badge.svg)](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml)

The package provides a range of functions for the manipulation of the kinematic properties of the steering geometry and suspension. 


# Functionality 
The package offers a variety of functions for the manipulation of the kinematic properties of the steering geometry and suspension.
It is possible to calculate disparate steering settings and subsequently view pertinent data within a single instance.
Furthermore, the steering geometry can be modified by altering the parameter values in order to optimise it if necessary. The optimisation is primarily based on the JuMP.jl package and utilises the Ipopt algorithm. The accuracy of the steering, along with other pivotal properties, can be depicted in plots to illustrate the resulting characteristics. Additionally, this package possesses the capability to calculate essential positions and parameter values essential for kinematic calculations as XML files. This facilitates, for instance, the immediate transfer of the steering geometry to CAD programs, which can be updated with the aid of the calculation.
 
# First Steps 
## 1. Initialisation
The initial stage of the process is to initialise all of the requisite sub-subsystems of the vehicle that directly influence the behaviour of the steering mechanism. 

### Steering

The initial step is to commence the configuration of the steering geometry, which entails defining the dimensions of the constituent components. The dimensions exert a direct impact on the kinematic characteristics of the steering mechanism. To gain a more comprehensive grasp of the steering geometry, it is recommended to consult the accompanying documentation, which provides a comprehensive and detailed account of the subject matter.

It is recommended that a random search or direct selection of functioning parameters be employed, given that not all parameter values will necessarily result in the desired functioning kinematics.

The steering class itself includes all the important joint positions and component dimensions. This allows different steering positions to be identified and compared.

```julia 
# random search, with the following argument
lower_bourder = (50.0, 50.0, 70.0, 195.0)
upper_bourder = (100.0, 200.0, 200.0, 260.0)
max_angleConfig = (10,35)

param = random_search(upper_bourder, lower_bourder, max_angleConfig)

# Initialisation of the steering, with the following argument
x_rotational_radius = param[1]
z_rotational_radius = param[2]
track_lever_length = param[3]
tie_rod_length = param[4]

steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
``` 
### Suspension

The compression of the damper also exerts an influence on the steering geometry, as it alters the direct positioning of the essential joints. 

```julia
# Initialisation of the suspension, with the following argument
compression = 30 # neutral damper positioning

suspension = Suspension(compression)
```

### Chassi
The chassis class includes the fundamental dimensions of the vehicle's chassis. These parameters are instrumental in calculating kinematic variables.
```julia
# Initialisation of the Chassi
chassi = Chassi()
```

## 2. Calculation of the kinematics
Once the individual components have been initialised, the steering geometry can be calculated depending on a steering setting     `angelConfig`. However, prior to this, the kinematics of the suspension must be calculated, as this has a direct influence on the subsequent calculation by changing the positions of the wishbones.


```julia
# steering setting
angleConfig = (0,10)

suspensionkinematics!(suspension)
steeringkinematics!(angleConfig, steering, suspension)

```


## 3. Updating the steering geometry positioning
Updating of steering geometry positioning by change in steering angle

```julia 
# new steering setting 
new_angleConfig = (0,25)

update!(new_angleConfig, steering, suspension)
```

## 4. Export 

```julia 
# exports xml list to path = @__DIR__ (default)
exportXML(steering)
exportXML(suspension)

```

# Optimization
