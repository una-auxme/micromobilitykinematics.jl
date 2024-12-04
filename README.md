# micromobilitykinematics


[![Documentation](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml/badge.svg)](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml)

The package provides a range of functions for the manipulation of the kinematic properties of the steering geometry and suspension. 


## Functionality 
The package offers a variety of functions for the manipulation of the kinematic properties of the steering geometry and suspension.
It is possible to calculate disparate steering settings and subsequently view pertinent data within a single instance.
Furthermore, the steering geometry can be modified by altering the parameter values in order to optimise it if necessary. The optimisation is primarily based on the JuMP.jl package and utilises the Ipopt algorithm. The accuracy of the steering, along with other pivotal properties, can be depicted in plots to illustrate the resulting characteristics. Additionally, this package possesses the capability to calculate essential positions and parameter values essential for kinematic calculations as XML files. This facilitates, for instance, the immediate transfer of the steering geometry to CAD programs, which can be updated with the aid of the calculation.



 

## How can I use micromobilitykinematics?
The code presented here is intended for illustrative purposes only. The accompanying documentation provides illustrative examples of potential applications, which offer a more comprehensive understanding of the package's functionality.

```julia 

# initialisation of the steering
x_rotational_radius = 56
z_rotational_radius = 165
track_lever_length = 185
tie_rod_length = 210
steering = Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)

# initialisation of the suspension
compression = 30 # neutral damper positioning
suspension = Suspension(compression)


# steering setting
angleConfig = (0,10)

suspensionkinematics!(suspension)
steeringkinematics!(angleConfig, steering, suspension)

# new steering setting 
new_angleConfig = (0,25)

# update steering kinematics 
update!(new_angleConfig, steering, suspension)

# exports xml list to path = @__DIR__ (default)
exportXML(steering)
exportXML(suspension)
```