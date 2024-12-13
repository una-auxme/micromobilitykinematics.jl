![](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/A9vugjjp_s38grl_hy8.jpg)

| | |
|---|---|
| Documentation | [![Build Docs](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml/badge.svg)](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.ym) [![Dev Docs](https://img.shields.io/badge/docs-dev-blue.svg)](https://thummeto.github.io/FMI.jl/dev/) |

# What is micromobilitykinematics.jl?
The fundamental operations of the micromobilitykinematics.jl library serve as the foundation for kinematic calculations and updates to a self-developed steering geometry of a micromobility vehicle. 

### Core Function
The functions include those for calculating the steering and suspension kinematics in both the neutral and moving state.All important information on the state of the vehicle at a specific steering setting can then be read from the instance. For example, the library offers the possibility to read the steering angles of the inner and outer wheels as well as important joint positions and dimensions of components. In addition, all attributes of the instance can be exported to an XML file. This facilitates, for example, the immediate transfer of the steering geometry to CAD programmes, which can be updated using the calculation.

### Optimization
Furthermore, the steering geometry can be modified by altering the parameter values in order to optimise it if necessary. The optimisation is primarily based on the JuMP.jl package and utilises the Ipopt algorithm. The accuracy of the steering, along with other pivotal properties, can be depicted in plots to illustrate the resulting characteristics. 

Collectively, these tools provide a robust framework for analysing and simulating this mechanical system. 



# How can I use micromobilitykinematics.jl?
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

# How is micromobilitykinematics.jl structured?




# Related publications?

| | |
|---|---|
| autor: |  Peter Krönes , Adrian Brune and Lars Mikelsons |
| title: |  Design and Optimization of a Steering Geometry for a Micromobility|
| journal: | Zehnte IFToMM D-A-CH Konferenz 2024 : 05./06. März 2024, Universität Rostock|
| date of publication: | 04.03.2024|
| url:| [DOI: 10.17185/duepublico/43383](https://doi.org/10.17185/duepublico/81695) |
| zite: | [BibTex](https://duepublico2.uni-due.de/receive/duepublico_mods_00081695?XSL.Transformer=bibtex) | | 

