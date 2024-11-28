# micromobilitykinematics


[![Documentation](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml/badge.svg)](https://github.com/adribrune/micromobilitykinematics.jl/actions/workflows/Documentation.yml)

The package provides a range of functions for the manipulation of the kinematic properties of the steering geometry and suspension. 


# Functionality 
The package offers a variety of functions for the manipulation of the kinematic properties of the steering geometry and suspension.
It is possible to calculate disparate steering settings and subsequently view pertinent data within a single instance.
Furthermore, the steering geometry can be modified by altering the parameter values in order to optimise it if necessary. The optimisation is primarily based on the JuMP.jl package and utilises the Ipopt algorithm. The accuracy of the steering, along with other pivotal properties, can be depicted in plots to illustrate the resulting characteristics. Additionally, this package possesses the capability to calculate essential positions and parameter values essential for kinematic calculations as XML files. This facilitates, for instance, the immediate transfer of the steering geometry to CAD programs, which can be updated with the aid of the calculation.
 
# Usage 