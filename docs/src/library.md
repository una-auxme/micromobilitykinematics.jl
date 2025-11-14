# micromobilitykinematics.jl Library Functions
This section outlines the library's functions along with their doc strings, enabling users to understand and leverage the capabilities of each function effectively.

## User level
The User Level functions offer an intuitive interface tailored for non-expert users. These functions simplify complex kinematic computations and visualizations, making the library accessible for practical applications in micromobility design, simulation, and optimization. By organizing functions into categories such as core calculations, plotting, and data export, this section helps users quickly locate the tools they need.

### core functions
The core functions in the micromobilitykinematics.jl library form the foundation for kinematic calculations and updates. They include functions for computing steering and suspension kinematics in both neutral and moved states. Additionally, the library provides angle_δi and angle_δo functions to calculate the steering angles of the inner and outer wheels, respectively. Together, these tools offer a robust framework for analyzing and simulating mechanical systems.

```@docs
random_search
steeringkinematics!
steeringkinematics
steeringkinematicsNEUTRAL!
steeringkinematicsMOVED!
suspensionkinematics!
suspensionkinematicsNEUTRAL!
suspensionkinematicsMOVED!
update!
angle_δi!
angle_δi
angle_δo!
angle_δo
```

### Optimization
The Optimization functions in the micromobilitykinematics.jl library are designed to help users optimize various parameters related to micromobility vehicle steering kinematics. These functions include tools for performing series optimizations (optim_series), grid-based optimization searches (grid_optim), and objective calculations (steering_objective, objective°). Additionally, checkConstraints and checkConstraints° ensure that optimization results satisfy necessary system constraints.
```@docs
create_model_for_pose
get_model_solution
optim_at_pose
optim_series_at_pose
grid_optim
create_model_for_range
optim_over_range
checkConstraints
checkConstraints°
ackermann_deviation
ackermann_deviation_for_pose
ackermann_deviation_over_range
```


### GUI
This package provides an extension that depends on GLMakie.
Because of this dependency, the interactive GUI will only work if GLMakie is installed beforehand.
Once GLMakie is available in the environment, the graphical user interface can be activated through the following function:
```@docs
lounch_gui
```



### Export
The export function exportXML enables users to efficiently save and share their configurations in an XML format. This functionality is particularly useful for integrating results into other software workflows or for long-term archiving, ensuring compatibility and traceability across platforms and projects.
```@docs
exportXML
```


