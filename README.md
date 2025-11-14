# micromobilitykinematics.jl
![](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/A9vugjjp_s38grl_hy8.jpg?raw=true)


| | |
|---|---|
|Documentation: |  [![Build Docs](https://github.com/una-auxme/micromobilitykinematics.jl/actions/workflows/Documentation.yml/badge.svg)](https://github.com/una-auxme/micromobilitykinematics.jl/actions/workflows/Documentation.ym) [![Dev Docs](https://img.shields.io/badge/docs-dev-blue.svg)](https://una-auxme.github.io/micromobilitykinematics.jl/) |
| Organisiation: | [![](https://github.com/una-auxme/micromobilitykinematics.jl/blob/main/docs/src/assets/149701353_V2.png?raw=true)](https://github.com/una-auxme) |


## ✨ What is micromobilitykinematics.jl?

`micromobilitykinematics.jl` provides a comprehensive toolkit for analysing, simulating, and 
optimising the **steering** and **suspension** kinematics of micromobility vehicles.  
It is designed to support concept development, geometric evaluation, research, and CAD-integrated workflows.

### 🔧 Core Capabilities

| Feature | Description |
|--------|-------------|
| 📐 **Steering Kinematics** | Computes inner/outer steering angles, joint positions, rotational axes and UCS information for arbitrary steering geometries. |
| 🦾 **Suspension Kinematics** | Models a double-wishbone suspension with dampers and a wheel mount; evaluates both neutral and dynamic states. |
| 🎯 **Optimisation** | Uses JuMP + Ipopt to optimise steering geometries over configurable steering ranges and parameter bounds. |
| 📤 **XML Export** | Exports all steering and suspension attributes to XML for seamless reconstruction in CAD systems. |
| 🖥️ **Interactive GUI** | GLMakie-based GUI enabling real-time parameter manipulation, visual feedback and immediate geometric analysis. |
| ⚠️ **Error Handling** | Structured diagnostic system providing error messages, traces and root-cause information. |

### 🎛️ What is the package designed for?

The package computes both **neutral** and **moving** kinematic states and exposes all relevant information  
directly via the instances — including wheel angles, component coordinates, joint geometry, and 
local coordinate systems. This enables transparent inspection of the geometry and supports automated 
engineering pipelines such as CAD updates or optimisation loops.

Its optimisation routines allow the steering geometry to be tuned by altering key parameters, 
evaluating steering accuracy or other metrics over a defined angle range.  
Visualisation tools complement this workflow by plotting characteristic steering behaviour.

The optional GUI provides an intuitive interface for exploring geometry configurations, offering 
live updates, plotting capabilities and clearly summarised state information.

> **Note:**  
> Together, these components form a coherent and extendable framework for micromobility vehicle 
> geometry design — from numerical analysis to interactive visual exploration.




# 🚀 How can I use micromobilitykinematics.jl?

This section provides a guided introduction to installing the package, running basic kinematics, and extending the workflow with optimisation and GUI interactions.  
The examples are intentionally minimal and can serve as templates for your own geometric setups.

---

## 📦 How to get started?
To install the package directly from the repository, run:
```julia 
    pkg> add https://github.com/una-auxme/micromobilitykinematics.jl.git
```
Then load the package:
```julia 
    using GLMakie
    using micromobilitykinematics
```
> **Info**  
> `GLMakie` is not required for the core kinematics functionality.  
> It is only needed for the optional GUI extension.
---

### 🔬 How does basic kinematics function work?
This example shows the minimal workflow to initialise a steering geometry, evaluate both suspension and steering kinematics, and export the resulting geometry for CAD use.

```julia 
    using micromobilitykinematics

    # initialisation of the steering 
    xR = 56.0 # Steeringarm_rotX 
    zR = 165.0 # Steeringarm_rotZ 
    leverL = 185.0 # Steeringlever 
    rodL = 210.0 # Steeringrod 

    steering = Steering(xR, zR, leverL, rodL)
    suspension = Suspension((30.0,30.0))

    angleConfig = (0.0, 0.0, 10.0)

    suspensionkinematics!(suspension)
    steeringkinematics!(angleConfig, steering, suspension)

    update!((0.0, 0.0, 25.0), steering, suspension)


    exportXML(steering)
    exportXML(suspension)
```
> **Note:**  
> `suspensionkinematics!` should be called **before** `steeringkinematics!`,  
> because the steering kinematics depend on the current suspension state  
> (wishbone positions, damper compression, wheel-ucs alignment).

> **Info:**  
> After running both kinematics, the instance contains **neutral** and **current** state information.  
> The XML export always includes both states, making it suitable for CAD reconstruction.

---

## 🎯 How to optimise a steering geometry

The optimisation system in `micromobilitykinematics.jl` allows automatic tuning of the
steering parameters using JuMP + Ipopt.

This workflow identifies feasible parameters and then refines them to minimise steering
error across a defined angle range.

```julia
    using micromobilitykinematics
    
    lower = (50.0, 50.0, 70.0, 195.0)
    upper = (100.0, 100.0, 200.0, 260.0)
    maxθ  = (15.0, 1.0, 35.0)

    p = random_search(upper, lower, maxθ)
    opt = optim_over_range(p..., maxθ)

    opt.input
    opt.objective
    opt.status

```

> **Note:**  
> The optimisation evaluates the steering geometry at every angle inside maxθ.    
> This means the returned configuration is not optimised for just one steering angle,
> but for the entire steering motion.  
> This makes the result more robust and suitable for real vehicle applications.

> **Info:**  
> Besides the high-level `optim_over_range` interface, `micromobilitykinematics.jl` also exposes  
> lower-level optimisation helpers such as `create_model_for_pose`, `optim_at_pose`,  
> `optim_series_at_pose`, `grid_optim`, and `create_model_for_range`.  
> Use them when you need pose-specific optimisation, parallel runs, or grid-based studies over many steering angles.


---

### 🔬 How does an advanced workflow look like?
Use this workflow when exploring new steering concepts or evaluating the influence of geometry changes on kinematic accuracy.
```julia 
    using micromobilitykinematics
    using GLMakie

    lower = (50.0, 50.0, 70.0, 195.0)
    upper = (100.0, 100.0, 200.0, 260.0)
    maxθ  = (15.0, 1.0, 35.0)

    p = random_search(upper, lower, maxθ)
    xR, zR, leverL, rodL = p

    opt = optim_over_range(xR, zR, leverL, rodL, maxθ)

    steering    = opt.steering
    suspension  = Suspension((30,30))
    chassis     = Chassis()

    update!((0.0, 1.0, 0.0), steering, suspension)

    lounch_gui((15.0, 5.0, 35.0), chassis, steering, suspension)

```
> **Note:**  
> The `random_search` step only identifies *feasible* parameter sets inside the given bounds.  
> The actual optimisation happens in `optim_over_range`, which refines these parameters to minimise the steering error across the full angle  
> range defined by `maxθ`.

> **Info:**  
> Before launching the GUI, make sure to call `update!` with a valid steering  
> configuration. The GUI displays **the current kinematic state** and relies on  
> the internally updated suspension and steering geometry.

---

## 📊 Type Hierarchy Diagram

Below is a compact overview of the internal type structure used in `micromobilitykinematics.jl`.  
It shows how steering, suspension and vehicle components are organised and how the optimisation and error system fit into the model.

```text
micromobilitykinematics.jl
├── ⚙️ Steering System
│   ├── Steering
│   │   Main container for all steering-related states, geometry & kinematics.
│   ├── RotationalComponent
│   │   Defines steering arm radii and pivot spacing.
│   ├── TrackLever
│   │   Length-parametrised steering lever.
│   ├── TieRod
│   │   Connects steering links; determines toe behaviour.
│   └── InitSteeringParam
│       Stores original input values for reconstruction & optimisation.
│
├── 🦾 Suspension System
│   ├── Suspension
│   │   Wrapper holding left/right wishbones, damper and wheel mount.
│   ├── LowerWishbone
│   │   Lower trapezoid control arm with damper fixture + joint geometry.
│   ├── UpperWishbone
│   │   Upper control arm including tilt orientation and linkage geometry.
│   ├── Damper
│   │   Spring-damper unit with compression, travel and fixture points.
│   └── WheelMount
│       Wheel carrier with offsets, camber angle and mounting geometry.
│
├── 🚗 Vehicle Assembly
│   ├── Chassis
│   │   Base geometry of the vehicle chassis (e.g., width).
│   ├── Measurements
│   │   Derived vehicle metrics: track width, wheelbase, turning radius.
│   └── Vehicle
│       Full vehicle representation combining chassis, steering & suspension.
│
├── 🎯 Optimisation
│   └── OptDa
│       Container for storing optimisation results (input, steering, objective, status).
│
└── ⚠️ Diagnostics
    └── ErrorInfo
        Structured error messages, backtraces and root-cause information.
```

## Related publications?

| | |
|---|---|
| autor: |  Peter Krönes , Adrian Brune and Lars Mikelsons |
| title: |  Design and Optimization of a Steering Geometry for a Micromobility|
| journal: | Zehnte IFToMM D-A-CH Konferenz 2024 : 05./06. März 2024, Universität Rostock|
| date of publication: | 04.03.2024|
| url:| [DOI: 10.17185/duepublico/43383](https://doi.org/10.17185/duepublico/81695) |
| zite: | [BibTex](https://duepublico2.uni-due.de/receive/duepublico_mods_00081695?XSL.Transformer=bibtex) | | 