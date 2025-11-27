# 🖥️ GUI — Interactive Steering & Suspension Explorer

The `micromobilitykinematics.jl` package provides an optional **GLMakie-based graphical interface** that allows you to interactively explore and analyse the steering and suspension geometry.  
This chapter explains what the GUI does, how to prepare the required data, and how to use it effectively.

---

## 🎨 1. Purpose of the GUI

The GUI is designed as an **interactive exploration and analysis tool**.  
It visualises the geometric and kinematic relationships between steering and suspension components.

The GUI allows you to:

- view the full steering and suspension geometry in 3D,  
- inspect joint positions, axes, and UCS frames,  
- sweep through steering angles interactively,  
- analyse live plots of kinematic behaviour,  
- validate optimisation results visually,  
- compare neutral and current states of the components.

> **Goal:**  
> Provide an intuitive and immediate understanding of the kinematic system without manually creating plots or examining raw data.

---

## ⚙️ 2. Requirements

The GUI is **optional** and only needs to be loaded when you want to use it.

| Component | Required? | Purpose |
|----------|-----------|---------|
| `micromobilitykinematics.jl` | ✔️ | Kinematics & geometry |
| `GLMakie` | ✔️ (GUI only) | Rendering & interaction |
| GPU / OpenGL backend | Recommended | Smooth 3D performance |

### Installation

```julia
pkg> add GLMakie
```

> **Note**
> The core functionality (kinematics, optimisation, XML export) works without GLMakie.  
> GUI components are fully isolated from the numerical kernel.


## 🛠️ 3. Preparing the GUI

he GUI relies on the current kinematic state of:

- ```Steering```
- ```Suspension```
- ```Chassis```

Before launching the GUI, these must be computed and updated:

```julia
    suspensionkinematics!(suspension)
    update!(angleConfig, steering, suspension)
```

> **Important**
> If these functions are not called first, the GUI will display outdated or incomplete geometry.


## 🚀 4. Launching the GUI

```julia
    using micromobilitykinematics
    using GLMakie

    # Example configuration
    steering    = Steering(56.0, 165.0, 185.0, 210.0)
    suspension  = Suspension((30.0, 30.0))
    chassis     = Chassis()

    angleConfig = (0.0, 1.0, 0.0)

    suspensionkinematics!(suspension)
    update!(angleConfig, steering, suspension)

    θ_max = (15.0, 5.0, 35.0)

    lounch_gui(θ_max, chassis, steering, suspension; path=@__DIR__)
```

After starting, a new window opens containing interactive 3D visualisation, plots, and parameter summaries.

