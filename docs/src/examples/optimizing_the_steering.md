# Optimizing the steering components


## Overview: When to use which optimisation method?

The optimization can be performed using  different methods:

| Method                     | Purpose                                                 | Recommended when…                                                 |
| -------------------------- | ------------------------------------------------------- | ----------------------------------------------------------------- |
| **`optim_over_range`**     | Optimises parameters across a full steering angle range | You want globally well-behaved steering for all angles            |
| **`optim_at_pose`**        | Optimises geometry at one fixed pose `(θx, θy, θz)`     | You need perfect behaviour at a critical angle (e.g. full lock)   |
| **`optim_series_at_pose`** | Runs many parallel optimisations at one pose            | The problem is non-convex and you want to explore multiple minima |
| **`grid_optim`**           | Executes optimisation over a full angle grid            | You want a full design landscape or want to generate datasets     |
> **Tip:**  
> Steering optimisation problems are often non-convex.   
> Running multiple starting points (random_search or threaded series runs) increases the chance of finding a good optimum.

---
### 🎯 Optimisation over a steering range — ```optim_over_range```
The optim_over_range method computes a weighted quadratic error over all steering angles (θx, θz), keeping θy constant.
A key aspect of this method is the exponential weighting, which prioritises configurations close to the straight-ahead position.

Because the weighting is based on an exponential decay, the influence of a steering configuration decreases rapidly as the steering angles increase.
This reflects real vehicle behaviour:
small steering angles are far more critical for accuracy, while large angles contribute only marginally to the steering performance.

The weighting table below shows how the importance of each steering pose decreases over the evaluated range:

| θx (deg) | θz (deg) | Weight (%) |
|----------|----------|------------|
| 0        | 0        | 100.00     |
| 0        | 10       | 88.48      |
| 0        | 20       | 61.28      |
| 0        | 35       | 22.31      |
| 5        | 0        | 84.65      |
| 5        | 10       | 74.89      |
| 5        | 20       | 51.87      |
| 5        | 35       | 18.89      |
| 10       | 0        | 51.34      |
| 10       | 10       | 45.42      |
| 10       | 20       | 31.46      |
| 10       | 35       | 11.46      |
| 15       | 0        | 22.31      |
| 15       | 10       | 19.74      |
| 15       | 20       | 13.67      |
| 15       | 35       | 4.98       |

> #### 🔎 **Note:** How is the weight calculated?
> The weighting values are computed using an **exponential decay function**, ensuring that small steering angles receive higher importance:  
> ```math
> w = \exp\!\left(
>   -\alpha\left[
>     \left(\frac{\theta_x}{\theta_{x,\max}}\right)^2 +
>     \left(\frac{\theta_z}{\theta_{z,\max}}\right)^2
>   \right]
> \right)
> ```
>  
> The parameter **α** controls how quickly the weight decreases.
> This formulation ensures that precision near the neutral steering position
> is strongly prioritised, while large-angle configurations contribute only marginally.


### Example: Using randomly generated starting parameters

```julia 
    using micromobilitykinematics

    lower_border = (50.0, 50.0, 70.0, 195.0)
    upper_border = (100.0, 100.0, 200.0, 260.0)
    θ_max        = (15.0, 0.0, 35.0)

    # find feasible parameters
    xR, zR, leverL, rodL = random_search(upper_border, lower_border, θ_max)

    # optimise
    opt = optim_over_range(xR, zR, leverL, rodL, θ_max)

    opt.input
    opt.objective
    opt.status
```

### Example: Full automatic version (no random search needed)

```julia

    opt = optim_over_range(upper_border, lower_border, θ_max)

```

> ### 🔎 Note: What does `OptDa` contain?
> The `OptDa` struct bundles all essential optimisation results in a single container:
>
> - **input** — the starting parameter tuple used for the optimisation  
> - **steering** — the optimised `Steering` instance  
> - **objective** — the final Ackermann error (weighted)  
> - **status** — Ipopt termination status  
> - **θ** — the steering pose or angle range used for evaluation  
>
> **Recommendation:**  
> Use `optim_over_range` for general steering-geometry design tasks.  
> It evaluates the geometry over the *entire steering range*, ensuring
> robust behaviour at all angles rather than only at a single pose.



---

### 🎯 Optimisation at a fixed pose — ```optim_at_pose```

On the other hand, `optim_at_pose` optimizes the steering geometry at a specific angular configuration (θx, θy, θz), as the name suggests. This method can be extended to perform optimization over a angle range using grid_optim.

Useful when:
- optimal geometry is needed at a specific steering lock,
- measurement data needs to be matched,
- or evaluating sensitivity of a single pose.


#### Example:

```julia
    using micromobilitykinematics

    lower_border = (50.0, 50.0, 70.0, 195.0)
    upper_border = (100.0, 100.0, 200.0, 260.0)
    θ_max        = (15.0, 0.0, 35.0)

    target_pose = (1.0, 1.0, 20.0)

    opt = optim_at_pose(target_pose, upper_border, lower_border, θ_max)

    opt.steering
    opt.objective
    opt.status

```

> #### **Note**
> ```optim_at_pose``` internally performs a ```random_search``` before building the nonlinear optimisation model.

### 🧵 Series optimisation — ```optim_series_at_pose```

Because the optimisation landscape is non-linear with multiple minima,
optim_series_at_pose executes many independent optimisations in parallel.

This increases reliability and explores alternative solutions.


#### Example:

```julia
    using micromobilitykinematics

    sol = optim_series_at_pose(4,
                            (5.0, 0.0, 20.0),
                            (100.0,100.0,200.0,260.0),
                            (50.0,50.0,70.0,195.0),
                            (15.0,0.0,35.0))

    for (id, entry) in sol
        @show id, entry.objective, entry.status
    end

```

> #### **Note**
> Each thread tries repeatedly until a valid solver status is returned.
> This makes the system robust against Ipopt instability and non-feasible initial guesses.


### 🌐 Grid-based optimisation — ```grid_optim```

grid_optim performs a 2D sweep over steering angles (θx, θz) and executes a threaded optimisation at each grid point.

This produces large datasets for:

- geometry analysis
- sensitivity maps
- parametric studies
- machine-learning training
- extensive design exploration

#### Example:

```julia

    lower_border = (50.0, 50.0, 70.0, 195.0)
    upper_border = (100.0, 100.0, 200.0, 260.0)
    θ_max        = (15.0, 0.0, 35.0)

    grid_optim(lower_border, upper_border, θ_max)

```