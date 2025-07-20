# Optimizing the steering components

The optimization can be performed using two different methods: `optim_over_range` and `optim_at_pose`.

The `optim_over_range` method computes a weighted quadratic error over all possible steering geometry angles (θx, θz), while keeping θy constant. In this approach, smaller angles are prioritized — that is, smaller angles receive higher weights and are therefore penalized more strongly for deviations. Smaller angle configurations are preferred because they are expected to provide more accurate steering behavior.

On the other hand, `optim_at_pose` optimizes the steering geometry at a specific angular configuration, as the name suggests. This method can be extended to perform optimization over a angle range using grid_optim.

## 1. Example with `optim_over_range`
In the `optim_over_range` function, the `OptDa` structure is used to bundle the outcome of the optimization: after setting up and solving the model,the solution is extracted, the steering geometry is updated, and the results are stored in an OptDa instance which is then returned. The `OptDa`structure is used to store key optimization results, including the computed angles (θx, θz), the input parameters, the resulting steering geometry, the objective value, and the optimization status.

```julia 
 
lower_bourder = (50.0, 50.0, 70.0, 195.0)  
upper_bourder = (100.0, 100.0, 200.0, 260.0)
max_angleConfig = (15.0, 0.0, 35.0)

x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length = random_search(upper_bourder, lower_bourder, max_angleConfig)

opt = optim_over_range(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length, max_angleConfig)

opt.input
opt.steering
opt.θ
opt.objective
opt.status

```
> **Note:** It is not strictly necessary to perform a random_search beforehand. The optimization can also be executed directly in the following form: ```julia optim_over_range(upper_bourder, lower_bourder, max_angleConfig)``` 






## 1. Example with `optim_over_range`
In the `optim_over_range` function, the `OptDa` structure is used to bundle the outcome of the optimization: after setting up and solving the model,the solution is extracted, the steering geometry is updated, and the results are stored in an OptDa instance which is then returned. The `OptDa`structure is used to store key optimization results, including the computed angles (θx, θz), the input parameters, the resulting steering geometry, the objective value, and the optimization status.

```julia 
 
lower_bourder = (50.0, 50.0, 70.0, 195.0)  
upper_bourder = (100.0, 100.0, 200.0, 260.0)
max_angleConfig = (15.0, 0.0, 35.0)

opt = optim_at_pose((1.0,1.0,20.0),upper_bourder, lower_bourder, max_angleConfig)

opt.input
opt.steering
opt.θ
opt.objective
opt.status

```

It is also possible to perform a grid-based optimization, in which `optim_at_pose` is executed at each steering position and the results are saved to a folder.

```julia 
 
lower_bourder = (50.0, 50.0, 70.0, 195.0)  
upper_bourder = (100.0, 100.0, 200.0, 260.0)
max_angleConfig = (15.0, 0.0, 35.0)

opt = grid_optim(upper_bourder, lower_bourder, max_angleConfig)

opt.input
opt.steering
opt.θ
opt.objective
opt.status

```