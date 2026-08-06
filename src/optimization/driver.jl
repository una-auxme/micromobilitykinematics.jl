################################################################################
###                         OPTIMIZATION METHOD 01: AT POSE                  ###
################################################################################

"""
    create_model_for_pose(ϕ::Tuple{T,T,T}, steering::Steering) where {T<:Number}

Constructs a nonlinear optimization model for minimizing the Ackermann steering deviation based on given spherical pose angles.

# Arguments
- `ϕ::Tuple{T,T,T}`: A tuple of target angles `(ϕx, ϕy, ϕz)` representing the desired steering pose. All values must be subtypes of `Number`.
- `steering::Steering`: A `Steering` struct containing initial values for the optimization parameters:
  - `x_rotational_radius`
  - `z_rotational_radius`
  - `track_lever_length`
  - `tie_rod_length`

# Returns
- `model::Model`: A JuMP nonlinear model configured with variables, constraints (if used), objective function, and solver settings (Ipopt).

# Notes
- Uses `Ipopt` as the optimization solver with custom tolerances.
- Registers the function `ackermann_deviation_for_pose` for use in the nonlinear objective.
- Fixed angles (ϕx, ϕy, ϕz) are treated as constants in the optimization.
- Variables are initialized using values extracted from the `Steering` object.

"""
function create_model_for_pose(ϕ::Tuple{T,T,T}, steering::Steering) where {T<:Number}

    # --- extract the maximum steering angles ---
    ϕx_, ϕy_, ϕz_ = ϕ
    start_x_rotational_radius,start_z_rotational_radius,start_track_lever_length,start_tie_rod_length = getValue(steering)

    # --- selection of the used solver ---
    model = Model(optimizer_with_attributes(Ipopt.Optimizer,
                                                             "tol" => 1e-3, #1e-2,           
                                                             "acceptable_tol" => 1e-1, 
                                                             "dual_inf_tol" =>1e-1, 
                                                             "compl_inf_tol"=>  1e-6,
                                                             "constr_viol_tol"=> 1e-6,
                                                             "max_iter" => 1000)
                                                             #"hessian_approximation" => "limited-memory")
                )

    # --- define the parameters to be optimised ---
    @variable(model, 50.0 <= x_rotational_radius <= 100.0)
    @variable(model, 50.0 <= z_rotational_radius <= 100.0)
    @variable(model, 70.0 <= track_lever_length <= 200.0)
    @variable(model, 195.0 <= tie_rod_length <= 260.0)

    # --- fixed variables used in optimisation ---
    @variable(model, ϕx)
    @variable(model, ϕy)
    @variable(model, ϕz)


    # --- used and unknown functions must be registered for optimisation ---
    register(model, :ackermann_deviation_for_pose, 7, ackermann_deviation_for_pose, autodiff=true)
    #register(model, :checkConstraints°, 4, checkConstraints°, autodiff=true)

    # --- define the objectie of the given NL-Problem ---
    @NLobjective(model, Min, ackermann_deviation_for_pose(ϕx, ϕy, ϕz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))
    
    # --- define the constraint of the given NL-Problem ---
    #@NLconstraint(model, C, checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 1 )

    # --- initialisation of the parameters ---
    set_start_value(x_rotational_radius, start_x_rotational_radius)
    set_start_value(z_rotational_radius, start_z_rotational_radius)
    set_start_value(track_lever_length, start_track_lever_length)
    set_start_value(tie_rod_length, start_tie_rod_length)

    fix(ϕx, ϕx_)
    fix(ϕy, ϕy_)
    fix(ϕz, ϕz_)

    return model
end

"""
    get_model_solution(model::Model)

Extracts the optimized parameter values, objective value, and solver status from a JuMP model.

# Arguments
- `model::Model`: A JuMP optimization model that has been solved.

# Returns
- `solution::Tuple{Float64, Float64, Float64, Float64}`: The optimized values of:
  1. `x_rotational_radius`
  2. `z_rotational_radius`
  3. `track_lever_length`
  4. `tie_rod_length`
- `objective::Float64`: The final value of the objective function.
- `status::MOI.TerminationStatusCode`: The solver's termination status, indicating whether the solution is optimal, infeasible, etc.

# Notes
- Assumes the model contains the variables in the expected order.
- The function must be called after the model has been successfully solved.
"""
function get_model_solution(model)
    # --- extract output data ---
    x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length, ~ = JuMP.all_variables(model)
    solution = (JuMP.value(x_rotational_radius),JuMP.value(z_rotational_radius),JuMP.value(track_lever_length),JuMP.value(tie_rod_length))
    objective = objective_value(model)
    status = termination_status(model)

    return solution,objective,status
end 



"""
    optim_at_pose(ϕ::Tuple{T,T,T}, 
                  upper_border::Tuple{<:Number, <:Number, <:Number, <:Number}, 
                  lower_border::Tuple{<:Number, <:Number, <:Number, <:Number}, 
                  ϕ_max::Tuple{<:Number, <:Number, <:Number}) where {T<:Number}

Performs a full optimization workflow for a given steering pose, including random search, model generation, solving, and result packaging.

# Arguments
- `ϕ::Tuple{T,T,T}`: Target steering pose defined by a tuple of angles `(ϕx, ϕy, ϕz)`.
- `upper_border::Tuple{<:Number, <:Number, <:Number, <:Number}`: Upper bounds for the parameters:
    1. `x_rotational_radius`
    2. `z_rotational_radius`
    3. `track_lever_length`
    4. `tie_rod_length`
- `lower_border::Tuple{<:Number, <:Number, <:Number, <:Number}`: Lower bounds for the same parameters (in same order).
- `ϕ_max::Tuple{<:Number, <:Number, <:Number}`: Maximum allowable steering angles in each direction (x, y, z).

# Returns
- `opda::OptDa`: An `OptDa` struct containing:
    - the selected input parameters,
    - the optimized `Steering` configuration,
    - the achieved objective value,
    - and the solver termination status.

# Notes
- Internally performs a random search to generate starting parameters.
- Creates a `Steering` and `Suspension` configuration, and solves a nonlinear optimization problem using `JuMP` and `Ipopt`.
- This function is multi-threading safe and logs the active thread ID.
"""
function optim_at_pose(ϕ::Tuple{T,T,T}, args...) where {T<:Number}

    # --- calaculate necessary input for optimization ---
    param = random_search(args...)
    steering = Steering(param...)
    suspension = Suspension((30,30))
    suspensionkinematics!(suspension)


    # --- generate optimization model ---
    println("Thread $(Threads.threadid()):> optimization begin")
    model = create_model_for_pose(ϕ,steering)

    # --- perform optimization ---
    JuMP.optimize!(model)

    # --- extract important data ---
    sol,objective,status = get_model_solution(model)

    # --- save data ---
    steering = Steering(sol...)
    update!(ϕ, steering, suspension)
    optda = OptDa(param,steering, objective, status)

    return optda

end 



"""
    optim_series_at_pose(num::Int, 
                         ϕ::Tuple{T,T,T}, 
                         upper_border::Tuple{Float64, Float64, Float64, Float64}, 
                         lower_border::Tuple{Float64, Float64, Float64, Float64}, 
                         ϕ_max::Tuple{<:Number, <:Number, <:Number}) where {T<:Number}

Runs the optimization process `num` times in parallel for a given steering pose using multithreading. Each run independently attempts to find an optimal solution based on randomized initial parameters within the provided bounds.

# Arguments
- `num::Int`: Number of optimization runs to perform in parallel.
- `ϕ::Tuple{T,T,T}`: Target steering angles `(ϕx, ϕy, ϕz)` for which optimization should be performed.
- `upper_border::Tuple{Float64, Float64, Float64, Float64}`: Upper bounds for the optimization parameters:
    1. `x_rotational_radius`
    2. `z_rotational_radius`
    3. `track_lever_length`
    4. `tie_rod_length`
- `lower_border::Tuple{Float64, Float64, Float64, Float64}`: Lower bounds for the same parameters (same order).
- `ϕ_max::Tuple{<:Number, <:Number, <:Number}`: Maximum allowed steering angles in each axis (optional; often `(0.0, 35.0, ...)` as a guideline).

# Returns
- `sol_dict::Dict{Int, OptDa}`: A dictionary mapping each thread/task ID to its corresponding `OptDa` optimization result.

# Notes
- Uses `Threads.@threads` for parallel execution.
- Retries each run until a valid result with acceptable solver status is found.
- Stores results in a shared dictionary using thread-safe access.
- **Side effect:** Optionally saves intermediate or final solutions externally (commented out in code).
- Expected solver status: `OPTIMAL`, `LOCALLY_SOLVED`, `NUMERICAL_ERROR`, `ITERATION_LIMIT`, or `ALMOST_LOCALLY_SOLVED`.
"""
function optim_series_at_pose(num::Int64,args...)

    # --- Initialize the dictionary for the respective solutions. ---
    sol_dict = Dict{Int,Any}()
    #println(args...)

    lk = ReentrantLock()
    @threads for id in 1:num
        success = false
        while !success
            try
                # --- finding steering that fulfil all dependencies ---
                optda = optim_at_pose(args...)

                # --- checking status ---
                if (optda.status != MOI.OPTIMAL &&
                     optda.status != MOI.LOCALLY_SOLVED && 
                     optda.status != MOI.NUMERICAL_ERROR && 
                     optda.status != MOI.ITERATION_LIMIT && 
                     optda.status != MOI.ALMOST_LOCALLY_SOLVED )

                     error("Thread $(Threads.threadid()):> $(optda.status))")
                end
                # --- write found steering in Dict ---
                lock(lk) do 
                    sol_dict[id] = optda
                    #save_best_objective()
                end 

                success = true
            catch err  
                @warn "Error in optim_series_at_pose: $err"
            end
        end 
    end 
    return sol_dict
end 



"""
    grid_optim(upper_border::Tuple{T,T,T,T}, 
               lower_border::Tuple{T,T,T,T}, 
               ϕ_max::Tuple{I,I,I}) where {T<:Number, I<:Number}

Performs a grid-based sweep over a range of steering angles and runs parallel optimization at each steering configuration. The results of each run are saved to disk.

# Arguments
- `upper_border::Tuple{T,T,T,T}`: Upper bounds for the optimization parameters:
    1. `x_rotational_radius`
    2. `z_rotational_radius`
    3. `track_lever_length`
    4. `tie_rod_length`
- `lower_border::Tuple{T,T,T,T}`: Lower bounds for the same parameters (same order).
- `ϕ_max::Tuple{I,I,I}`: Maximum values for the steering angles `(ϕx, ϕy, ϕz)` in degrees. The grid will be generated from `0` to `ϕx_max` and `ϕz_max` (in steps of 1.0°), while `ϕy` remains fixed.

# Behavior
- Constructs a 2D grid of `(ϕx, ϕz)` angle pairs with a step size of `1.0°`.
- For each steering angle (excluding `(0, 0)`), creates a directory for that row of the grid and runs `optim_series` twice (`num=2`) for the full `ϕ = (ϕx, ϕy, ϕz)`.
- Saves each result in a `.jld2` file to a structured directory based on the grid position.
- Paths are built relative to the current file location (`@__DIR__`), under a `data/` subfolder.

# Side Effects
- Creates folders and writes `.jld2` result files to disk using `JLD2.@save`.

# File Naming Convention
- Folder: `data/(ϕx,n)` where `ϕx` is the x-angle of the row.
- File: `opt_series(ϕx,ϕy,ϕz).jld2` — the result of each optimization series for a grid point.
"""
function grid_optim(upper_border::Tuple{T, T, T, T},lower_border::Tuple{T, T, T, T}, ϕ_max::Tuple{I,I,I}) where {T<:Number, I<:Number}
    # --- extract the maximum steering angles ---
    ϕx_max , ϕy, ϕz_max = ϕ_max

    # --- Angle preprocessing ---
    ϕx_max = Int(round(ϕx_max))
    ϕz_max = Int(round(ϕz_max))

    step_size = 1.0
    # --- generate steering angles grid ---
    ϕ_tuple = [(i, j) for i in 0.0:step_size:ϕx_max, j in 0.0:step_size:ϕz_max]

    # --- ---
    for i in 1:Int((ϕx_max/step_size)+1)
        data_name = "data\\data($((i-1)*step_size),n)"
        path_data = joinpath(@__DIR__, data_name)
        mkdir(path_data)
        # --- iterate `optim_series_at_pose` over steering angle tuple ---
        for ϕ in ϕ_tuple[i,:] 
            if ϕ != (0,0)
                # --- input preprocessing ---
                ϕx,ϕz = ϕ
                ϕ_ = (ϕx,ϕy,ϕz)

                # --- perform optimization series at given angle position---
                opt_series = optim_series_at_pose(2,ϕ_,upper_border,lower_border,ϕ_max)

                # --- save data ---
                pathTOdata = joinpath(path_data,"opt_series($(ϕx),$(ϕy),$(ϕz)).jld2")
                @save pathTOdata opt_series
            end   
        end
    end 
end

################################################################################
###                     OPTIMIZATION METHOD 02: OVER RANGE                  ###
################################################################################







"""
    create_model_for_range(ϕ::Tuple{T,T,T}, steering::Steering) where {T<:Number}

Constructs a nonlinear optimization model to minimize the Ackermann deviation over a range of steering angles. The model is built using the Ipopt solver and is initialized based on a given `Steering` configuration.

# Arguments
- `ϕ::Tuple{T,T,T}`: A tuple of maximum steering angles `(ϕx_max, ϕy_max, ϕz_max)` to define the evaluation range for the optimization.
- `steering::Steering`: A `Steering` object that provides the starting values for the parameters:
    1. `x_rotational_radius`
    2. `z_rotational_radius`
    3. `track_lever_length`
    4. `tie_rod_length`

# Returns
- `model::Model`: A JuMP model prepared for nonlinear optimization, including:
    - Decision variables with bounds,
    - A nonlinear objective function based on `ackermann_deviation_over_range`,
    - Fixed steering angles derived from the input `ϕ`,
    - Starting values set from the provided `Steering` configuration.

# Notes
- The objective function `ackermann_deviation_over_range` must be defined and differentiable (registered with `autodiff=true`).
- The solver used is `Ipopt` with moderate tolerances and a maximum iteration limit of 70.
- Constraints are currently deactivated but can be re-enabled if needed.
- This function is used when the goal is to evaluate Ackermann deviation over a full steering range, rather than a fixed pose.

"""
function create_model_for_range(ϕ::Tuple{T,T,T}, steering::Steering) where {T<:Number}


    # --- extract the maximum steering angles ---
    ϕx_max, ϕy_max, ϕz_max = ϕ
    start_x_rotational_radius, start_z_rotational_radius, start_track_lever_length, start_tie_rod_length = getValue(steering)

    # --- selection of the used solver ---
    model = Model(optimizer_with_attributes(Ipopt.Optimizer,
                                                             "tol" => 1e-3, #1e-2,           
                                                             "acceptable_tol" => 1e-1, 
                                                             "dual_inf_tol" =>1e-1, 
                                                             "compl_inf_tol"=>  1e-6,
                                                             "constr_viol_tol"=> 1e-6,
                                                             "max_iter" => 70)
                                                             #"hessian_approximation" => "limited-memory")
                )

    # --- define the parameters to be optimised ---
    @variable(model, 50.0 <= x_rotational_radius <= 100.0)
    @variable(model, 50.0 <= z_rotational_radius <= 100.0)
    @variable(model, 70.0 <= track_lever_length <= 200.0)
    @variable(model, 195.0 <= tie_rod_length <= 260.0)

    # --- fixed variables used in optimisation ---
    @variable(model, ϕx)
    @variable(model, ϕy)
    @variable(model, ϕz)


    # --- used and unknown functions must be registered for optimisation ---
    register(model, :ackermann_deviation_over_range, 7, ackermann_deviation_over_range, autodiff=true)
    #register(model, :checkConstraints°, 4, checkConstraints°, autodiff=true)

    # --- define the objectie of the given NL-Problem ---
    @NLobjective(model, Min, ackermann_deviation_over_range(ϕx, ϕy, ϕz, x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length))
    
    # --- define the constraint of the given NL-Problem ---
    #@NLconstraint(model, C, checkConstraints°(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) >= 1 )

    # --- initialisation of the parameters ---
    set_start_value(x_rotational_radius, start_x_rotational_radius)
    set_start_value(z_rotational_radius, start_z_rotational_radius)
    set_start_value(track_lever_length, start_track_lever_length)
    set_start_value(tie_rod_length, start_tie_rod_length)

    fix(ϕx, ϕx_max)
    fix(ϕy, ϕy_max)
    fix(ϕz, ϕz_max)

    return model
end



"""
    optim_over_range(upper_border::Tuple{T,T,T,T}, 
                     lower_border::Tuple{T,T,T,T}, 
                     ϕ_max::Tuple{I,I,I}) where {T<:Number, I<:Number}

Performs an optimization over a defined steering angle range, targeting minimal Ackermann deviation across that range. Random initial parameters are selected within bounds, and the result is returned as an `OptDa` object.

# Arguments
- `upper_border::Tuple{T,T,T,T}`: Upper bounds for the optimization parameters:
    1. `x_rotational_radius`
    2. `z_rotational_radius`
    3. `track_lever_length`
    4. `tie_rod_length`
- `lower_border::Tuple{T,T,T,T}`: Lower bounds for the same parameters.
- `ϕ_max::Tuple{I,I,I}`: Maximum steering angles `(ϕx_max, ϕy_max, ϕz_max)` that define the full evaluation range.

# Returns
- `optda::OptDa`: An `OptDa` struct containing:
    - The randomly selected parameter values,
    - The optimized `Steering` configuration,
    - The final objective value (Ackermann deviation),
    - The solver's termination status.

# Notes
- Uses robust random search followed by derivative-free constrained optimization.
- The actual `suspension` and `chassis` can be supplied as keyword arguments.
- `OptDa.feasibility` contains the final suspension-travel certification report.

"""
function optim_over_range(upper_border::Tuple{T,T,T,T},
                          lower_border::Tuple{T,T,T,T},
                          ϕ_max::Tuple{I,I,I};
                          suspension = Suspension((30.0, 30.0)),
                          chassis = Chassis(),
                          domain = OptimizationDomain(ϕ_max),
                          kwargs...) where {T<:Number, I<:Number}
    parameters = robust_random_search(
        upper_border,
        lower_border,
        ϕ_max;
        suspension = suspension,
        chassis = chassis,
        domain = domain,
    )
    return robust_optim_over_range(
        parameters,
        ϕ_max;
        lower_border = lower_border,
        upper_border = upper_border,
        suspension = suspension,
        chassis = chassis,
        domain = domain,
        kwargs...,
    )
end



"""
    optim_over_range(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length, ϕ_max::Tuple)

Performs an optimization over a defined steering angle range using manually specified suspension parameters. The goal is to minimize the Ackermann deviation across the full range of steering angles.

# Arguments
- `x_rotational_radius::Number`: Initial guess for the x-axis rotational radius.
- `z_rotational_radius::Number`: Initial guess for the z-axis rotational radius.
- `track_lever_length::Number`: Initial length of the track lever.
- `tie_rod_length::Number`: Initial length of the tie rod.
- `ϕ_max::Tuple`: Maximum steering angles `(ϕx_max, ϕy_max, ϕz_max)` to define the evaluation range.

# Returns
- `optda::OptDa`: A struct containing:
    - The initial input parameters as provided,
    - The optimized `Steering` configuration,
    - The final value of the optimization objective (Ackermann deviation),
    - The solver’s termination status.

# Notes
- Uses the specified values as the initial point for robust constrained optimization.
- The actual `suspension`, `chassis`, and `OptimizationDomain` can be supplied.
- The result is certified over the configured steering and suspension-travel range.

"""
function optim_over_range(x_rotational_radius,
                          z_rotational_radius,
                          track_lever_length,
                          tie_rod_length,
                          ϕ_max::Tuple;
                          suspension = Suspension((30.0, 30.0)),
                          chassis = Chassis(),
                          domain = OptimizationDomain(ϕ_max),
                          lower_border = (50.0, 50.0, 70.0, 195.0),
                          upper_border = (100.0, 100.0, 200.0, 260.0),
                          kwargs...)
    parameters = (x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length)
    return robust_optim_over_range(
        parameters,
        ϕ_max;
        lower_border = lower_border,
        upper_border = upper_border,
        suspension = suspension,
        chassis = chassis,
        domain = domain,
        kwargs...,
    )
end
