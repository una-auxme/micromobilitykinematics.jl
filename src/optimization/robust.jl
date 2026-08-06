using Random

"""
    OptimizationDomain(varphi_max; ...)

Defines the steering-input and suspension-travel envelope used by robust steering
optimization. At steering inputs above `high_steering_threshold_deg`, the narrower
high-steering compression range is used.
"""
struct OptimizationDomain
    varphi_x_range_deg::Tuple{Float64,Float64}
    varphi_y_deg::Float64
    varphi_z_range_deg::Tuple{Float64,Float64}
    compression_range_percent::Tuple{Float64,Float64}
    high_steering_compression_range_percent::Tuple{Float64,Float64}
    high_steering_threshold_deg::Float64
    reference_compression_percent::Float64
    geometry_clearance_mm::Float64
    angle_order_margin_deg::Float64
    enforce_inner_outer_order::Bool
    min_steering_gain_deg_per_deg::Float64
    objective_compressions_percent::Vector{Float64}
    certify_negative_varphi_z::Bool
end

function OptimizationDomain(varphi_max::Tuple;
                            varphi_x_range_deg = (0.0, float(varphi_max[1])),
                            varphi_y_deg = float(varphi_max[2]),
                            varphi_z_range_deg = (0.0, float(varphi_max[3])),
                            compression_range_percent = (10.0, 90.0),
                            high_steering_compression_range_percent = (20.0, 70.0),
                            high_steering_threshold_deg = 15.0,
                            reference_compression_percent = 30.0,
                            geometry_clearance_mm = 0.25,
                            angle_order_margin_deg = 0.0,
                            enforce_inner_outer_order = false,
                            min_steering_gain_deg_per_deg = 1e-3,
                            objective_compressions_percent = [10.0, 30.0, 90.0],
                            certify_negative_varphi_z = true)
    x_range = Float64.(varphi_x_range_deg)
    z_range = Float64.(varphi_z_range_deg)
    compression_range = Float64.(compression_range_percent)
    high_compression_range = Float64.(high_steering_compression_range_percent)

    x_range[1] <= x_range[2] || error("varphi_x_range_deg must be ordered")
    z_range[1] <= z_range[2] || error("varphi_z_range_deg must be ordered")
    compression_range[1] <= compression_range[2] || error("compression_range_percent must be ordered")
    high_compression_range[1] <= high_compression_range[2] || error("high_steering_compression_range_percent must be ordered")
    geometry_clearance_mm >= 0 || error("geometry_clearance_mm must be non-negative")
    min_steering_gain_deg_per_deg >= 0 || error("min_steering_gain_deg_per_deg must be non-negative")

    return OptimizationDomain(
        Tuple(x_range),
        float(varphi_y_deg),
        Tuple(z_range),
        Tuple(compression_range),
        Tuple(high_compression_range),
        float(high_steering_threshold_deg),
        float(reference_compression_percent),
        float(geometry_clearance_mm),
        float(angle_order_margin_deg),
        enforce_inner_outer_order,
        float(min_steering_gain_deg_per_deg),
        Float64.(objective_compressions_percent),
        certify_negative_varphi_z,
    )
end

struct OperatingPoint
    varphi_x_deg::Float64
    varphi_y_deg::Float64
    varphi_z_deg::Float64
    left_compression_percent::Float64
    right_compression_percent::Float64
end

struct FeasibilityViolation
    point::OperatingPoint
    condition::Symbol
    margin::Float64
end

struct FeasibilityReport
    valid::Bool
    min_margin::Float64
    violations::Vector{FeasibilityViolation}
    evaluated_points::Int
    failed_points::Int
end

mutable struct CandidateEvaluation
    objective::Float64
    feasibility::FeasibilityReport
end

mutable struct RobustEvaluationCache
    parameters::Vector{Float64}
    evaluation::Union{CandidateEvaluation,Nothing}
end

RobustEvaluationCache() = RobustEvaluationCache(Float64[], nothing)

function robust_range(first_value, last_value, step)
    step > 0 || error("step must be greater than zero")
    values = collect(float(first_value):float(step):float(last_value))
    isempty(values) && return [float(first_value), float(last_value)]
    isapprox(last(values), float(last_value); atol = 1e-10) || push!(values, float(last_value))
    return unique(values)
end

function compression_range_for_varphi_z(domain::OptimizationDomain, varphi_z)
    if abs(varphi_z) > domain.high_steering_threshold_deg
        return domain.high_steering_compression_range_percent
    end
    return domain.compression_range_percent
end

function compression_allowed(domain::OptimizationDomain, varphi_z, compression)
    limits = compression_range_for_varphi_z(domain, varphi_z)
    return limits[1] - 1e-9 <= compression <= limits[2] + 1e-9
end

function representative_values(limits, reference)
    low, high = limits
    middle = (low + high) / 2
    return sort(unique([low, clamp(reference, low, high), middle, high]))
end

function signed_varphi_z_values(domain::OptimizationDomain, positive_values)
    values = Float64.(positive_values)
    if domain.certify_negative_varphi_z
        append!(values, -value for value in positive_values if value > 0)
    end
    return sort(unique(values))
end

"""
    operating_points(domain; angle_step_deg=1, compression_step_percent=1)

Builds the certification grid for a robust optimization domain. Compression is
symmetric by default; negative `varphi_z` states can be included for certification.
"""
function operating_points(domain::OptimizationDomain;
                          angle_step_deg = 1.0,
                          compression_step_percent = 1.0,
                          include_negative_varphi_z = domain.certify_negative_varphi_z)
    varphi_x_values = robust_range(domain.varphi_x_range_deg..., angle_step_deg)
    positive_z_limits = (max(0.0, domain.varphi_z_range_deg[1]), domain.varphi_z_range_deg[2])
    positive_z_values = robust_range(positive_z_limits..., angle_step_deg)
    varphi_z_values = include_negative_varphi_z ?
                      signed_varphi_z_values(domain, positive_z_values) : positive_z_values
    points = OperatingPoint[]

    for varphi_x in varphi_x_values, varphi_z in varphi_z_values
        compression_limits = compression_range_for_varphi_z(domain, varphi_z)
        compression_values = robust_range(compression_limits..., compression_step_percent)
        for compression in compression_values
            push!(points, OperatingPoint(
                varphi_x,
                domain.varphi_y_deg,
                varphi_z,
                compression,
                compression,
            ))
        end
    end
    return points
end

function initial_operating_points(domain::OptimizationDomain)
    x_values = representative_values(domain.varphi_x_range_deg, first(domain.varphi_x_range_deg))
    z_max = domain.varphi_z_range_deg[2]
    positive_z_values = sort(unique([
        max(0.0, domain.varphi_z_range_deg[1]),
        min(domain.high_steering_threshold_deg, z_max),
        z_max,
    ]))
    z_values = signed_varphi_z_values(domain, positive_z_values)
    points = OperatingPoint[]

    for varphi_x in x_values, varphi_z in z_values
        compression_limits = compression_range_for_varphi_z(domain, varphi_z)
        compression_values = representative_values(compression_limits, domain.reference_compression_percent)
        for compression in compression_values
            push!(points, OperatingPoint(
                varphi_x,
                domain.varphi_y_deg,
                varphi_z,
                compression,
                compression,
            ))
        end
    end
    return unique(points)
end

function objective_operating_points(domain::OptimizationDomain; angle_step_deg = 5.0)
    x_values = robust_range(domain.varphi_x_range_deg..., angle_step_deg)
    z_values = robust_range(max(0.0, domain.varphi_z_range_deg[1]), domain.varphi_z_range_deg[2], angle_step_deg)
    points = OperatingPoint[]

    for varphi_x in x_values, varphi_z in z_values
        for compression in domain.objective_compressions_percent
            compression_allowed(domain, varphi_z, compression) || continue
            push!(points, OperatingPoint(
                varphi_x,
                domain.varphi_y_deg,
                varphi_z,
                compression,
                compression,
            ))
        end
    end
    return unique(points)
end

function prepared_suspension(base_suspension::Suspension, point::OperatingPoint)
    suspension = deepcopy(base_suspension)
    suspension.damper[1].compression = point.left_compression_percent
    suspension.damper[2].compression = point.right_compression_percent
    suspensionkinematics!(suspension)
    return suspension
end

function suspension_cache(base_suspension::Suspension, points)
    cache = Dict{Tuple{Float64,Float64},Suspension}()
    for point in points
        key = (point.left_compression_percent, point.right_compression_percent)
        haskey(cache, key) || (cache[key] = prepared_suspension(base_suspension, point))
    end
    return cache
end

function circle_sphere_margins(steering::Steering, side_index, sphere_center)
    circle_center = steering.track_lever_mounting_points_ucs[side_index]
    circle_radius = abs(float(steering.track_lever.length))
    sphere_radius = abs(float(steering.tie_rod.length))
    normal = steering.base_vec_wheel_ucs[side_index][:, 3]
    normal = normal / norm(normal)
    plane_distance = dot(normal, sphere_center - circle_center)
    plane_margin = sphere_radius - abs(plane_distance)

    if plane_margin < 0
        return (plane = plane_margin, external = plane_margin, internal = plane_margin)
    end

    projected_radius = sqrt(max(0.0, sphere_radius^2 - plane_distance^2))
    projected_center = sphere_center - plane_distance * normal
    center_distance = norm(circle_center - projected_center)
    external_margin = circle_radius + projected_radius - center_distance
    internal_margin = center_distance - abs(circle_radius - projected_radius)

    return (plane = plane_margin, external = external_margin, internal = internal_margin)
end

function steering_geometry_margin(steering::Steering)
    minimum_margin = Inf
    minimum_condition = :none

    for side_index in 1:2
        margins = circle_sphere_margins(steering, side_index, steering.sphere_joints[side_index])
        for margin_name in (:plane, :external, :internal)
            margin = getproperty(margins, margin_name)
            if margin < minimum_margin
                minimum_margin = margin
                minimum_condition = Symbol(:moved_, side_index == 1 ? :left : :right, :_, margin_name)
            end
        end
    end
    return minimum_margin, minimum_condition
end

function ackermann_deviation_from_angles(chassis::Chassis, steering::Steering, inner_angle, outer_angle)
    if abs(inner_angle) <= 1e-8 || abs(outer_angle) <= 1e-8 || isapprox(inner_angle, outer_angle; atol = 1e-10)
        return NaN
    end

    measurements = Measurements(chassis, steering)
    wheel_base = measurements.wheel_base
    track_width = measurements.track_width
    delta_x_inner = wheel_base / tand(inner_angle)
    delta_x_outer = wheel_base / tand(outer_angle)
    inner_slope = -wheel_base / delta_x_inner
    outer_slope = -wheel_base / delta_x_outer
    shift_x = delta_x_outer - (delta_x_inner + track_width)
    intercept = -outer_slope * shift_x
    intersection_x = intercept / (inner_slope - outer_slope)
    return outer_slope * intersection_x + intercept
end

function wheel_angle_order(point::OperatingPoint, steering::Steering)
    if point.varphi_z_deg < 0
        return steering.δo, steering.δi
    end
    return steering.δi, steering.δo
end

function is_turning_radius_point(domain::OptimizationDomain, point::OperatingPoint)
    return isapprox(point.varphi_x_deg, domain.varphi_x_range_deg[1]; atol = 1e-9) &&
           isapprox(abs(point.varphi_z_deg), domain.varphi_z_range_deg[2]; atol = 1e-9)
end

function evaluate_operating_point(parameters,
                                  point::OperatingPoint,
                                  suspension::Suspension,
                                  chassis::Chassis,
                                  domain::OptimizationDomain)
    preliminary_steering = Steering(parameters...)

    try
        preliminary_steering = kinematicsUNTILmount°(
            (point.varphi_x_deg, point.varphi_y_deg, point.varphi_z_deg),
            preliminary_steering,
            suspension,
        )
    catch
        return (valid = false, margin = -100.0, condition = :mount_kinematics,
                delta_inner = NaN, delta_outer = NaN, ackermann = NaN)
    end

    geometry_margin, geometry_condition = steering_geometry_margin(preliminary_steering)
    normalized_geometry_margin = (geometry_margin - domain.geometry_clearance_mm) / max(1.0, domain.geometry_clearance_mm)
    if normalized_geometry_margin < 0
        return (valid = false, margin = normalized_geometry_margin, condition = geometry_condition,
                delta_inner = NaN, delta_outer = NaN, ackermann = NaN)
    end

    steering = Steering(parameters...)
    try
        update!(
            (point.varphi_x_deg, point.varphi_y_deg, point.varphi_z_deg),
            steering,
            suspension,
        )
    catch
        return (valid = false, margin = -100.0, condition = :full_kinematics,
                delta_inner = NaN, delta_outer = NaN, ackermann = NaN)
    end

    delta_inner, delta_outer = wheel_angle_order(point, steering)
    minimum_margin = normalized_geometry_margin
    minimum_condition = geometry_condition

    if domain.enforce_inner_outer_order &&
       point.varphi_z_deg >= 0 &&
       max(abs(delta_inner), abs(delta_outer)) > 0.1
        angle_margin = delta_inner - delta_outer - domain.angle_order_margin_deg
        normalized_angle_margin = angle_margin / max(0.1, domain.angle_order_margin_deg)
        if normalized_angle_margin < minimum_margin
            minimum_margin = normalized_angle_margin
            minimum_condition = :wheel_angle_order
        end
    end

    if is_turning_radius_point(domain, point) && abs(delta_outer) > 1e-6
        measurements = Measurements(chassis, steering)
        actual_radius = measurements.wheel_base / sind(abs(delta_outer))
        radius_margin = (measurements.turning_radius - actual_radius) / 100.0
        if radius_margin < minimum_margin
            minimum_margin = radius_margin
            minimum_condition = :turning_radius
        end
    end

    ackermann = ackermann_deviation_from_angles(chassis, steering, abs(delta_inner), abs(delta_outer))
    return (
        valid = minimum_margin >= 0,
        margin = minimum_margin,
        condition = minimum_condition,
        delta_inner = float(delta_inner),
        delta_outer = float(delta_outer),
        ackermann = float(ackermann),
    )
end

function add_gain_violations!(violations,
                              evaluations,
                              domain::OptimizationDomain)
    groups = Dict{Tuple{Float64,Float64,Float64},Vector{OperatingPoint}}()
    for point in keys(evaluations)
        point.varphi_z_deg >= 0 || continue
        key = (point.varphi_x_deg, point.left_compression_percent, point.right_compression_percent)
        push!(get!(groups, key, OperatingPoint[]), point)
    end

    minimum_margin = Inf
    for points in values(groups)
        sort!(points; by = point -> point.varphi_z_deg)
        for index in 1:(length(points) - 1)
            current_point = points[index]
            next_point = points[index + 1]
            current = evaluations[current_point]
            next = evaluations[next_point]
            current.valid && next.valid || continue
            delta_varphi = next_point.varphi_z_deg - current_point.varphi_z_deg
            delta_varphi > 1e-9 || continue

            inner_gain = (next.delta_inner - current.delta_inner) / delta_varphi
            outer_gain = (next.delta_outer - current.delta_outer) / delta_varphi
            gain_margin = min(inner_gain, outer_gain) - domain.min_steering_gain_deg_per_deg
            normalized_margin = gain_margin / max(0.01, domain.min_steering_gain_deg_per_deg)
            minimum_margin = min(minimum_margin, normalized_margin)

            if normalized_margin < 0
                push!(violations, FeasibilityViolation(current_point, :steering_gain_start, normalized_margin))
                push!(violations, FeasibilityViolation(next_point, :steering_gain, normalized_margin))
            end
        end
    end
    return minimum_margin
end

"""
    evaluate_feasibility(parameters, points, suspension, chassis, domain)

Evaluates steering-connection clearance, turning-radius and steering-gain
constraints at the supplied operating points. Margins are normalized; a margin
greater than or equal to zero is feasible.
"""
function evaluate_feasibility(parameters,
                              points::Vector{OperatingPoint},
                              suspension::Suspension,
                              chassis::Chassis,
                              domain::OptimizationDomain;
                              prepared_suspensions = nothing)
    suspensions = prepared_suspensions === nothing ?
                  suspension_cache(suspension, points) : prepared_suspensions
    evaluations = Dict{OperatingPoint,Any}()
    violations = FeasibilityViolation[]
    minimum_margin = Inf
    failed_points = 0

    for point in points
        key = (point.left_compression_percent, point.right_compression_percent)
        evaluation = evaluate_operating_point(parameters, point, suspensions[key], chassis, domain)
        evaluations[point] = evaluation
        minimum_margin = min(minimum_margin, evaluation.margin)
        if !evaluation.valid
            failed_points += 1
            push!(violations, FeasibilityViolation(point, evaluation.condition, evaluation.margin))
        end
    end

    gain_margin = add_gain_violations!(violations, evaluations, domain)
    isfinite(gain_margin) && (minimum_margin = min(minimum_margin, gain_margin))
    sort!(violations; by = violation -> violation.margin)
    return FeasibilityReport(
        isempty(violations) && minimum_margin >= 0,
        minimum_margin,
        violations,
        length(points),
        failed_points,
    ), evaluations
end

function robust_ackermann_objective(evaluations, points, domain::OptimizationDomain)
    weighted_error = 0.0
    total_weight = 0.0
    varphi_x_scale = max(abs(domain.varphi_x_range_deg[2]), 1.0)
    varphi_z_scale = max(abs(domain.varphi_z_range_deg[2]), 1.0)

    for point in points
        evaluation = evaluations[point]
        isfinite(evaluation.ackermann) || continue
        weight = exp(-1.5 * ((point.varphi_x_deg / varphi_x_scale)^2 +
                             (point.varphi_z_deg / varphi_z_scale)^2))
        weighted_error += weight * evaluation.ackermann^2
        total_weight += weight
    end

    total_weight > 0 || return 1e8
    return sqrt(weighted_error / total_weight)
end

function evaluate_candidate(parameters,
                            active_points,
                            objective_points,
                            suspension,
                            chassis,
                            domain;
                            prepared_suspensions = nothing)
    all_points = unique(vcat(active_points, objective_points))
    report, evaluations = evaluate_feasibility(
        parameters,
        all_points,
        suspension,
        chassis,
        domain;
        prepared_suspensions = prepared_suspensions,
    )
    objective = robust_ackermann_objective(evaluations, objective_points, domain)
    if !isfinite(objective)
        objective = 1e8
    end
    return CandidateEvaluation(objective, report)
end

function cached_candidate_evaluation!(cache::RobustEvaluationCache,
                                      parameters,
                                      active_points,
                                      objective_points,
                                      suspension,
                                      chassis,
                                      domain,
                                      prepared_suspensions)
    values = collect(Float64.(parameters))
    if cache.evaluation !== nothing && values == cache.parameters
        return cache.evaluation
    end
    cache.parameters = values
    cache.evaluation = evaluate_candidate(
        values,
        active_points,
        objective_points,
        suspension,
        chassis,
        domain;
        prepared_suspensions = prepared_suspensions,
    )
    return cache.evaluation
end

function find_robust_start(start_parameters,
                           lower_border,
                           upper_border,
                           active_points,
                           suspension,
                           chassis,
                           domain;
                           max_attempts = 200,
                           rng = Random.default_rng())
    prepared_suspensions = suspension_cache(suspension, active_points)
    best_parameters = Float64.(start_parameters)
    best_report, _ = evaluate_feasibility(
        best_parameters,
        active_points,
        suspension,
        chassis,
        domain;
        prepared_suspensions = prepared_suspensions,
    )
    best_report.valid && return Tuple(best_parameters), best_report

    for _ in 1:max_attempts
        candidate = [rand(rng) * (upper - lower) + lower for (lower, upper) in zip(lower_border, upper_border)]
        report, _ = evaluate_feasibility(
            candidate,
            active_points,
            suspension,
            chassis,
            domain;
            prepared_suspensions = prepared_suspensions,
        )
        if report.min_margin > best_report.min_margin
            best_parameters = candidate
            best_report = report
        end
        report.valid && return Tuple(candidate), report
    end
    return Tuple(best_parameters), best_report
end

function optimize_active_set(start_parameters,
                             lower_border,
                             upper_border,
                             active_points,
                             objective_points,
                             suspension,
                             chassis,
                             domain;
                             maxeval = 250,
                             xtol_rel = 1e-4)
    cache = RobustEvaluationCache()
    all_points = unique(vcat(active_points, objective_points))
    prepared_suspensions = suspension_cache(suspension, all_points)
    optimizer = NLopt.Opt(:LN_COBYLA, 4)
    NLopt.lower_bounds!(optimizer, collect(Float64.(lower_border)))
    NLopt.upper_bounds!(optimizer, collect(Float64.(upper_border)))
    NLopt.xtol_rel!(optimizer, xtol_rel)
    NLopt.maxeval!(optimizer, maxeval)

    objective_callback = function (parameters, gradient)
        evaluation = cached_candidate_evaluation!(
            cache, parameters, active_points, objective_points,
            suspension, chassis, domain, prepared_suspensions,
        )
        violation_penalty = max(0.0, -evaluation.feasibility.min_margin)
        return evaluation.objective + 1e4 * violation_penalty^2
    end
    NLopt.min_objective!(optimizer, objective_callback)

    constraint_callback = function (parameters, gradient)
        evaluation = cached_candidate_evaluation!(
            cache, parameters, active_points, objective_points,
            suspension, chassis, domain, prepared_suspensions,
        )
        return -evaluation.feasibility.min_margin
    end
    NLopt.inequality_constraint!(optimizer, constraint_callback, 1e-6)

    minimum, parameters, status = NLopt.optimize(optimizer, collect(Float64.(start_parameters)))
    evaluation = evaluate_candidate(
        parameters, active_points, objective_points,
        suspension, chassis, domain;
        prepared_suspensions = prepared_suspensions,
    )
    return Tuple(parameters), minimum, status, evaluation
end

function add_worst_violations(active_points, report; count = 12)
    isempty(report.violations) && return active_points
    selected = first(report.violations, min(count, length(report.violations)))
    return unique(vcat(active_points, [violation.point for violation in selected]))
end

"""
    robust_random_search(upper_border, lower_border, varphi_max; ...)

Finds a feasible starting point on the coarse operating-point set. The search is
bounded by `max_attempts` and reports failure instead of looping indefinitely.
"""
function robust_random_search(upper_border,
                              lower_border,
                              varphi_max;
                              suspension = Suspension((30.0, 30.0)),
                              chassis = Chassis(),
                              domain = OptimizationDomain(varphi_max),
                              max_attempts = 500,
                              rng = Random.default_rng(),
                              info = false)
    active_points = initial_operating_points(domain)
    midpoint = Tuple((Float64.(lower_border) .+ Float64.(upper_border)) ./ 2)
    parameters, report = find_robust_start(
        midpoint,
        lower_border,
        upper_border,
        active_points,
        suspension,
        chassis,
        domain;
        max_attempts = max_attempts,
        rng = rng,
    )

    info && println("Robust random search: margin=$(report.min_margin), valid=$(report.valid)")
    report.valid || error("No feasible robust start found after $max_attempts attempts; best normalized margin $(report.min_margin)")
    return parameters
end


"""
    robust_optim_over_range(start_parameters, varphi_max; ...)

Optimizes Ackermann deviation with derivative-free constrained optimization.
Violating points found by progressively denser audits are added to the active set;
the returned `OptDa.feasibility` contains the final dense certification report.
"""
function robust_optim_over_range(start_parameters,
                                 varphi_max;
                                 lower_border = (50.0, 50.0, 70.0, 195.0),
                                 upper_border = (100.0, 100.0, 200.0, 260.0),
                                 suspension = Suspension((30.0, 30.0)),
                                 chassis = Chassis(),
                                 domain = OptimizationDomain(varphi_max),
                                 active_set_rounds = 3,
                                 maxeval_per_round = 250,
                                 objective_angle_step_deg = 5.0,
                                 audit_angle_step_deg = 2.0,
                                 audit_compression_step_percent = 5.0,
                                 final_angle_step_deg = 1.0,
                                 final_compression_step_percent = 1.0,
                                 violations_per_round = 12,
                                 random_start_attempts = 200,
                                 rng = Random.default_rng(),
                                 info = true)
    active_points = initial_operating_points(domain)
    objective_points = objective_operating_points(domain; angle_step_deg = objective_angle_step_deg)
    parameters, start_report = find_robust_start(
        start_parameters,
        lower_border,
        upper_border,
        active_points,
        suspension,
        chassis,
        domain;
        max_attempts = random_start_attempts,
        rng = rng,
    )
    info && println("Robust start: valid=$(start_report.valid), margin=$(round(start_report.min_margin, digits=4))")

    last_status = :NOT_RUN
    last_objective = Inf
    completed_rounds = 0

    audit_points = operating_points(
        domain;
        angle_step_deg = audit_angle_step_deg,
        compression_step_percent = audit_compression_step_percent,
        include_negative_varphi_z = domain.certify_negative_varphi_z,
    )

    for round_index in 1:active_set_rounds
        completed_rounds = round_index
        parameters, last_objective, last_status, _ = optimize_active_set(
            parameters,
            lower_border,
            upper_border,
            active_points,
            objective_points,
            suspension,
            chassis,
            domain;
            maxeval = maxeval_per_round,
        )
        audit_report, _ = evaluate_feasibility(parameters, audit_points, suspension, chassis, domain)
        info && println(
            "Active-set round $round_index: objective=$(round(last_objective, digits=3)), " *
            "audit margin=$(round(audit_report.min_margin, digits=4)), violations=$(length(audit_report.violations))",
        )
        audit_report.valid && break
        active_points = add_worst_violations(
            active_points,
            audit_report;
            count = violations_per_round,
        )
    end

    final_points = operating_points(
        domain;
        angle_step_deg = final_angle_step_deg,
        compression_step_percent = final_compression_step_percent,
        include_negative_varphi_z = domain.certify_negative_varphi_z,
    )
    final_report, final_evaluations = evaluate_feasibility(
        parameters,
        final_points,
        suspension,
        chassis,
        domain,
    )
    final_objective_points = objective_operating_points(domain; angle_step_deg = objective_angle_step_deg)
    final_objective = robust_ackermann_objective(
        final_evaluations,
        intersect(final_objective_points, final_points),
        domain,
    )
    isfinite(final_objective) || (final_objective = last_objective)

    result_steering = Steering(parameters...)
    result_suspension = deepcopy(suspension)
    result_suspension.damper[1].compression = domain.reference_compression_percent
    result_suspension.damper[2].compression = domain.reference_compression_percent
    result_pose = (
        domain.varphi_x_range_deg[2],
        domain.varphi_y_deg,
        domain.varphi_z_range_deg[2],
    )
    try
        update!(result_pose, result_steering, result_suspension)
    catch
        result_steering.ϕx = result_pose[1]
        result_steering.ϕy = result_pose[2]
        result_steering.ϕz = result_pose[3]
    end

    result_status = final_report.valid ? last_status : :INFEASIBLE_AFTER_AUDIT
    result = OptDa(
        Tuple(Float64.(start_parameters)),
        result_steering,
        final_objective,
        result_status;
        feasibility = final_report,
        iterations = completed_rounds,
    )
    info && println(
        "Final certification: valid=$(final_report.valid), points=$(final_report.evaluated_points), " *
        "margin=$(round(final_report.min_margin, digits=4)), violations=$(length(final_report.violations))",
    )
    return result
end
