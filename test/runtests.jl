using Test
using Random
using micromobilitykinematics

const MMK = micromobilitykinematics
const CURRENT_PARAMETERS = (
    62.81680256916951,
    100.00000099935133,
    108.80559236847354,
    227.8382026583041,
)

@testset "Optimization operating domain" begin
    domain = OptimizationDomain(
        (2.0, 1.0, 3.0);
        compression_range_percent = (10.0, 12.0),
        high_steering_compression_range_percent = (11.0, 12.0),
        high_steering_threshold_deg = 2.0,
    )
    points = operating_points(
        domain;
        angle_step_deg = 1.0,
        compression_step_percent = 1.0,
    )

    @test length(points) == 57
    @test any(point -> point.varphi_z_deg < 0, points)
    @test all(point -> abs(point.varphi_z_deg) <= 2 || point.left_compression_percent >= 11, points)
end

@testset "Prepared suspension follows damper compression" begin
    base = Suspension((30.0, 30.0))
    low_point = OperatingPoint(0.0, 1.0, 0.0, 20.0, 20.0)
    high_point = OperatingPoint(0.0, 1.0, 0.0, 70.0, 70.0)
    low = MMK.prepared_suspension(base, low_point)
    high = MMK.prepared_suspension(base, high_point)

    @test low.damper[1].length ≈ low.damper[1].nominal_length - 0.2 * low.damper[1].travel
    @test high.damper[1].length ≈ high.damper[1].nominal_length - 0.7 * high.damper[1].travel
    @test low.lowerwishbone[1].sphere_joint != high.lowerwishbone[1].sphere_joint
end

@testset "Optimization precheck matches moved full geometry" begin
    point = OperatingPoint(10.0, 1.0, 35.0, 20.0, 20.0)
    suspension = MMK.prepared_suspension(Suspension((30.0, 30.0)), point)

    full = Steering(CURRENT_PARAMETERS...)
    update!((point.varphi_x_deg, point.varphi_y_deg, point.varphi_z_deg), full, deepcopy(suspension))

    partial = MMK.kinematicsUNTILmount°(
        (point.varphi_x_deg, point.varphi_y_deg, point.varphi_z_deg),
        Steering(CURRENT_PARAMETERS...),
        suspension,
    )
    @test partial.track_lever_mounting_points_ucs[1] ≈ full.track_lever_mounting_points_ucs[1] atol = 1e-9
    @test partial.track_lever_mounting_points_ucs[2] ≈ full.track_lever_mounting_points_ucs[2] atol = 1e-9
    @test partial.sphere_joints[1] ≈ full.sphere_joints[1] atol = 1e-9
    @test partial.sphere_joints[2] ≈ full.sphere_joints[2] atol = 1e-9
end

@testset "Feasibility changes over suspension travel" begin
    domain = OptimizationDomain((10.0, 1.0, 35.0))
    low = OperatingPoint(10.0, 1.0, 35.0, 20.0, 20.0)
    high = OperatingPoint(10.0, 1.0, 35.0, 70.0, 70.0)
    report, evaluations = evaluate_feasibility(
        CURRENT_PARAMETERS,
        [low, high],
        Suspension((30.0, 30.0)),
        Chassis(),
        domain,
    )

    @test evaluations[low].valid
    @test !evaluations[high].valid
    @test evaluations[high].condition == :moved_right_external
    @test !report.valid
end

@testset "Robust random search returns a feasible coarse start" begin
    domain = OptimizationDomain((10.0, 1.0, 35.0))
    parameters = random_search(
        (100.0, 100.0, 200.0, 260.0),
        (50.0, 50.0, 70.0, 195.0),
        (10.0, 1.0, 35.0);
        domain = domain,
        max_attempts = 300,
        rng = MersenneTwister(42),
    )
    report, _ = evaluate_feasibility(
        parameters,
        initial_operating_points(domain),
        Suspension((30.0, 30.0)),
        Chassis(),
        domain,
    )

    @test report.valid
    @test all(((50.0, 50.0, 70.0, 195.0) .<= parameters) .&
              (parameters .<= (100.0, 100.0, 200.0, 260.0)))
end

@testset "Short robust optimization run" begin
    domain = OptimizationDomain(
        (2.0, 1.0, 5.0);
        compression_range_percent = (25.0, 35.0),
        high_steering_compression_range_percent = (25.0, 35.0),
        high_steering_threshold_deg = 5.0,
        objective_compressions_percent = [25.0, 30.0, 35.0],
        geometry_clearance_mm = 0.0,
        min_steering_gain_deg_per_deg = 0.0,
        certify_negative_varphi_z = false,
    )
    start = (71.47440223906428, 83.64158509179684, 110.56250959183535, 229.33402456269965)
    result = optim_over_range(
        start...,
        (2.0, 1.0, 5.0);
        domain = domain,
        active_set_rounds = 1,
        maxeval_per_round = 12,
        objective_angle_step_deg = 2.0,
        audit_angle_step_deg = 1.0,
        audit_compression_step_percent = 5.0,
        final_angle_step_deg = 1.0,
        final_compression_step_percent = 5.0,
        random_start_attempts = 20,
        rng = MersenneTwister(7),
        info = false,
    )

    @test result.feasibility isa FeasibilityReport
    @test result.feasibility.evaluated_points == 54
    @test result.iterations == 1
    @test isfinite(result.objective)
    parameters = getValue(result.steering)
    @test all(((50.0, 50.0, 70.0, 195.0) .<= parameters) .&
              (parameters .<= (100.0, 100.0, 200.0, 260.0)))
end
