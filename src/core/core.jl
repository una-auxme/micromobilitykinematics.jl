############ TODO ###################

############ Steering Component Positions #############################
#
#                                           
#                                                                   ^
#                                                                   |
#                                                                   |  z- Achse
#                                                                   |
#         |                                             y- Achse    |
#         |                                        <-------------(steering_ucs) 
#         |                                             ^           | |         x- Achse
#         |                                             |           |      |
#         |                                             |           |           |->
#         |                                     x_rotational_raidus |   
#         |                                             |           |
#         |                                             |           |
#         |                                             |           |
#         |                                       ---------------  (x)                                                                           (lever_mount_UCS)
# (lever_mount_UCS)                                                    |                                                                                  |
#         |       |     track_lever                                         |        z_rotational_radius                                               |    track_lever 
#         |               |                                                      |                                                                  |
#         |                    (CJ_l) --------------------------------    (SJ_l)-----------(z)-----------(SJ_r)   -------------------------------- (CJ_r)
#         |                     |                                           |                             |    
#         |                     |<-------------- tie_rod --------------->    |<--- distance_between_ ----->|
#         |                                                                       joint_pivot_points
#         |                                       
#         |                                       
#         |                                       
#         |                                       
#         |                                       
########################################################################
abstract type AbstractSteering end

abstract type AbstractComponent <: AbstractSteering end

abstract type AbstractRotationalComponent <: AbstractComponent end
abstract type AbstractTieRod <: AbstractComponent end
abstract type AbstractTrackLever <: AbstractComponent end




# Type VariableRef needed for JuMP.jl 
mutable struct RotationalComponent <: AbstractComponent
    x_rotational_radius::Any                                        # rotation around the x-axis  
    z_rotational_radius::Any                                        # rotation around the z-axis
    to_joint_pivot_point::Any                                       # distance from the component center (z) to joint pivot
    distance_between_joint_pivot_points::Any                        # distance between both pivot points (JR_l) and (JR_r)



    function RotationalComponent(x_rotational_radius, z_rotational_radius)
        
        inst = new()

        inst.x_rotational_radius = x_rotational_radius
        inst.z_rotational_radius = z_rotational_radius

        inst.to_joint_pivot_point = 23.5
        inst.distance_between_joint_pivot_points = inst.to_joint_pivot_point * 2
   
        return inst
    end

end

# Type VariableRef needed for JuMP.jl 
for (type, supertype) in zip([:TieRod, :TrackLever],[:AbstractTieRod, :AbstractTrackLever])


    @eval mutable struct $type <: $supertype
        length::Any    
        function $type(length)
            new(length)
        end
    end
end



mutable struct Steering <: AbstractSteering
    ##### Components
    rotational_component::Union{RotationalComponent,Nothing}
    track_lever::Union{TrackLever,Nothing}
    tie_rod::Union{TieRod,Nothing}

    ######## depends on the kinematics
    θx::Any                  # Angle of rotation of the rotation component around the x-axis
    θy::Any                  # Angle of rotation of the rotation component around the x-axis
    θz::Any                  # Angle of rotation of the rotation component around the z-axis

    δi::Any                 # inner steering angle of the wheel
    δo::Any                   # outer steering angle of the wheel


    vec_x_rotational::Union{Vector{Any},Nothing} 
    vec_z_rotational::Union{Vector{Any},Nothing} 

    vec_x_rotational_neutral::Union{Vector{Any},Nothing} 
    vec_z_rotational_neutral::Union{Vector{Any},Nothing} 

    sphere_joints::Union{Tuple{Vector{Any},Vector{Any}},Nothing}          # (SJ_l, SJ_r) = (left, right)
    circle_joints::Union{Tuple{Vector{Any},Vector{Any}},Nothing}          # (SC_l, SC_r) = (left, right)

    sphere_joints_neutral::Union{Tuple{Vector{Any},Vector{Any}}, Nothing}          # (SJ_l, SJ_r) = (left, right) VariableRef
    circle_joints_neutral::Union{Tuple{Vector{Any},Vector{Any}}, Nothing}          # (SC_l, SC_r) = (left, right)

    ######## depends on the Suspension kinematics
    wheel_ucs_position::Union{Tuple{Vector{Any},Vector{Any}}, Nothing}
    base_vec_wheel_ucs::Union{Tuple{Matrix{Any},Matrix{Any}}, Nothing}    # Basis vectors of the wheel coordinate system


    ######## Positions of the Userdefined Coordinate System (UCS)

    wishbone_ucs_position::Union{Tuple{Vector{Any},Vector{Any}}, Nothing}               # (left, right) in steering ucs
    track_lever_mounting_points_ucs::Union{Tuple{Vector{Any},Vector{Any}}, Nothing}     # (left, right) Mounting point of the tracklever in steering ucs
    

    ######## objective 

    objective::Union{Any, Nothing}                              


    ######## function 

    kinematics!::Function

    function Steering(x_rotational_radius, z_rotational_radius, track_lever_length, tie_rod_length) 
        inst = new()
        inst.rotational_component = RotationalComponent(x_rotational_radius, z_rotational_radius)
        inst.track_lever = TrackLever(track_lever_length)
        inst.tie_rod = TieRod(tie_rod_length)


        inst.θx = nothing
        inst.θy = nothing
        inst.θz = nothing

        inst.δi = nothing 
        inst.δo = nothing      
        
        inst.vec_x_rotational = nothing
        inst.vec_z_rotational = nothing

        inst.vec_x_rotational_neutral = nothing
        inst.vec_x_rotational_neutral = nothing

        inst.sphere_joints = nothing
        inst.circle_joints = nothing

        inst.sphere_joints_neutral = nothing
        inst.circle_joints_neutral = nothing

        inst.wheel_ucs_position = nothing
        inst.base_vec_wheel_ucs = nothing

        inst.wishbone_ucs_position = ([-36.0, 140.0, -139.0], [-36.0, -140.0, -139.0])
        inst.track_lever_mounting_points_ucs = nothing

        inst.objective = nothing

        inst.kinematics! = steeringkinematics!

        return inst
    end

end

############ Suspension Component Positions #############################



#########################################################################
abstract type AbstractSuspension end

abstract type AbstractDamper <: AbstractSuspension end
abstract type AbstractWishbone <: AbstractSuspension end
abstract type AbstractLowerWishbone <: AbstractWishbone end
abstract type AbstractUpperWishbone <: AbstractWishbone end
abstract type AbstractWheelMount <: AbstractSuspension end


mutable struct Damper <: AbstractDamper

    id::Any

    nominal_length::Any                                 # Damper nominal length (not compressed) [mm]
    travel::Any                                         # Damper travel [mm]
    compression::Any                                    # Damper Compression [%]
    length::Any  
    length_neutral::Any                                 # Damper Length with compression as set in LEFTcompression

    upper_fixture::Union{Vector{Any}, Nothing}                          # Damper upper fixture
    lower_fixture::Union{Vector{Any}, Nothing}                          # Damper lower fixture point 
    lower_fixture_neutral::Union{Vector{Any}, Nothing} 

    function Damper(compression::T) where {T <: Any }
        inst = new()

        inst.id = nothing 

        inst.nominal_length = 210.0
        inst.travel = 55.0
        inst.compression = compression
        inst.length = inst.nominal_length - (inst.compression / 100) * inst.travel
        inst.length_neutral = inst.nominal_length - (30.0 / 100) * inst.travel
        inst.upper_fixture = [37.0;30.0; 160.0]
        inst.lower_fixture = nothing
        inst.lower_fixture_neutral = nothing
        return inst
    end
end


mutable struct LowerWishbone <: AbstractLowerWishbone

    id::Union{Symbol,Nothing}

    ######## 
    
    ########    
    bearing_rear::Union{Vector{Any}, Nothing}                                 # Rear Bearing of the Wishbone -> Center CoordinateSystem 
    bearing_distance_x::Any                                                   # Distance in X-Axis Lower Wishbone Bearings
    bearing_front::Union{Vector{Any}, Nothing}                                # Front Bearing of the Wishbone
    rotation_axis::Union{Vector{Any}, Nothing}                              # Rotation Axis Lower Wishbone UNIT VECTOR
    distance_to_joint_y::Any                                                # Distance rotation_axis to sphere_joint [mm] in Wishbone CoordinateSystem
    distance_rotation_axis_to_lower_damper_fixture::Any                      # Distance rotation_axis to LowerDamperFixPoint [mm]
    distance_to_joint_x::Any                                                # Distance on x-Axis bearing_rear to Joint [mm] 
    
    sphere_joint_neutral::Union{Vector{Any}, Nothing}
    lower_fixture_neutral::Union{Vector{Any}, Nothing}
    ######## depends on the kinematics
    sphere_joint::Union{Vector{Any}, Nothing}                               # Sphere Joint at the end of the lower Wishbone (connection to wheel mount)
    lower_fixture::Union{Vector{Any}, Nothing}                              # Damper lower fixture point 
    #rotation_axis_TO_sphere_joint::Union{Vector{Float64}, Nothing}         # directuion vector to  Spherejoint from the LowerWishboneBearingRear x-Achse

    function LowerWishbone()
        inst = new()

        inst.id = nothing

        inst.bearing_rear = [0.0;0.0;0.0]
        inst.bearing_distance_x = 74.00
        inst.bearing_front = [inst.bearing_distance_x;0.0;0.0]
        inst.rotation_axis = (inst.bearing_front - inst.bearing_rear) / norm(inst.bearing_front - inst.bearing_rear)
        inst.distance_to_joint_y = 140.0  
        inst.distance_rotation_axis_to_lower_damper_fixture = 85.0   
        inst.distance_to_joint_x =  37.00 

        inst.sphere_joint_neutral = nothing
        inst.lower_fixture_neutral = nothing

        inst.sphere_joint = nothing
        inst.lower_fixture = nothing

        return inst
    end

end

mutable struct UpperWishbone <: AbstractUpperWishbone

    id::Union{Symbol,Nothing}

    bearing_rear::Union{Vector{Any}, Nothing}                               # Rear Bearing of the Wishbone -> Center CoordinateSystem 
    bearing_distance_x::Any                                                 # Distance in X-Axis Lower Wishbone Bearings
    bearing_front::Union{Vector{Any}, Nothing}                              # Front Bearing of the Wishbone
    rotation_axis::Union{Vector{Any}, Nothing}                              # Rotation Axis Lower Wishbone UNIT VECTOR 
    distance_to_joint_y::Any                                                # Distance LEFTrotation_axis to LEFTsphere_joint [mm] in Wishbone CoordinateSystem
    distance_to_joint_x::Any                                                # Distance on x-Axis bearing_rear to Joint [mm] 
    

    tiltx::Any                                              # angle LEFTUpperWishboneRotationAxis to xz-plane of LEFTLowerWishboneBearingRear 
    tilty::Any                                              # angle LEFTUpperWishboneRotationAxis to xy-plane of LEFTLowerWishboneBearingRear 
    tiltZ::Any                                              # angle LEFTUpperWishboneRotationAxis to yz-plane of LEFTLowerWishboneBearingRear set to zero (here) has to be calculated after calculating bearing positions

    ######## depends on the kinematics
    sphere_joint::Union{Vector{Any}, Nothing}                               # Sphere Joint at the end of the lower Wishbone (connection to wheel mount)
    sphere_joint_neutral::Union{Vector{Any}, Nothing}   

    function UpperWishbone()
        inst = new()

        inst.id = nothing 

        inst.bearing_rear = [0.0; 0.0; 139.00] 
        inst.bearing_distance_x = 74.00
        inst.bearing_front = [inst.bearing_distance_x;0.0;139.00]
        inst.rotation_axis = (inst.bearing_front - inst.bearing_rear) / norm(inst.bearing_front - inst.bearing_rear)
        inst.distance_to_joint_y = 140.0  
        inst.distance_to_joint_x =  37.00 
        
        inst.tiltx = 0.0
        inst.tilty = 90-acosd(abs(dot([0;1;0],inst.rotation_axis)))
        inst.tiltZ = -90+acosd(abs(dot([0;0;1],inst.rotation_axis)))

        inst.sphere_joint = nothing
        inst.sphere_joint_neutral = nothing
        return inst
    end
end


mutable struct WheelMount <: AbstractWheelMount
    length::Any
    camper_angle::Any
    offset_x::Any
    offset_y::Any
    offset_z::Any
    to_angle::Any

    function WheelMount()
        inst = new()

        inst.length = 139.00
        inst.camper_angle = 0.0
        inst.offset_x = 0.0
        inst.offset_y = 50.0
        inst.offset_z = inst.length / 2
        inst.to_angle = 0.0

        return inst
    end
end

mutable struct Suspension <: AbstractSuspension
    ##### Components 
    lowerwishbone::Union{Tuple{LowerWishbone,LowerWishbone},Nothing}    # (left, right)
    upperwishbone::Union{Tuple{UpperWishbone,UpperWishbone},Nothing}    # (left, right)
    damper::Union{Tuple{Damper,Damper},Nothing}                         # (left, right)
    wheelmount::Union{WheelMount,Nothing}                        
    
    kinematics!::Function

    function Suspension(compressions::Tuple)
        inst = new()

        inst.lowerwishbone = (LowerWishbone(),LowerWishbone())      # (left, right)

        inst.upperwishbone = (UpperWishbone(), UpperWishbone())     # (left, right)

        inst.damper = (Damper(compressions[1]), Damper(compressions[2]))                          # (left, right)

        inst.wheelmount = WheelMount()

        inst.kinematics! = suspensionkinematics!

        return inst
    end

end


############ micromobility vehicle ######################################



#########################################################################
abstract type AbstractVehicle end

abstract type AbstractChassis <: AbstractVehicle end
abstract type AbstractMeasurements <: AbstractVehicle end







mutable struct Chassis <: AbstractChassis
    ##### Dimensions
    width::Any
    length::Any
    radius::Any

    ##### Components


    function Chassis()
        inst = new()
        inst.width = 280.0              # distance between both LowerWishbone.bearing_rear 
        return inst
    end
end


mutable struct Measurements <: AbstractMeasurements

    track_width::Any
    wheel_base::Any
    turning_radius::Any


    function Measurements(chassi::Chassis, steering::Steering)
        inst = new()
        inst.track_width = chassi.width + 2 * abs(steering.wheel_ucs_position[1][2]) 
        inst.wheel_base = 1000.0            
        inst.turning_radius = 3000.0        # desired track radius

        return inst
    end
    
end


mutable struct Vehicle <: AbstractVehicle

    ##### Measurments
    measurements ::Union{Measurements,Nothing}

    ##### Chassis
    chassi::Union{Chassis, Nothing}
    
    ##### Steering
    steering::Union{Steering,Nothing}

    ##### Suspension
    suspension::Union{Tuple{Suspension,Suspension},Nothing}


    function Vehicle(measurements::Measurements, chassi::Chassis, steering::Steering, suspensions::Tuple{Suspension,Suspension})
        inst = new()

        inst.measurements = measurements

        inst.chassi = chassi

        inst.steering = steering

        inst.suspension = suspensions
        
        return inst
    end
end



############ Optimisation  ######################################



#########################################################################


mutable struct OptDa # optimisation data
    θ::Union{Tuple{Int64,Int64},Nothing}
    input::Union{Tuple{<:Number,<:Number,<:Number,<:Number},Nothing}
    steering::Union{Steering, Nothing}
    objective::Union{<:Number, Nothing}
    status::Union{Any,Nothing}

    function OptDa(input::Tuple{<:Number,<:Number,<:Number,<:Number}, steering::Steering, objective::Any, status::Any)
        inst = new()
        inst.θ = (steering.θx, steering.θz)
        inst.input = input
        inst.steering = steering
        inst.objective = objective 
        inst.status = status
        return inst
    end
end