############ Steering Component Positions #############################
#
#                                           
#                                                                   ^
#                                                                   |
#                                                                   |  z- Achse
#                                                                   |
#         |                                             y- Achse    |
#         |                                        <-------------(rot_comp_UCS) 
#         |                                             ^           | |         x- Achse
#         |                                             |           |      |
#         |                                             |           |           |->
#         |                                     x_rotational_raidus |   
#         |                                             |           |
#         |                                             |           |
#         |                                             |           |
#         |                                       ---------------  (x)
# (lever_mount_UCS)                                                           |
#         |       |     track_lever                                         |        z_rotational_radius
#         |               |                                                      |
#         |                    (JC_l) --------------------------------    (JS_l)-----------(z)-----------(JS_r)
#         |                     |                                           |                             |    
#         |                     |<-------------- tie_rod --------------->    |<--- distance_between_ ----->|
#         |                                                                       joint_pivot_points
#         |                                       
#         |                                       
#         |                                       
#         |                                       
#         |                                       
########################################################################

mutable struct RotationalComponent
    x_rotational_radius::Union{Float64, Nothing}                    # rotation around the x-axis
    z_rotational_radius::Union{Float64, Nothing}                    # rotation around the z-axis
    to_joint_pivot_points::Union{Float64, Nothing}                  # distance from the component center (z) to joint pivot
    distance_between_joint_pivot_points::Union{Float64, Nothing}    # distance between both pivot poits (JR_l and (JR_r)



    function RotationalComponent(xSteeringRadius::Float64,zSteeringRadius::Float64)
        inst = new()

        inst.xSteeringRadius = xSteeringRadius
        inst.zSteeringRadius = zSteeringRadius

        inst.to_joint_pivot_point = 23.5
        inst.distance_between_joint_pivot_points = inst.to_joint_pivot_points * 2
   
        return inst
    end

end

mutable struct TieRod
    length::Union{Float64, Nothing}                             # lenght of the component

    function TieRod(length)
        inst = new()
        inst.length = length
        return inst
    end
end

mutable struct TrackLever
    length::Union{Float64, Nothing}                            # lenght of the component

    function TrackLever(length)
        inst = new()
        inst.length = length
        return inst
    end
end


mutable struct Steering
    ##### Components
    rotational_component::Union{RotationalComponent,Nothing}
    track_lever::Union{TrackLever,Nothing}
    tie_rod::Union{TieRod,Nothing}

    ######## depends on the kinematics

    sphere_joint::Union{Vector{Float64},Nothing}          #JR_l
    circle_joint::Union{Vector{Float64},Nothing}          #JS_l

    ######## depends on the Suspension kinematics



    ######## Positions of the Userdefined Coordinate System (UCS)

    rotational_component_ucs::Union{Vector{Float64},Nothing}    
    lever_mounting_point_ucs::Union{Vector{Float64},Nothing} 


    kinematics!::Funtion

    function Steering(xSteeringRadius::Float64, zSteeringRadius::Float64, track_lever_length::Float64, tie_rod_length::Float64)
        inst = new()
        inst.rotational_component = RotationalComponent(xSteeringRadius, zSteeringRadius)
        inst.track_lever = TrackLever(track_lever_length)
        inst.tie_rod = TieRod(tie_rod_length)

        inst.sphere_joint = nothing
        inst.circle_joint = nothing

        inst.rotational_component_ucs = [36.0, -140.0, 139.0]

        inst.kinematics! = steeringkinematics!

        return inst
    end

end


############ Suspension Component Positions #############################



#########################################################################



mutable struct Damper

    nominal_length::Union{Float64, Nothing}                                 # Damper nominal length (not compressed) [mm]
    travel::Union{Float64, Nothing}                                         # Damper travel [mm]
    compression::Union{Float64, Nothing}                                    # Damper Compression [%]
    length::Union{Float64, Nothing}                                         # Damper Length with compression as set in LEFTcompression

    upper_fixture::Union{Vector{Float64}, Nothing}                          # Damper upper fixtur
    lower_fixture::Union{Vector{Float64}, Nothing}                          # Damper lower fixture point 

    function Damper()
        inst = new()

        inst.nominal_length = 210.0
        inst.travel = 55.0
        inst.compression = 30.0
        inst.length = inst.nominal_length - (inst.compression / 100) * inst.travel
        inst.upper_fixture = [37.0;30.0; 160.0]
        inst.lower_fixture = nothing
        return inst
    end
end


mutable struct LowerWishbone

    ######## 
    damper::Union{Damper, Nothing}   



    ########    
    bearing_rear::Union{Vector{Float64}, Nothing}                               # Rear Bearing of the Wishbone -> Center CoordinateSystem 
    bearing_distance_x::Union{Float64, Nothing}                                 # Distance in X-Axis Lower Wishbone Bearings
    bearing_front::Union{Vector{Float64}, Nothing}                              # Front Bearing of the Wishbone
    rotation_axis::Union{Vector{Float64}, Nothing}                              # Rotation Axis Lower Wishbone UNIT VECTOR
    distance_to_joint_y::Union{Float64, Nothing}                                # Distance LEFTrotation_axis to LEFTsphere_joint [mm] in Wishbone CoordinateSystem
    distance_rotation_axis_to_lower_damper_fixure::Union{Float64, Nothing}      # Distance LEFTrotation_axis to LowerDamperFixPoint [mm]
    distance_to_joint_x::Union{Float64, Nothing}                                # Distance on x-Axis bearing_rear to Joint [mm] 
    
    
    
    ######## depends on the kinematics
    sphere_joint::Union{Vector{Float64}, Nothing}                               # Sphere Joint at the end of the lower Wishbone (connection to wheel mount)
    lower_fixture::Union{Vector{Float64}, Nothing}                              # Damper lower fixture point 
    rotation_axis_TO_sphere_joint::Union{Vector{Float64}, Nothing}              # directuion vector to  Spherejoint from the LEFTLowerWishboneBearingRear x-Achse

    function LowerWishbone()
        inst = new()

        inst.bearing_rear = [0.0;0.0;0.0]
        inst.bearing_distance_x = 74.00
        inst.bearing_front = [inst.bearing_distance_x;0.0;0.0]
        inst.rotation_axis = (inst.bearing_front - inst.bearing_rear) / norm(inst.bearing_front - inst.bearing_rear)
        inst.distance_to_joint_y = 140.0  
        inst.distance_rotation_axis_to_lower_damper_fixure = 85.0   
        inst.distance_to_joint_x =  37.00 

        inst.sphere_joint = nothing
        inst.lower_fixture = nothing

        return inst
    end

end

mutable struct UpperWishbone
    bearing_rear::Union{Vector{Float64}, Nothing}                               # Rear Bearing of the Wishbone -> Center CoordinateSystem 
    bearing_distance_x::Union{Float64, Nothing}                                 # Distance in X-Axis Lower Wishbone Bearings
    bearing_front::Union{Vector{Float64}, Nothing}                              # Front Bearing of the Wishbone
    rotation_axis::Union{Vector{Float64}, Nothing}                              # Rotation Axis Lower Wishbone UNIT VECTOR 
    distance_to_joint_y::Union{Float64, Nothing}                                # Distance LEFTrotation_axis to LEFTsphere_joint [mm] in Wishbone CoordinateSystem
    distance_to_joint_x::Union{Float64, Nothing}                                # Distance on x-Axis bearing_rear to Joint [mm] 
    

    tiltx::Union{Float64, Nothing}                                              # angle LEFTUpperWishboneRotationAxis to xz-plane of LEFTLowerWishboneBearingRear 
    tilty::Union{Float64, Nothing}                                              # angle LEFTUpperWishboneRotationAxis to xy-plane of LEFTLowerWishboneBearingRear 
    tiltZ::Union{Float64, Nothing}                                              # angle LEFTUpperWishboneRotationAxis to yz-plane of LEFTLowerWishboneBearingRear set to zero (here) has to be calculated after calculating bearing positions

    ######## depends on the kinematics
    sphere_joint::Union{Vector{Float64}, Nothing}                               # Sphere Joint at the end of the lower Wishbone (connection to wheel mount)


    function UpperWishbone()
        inst = new()


        inst.bearing_rear = [0.0; 0.0; 139.00] 
        inst.bearing_distance_x = 74.00
        inst.bearing_front = [inst.bearing_distance_x;0.0;139.00]
        inst.rotation_axis = (inst.bearing_front - inst.bearing_rear) / norm(inst.bearing_front - inst.bearing_rear)
        inst.distance_to_joint_y = 140.0  
        inst.distance_to_joint_x =  37.00 
        
        tiltx = 0.0
        tilty = 90-acosd(abs(dot([0;1;0],inst.rotation_axis)))
        tiltZ = -90+acosd(abs(dot([0;0;1],inst.rotation_axis)))

        inst.sphere_joint = nothing

        return inst
    end
end

mutable struct Chassi
    ##### Dimensions
    width::Union{Float64, Nothing}
    length::Union{Float64, Nothing}

    ##### Components


    function Chassi()
        inst = new()
        inst.width = 280.0
        inst.length = 1300.0

        return inst
    end
end

mutable struct WheelMount
    length::Union{Float64, Nothing}
    camper_angle::Union{Float64, Nothing}
    offset_x::Union{Float64, Nothing}
    offset_y::Union{Float64, Nothing}
    offset_z::Union{Float64, Nothing}
    to_angle::Union{Float64, Nothing}

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

mutable struct Suspension
    ##### Components 
    lowerwishbone::Union{LowerWishbone,Nothing}
    upperwishbone::Union{UpperWishbone,Nothing}
    damper::Union{Damper,Nothing}
    
    kinematics!::Function

    function Suspension()
        inst = new()

        inst.lowerwishbone = LowerWishbone()
        inst.upperwishbone = UpperWishbone()
        inst.damper = Damper()
        inst.kinematics! = suspensionkinematics!

        return inst
    end

end


############ Steering #############################



#########################################################################

