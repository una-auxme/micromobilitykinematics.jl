function steeringkinematics!(θx::Float64, θz::Float64, steering::Steering, suspension::Suspension)

    θx =    deg2rad(θx)
    θz =    deg2rad(θz)

    #############
    rotational_component_ucs = [[1;0;0] [0;1;0] [0;0;1]]

    vec_x_rotational_neutral = [0.0; 0.0; -steering.rotational_component.x_rotational_radius]
    vec_z_rotational_neutral = [-steering.rotational_component.z_rotational_radius; 0.0; 0.0] + vec_x_rotational_neutral 

    left_sphere_joints_neutral = [0.0; steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral
    right_sphere_joints_neutral = [0.0; -steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral

    steering.sphere_joints_neutral = (left_sphere_joints_neutral, right_sphere_joints_neutral)

    ##############
    ~, vec_x_rotational = rotate3(vec_x_rotational_neutral, rotational_component_ucs[1,:], θx)

    ~, vec_z_rotational = rotate3(vec_z_rotational_neutral, rotational_component_ucs[1,:], θx)
    ~, vec_z_rotational = rotate3(vec_z_rotational, vec_x_rotational, θz)
    
    ~, left_sphere_joints = rotate3(steering.sphere_joints_neutral[1], rotational_component_ucs[1,:], θx)
    ~, left_sphere_joints = rotate3(left_sphere_joints, vec_x_rotational, θz)

    ~, right_sphere_joints = rotate3(steering.sphere_joints_neutral[2], rotational_component_ucs[1,:], θx)
    ~, right_sphere_joints = rotate3(right_sphere_joints, vec_x_rotational, θz)


    steering.sphere_joints = (left_sphere_joints, right_sphere_joints) 

    ###############
    wheel_ucs = [[1;0;0] [0;1;0] [0;0;1]]


    suspension.kinematics!(suspension)


    wheel_ucs_position = (suspension.lowerwishbone[1].sphere_joint, suspension.lowerwishbone[2].sphere_joint)

    
    left_axis_z = (suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint) / norm(suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint)
    right_axis_z = (suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint) / norm(suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint)
    wheel_ucs_axis_z = (left_axis_z,  right_axis_z)

    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[1])
    left_base_vec = [base_vec_x base_vec_y base_vec_z]

    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[2])
    right_base_vec = [base_vec_x base_vec_y base_vec_z]

    base_vec_wheel_ucs = (left_base_vec, right_base_vec)    # (left, right)

    ##############
    lower_end_of_rotational_component = (steering.wishbone_ucs_position[1].*[-1,-1,-1] - [0.0,0.0,steering.rotational_component.x_rotational_radius])
    
    intersection = lineplane(suspension.lowerwishbone[1].sphere_joint_neutral,
                                base_vec_wheel_ucs[1][:,3],[0.0,0.0,1.0], 
                                lower_end_of_rotational_component)
    
    vec_offset = intersection - suspension.lowerwishbone[1].sphere_joint_neutral
    offset_length = norm(vec_offset)
    ##############
    
    track_lever_mount = [0.0; 0.0; offset_length]

    

    #left
    track_lever_mount_IN_wheel_ucs = EulerExtrinsicRotations(track_lever_mount, 
                                                            base_vec_wheel_ucs[1][:,1],
                                                            base_vec_wheel_ucs[1][:,2],
                                                            base_vec_wheel_ucs[1][:,3], 
                                                            wheel_ucs[:,1],
                                                            wheel_ucs[:,2], 
                                                            wheel_ucs[:,3],
                                                            tran = true) 


    track_lever_mount_IN_wishbone_ucs = wheel_ucs_position[1] + track_lever_mount_IN_wheel_ucs

    track_lever_mount_IN_steering_ucs = steering.wishbone_ucs_position[1] + track_lever_mount_IN_wishbone_ucs


    circle_joints_1, circle_joints_2 = circsphere(track_lever_mount_IN_steering_ucs,
                                steering.track_lever.length,
                                base_vec_wheel_ucs[1][:,3],
                                steering.sphere_joints[1],
                                steering.tie_rod.length)


    if circle_joints_2[1] - track_lever_mount_IN_steering_ucs[1] < circle_joints_1[1] - track_lever_mount_IN_steering_ucs[1]                          
        left_circle_joints = circle_joints_2
    else
        left_circle_joints = circle_joints_1
    end

    #right

    track_lever_mount_IN_wheel_ucs = EulerExtrinsicRotations(track_lever_mount, 
                                                                base_vec_wheel_ucs[2][:,1],
                                                                base_vec_wheel_ucs[2][:,2],
                                                                base_vec_wheel_ucs[2][:,3], 
                                                                wheel_ucs[:,1],
                                                                wheel_ucs[:,2], 
                                                                wheel_ucs[:,3],
                                                                 tran = true) 


    track_lever_mount_IN_wishbone_ucs = wheel_ucs_position[2] + track_lever_mount_IN_wheel_ucs

    track_lever_mount_IN_steering_ucs = steering.wishbone_ucs_position[2] + track_lever_mount_IN_wishbone_ucs.*[1,-1,1]

    circle_joints_1, circle_joints_2 = circsphere(track_lever_mount_IN_steering_ucs,
                                                    steering.track_lever.length,
                                                    base_vec_wheel_ucs[2][:,3],
                                                    steering.sphere_joints[2],
                                                    steering.tie_rod.length)


    if circle_joints_2[1] - track_lever_mount_IN_steering_ucs[1] < circle_joints_1[1] - track_lever_mount_IN_steering_ucs[1]                          
        right_circle_joints = circle_joints_2
    else
        right_circle_joints = circle_joints_1
    end

    steering.circle_joints = (left_circle_joints, right_circle_joints)
end

