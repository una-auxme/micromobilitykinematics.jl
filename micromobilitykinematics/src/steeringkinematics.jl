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
    wheel_ucs = [[1.0;0.0;0.0] [0.0;1.0;0.0] [0.0;0.0;1.0]]


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
    
    #line
    line = Line(suspension.lowerwishbone[1].sphere_joint_neutral, left_base_vec[:,3])
 
    #plane
    plane = Plane(lower_end_of_rotational_component, [0.0,0.0,1.0])

    inter = intersection(line, plane)
    
    vec_offset = inter - suspension.lowerwishbone[1].sphere_joint_neutral
    offset_length = norm(vec_offset)
    ##############
    
    track_lever_mount = [0.0; 0.0; offset_length]

    

    for (circle_joint,index,shift) in zip([:left, :right], [1,2], [[1,1,1],[1,-1,1]])

        track_lever_mount_IN_wheel_ucs = applyMatrixRotation(track_lever_mount,base_vec_wheel_ucs[index], wheel_ucs)

        track_lever_mount_IN_wishbone_ucs = wheel_ucs_position[index] + track_lever_mount_IN_wheel_ucs
        track_lever_mount_IN_steering_ucs = steering.wishbone_ucs_position[index] + track_lever_mount_IN_wishbone_ucs.*(shift)


        #Circle 
        circ = Circle(track_lever_mount_IN_steering_ucs,steering.track_lever.length,base_vec_wheel_ucs[index][:,3])

        #Sphere
        sphere = Sphere(steering.sphere_joints[index],steering.tie_rod.length)


        circle_joints_1, circle_joints_2 = intersection(circ, sphere)
        

        if circle_joints_2[1] - track_lever_mount_IN_steering_ucs[1] < circle_joints_1[1] - track_lever_mount_IN_steering_ucs[1]                          
            @eval $circle_joint = $circle_joints_2
        else
            @eval $circle_joint = $circle_joints_1
        end
    end

    steering.circle_joints = (collect(left), collect(right))
end

