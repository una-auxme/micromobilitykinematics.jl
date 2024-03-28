
"""
    suspensionkinematicsMOVED!(suspension::Suspension)

calculates all the movement-dependent positions of the joints in MOVED-Position

#Arguments:
-`suspension::Suspension`: Suspension of the micromobility vihicle


"""
function suspensionkinematicsMOVED!(suspension::Suspension)
    
    for (side,index) in zip([:Left, :Right], [1,2])

        suspension.damper[index].id = side
        suspension.upperwishbone[index].id = side
        suspension.lowerwishbone[index].id = side

        center = suspension.damper[index].upper_fixture
        radius = suspension.damper[index].length
        normal =  suspension.lowerwishbone[index].rotation_axis
        circ_damper = GeoSpatialRelations.Circle(center, radius, normal)


        center = [suspension.damper[index].upper_fixture[1]; suspension.lowerwishbone[index].bearing_rear[2]; suspension.lowerwishbone[index].bearing_rear[3]]
        radius = suspension.lowerwishbone[index].distance_rotation_axis_to_lower_damper_fixure
        normal =  suspension.lowerwishbone[index].rotation_axis
        circ_lowerwishbone = GeoSpatialRelations.Circle(convert(Vector{Real},center), radius, normal)

        dampel_lower_fixure = intersection(circ_damper, circ_lowerwishbone)
    


        dampel_lower_fixure = dampel_lower_fixure[1]
        suspension.damper[index].lower_fixture = dampel_lower_fixure
        suspension.lowerwishbone[index].lower_fixture = dampel_lower_fixure
    
        rotation_axis_TO_sphere_joint = [0;suspension.damper[index].lower_fixture[2]; suspension.damper[index].lower_fixture[3]]
        rotation_axis_TO_sphere_joint =  (-suspension.lowerwishbone[index].bearing_rear + rotation_axis_TO_sphere_joint) / norm(-suspension.lowerwishbone[index].bearing_rear + rotation_axis_TO_sphere_joint)
        suspension.lowerwishbone[index].sphere_joint = suspension.lowerwishbone[index].bearing_rear + suspension.lowerwishbone[index].rotation_axis * suspension.lowerwishbone[index].distance_to_joint_x + rotation_axis_TO_sphere_joint * suspension.lowerwishbone[index].distance_to_joint_y
        


        center = suspension.upperwishbone[index].bearing_rear + suspension.upperwishbone[index].distance_to_joint_x * ((suspension.upperwishbone[index].bearing_front - suspension.upperwishbone[index].bearing_rear) / norm(suspension.upperwishbone[index].bearing_front - suspension.upperwishbone[index].bearing_rear))
        radius = suspension.upperwishbone[index].distance_to_joint_y
        normal = suspension.upperwishbone[index].rotation_axis
        circ_upperwishbone = GeoSpatialRelations.Circle(convert(Vector{Real},center), radius, normal)


        center = suspension.lowerwishbone[index].sphere_joint
        radius = suspension.wheelmount.length

        sphere_lower_sphere_joint = GeoSpatialRelations.Sphere(center, radius)
    
        suspension.upperwishbone[index].sphere_joint = intersection(circ_upperwishbone, sphere_lower_sphere_joint)[1]
    end
end


"""
    suspensionkinematicsNEUTRAL!(suspension::Suspension)

calculates all the movement-dependent positions of the joints in NEUTRAL-Position

#Arguemnts:
-`suspension::Suspension`: Suspension of the micromobility vihicle


"""
function suspensionkinematicsNEUTRAL!(suspension::Suspension)

    for (side,index) in zip([:Left, :Right], [1,2])

        suspension.damper[index].id = side
        suspension.upperwishbone[index].id = side
        suspension.lowerwishbone[index].id = side

        #circle 1
        center = suspension.damper[index].upper_fixture
        radius = suspension.damper[index].length_neutral
        normal =  suspension.lowerwishbone[index].rotation_axis
        circ_damper = Circle(center, radius, normal)

        #circle 2
        center = [suspension.damper[index].upper_fixture[1]; suspension.lowerwishbone[index].bearing_rear[2]; suspension.lowerwishbone[index].bearing_rear[3]]
        radius = suspension.lowerwishbone[index].distance_rotation_axis_to_lower_damper_fixure
        normal =  suspension.lowerwishbone[index].rotation_axis
        circ_lowerwishbone = Circle(convert(Vector{Real},center), radius, normal)

        dampel_lower_fixure = intersection(circ_damper, circ_lowerwishbone)
    


        dampel_lower_fixure = dampel_lower_fixure[1]
        suspension.damper[index].lower_fixture_neutral = dampel_lower_fixure
        suspension.lowerwishbone[index].lower_fixture_neutral = dampel_lower_fixure
    
        rotation_axis_TO_sphere_joint = [0;suspension.damper[index].lower_fixture_neutral[2]; suspension.damper[index].lower_fixture_neutral[3]]
        rotation_axis_TO_sphere_joint =  (-suspension.lowerwishbone[index].bearing_rear + rotation_axis_TO_sphere_joint) / norm(-suspension.lowerwishbone[index].bearing_rear + rotation_axis_TO_sphere_joint)
        suspension.lowerwishbone[index].sphere_joint_neutral = suspension.lowerwishbone[index].bearing_rear + suspension.lowerwishbone[index].rotation_axis * suspension.lowerwishbone[index].distance_to_joint_x + rotation_axis_TO_sphere_joint * suspension.lowerwishbone[index].distance_to_joint_y
        

        #circle 
        center = suspension.upperwishbone[index].bearing_rear + suspension.upperwishbone[index].distance_to_joint_x * ((suspension.upperwishbone[index].bearing_front - suspension.upperwishbone[index].bearing_rear) / norm(suspension.upperwishbone[index].bearing_front - suspension.upperwishbone[index].bearing_rear))
        radius = suspension.upperwishbone[index].distance_to_joint_y
        normal = suspension.upperwishbone[index].rotation_axis
        circ_upperwishbone = Circle(convert(Vector{Real},center), radius, normal)

        #Sphere 
        center = suspension.lowerwishbone[index].sphere_joint_neutral
        radius = suspension.wheelmount.length
        sphere_lower_sphere_joint = Sphere(center, radius)
    
        suspension.upperwishbone[index].sphere_joint_neutral = intersection(circ_upperwishbone, sphere_lower_sphere_joint)[1]
    end
end

"""
    suspensionkinematics!(suspension::Suspension)

    suspensionkinematicsNEUTRAL!(suspension::Suspension)

    calculates all the movement-dependent positions of the joints in NEUTRAL- and MOVED-Postion
    
#Arguments:
-`suspension::Suspension`: Suspension of the micromobility vihicle
"""
function suspensionkinematics!(suspension::Suspension)
    suspensionkinematicsNEUTRAL!(suspension)
    suspensionkinematicsMOVED!(suspension)
end