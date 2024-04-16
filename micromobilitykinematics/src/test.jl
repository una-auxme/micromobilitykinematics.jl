s1 = Suspension(30.0)

s2 = Steering(53.0, 151.0, 169.0, 249.0)

s2.kinematics!(20.0,40.0,s2,s1)


s2.track_lever_mounting_points_ucs


angle_δo(s2)

s2.circle_joints

s2.circle_joints_neutral

s2.base_vec_wheel_ucs

s2.sphere_joints
s2.sphere_joints_neutral

a = (s2.rotational_component.x_rotational_radius,s2.rotational_component.z_rotational_radius, s2.track_lever.length, s2.tie_rod.length)




