init_angle = [-pi/2, 0, pi/2];
w_b = [pi/, 0, 0];
init_quat = eul2quat(-init_angle, 'ZYX');
dq = quat_kinematics(w_b', init_quat');
q_new = init_quat + dq'
deg_new = rad2deg(quat2eul(q_new,'ZYX'))

angle_expect = [-pi/4, 0, pi/2];
q_expect = eul2quat(-angle_expect,'ZYX')

q_deg_new = eul2quat(deg2rad(deg_new))