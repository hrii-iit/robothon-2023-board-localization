import numpy as np

def TF_cam_W(x_1, x_2, y_1, y_2, z_1, z_m, z_2, cx, cy, fx, fy):
    point1 = np.array([x_1, y_1])
    point2 = np.array([x_2, y_2])
    
    angle = (np.arctan2(point2[1] - point1[1], point2[0] - point1[0]) +np.pi)
    angle = angle   ##Sth have been commented
    euler_angles = [0, 0, angle]
    z_1 = 480
    z_2 = 490

    O = np.array([0.0, 0.0, 0.0, 1.00]).reshape((4,1))
    translation = np.array([[x_1*0.001, y_1*0.001, (100.0*0.001)-z_m, 1.00]])
    R = euler_to_rotation_matrix(euler_angles)
    translation_o = np.array([[-0.024, -0.097, -100.0*0.001, 1.00]])
    R_a = np.vstack((R, np.array([[0, 0, 0]])))
    T_M = np.hstack((R_a , translation.T))
    R_O = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    R_O = np.vstack((R_O, np.array([[0, 0, 0]])))
    T_O = np.hstack((R_O , translation_o.T))
    T_T = np.dot(T_M, T_O)
    O_new = np.dot(T_T, O)
    R_A = T_T[0:3, 0:3]
    q = from_rotation_matrix(R_A)
    transform_msg.header.frame_id = 'world'
    transform_msg.child_frame_id = 'task_board_base_link'
    transform_msg.header.stamp = rospy.Time.now()

    transform_msg.transform.translation.x = O_new[0, 0]
    transform_msg.transform.translation.y = O_new[1, 0]
    transform_msg.transform.translation.z = O_new[2, 0]
    transform_msg.transform.rotation.x = q.x
    transform_msg.transform.rotation.y = q.y
    transform_msg.transform.rotation.z = q.z
    transform_msg.transform.rotation.w = q.w


    return transform_msg, T_T



    

    

