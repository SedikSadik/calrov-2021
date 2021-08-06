void AP_Motors6DOF::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // ÖNCEDEN TANIMLI MOTORLARIN OLMAMASININ KONTROLÜ
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    // MOTORLARIN TANIMLANMASI
    switch ((sub_frame_t)frame_class) {
        //                 Motorlar #        Roll   Pitch    Yaw   Throttle Forward  Lateral Testing
    case SUB_FRAME_BLUEROV1:
        _frame_class_string = "TALAY1";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,  0    ,  0    , -1.0f ,   0    ,   1.0f ,   0   ,  1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,  0    ,  0    ,  1.0f ,   0    ,   1.0f ,   0   ,  2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3, -0.5f ,  0 .5f,  0    ,   0.45f,   0    ,   0   ,  3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,  0.5f ,  0.5f ,  0    ,   0.45f,   0    ,   0   ,  4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,  0    , -1.0f ,  0    ,   1.0f ,   0    ,   0   ,  5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6, -0.25f,  0    ,  0    ,   0    ,   0    ,   1.0f,  6);
        break;