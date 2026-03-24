#include "fr3_husky_controller/utils/motion_primitives.h"

namespace primitives
{
    Eigen::Isometry3d approach_motion(const double vel, // target velocity
                                      const double t,   // current time
                                      const double t_0) // initial time
    {

        // Purpose
        //     Generate a linear otion of an end-effector toward an object
        // Output
        //     force w.r.t end-effector. It should be transformed into a command force w.r.t the robot base.

        Eigen::Isometry3d x;
        Eigen::Vector2d result;

        x = Eigen::Isometry3d::Identity();
        result = trapezoid(t, t_0, vel);

        x.translation()(2) = result(0);

        return x;
    }

    Eigen::Isometry3d spiral_motion(const double pitch,
                                    const double lin_vel,
                                    const double t,
                                    const double t_0,
                                    const double duration,
                                    double direction)
    // Generate spiral trajectory w.r.t the end-effector frame.
    // The end-effector will move on the X-Y plane keeping its initial orientation.
    // Also, no movement along Z-axis.
    {
        Eigen::Isometry3d x;
        Eigen::Vector2d traj;

        x = Eigen::Isometry3d::Identity();

        traj = dyros_math::spiral(t, t_0, t_0 + duration, Eigen::Vector2d{0, 0}, lin_vel, pitch, direction);
        x.translation().head<2>() = traj;

        return x;
    }

    
    Eigen::Vector2d trapezoid(const double t,
                              const double t_0,
                              const double v_sat)
    {
        double t_e, t_sat;
        double p, v;
        Eigen::Vector2d result; // p, v

        t_sat = v_sat * 10; // to make acc be 10cm/s^2

        t_e = t - t_0;

        if (t_e < t_sat)
        {
            v = (v_sat) / (t_sat)*t_e;
            p = 0.5 * t_e * v;
        }
        else
        {
            v = v_sat;
            p = 0.5 * v_sat * t_sat + v_sat * (t_e - t_sat);
        }

        result(0) = p;
        result(1) = v;

        return result;
    }

    double push(const double force,
                const double t,
                const double t_0,
                const double duration)
    {
        double f_desired;
        f_desired = dyros_math::cubic(t, t_0, t_0 + duration, 0.0, force, 0.0, 0.0);
        return f_desired;
    }

    Eigen::Isometry3d hold_pose()
    {
        Eigen::Isometry3d x_ee;

        x_ee = Eigen::Isometry3d::Identity();
        return x_ee;
    }

    void accum_tau(Eigen::Vector7d &accum_tau,
                   const Eigen::Ref<const Eigen::Vector7d> &tau_ext)
    {
        for (int i = 0; i < 7; i++)
            accum_tau(i) += tau_ext(i);
    }

    void accum_wrench(Eigen::Vector6d &accum_wrench,
                      const Eigen::Ref<const Eigen::Vector6d> &f_ext)
    {
        for (int i = 0; i < 6; i++)
            accum_wrench(i) += f_ext(i);
    }

    Eigen::Vector3d wrtEndEffector(const Eigen::Isometry3d &world,
                                   const Eigen::Vector3d &x,
                                   bool debug)
    {
        Eigen::Matrix3d rot, rot_inv;
        Eigen::Vector3d y;

        rot = world.linear(); // R_w2ee
        rot_inv = rot.transpose(); // R_ee2w
        y = rot_inv * x;
        
        if(debug == true){
            std::cout<<"--------------------"<<std::endl;
            std::cout<<"rotation w.r.t world : \n"<<rot<<std::endl;
            std::cout<<"rotation w.r.t local : \n"<<rot_inv<<std::endl;
            std::cout<<"before : "<< x.transpose()<<std::endl;
            std::cout<<"after  : "<< y.transpose()<<std::endl;  

        }
        return y;
    }

    Eigen::Vector3d wrtBaseFrame(const Eigen::Isometry3d &local,
                                 const Eigen::Vector3d &x)
    {
        Eigen::Matrix3d rot;
        Eigen::Vector3d y;

        rot = local.linear();  // ee2Rw
        rot = rot.transpose(); // Rw2ee
        y = rot * x;
        return y;
    }

    bool detectHole(const Eigen::Vector3d &f_ext,
                    const double threshold)
    {
        bool result;
        double f;

        f = sqrt(pow(f_ext(0), 2) + pow(f_ext(1), 2)); // reaction force from X-Y plane wrt {A}

        if (f >= threshold)
        {
            result = true;
        }
        else
            result = false;

        return result;
    }

    Eigen::Matrix3d wiggle_motion(const double angle,
                                  const double w, // rad/s
                                  const double t,
                                  const double t_0)
    // Output is a 3x3 matrix for wiggle angle of the end-effector.
    // The reference frame is end-effector's frame
    {
        Eigen::Vector3d x_axis, y_axis;
        Eigen::Matrix3d target_rot, rot_x, rot_y;
        double t_e;
        double roll, pitch;

        x_axis = Eigen::Vector3d::UnitX();
        y_axis = Eigen::Vector3d::UnitY();

        t_e = t - t_0;
        roll = angle * sin(w * t_e);

        if(w*t_e <= M_PI_4)
            pitch = angle * sin(w * t_e);
        else
            pitch = angle * cos(w * t_e);

        rot_x = dyros_math::angleaxis2rot(x_axis, roll); // wrt {A}
        rot_y = dyros_math::angleaxis2rot(y_axis, pitch); // wrt {A}
        target_rot = rot_x * rot_y;

        return target_rot; // w.r.t end-effector frame
    }

    Eigen::Isometry3d tilt_motion(const Eigen::Vector3d cor, // center of rotation from the end-effector frame, (tcp)
                                  const Eigen::Vector3d axis,
                                  const double angle)
    {
        Eigen::Isometry3d target_transform; // transformation matrix w.r.t end-effector frame
        Eigen::Matrix3d r_cor; // target orientation w.r.t cor
        Eigen::Vector3d p_cor; // target position w.r.t cor

        r_cor = dyros_math::angleaxis2rot(axis, angle); 
        p_cor = r_cor * (-cor); // -cor == ee position w.r.t cor

        target_transform.translation() = cor + p_cor;
        target_transform.linear() = r_cor;

        return target_transform;
    }

    Eigen::Matrix3d yawing_motion(const double angle,
                                  const double w, // rad/s
                                  const double t,
                                  const double t_0)
    // Output is a 3x3 matrix for wiggle angle of the end-effector.
    // The reference frame is end-effector's frame
    {
        Eigen::Vector3d z_axis;
        Eigen::Matrix3d rot_z;
        double t_e;
        double yaw;

        z_axis = Eigen::Vector3d::UnitZ();

        t_e = t - t_0;
        yaw = angle * sin(w * t_e);


        rot_z = dyros_math::angleaxis2rot(z_axis, yaw); // wrt {A}
        return rot_z; // w.r.t end-effector frame
    }



    Eigen::MatrixXd pouring_motion(double pouring_angle,                       //Pouring angle [deg]
                                const Eigen::Vector3d& pouring_axis,			   //Pouring axis w.r.t {Base}
                                const Eigen::Vector3d& pouring_radius,		   //Pouring axis w.r.t {Base}
                                const Eigen::Vector3d& pouring_translation,	   //Translation of rotation axis during pouring w.r.t {Base}
                                const Eigen::Vector3d& pos_0,                   //Initial EE position
                                const Eigen::Matrix3d& rot_0,                   //Initial EE rotation
                                const double t,
                                const double t_0,
                                double duration,
                                bool return_dot)
    {   
        Eigen::Vector3d pouring_axis_ee;			//Pouring axis w.r.t {Initial EE}
        pouring_axis_ee = rot_0.transpose() * pouring_axis;
        Eigen::Vector3d pouring_radius_ee;			//Pouring radius w.r.t {Initial EE}
        pouring_radius_ee = rot_0.transpose() * pouring_radius;

        std::cout << "pos_0" << pos_0.transpose() << std::endl;
        std::cout << "rot_0\n" << rot_0.transpose() << std::endl;
        std::cout << "axis_ee" << pouring_axis_ee.transpose() << std::endl;
        std::cout << "cor_ee" << pouring_radius_ee.transpose() << std::endl << std::endl;

        Eigen::Matrix4d x_trajectory_temp;			//Pouring trajectory w.r.t {Base} (Function Output)
        Eigen::Matrix<double, 3, 2> x_dot_trajectory_temp;	//Pouring dot trajectory w.r.t {Base} (Function Output)

    
        double angle_quintic;
        angle_quintic = dyros_math::quintic(t, t_0, t_0 + duration, 0.0, pouring_angle * DEG2RAD, 0.0, 0.0, 0.0, 0.0);

        Eigen::Isometry3d axisRot_ee;			    //Axis rotation trajectory w.r.t {initial EE}
        axisRot_ee = dyros_math::rigidRotation(pouring_radius_ee, pouring_axis_ee, angle_quintic);	//radius, axis must be w.r.t {initial EE}

        Eigen::Isometry3d axisRot;				    //Axis rotation trajectory w.r.t {base}
        axisRot.linear() = rot_0 * axisRot_ee.linear();
        axisRot.translation() = pos_0 + (rot_0 * axisRot_ee.translation());

        x_trajectory_temp = axisRot.matrix();	    //Desired EE trajectory output


        double angle_quinticDot;
        angle_quinticDot = dyros_math::quinticDot(t, t_0, t_0 + duration, 0.0, pouring_angle * DEG2RAD, 0.0, 0.0, 0.0, 0.0);

        Eigen::Vector3d pouring_w;				    //pouring w vector w.r.t {base}
        pouring_w = angle_quinticDot * (rot_0 * pouring_axis_ee);

        Eigen::Vector3d pouring_center;			    //Pouring center w.r.t {base}
        pouring_center = pos_0 + (rot_0 * pouring_radius_ee);

        x_dot_trajectory_temp.col(1) = pouring_w.cross(x_trajectory_temp.block<3, 1>(0, 3) - pouring_center);	//Desired linear velocity trajectory output

        Eigen::Isometry3d rot_f_ee;				    //Final orientation w.r.t {initial EE}
        Eigen::Isometry3d rot_f;				    //Final orientation w.r.t {initial EE}
        rot_f_ee = dyros_math::rigidRotation(pouring_radius_ee, pouring_axis_ee, pouring_angle * DEG2RAD);
        rot_f.linear() = rot_0 * rot_f_ee.linear();

        rot_f.translation() = pos_0 + rot_f_ee.translation();

        x_dot_trajectory_temp.col(0) = dyros_math::rotationQuinticDot(t, t_0, t_0 + duration, rot_0, x_trajectory_temp.block<3, 3>(0, 0), rot_f.linear());
                                                                                                                //Desired angular velocity trajectory output
        for (int i = 0; i < 3; i++)
        {   
            x_trajectory_temp(i,3) += dyros_math::quintic(t, t_0, t_0 + duration, 0.0, pouring_translation(i), 0.0, 0.0, 0.0, 0.0);
            x_dot_trajectory_temp(i,1) += dyros_math::quinticDot(t, t_0, t_0 + duration, 0.0, pouring_translation(i), 0.0, 0.0, 0.0, 0.0);
        }

        if (return_dot == true) {return x_dot_trajectory_temp;}
        return x_trajectory_temp;
    }

}
