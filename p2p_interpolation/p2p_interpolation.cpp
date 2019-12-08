#include "p2p_interpolation.h"

P2P_interpolator::P2P_interpolator()
{

}

double P2P_interpolator::constant_vel(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}

double P2P_interpolator::ramp_up_vel(double t, double t0, double t1) {
    return pow((t-t0)/(t1-t0), 2);
}

double P2P_interpolator::ramp_down_vel(double t, double t0, double t1) {
    return 1-pow((t1-t)/(t1-t0), 2);
}

rw::math::Vector3D<double> P2P_interpolator::parabolic_blend(double tmT, rw::math::Vector3D<double> p_i, rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2, double tau) {
    return (v2-v1)/(4*tau)*pow(tmT+tau, 2)+v1*tmT+p_i;
}

rw::math::Q P2P_interpolator::parabolic_blend(double tmT, rw::math::Q p_i, rw::math::Q v1, rw::math::Q v2, double tau) {
    return (v2-v1)/(4*tau)*pow(tmT+tau, 2)+v1*tmT+p_i;
}

std::map<int, rw::math::Q> P2P_interpolator::q_interpolation(std::vector<rw::math::Q> points, std::vector<double> times) {

    double time_step = 0.1;
    double time = 0;
    double t = 0;
    int time_index = 0;

    // Linear interpolation between the points:
    std::map<int, rw::math::Q> interpolation;

    for (size_t j = 0; j < times.size(); j++) {

        // Calculate the number of steps for the next loop.
        int interval = int(std::round((time+times[j])-time));
        //std::cout << interval << std::endl;

        for (int i = 0; i <= interval/time_step; i++) {
            //std::cout << points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j]) << ", " << t << ", " << i << std::endl;
            //std::cout << points[j] << points[j+1] << constant_vel(i, time, times[j])*(points[j+1]-points[j]) << std::endl;
            interpolation.insert(std::pair<int, rw::math::Q>(time_index, points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j])));
            t += time_step;
            time_index++;
        }
        time_index--;
        time += times[j];
        t = time;
    }

    double T = 0;
    for (size_t i = 1; i < points.size()-1; i++) {
        T += times[i-1];
        double tau = times[i-1] < times[i] ? times[i-1]/3 : times[i]/3;
        tau = std::round(tau*10)/10;

        double blend_time = T-tau;

        rw::math::Q v1 = (points[i]-points[i-1])/times[i-1];
        rw::math::Q v2 = (points[i+1]-points[i])/times[i];

        //std::cout << T << ", " << blend_time << ", " << tau << std::endl;
        //std::cout << interpolation.find(int((T-tau)*10))->first << ", " << interpolation.find(int((T-tau)*10))->second << std::endl;

        int blend_steps = 0;
        while (blend_steps++ <= int(std::round((2*tau)/time_step))) {
            // Insert the blend into the interpolation.
            interpolation.find(int(std::round(blend_time*10)))->second = parabolic_blend(blend_time-T, points[i], v1, v2, tau);
            //std::cout << parabolic_blend(blend_time-T, points[i], v1, v2, tau) << ", " << blend_time << ", " << T+tau << std::endl;
            blend_time += time_step;
        }
        //std::cout << interpolation.find(int((T+tau)*10))->first << ", " << interpolation.find(int((T+tau)*10))->second << std::endl;
        //std::cout << "Next point" << std::endl;
    }

    /*
    std::map<int, rw::math::Q>::iterator it;
    for (it = interpolation.begin(); it != interpolation.end(); it++) {
        std::cout << it->first << ", " << it->second << std::endl;
    }
    */

    return interpolation;
}

std::map<int, rw::math::Vector3D<double>> P2P_interpolator::xyz_interpolation(std::vector<rw::math::Vector3D<double>> points, std::vector<double> times) {

    double time_step = 0.1;
    double time = 0;
    double t = 0;
    int time_index = 0;

    // Linear interpolation between the points:
    std::map<int, rw::math::Vector3D<double>> interpolation;

    for (size_t j = 0; j < times.size(); j++) {

        // Calculate the number of steps for the next loop.
        int interval = int(std::round((time+times[j])-time));
        //std::cout << interval << std::endl;

        for (int i = 0; i <= interval/time_step; i++) {
            //std::cout << points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j]) << ", " << t << ", " << i << std::endl;
            //std::cout << points[j] << points[j+1] << constant_vel(i, time, times[j])*(points[j+1]-points[j]) << std::endl;
            interpolation.insert(std::pair<int, rw::math::Vector3D<double>>(time_index, points[j]+constant_vel(t, time, time+times[j])*(points[j+1]-points[j])));
            t += time_step;
            time_index++;
        }
        time_index--;
        time += times[j];
        t = time;
    }

    double T = 0;
    for (size_t i = 1; i < points.size()-1; i++) {
        T += times[i-1];
        double tau = times[i-1] < times[i] ? times[i-1]/3 : times[i]/3;
        tau = std::round(tau*10)/10;

        double blend_time = T-tau;

        rw::math::Vector3D<double> v1 = (points[i]-points[i-1])/times[i-1];
        rw::math::Vector3D<double> v2 = (points[i+1]-points[i])/times[i];

        //std::cout << T << ", " << blend_time << ", " << tau << std::endl;
        //std::cout << interpolation.find(int((T-tau)*10))->first << ", " << interpolation.find(int((T-tau)*10))->second << std::endl;

        int blend_steps = 0;
        while (blend_steps++ <= int(std::round((2*tau)/time_step))) {
            // Insert the blend into the interpolation.
            interpolation.find(int(std::round(blend_time*10)))->second = parabolic_blend(blend_time-T, points[i], v1, v2, tau);
            //std::cout << parabolic_blend(blend_time-T, points[i], v1, v2, tau) << ", " << blend_time << ", " << T+tau << std::endl;
            blend_time += time_step;
        }
        //std::cout << interpolation.find(int((T+tau)*10))->first << ", " << interpolation.find(int((T+tau)*10))->second << std::endl;
        //std::cout << "Next point" << std::endl;
    }

    /*
    std::map<int, rw::math::Vector3D<double>>::iterator it;
    for (it = interpolation.begin(); it != interpolation.end(); it++) {
        std::cout << it->first << ", " << it->second << std::endl;
    }
    */

    return interpolation;
}

std::map<int, rw::math::Quaternion<double>> P2P_interpolator::quat_interpolation(std::vector<rw::math::Quaternion<double>> rotations, std::vector<double> times) {

    std::map<int, rw::math::Quaternion<double>> interpolation;

    double time_step = 0.1;

    int time_index = 0;

    double time = 0;
    double t2 = 0;

    for (size_t j = 0; j < rotations.size()-1; j++) {
        //std::cout << time_index << std::endl;
        double steps = times[j]/time_step;
        double t = 0;
        rw::math::Quaternion<double> q1 = rotations[j];
        rw::math::Quaternion<double> q2 = rotations[j+1];
        double angle = std::acos( q1.getQw() * q2.getQw() + q1.getQx() * q2.getQx() + q1.getQy() * q2.getQy() + q1.getQz() * q2.getQz() );
        if (angle >= 0.001) {
            for (int i = 0; i <= steps; i++) {
                interpolation.insert(std::pair<int, rw::math::Quaternion<double>>(time_index, quat_int(q1, q2, t)));
                //std::cout << quat_int(q1, q2, t) << std::endl;
                t += 1/steps;
                time_index++;
            }
            //std::cout << "slerp, " <<  j << std::endl;
        } else {
            for (int i = 0; i <= steps; i++) {
                interpolation.insert(std::pair<int, rw::math::Quaternion<double>>(time_index, q1+constant_vel(t2, time, time+times[j])*(q2-q1)));
                //std::cout << q1+constant_vel(t2, time, time+times[j])*(q2-q1) << std::endl;
                t2 += time_step;
                time_index++;
            }
            //std::cout << "lerp, " <<  j << std::endl;
        }
        time_index--;
        time += times[j];
        t2 = time;
    }

    return interpolation;
}

rw::math::Quaternion<double> P2P_interpolator::quat_int(rw::math::Quaternion<double> q1, rw::math::Quaternion<double> q2, double T) {
    double angle = std::acos( q1.getQw() * q2.getQw() + q1.getQx() * q2.getQx() + q1.getQy() * q2.getQy() + q1.getQz() * q2.getQz() );
    std::cout << "Angle: " << angle << std::endl;
    //std::cout << std::sin(angle) << std::endl;
    return (std::sin((1-T)*angle))/(std::sin(angle))*q1 + (std::sin(T*angle))/(std::sin(angle))*q2;
}

rw::trajectory::TimedStatePath P2P_interpolator::inverse_kin(rw::models::SerialDevice::Ptr robot, rw::kinematics::State init_state, rw::math::Q init_joint_config, std::map<int, rw::math::Vector3D<double>> xyz, std::map<int, rw::math::Quaternion<double>> quat) {

    file_output.open(toolSpace_filename);
    rw::math::Transform3D<double> FK;

    rw::trajectory::TimedStatePath output_path;
    rw::kinematics::State state = init_state;
    rw::invkin::ClosedFormIKSolverUR::Ptr ikSolver = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot, state));


    rw::math::Q last_best = init_joint_config;
    for (int i = 0; i < int(xyz.size()); i++) {

        rw::math::Transform3D<double> trans(xyz.find(i)->second/100., quat.find(i)->second.toRotation3D());
        //std::cout << trans << std::endl;
        std::vector<rw::math::Q> ik_result = ikSolver->solve(trans, state);
        //std::cout << ik_result.size() << std::endl;

        size_t best_result = 0;
        rw::math::Q q_best;
        rw::math::Q q_dif;
        q_best = last_best-ik_result[0];
        for (size_t j = 1; j < ik_result.size(); j++) {
            q_dif = last_best-ik_result[j];
            if (q_dif.norm1() < q_best.norm1()) {
                q_best = q_dif;
                best_result = j;
                //std::cout << i << " - " << q_best.norm1() << std::endl;
            }
        }
        last_best = ik_result[best_result];
        //std::cout << last_best << std::endl;

        robot->setQ(ik_result[best_result], state);
        output_path.push_back(rw::trajectory::TimedState(double(xyz.find(i)->first)/10., state));

        std::cout << robot->baseTend(state) << std::endl;
        FK = robot->baseTend(state);
        file_output << double(xyz.find(i)->first)/10. << "," << FK.P()[0] << "," << FK.P()[1] << "," << FK.P()[2] <<
                       "," << FK.R().getRow(0)[0] << "," << FK.R().getRow(0)[1] << "," << FK.R().getRow(0)[2] <<
                       "," << FK.R().getRow(1)[0] << "," << FK.R().getRow(1)[1] << "," << FK.R().getRow(1)[2] <<
                       "," << FK.R().getRow(2)[0] << "," << FK.R().getRow(2)[1] << "," << FK.R().getRow(2)[2] << std::endl;
    }
    file_output.close();
    return output_path;
}

