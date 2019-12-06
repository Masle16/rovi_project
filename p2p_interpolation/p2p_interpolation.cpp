#include "p2p_interpolation.h"

P2P_interpolator::P2P_interpolator()
{

}

void P2P_interpolator::ts_interpolation(std::vector<rw::math::Vector3D<double>> p, std::vector<rw::math::RPY<double>> r) {

    double time = 10;
    double time_step = 0.1;


    std::vector<rw::math::Vector3D<double>> positions1;
    std::vector<rw::math::Vector3D<double>> positions2;
    std::vector<rw::math::Vector3D<double>> ramp_up;

    // Linear interpolation

    for (double i = 0; i < time; i += time_step) {
        std::cout << p[0]+constant_vel(i, 0, time)*(p[1]-p[0]) << std::endl;
        positions1.push_back(p[0]+constant_vel(i, 0, time)*(p[1]-p[0]));
        //std::cout << i << std::endl;
    }

    std::cout << "Velocity between points on the linear interpolation: " << (p[1]-p[0])/time << std::endl;

    for (double i = 0; i < 4; i += time_step) {
        ramp_up.push_back(p[0]+ramp_up_vel(i, 0, 4)*(positions1[50]-p[0]));
        std::cout << p[0]+ramp_up_vel(i, 0, 4)*(positions1[50]-p[0]) << std::endl;
    }

    std::cout << "Velocity between points on the end of the ramp up: " << (*ramp_up.end()-*(ramp_up.end()-1))/time_step << std::endl;
    std::cout << *ramp_up.end() << positions1[50] << std::endl;

    for (double i = 0; i < time; i += time_step) {
        //std::cout << p[1]+constant_vel(i, 0, time)*(p[2]-p[1]) << std::endl;
        positions2.push_back(p[1]+constant_vel(i, 0, time)*(p[2]-p[1]));
    }

    // Parabolic blend

    double T = 10; // Intersection in time.
    double tau = 3; // The width of the blend.
    rw::math::Vector3D<double> v1 = (p[1]-p[0])/(10);
    rw::math::Vector3D<double> v2 = (p[2]-p[1])/(10);

    std::cout << positions1[70] << std::endl;
    std::cout << parabolic_blend(7-T, p[1], v1, v2, tau) << std::endl;
    std::cout << positions2[30] << std::endl;
    std::cout << parabolic_blend(13-T, p[1], v1, v2, tau) << std::endl;

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

    std::map<int, rw::math::Q>::iterator it;
    for (it = interpolation.begin(); it != interpolation.end(); it++) {
        std::cout << it->first << ", " << it->second << std::endl;
    }

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

    std::map<int, rw::math::Vector3D<double>>::iterator it;
    for (it = interpolation.begin(); it != interpolation.end(); it++) {
        std::cout << it->first << ", " << it->second << std::endl;
    }

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
    //std::cout << std::sin(angle) << std::endl;
    return (std::sin((1-T)*angle))/(std::sin(angle))*q1 + (std::sin(T*angle))/(std::sin(angle))*q2;
}

void P2P_interpolator::print_q_to_rwplay(rw::models::WorkCell::Ptr wc, std::map<int, rw::math::Q> path) {


}

