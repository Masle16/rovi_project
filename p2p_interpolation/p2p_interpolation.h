#ifndef P2P_INTERPOLATION_H
#define P2P_INTERPOLATION_H

#include <rw/rw.hpp>
#include <iostream>
#include <vector>
#include <map>

class P2P_interpolator
{
public:
    P2P_interpolator();

    void ts_interpolation(std::vector<rw::math::Vector3D<double>> p, std::vector<rw::math::RPY<double>> r);
    rw::math::Vector3D<double> parabolic_blend(double tmT, rw::math::Vector3D<double> p_i, rw::math::Vector3D<double> v1, rw::math::Vector3D<double> v2, double tau);

    rw::math::Q parabolic_blend(double tmT, rw::math::Q p_i, rw::math::Q v1, rw::math::Q v2, double tau);

    std::map<int, rw::math::Q> q_interpolation(std::vector<rw::math::Q> points, std::vector<double> times);

    std::map<int, rw::math::Vector3D<double>> xyz_interpolation(std::vector<rw::math::Vector3D<double>> points, std::vector<double> times);

    std::map<int, rw::math::Quaternion<double>> quat_interpolation(std::vector<rw::math::Quaternion<double>> rotations, std::vector<double> times);

    rw::math::Quaternion<double> quat_int(rw::math::Quaternion<double> q1, rw::math::Quaternion<double> q2, double T);

    void print_q_to_rwplay(rw::models::WorkCell::Ptr wc, std::map<int, rw::math::Q> path);

private:

    double constant_vel(double t, double t0, double t1);
    double ramp_up_vel(double t, double t0, double t1);
    double ramp_down_vel(double t, double t0, double t1);

};

#endif // P2P_INTERPOLATION_H
