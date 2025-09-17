#pragma once
#include <msgpack.hpp>
#include <array>
#include <cmath>

class Transform {
    public:
        float x, y, z; // translation
        float qx, qy, qz, qw; // rotation in quaternion

        MSGPACK_DEFINE(x, y, z, qx, qy, qz, qw);

        // Normalize quaternion to unit length (avoid drift)
        void normalize() {
            float n = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            if (n > 0.0f) {
                qx /= n; qy /= n; qz /= n; qw /= n;
            } else {
                // fallback to identity on degenerate
                qx = qy = qz = 0.0f; qw = 1.0f;
            }
        }

        // Rotate a vector by this transform's quaternion
        std::array<float,3> rotateVector(const std::array<float,3>& v) const {
            // Using: v' = 2*dot(u,v)*u + (s^2 - dot(u,u))*v + 2*s*(u × v)
            float ux = qx, uy = qy, uz = qz, s = qw;
            float vx = v[0], vy = v[1], vz = v[2];

            float dot_uv = ux*vx + uy*vy + uz*vz;
            float dot_uu = ux*ux + uy*uy + uz*uz;

            // cross(u, v)
            float cx = uy*vz - uz*vy;
            float cy = uz*vx - ux*vz;
            float cz = ux*vy - uy*vx;

            float k1 = 2.0f * dot_uv;
            float k2 = (s*s - dot_uu);
            float k3 = 2.0f * s;

            return { k1*ux + k2*vx + k3*cx,
                     k1*uy + k2*vy + k3*cy,
                     k1*uz + k2*vz + k3*cz };
        }

        // Apply full SE(3) transform to a point
        std::array<float,3> transformPoint(const std::array<float,3>& p) const {
            auto r = rotateVector(p);
            return { r[0] + x, r[1] + y, r[2] + z };
        }

        // Overload the * operator to compose two transforms: this ∘ other
        // Result applies 'other' first, then 'this'.
        Transform operator*(const Transform& other) const {
            Transform result;

            // Rotation composition (quaternion multiplication): q = q1 ⊗ q2
            result.qw = qw * other.qw - qx * other.qx - qy * other.qy - qz * other.qz;
            result.qx = qw * other.qx + qx * other.qw + qy * other.qz - qz * other.qy;
            result.qy = qw * other.qy - qx * other.qz + qy * other.qw + qz * other.qx;
            result.qz = qw * other.qz + qx * other.qy - qy * other.qx + qz * other.qw;
            result.normalize();

            // Translation composition: t = t1 + R1 * t2
            auto t2_rot = rotateVector({other.x, other.y, other.z});
            result.x = x + t2_rot[0];
            result.y = y + t2_rot[1];
            result.z = z + t2_rot[2];

            return result;
        }
};