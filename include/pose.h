#pragma once

#include "Eigen/Dense"
#include "mathtils.h"
#include <iomanip>

class Pose {
    private:
        Eigen::Vector2f _pos;
        float _theta;

    public:
        Pose(float x, float y, float theta) : _pos(x, y), _theta(theta) {}
        Pose(const Eigen::Vector2f& pos, float theta) : _pos(pos), _theta(theta) {}
        Pose(const Pose& other) : _pos(other._pos), _theta(other._theta) {}

        const Eigen::Vector2f& get_pos(void) const { return _pos; }
        const Eigen::Vector2f& pos(void) const { return _pos; }
        float x(void) const { return _pos.x(); }
        float y(void) const { return _pos.y(); }
        float get_theta(void) const { return _theta; }
        float theta(void) const { return _theta; }

        void set_pos(const Eigen::Vector2f& pos) { this->_pos = pos; }
        void set_theta(float theta) { this->_theta = theta; }

        float angle_diff(float other) const {
            return minimum_mod_diff(_theta, other, M_2_PI);
        }

        float angle_diff(const Pose& other) const {
            return angle_diff(other._theta);
        }

        float angle_diff(const Eigen::Vector2f& other) const {
            return angle_diff(atan2f(other.x() - _pos.x(), other.y() - _pos.y()));
        }

        float angle_diff(float x, float y) const {
            return angle_diff(atan2f(x - _pos.x(), y - _pos.y()));
        }

        float distance(const Pose& other) const {
            return (_pos - other._pos).norm();
        }

        float distance(const Eigen::Vector2f& other) const {
            return (_pos - other).norm();
        }

        float distance(float x, float y) const {
            return (_pos - Eigen::Vector2f(x, y)).norm();
        }

        Pose& operator=(const Pose& other) {
            _pos = other._pos;
            _theta = other._theta;
            return *this;
        }

        Pose operator+(const Pose& other) const {
            return Pose(_pos + other._pos, _theta + other._theta);
        }

        Pose operator-(const Pose& other) const {
            return Pose(_pos - other._pos, _theta - other._theta);
        }

        Pose operator*(float scalar) const {
            return Pose(_pos * scalar, _theta * scalar);
        }

        Pose operator/(float scalar) const {
            return Pose(_pos / scalar, _theta / scalar);
        }

        Pose operator+=(const Pose& other) {
            _pos += other._pos;
            _theta += other._theta;
            return *this;
        }

        Pose operator+=(const Eigen::Vector2f& other) {
            _pos += other;
            return *this;
        }

        Pose operator-=(const Pose& other) {
            _pos -= other._pos;
            _theta -= other._theta;
            return *this;
        }

        Pose operator*=(float scalar) {
            _pos *= scalar;
            _theta *= scalar;
            return *this;
        }

        Pose operator/=(float scalar) {
            _pos /= scalar;
            _theta /= scalar;
            return *this;
        }

        bool operator==(const Pose& other) const {
            return _pos.x() == other._pos.x() && _pos.y() == other._pos.y() && _theta == other._theta;
        }

        bool operator!=(const Pose& other) const {
            return !(*this == other);
        }

        friend std::ostream& operator<<(std::ostream& os, const Pose& pose) {
            os << std::fixed << std::setprecision(4);
            os << "Pose(" << pose._pos.x() << ", " << pose._pos.y() << ", " << pose._theta << ")";
            return os;
        }
};