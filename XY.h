// Copyright 2016 Carrie Rebhuhn
#ifndef MATH_XY_H_
#define MATH_XY_H_
#include <utility>

namespace easymath {
//! A class for locations. Contains many overloads for vector arithmetic.
class XY : public std::pair<double, double> {
 public:
    XY(const double &x, const double &y) :
        pair<double, double>(x, y), x(x), y(y) {}

    XY() {};

    double x, y;

    //! Vector subtraction
    XY operator-(const XY &rhs) const {
        return XY(x - rhs.x, y - rhs.y);
    }

    //! Orders first by x values, then by y values
    bool operator<(const XY &rhs) const {
        if (x != rhs.x) return x < rhs.x;
        return y < rhs.y;
    }

    //! Checks equality of both elements
    bool operator==(const XY &rhs) const {
        return x == rhs.x && y == rhs.y &&
            first == rhs.first && second == rhs.second;
    }

    //! Scalar multiplication
    XY operator*(double rhs) const {
        return XY(x*rhs, y*rhs);
    }

    //! Dot multiplication
    double operator*(const XY &V) const {
        return x*V.x + y*V.y;
    }

    //! Vector addition
    XY operator+(const XY &rhs) const {
        return XY(x + rhs.x, y + rhs.y);
    }

    //! Assignment operator
    XY& operator=(const XY &rhs) {
        x = rhs.x;
        y = rhs.y;
        first = rhs.first;
        second = rhs.second;
        return *this;
    }
};
}  // namespace easymath
#endif  // MATH_XY_H_
