#include "Utils.h"


std::vector<Point3D> rdp( const std::vector<Point3D>& points, double epsilon )
{
    std::vector<Point3D> out;
    if( points.size() < 2 ) {
        out = points;
        return out;
    }

    double dx = points.back().x - points.front().x;
    double dy = points.back().y - points.front().y;
    double mag = std::pow( std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5 );
    if( mag > 0.0 ) {
        dx /= mag;
        dy /= mag;
    }

    double d_max = 0.0;
    size_t d_max_idx = 0;
    for( int idx = 0; idx < points.size()-1; idx++ ) {
        Point3D pt = points.at(idx);
        double pvx = pt.x - points.front().x;
        double pvy = pt.y - points.front().y;
        double pv_dot = dx * pvx + dy * pvy;
        double dsx = pv_dot * dx;
        double dsy = pv_dot * dy;
        double ax = pvx - dsx;
        double ay = pvy - dsy;

        double d = std::pow( std::pow(ax, 2.0) + std::pow(ay, 2.0), 0.5 );
        if( d > d_max ) {
            d_max = d;
            d_max_idx = idx;
        }
    }

    if( d_max > epsilon ) {
        std::vector<Point3D> first_line( points.begin(), points.begin()+d_max_idx+1 );
        std::vector<Point3D> last_line( points.begin()+d_max_idx, points.end() );
        std::vector<Point3D> results_1 = rdp(first_line, epsilon);
        std::vector<Point3D> results_2 = rdp(last_line, epsilon);
        out.assign(results_1.begin(), results_1.end()-1);
        out.insert(out.end(), results_2.begin(), results_2.end());
    } else {
        out.push_back(points.front());
        out.push_back(points.back());
    }

    return out;
}