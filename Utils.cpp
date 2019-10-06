#include "Utils.h"


std::vector<std::pair<double, double>> rdp( const std::vector<std::pair<double, double>>& points, double epsilon )
{
    std::vector<std::pair<double, double>> out;
    if( points.size() < 2 ) {
        out = points;
        return out;
    }

    double dx = points.back().first - points.front().first;
    double dy = points.back().second - points.front().second;
    double mag = std::pow( std::pow(dx, 2.0) + std::pow(dy, 2.0), 0.5 );
    if( mag > 0.0 ) {
        dx /= mag;
        dy /= mag;
    }

    double d_max = 0.0;
    size_t d_max_idx = 0;
    for( int idx = 0; idx < points.size()-1; idx++ ) {
        std::pair<double, double> pt = points.at(idx);
        double pvx = pt.first - points.front().first;
        double pvy = pt.second - points.front().second;
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
        std::vector<std::pair<double, double>> first_line( points.begin(), points.begin()+d_max_idx+1 );
        std::vector<std::pair<double, double>> last_line( points.begin()+d_max_idx, points.end() );
        std::vector<std::pair<double, double>> results_1 = rdp(first_line, epsilon);
        std::vector<std::pair<double, double>> results_2 = rdp(last_line, epsilon);
        out.assign(results_1.begin(), results_1.end()-1);
        out.insert(out.end(), results_2.begin(), results_2.end());
    } else {
        out.push_back(points.front());
        out.push_back(points.back());
    }

    return out;
}