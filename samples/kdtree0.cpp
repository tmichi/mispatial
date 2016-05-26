#include <mi/Kdtree.hpp>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
int main ( int argc, char** argv )
{

        std::vector<Eigen::Vector3d> p;
        for( int i = 0 ; i < 10000 ; i++ ) {
                double x = ( rand() % 2001 - 1000 ) * 0.001;
                double y = ( rand() % 2001 - 1000 ) * 0.001;
                double z = ( rand() % 2001 - 1000 ) * 0.001;
                p.push_back( Eigen::Vector3d( x,y,z ) );
        }

        mi::Kdtree<Eigen::Vector3d> kdtree( p ) ;

        Eigen::Vector3d v( 0.1, 0.1, 0.1 ) ;
        std::list<Eigen::Vector3d> result;

        kdtree.find( v, 0.1, result, true );
        for( std::list<Eigen::Vector3d>::iterator iter = result.begin(); iter != result.end() ; ++iter ) {
                Eigen::Vector3d& v0 = *iter;
                Eigen::Vector3d d = *iter - v;
                double sqrtDist = d.x()*d.x()+d.y()*d.y()+d.z()*d.z();
                std::cerr<<v0.x()<<" "<<v0.y()<<" "<<v0.z()<<" "<<sqrtDist<<std::endl;
        }
        return 0;
}
