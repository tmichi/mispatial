/**
 * @file OctreeExporter.hpp
 * @author Takashi Michikawa <michikawa@acm.org>
 */
#pragma once
#ifndef MI_OCTREEEXPORTER_HPP
#define MI_OCTREEEXPORTER_HPP 1
#include <mi/Exporter.hpp>
#include "Octree.hpp"

namespace mi
{
        /**
         * @class OctreeExporter OctreeExporter.hpp "OctreeExporter.hpp"
         *
         */
        template <typename T>
        class OctreeExporter : public Exporter
        {
        public:
                /**
                 * @brief Constructor.
                 * @param [in] data Octree data.
                 */
                explicit OctreeExporter ( Octree<T>& data ) : Exporter ( true ), _data ( data ) {
                        return;
                }
        protected:
                Octree<T>& getOctree( void ) {
                        return this->_data;
                }
        private:
                bool writeHeader ( std::ofstream& fout ) {
                        const int dimension = this->getOctree().getDimension();
                        const T emptyValue  = this->getOctree().getEmptyValue();
                        if ( !fout.write( ( char* )( &dimension ) , sizeof( int ) ) ) return false;
                        if ( !fout.write( ( char* )( &emptyValue ) , sizeof( T )  ) ) return false;
                        return fout.good();
                }

                bool writeBody ( std::ofstream& fout ) {
                        std::deque<OctreeNodeType> code;
                        std::deque<T> value;

                        this->getOctree().optimize();
                        this->getOctree().serialize( code, value );

                        typename std::deque<OctreeNodeType>::iterator citer = code.begin();
                        typename std::deque<T>::iterator viter = value.begin();
                        while( 1 ) {
                                if ( citer == code.end() ) break;
                                //@bug ! if ( viter == value.end() ) break;
                                OctreeNodeType c = *citer;
                                ++citer;
                                if( ! fout.write ( ( char* )( &c ), sizeof( OctreeNodeType ) ) )return false;
                                if( c == OCTREE_LEAF ) {
                                        if ( viter == value.end() ) break;
                                        T v = *viter;
                                        ++viter;
                                        if( !fout.write ( ( char* )&v, sizeof( T ) ) )return false;
                                }
                        }
                        // two iterators must reach at the end.
                        if ( citer != code.end() || viter != value.end() ) {
                                std::cerr<<"format error."<<std::endl;
                                return false;
                        }
                        return fout.good();
                }
                virtual std::string toString ( void ) const {
                        return std::string ( "oct" );
                }
        private:
                Octree<T>& _data;
        };
};
#endif// MI_OCTREEEXPORTER_HPP

