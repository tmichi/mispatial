/**
 * @file OctreeImporter.hpp
 * @author Takashi Michikawa <michikawa@acm.org>
 */
#pragma once
#ifndef MI_OCTREE_IMPORTER_HPP
#define MI_OCTREE_IMPORTER_HPP 1

#include <string>
#include "Octree.hpp"
#include <mi/Importer.hpp>
namespace mi
{
        /**
         * @class OctreeImporter
         * @brief Octree importer.
         */
        template <typename T>
        class OctreeImporter : public Importer
        {
        public:
                /**
                 * @brief Constructor.
                 * @param [in] data Octree data.
                 */
                explicit OctreeImporter ( Octree<T>& data ) : Importer ( true ), _data ( data ) {
                        return;
                }
        protected:
                Octree<T>& getOctree( void ) {
                        return this->_data;
                }
        private:
                virtual bool readHeader ( std::ifstream& fin ) {
                        int dimension;
                        T emptyValue;
                        if( !fin.read ( ( char* ) & ( dimension ), sizeof( int ) ) ) return false;
                        if( !fin.read ( ( char* ) & ( emptyValue ), sizeof( T ) ) ) return false;
                        this->getOctree().init( dimension, emptyValue );
                        return true;
                }

                virtual bool readBody ( std::ifstream& fin ) {
                        typename std::deque< OctreeNodeType > code;
                        std::deque<T> value;
                        while( 1 ) {
                                OctreeNodeType c;
                                T v;
                                if( !fin.read ( ( char* )&c, sizeof( OctreeNodeType ) ) ) break;
                                code.push_back( c );
                                if( c == OCTREE_LEAF ) {
                                        if( !fin.read ( ( char* )&v, sizeof( T ) ) ) break;
                                        value.push_back( v );
                                }
                                if ( fin.eof() ) break;
                        }
                        return this->getOctree().loadSerializedData ( code, value );
                }

                virtual std::string toString ( void ) const {
                        return std::string ( "oct" );
                }
        private:
                Octree<T>& _data;
        };
};
#endif// MI_OCTREE_IMPORTER_HPP
