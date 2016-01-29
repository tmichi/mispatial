/**
 * @file Octree.hpp
 * @author Takashi Michikawa
 */
#pragma once
#ifndef MI_OCTREE_HPP
#define MI_OCTREE_HPP 1

#include <deque>
#include <cmath>
#include <iostream>

namespace mi
{
        typedef unsigned char OctreeNodeType;

        static OctreeNodeType const OCTREE_INVALID = 0xFF;
        static OctreeNodeType const OCTREE_LEAF  = 0x02;
        static OctreeNodeType const OCTREE_EMPTY = 0x01; // LEAF and contains emptyValue : use only serialization
        static OctreeNodeType const OCTREE_INTERMEDIATE = 0x00;

        /**
         * @class Octree Octree.hpp <mi/Octree.hpp>
         * @brief Octree implementation.
         */
        template < typename T >
        class Octree
        {
        private:
                class node;
                Octree ( const Octree& that ) ;
                void operator = ( const Octree& that ) ;
        public:
                explicit Octree ( const int dimension = 1, const T emptyValue = T() ) : _root( NULL ) {
                        this->init( dimension, emptyValue );
                        return;
                }

                ~Octree ( void ) {
                        if ( this->_root != NULL ) delete this->_root;
                        return;
                }

                void init (  const int dimension = 1, const T emptyValue = T() ) {
                        if( this->_root != NULL ) delete this->_root;
                        this->_dimension  = static_cast<int> ( pow ( 2.0, static_cast<int> ( std::log( static_cast<double>( dimension - 1 ) ) / std::log( 2.0 ) + 1 ) ) );
                        this->_emptyValue = emptyValue;
                        this->_root       = new node ( this->getEmptyValue() );
                        return;
                }

                T get ( const int x, const int y, const int z ) const {
                        if ( !this->isValid ( x, y, z ) ) return this->getEmptyValue();
                        return this->getRoot()->get ( x, y, z, this->getDimension() ) ;
                }

                void set ( const int x, const int y, const int z, const T v ) {
                        if ( !this->isValid ( x,y,z ) ) return;
                        this->getRoot()->set ( x, y, z, v , this->getDimension() );
                        return;
                }

                void set ( const int mnx, const int mny, const int mnz, const int mxx, const int mxy, const int mxz, const T v ) {
                        if ( !this->isValid ( mnx, mny, mnz ) ) return;
                        if ( !this->isValid ( mxx, mxy, mxz ) ) return;
                        this->getRoot()->set ( mnx, mny, mnz, mxx, mxy, mxz, v , this->getDimension() );
                        return;
                }

                inline bool isValid ( const int x, const int y, const int z ) const {
                        return  ( 0 <= x && x < this->getDimension() ) && ( 0 <= y && y < this->getDimension() ) && ( 0 <= z && z < this->getDimension() ) ;
                }

                inline bool isEmpty ( const int x, const int y, const int z ) {
                        return ( this->get ( x, y, z ) == this->getEmptyValue() );
                }

                void getBoundingBox ( int& mnx, int& mny, int& mnz, int& mxx, int& mxy, int& mxz, bool optimized = true ) {
                        mnx = mny = mnz = 0;
                        mxx = mxy = mxz = this->getDimension() - 1;
                        if ( !optimized ) return;
                        this->getRoot()->bounding_box ( this->getEmptyValue(), mnx, mny, mnz, mxx, mxy, mxz, this->getDimension() );
                        return;
                }

                inline size_t count ( const T value ) const {
                        return this->getRoot()->count ( value, this->getDimension() );
                }

                void optimize ( const bool opt = true ) {
                        if ( opt ) this->getRoot()->optimize();
                        return;
                }

                inline int getDimension ( void ) const {
                        return this->_dimension;
                }

                inline T getEmptyValue ( void ) const {
                        return this->_emptyValue;
                }

                // This is used only for OctreeExporter
                bool serialize ( std::deque<OctreeNodeType>& code, typename std::deque<T>& value ) {
                        return this->getRoot()->serialize( code, value, this->getEmptyValue() );
                }

                bool loadSerializedData ( std::deque<OctreeNodeType>& code, typename std::deque<T>& value ) {
                        std::deque<OctreeNodeType>::iterator citer = code.begin();
                        std::deque<OctreeNodeType>::iterator cend = code.end();
                        typename std::deque<T>::iterator viter = value.begin();
                        typename std::deque<T>::iterator vend = value.end();

                        if( !this->getRoot()->loadSerializedData( citer, viter, cend, vend, this->getEmptyValue() ) ) return false;
                        return citer == code.end() && viter == value.end();
                }
        private:
                inline node* getRoot( void ) const {
                        return this->_root;
                }

                class node
                {
                private:
                        node ( const node& that );
                        void operator = ( const node& that );
                public:
                        explicit node ( const T value = T() ) : _value( value ), _child( NULL ) {
                                return;
                        }

                        ~node ( void ) {
                                this->remove_children();
                                return;
                        }

                        T get ( const int x, const int y, const int z, const int dimension ) const {
                                if ( this->is_leaf() ) {
                                        return this->get_value();
                                } else {
                                        const int d = dimension/2;
                                        const int idx = this->get_index( x, y, z, d );
                                        return this->child( idx ).get ( x % d, y % d, z % d, d );
                                }
                        }

                        void set ( const int x, const int y, const int z, const T v, const int dimension ) {
                                if ( dimension == 1 ) {
                                        this->set_value( v );
                                } else {
                                        this->create_children();
                                        const int d =  dimension / 2;
                                        const int idx = this->get_index( x, y, z, d );
                                        this->child( idx ).set ( x % d, y % d, z % d, v, d );
                                }
                                return;
                        }

                        void set ( const int mnx, const int mny, const int mnz, const int mxx, const int mxy, const int mxz, const T v, const int dimension ) {
                                const int d = dimension / 2 ;
                                if ( mnx == 0             && mny == 0             && mnz == 0             &&
                                     mxx == dimension - 1 && mxy == dimension - 1 && mxz == dimension - 1 ) {
                                        this->set_value( v );
                                        return;
                                } else {
                                        this->create_children();
                                        for( int z = 0 ; z < 2 ; ++z ) {
                                                for( int y = 0 ; y < 2 ; ++y ) {
                                                        for( int x = 0 ; x < 2 ; ++x ) {
                                                                const int nmnx = d * x;
                                                                const int nmny = d * y;
                                                                const int nmnz = d * z;

                                                                const int nmxx = d * x + d - 1;
                                                                const int nmxy = d * y + d - 1;
                                                                const int nmxz = d * z + d - 1;

                                                                int bminx, bminy, bminz, bmaxx, bmaxy, bmaxz;
                                                                if ( !this->get_overlap ( mnx, mxx, nmnx, nmxx, bminx, bmaxx ) ) continue;
                                                                if ( !this->get_overlap ( mny, mxy, nmny, nmxy, bminy, bmaxy ) ) continue;
                                                                if ( !this->get_overlap ( mnz, mxz, nmnz, nmxz, bminz, bmaxz ) ) continue;

                                                                const int idx = this->get_index( x,y,z,1 );
                                                                this->child( idx ).set ( bminx % d, bminy % d, bminz % d, bmaxx % d, bmaxy % d, bmaxz % d, v, d );
                                                        }
                                                }
                                        }
                                        return;
                                }
                        }

                        bool bounding_box ( const T emptyValue,
                                            int& mnx, int& mny, int& mnz,
                                            int& mxx, int& mxy, int& mxz, const int dimension ) {
                                if ( this->is_leaf() ) {
                                        if ( this->get_value() == emptyValue ) return false;
                                        mnx = mny = mnz = 0;
                                        mxx = mxy = mxz = dimension - 1;
                                        return true;
                                } else {
                                        const int d = dimension /2 ;
                                        mnx = mny = mnz = dimension - 1;
                                        mxx = mxy = mxz = 0;
                                        for( int z = 0 ; z < 2 ; ++z ) {
                                                for( int y = 0 ; y < 2 ; ++y ) {
                                                        for( int x = 0 ; x < 2 ; ++x ) {
                                                                int lnx, lny, lnz, lxx, lxy, lxz;
                                                                // skip when the child node is empty.
                                                                const int idx = this->get_index( x,y,z,1 );
                                                                if ( !this->child( idx ).bounding_box ( emptyValue, lnx, lny, lnz, lxx, lxy, lxz, d ) ) continue;

                                                                const int offx = d * x;
                                                                const int offy = d * y;
                                                                const int offz = d * z;

                                                                mnx = std::min( lnx + offx, mnx );
                                                                mny = std::min( lny + offy, mny );
                                                                mnz = std::min( lnz + offz, mnz );

                                                                mxx = std::max( lxx + offx, mxx );
                                                                mxy = std::max( lxy + offy, mxy );
                                                                mxz = std::max( lxz + offz, mxz );
                                                        }
                                                }
                                        }
                                        return mnx <= mxx && mny <= mxy && mnz <= mxz; // bounding box exists or not.
                                }
                        }

                        size_t count ( const T value, const int dimension ) const {
                                size_t result = 0;
                                if ( this->is_leaf() ) {
                                        if ( this->_value == value ) result =  dimension * dimension * dimension;
                                } else {
                                        for ( int i = 0 ; i < NUM_CHILDREN ; ++i ) {
                                                result += this->child( i ).count ( value, dimension / 2 );
                                        }
                                }
                                return result;
                        }

                        bool optimize ( void ) {
                                if ( !this->is_leaf() ) {
                                        for ( int i = 0 ; i < NUM_CHILDREN ; ++i ) {
                                                if ( !this->child( i ).optimize() ) return false; // child node cannot be optimized
                                                if (  this->child( 0 ).get_value() != this->child( i ).get_value() ) return false; // value of child nodes are different
                                        }
                                        this->set_value( this->child( 0 ).get_value() );
                                        this->remove_children();
                                }
                                return true;
                        }

                        bool serialize ( std::deque<OctreeNodeType>& code, std::deque<T>& value, const T emptyValue ) {
                                if ( this->is_leaf() ) {
                                        if( this->get_value() == emptyValue ) {
                                                code.push_back( OCTREE_EMPTY );
                                        } else {
                                                code.push_back ( OCTREE_LEAF );
                                                value.push_back( this->get_value() );
                                        }
                                } else {
                                        code.push_back( OCTREE_INTERMEDIATE );
                                        for ( int i = 0 ; i < NUM_CHILDREN ; ++i ) {
                                                if ( !this->child( i ).serialize ( code, value, emptyValue ) ) return false;
                                        }
                                }
                                return true;
                        }


                        bool loadSerializedData ( std::deque<OctreeNodeType>::iterator& citer,  typename std::deque<T>::iterator& viter ,
                                                  std::deque<OctreeNodeType>::iterator& cend,   typename std::deque<T>::iterator& vend ,
                                                  const T emptyValue ) {
                                if( citer == cend ) {
                                        std::cerr<<"iterator reaches at the end."<<std::endl;
                                        return false;
                                }
                                const OctreeNodeType type = *citer;
                                ++citer;
                                if ( type == OCTREE_EMPTY ) {
                                        this->set_value( emptyValue );
                                } else if ( type == OCTREE_LEAF ) {
                                        if( viter == vend ) {
                                                std::cerr<<"iterator reaches at the end."<<std::endl;
                                                return false;
                                        }
                                        this->set_value( *viter );
                                        ++viter;
                                } else if ( type == OCTREE_INTERMEDIATE ) {
                                        this->create_children();
                                        for ( int i = 0 ; i < NUM_CHILDREN ; ++i ) {
                                                if ( !this->child( i ).loadSerializedData ( citer, viter, cend, vend, emptyValue ) ) {
                                                        std::cerr<<"failed to load child node data."<<std::endl;
                                                        return false;
                                                }
                                        }
                                } else {
                                        std::cerr<<"invalid node was found."<<std::endl;
                                        return false;
                                }
                                return true;
                        }
                private:
                        inline T get_value ( void ) const {
                                return this->_value;
                        }

                        inline void set_value ( const T value ) {
                                this->_value = value;
                        }

                        inline int get_index ( const int x, const int y, const int z, const int sep ) const {
                                int idx = 0;
                                if( sep <= x ) idx += 1;
                                if( sep <= y ) idx += 2;
                                if( sep <= z ) idx += 4;
                                return idx;
                        }

                        inline bool get_overlap ( int mn0, int mx0, int mn1, int mx1, int& mn2, int& mx2 ) {
                                mn2 = mn0 > mn1 ? mn0 : mn1; //
                                mx2 = mx0 < mx1 ? mx0 : mx1;
                                return ( mn2 <= mx2 ) ;
                        }

                        inline node& child( const int idx ) const {
                                return this->_child[idx];
                        }

                        inline bool is_leaf( void ) const {
                                return ( this->_child == NULL ); // the node is leaf when the child is null
                        }

                        inline void create_children ( void ) {
                                if ( this->is_leaf() ) {
                                        this->_child = new node [NUM_CHILDREN];
                                        for( int i = 0 ; i < NUM_CHILDREN ; ++i ) this->child( i ).set_value( this->get_value() );
                                }
                                return;
                        }

                        inline void remove_children ( void ) {
                                if ( !this->is_leaf() ) delete[] this->_child;
                                this->_child = NULL;
                                return;
                        }
                private:
                        static const int NUM_CHILDREN = 8;
                        T	_value;
                        node*	_child;
                };// node
        private:
                node*	_root;
                T	_emptyValue;
                int	_dimension;
        };// Octree
};
#endif// MI_OCTREE_HPP
