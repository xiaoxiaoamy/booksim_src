// $Id: kncube.cpp 5516 2013-10-06 02:14:48Z dub $

/*
 Copyright (c) 2007-2012, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*busmesh.cpp
 *
 bus_mesh_hybrid*
 *
 */

#include "booksim.hpp"
#include <vector>
#include <sstream>
#include "busmesh.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"


BusMesh::BusMesh( const Configuration &config, const string & name, bool mesh ) :
Network( config, name )
{
  _mesh = mesh;

  _ComputeSize( config );
  _Alloc( );
  _BuildNet( config );
}

void BusMesh::_ComputeSize( const Configuration &config )
{
  _k = config.GetInt( "k" );
  _n = config.GetInt( "n" );
  _node_per_bus = config.GetInt( "node_per_bus" );

  gK = _k; gN = _n;
  //Router Number: size = routers for bus (_nodes) + routers in mesh;
  _size     = powi( _k, _n )*(_node_per_bus + 1);   //size of mesh
  // mesh + bus
  _channels = 2*_n*_size + 2*_size; 

  _nodes = _node_per_bus * powi( _k, _n );

  _router_in_mesh = powi( _k, _n );
  _router_in_bus = _nodes;
}

void BusMesh::RegisterRoutingFunctions() {

}
void BusMesh::_BuildNet( const Configuration &config )
{
  int left_node;
  int right_node;

  int right_input;
  int left_input;

  int right_output;
  int left_output;

  ostringstream router_name;

  //latency type, noc or conventional network
  bool use_noc_latency;
  use_noc_latency = (config.GetInt("use_noc_latency")==1);

  //build mesh 
  for ( int node = 0; node < _router_in_mesh; ++node ) {

    router_name << "router";
    
    if ( _k > 1 ) {
      for ( int dim_offset = _router_in_mesh / _k; dim_offset >= 1; dim_offset /= _k ) {
        router_name << "_" << ( node / dim_offset ) % _k;
      }
    }

    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
        				node, 2*_n + 1, 2*_n + 1 );
    _timed_modules.push_back(_routers[node]);

    router_name.str("");

    for ( int dim = 0; dim < _n; ++dim ) {

      //find the neighbor 
      left_node  = _LeftNode( node, dim );
      right_node = _RightNode( node, dim );

      //
      // Current (N)ode
      // (L)eft node
      // (R)ight node
      //
      //   L--->N<---R
      //   L<---N--->R
      //

      // torus channel is longer due to wrap around
      int latency = _mesh ? 1 : 2 ;

      //get the input channel number
      right_input = _LeftChannel( right_node, dim );
      left_input  = _RightChannel( left_node, dim );

      //add the input channel
      _routers[node]->AddInputChannel( _chan[right_input], _chan_cred[right_input] );
      _routers[node]->AddInputChannel( _chan[left_input], _chan_cred[left_input] );
//      cout<<"Debug input channel: Add input channel for mech node "<<node<<endl;

      //set input channel latency
      if(use_noc_latency){
        _chan[right_input]->SetLatency( latency );
        _chan[left_input]->SetLatency( latency );
        _chan_cred[right_input]->SetLatency( latency );
        _chan_cred[left_input]->SetLatency( latency );
      } else {
        _chan[left_input]->SetLatency( 1 );
        _chan_cred[right_input]->SetLatency( 1 );
        _chan_cred[left_input]->SetLatency( 1 );
        _chan[right_input]->SetLatency( 1 );
      }
      //get the output channel number
      right_output = _RightChannel( node, dim );
      left_output  = _LeftChannel( node, dim );
      
      //add the output channel
      _routers[node]->AddOutputChannel( _chan[right_output], _chan_cred[right_output] );
      _routers[node]->AddOutputChannel( _chan[left_output], _chan_cred[left_output] );

      //set output channel latency
      if(use_noc_latency){
        _chan[right_output]->SetLatency( latency );
        _chan[left_output]->SetLatency( latency );
        _chan_cred[right_output]->SetLatency( latency );
        _chan_cred[left_output]->SetLatency( latency );
      } else {
        _chan[right_output]->SetLatency( 1 );
        _chan[left_output]->SetLatency( 1 );
        _chan_cred[right_output]->SetLatency( 1 );
        _chan_cred[left_output]->SetLatency( 1 );

      }
    }
    
  }

  //build buses 
  for ( int node = 0; node < _router_in_bus; ++node ) {

    //Bus Routers' name: router_x_y_(0,1,...,(_node_per_bus-1))
    router_name << "router";
    
    if ( _k > 1 ) {
      for ( int dim_offset = _router_in_mesh / _k; dim_offset >= 1; dim_offset /= _k ) {
        router_name << "_" << ( (node/_node_per_bus) / dim_offset ) % _k;
      }
    }
    router_name << "_" << node%_node_per_bus ;

    //2 ports in both input and output
    _busrouters[node] = Router::NewBusRouter( config, this, router_name.str( ), 
        				node, 3, 3 , node%_node_per_bus);
    _timed_modules.push_back(_busrouters[node]);

    router_name.str("");

    //find the neighbor 
    left_node  = _BusLeftNode( node );
    right_node = _BusRightNode( node );

    //
    // Current (N)ode
    // (L)eft node
    // (R)ight node
    //
    //   L--->N<---R
    //   L<---N--->R
    //

    //get the input channel number
    right_input = _BusLeftChannel( right_node );
    left_input  = _BusRightChannel( left_node );

    //add the input channel
    _busrouters[node]->AddInputChannel( _chan[right_input], _chan_cred[right_input] );
    _busrouters[node]->AddInputChannel( _chan[left_input], _chan_cred[left_input] );
//    cout<<"Debug input channel: Add input channel for bus node "<<node<<endl;

    //set input channel latency
    _chan[left_input]->SetLatency( 1 );
    _chan_cred[right_input]->SetLatency( 1 );
    _chan_cred[left_input]->SetLatency( 1 );
    _chan[right_input]->SetLatency( 1 );

    //get the output channel number
    right_output = _BusRightChannel( node );
    left_output  = _BusLeftChannel( node );
    
    //add the output channel
    _busrouters[node]->AddOutputChannel( _chan[right_output], _chan_cred[right_output] );
    _busrouters[node]->AddOutputChannel( _chan[left_output], _chan_cred[left_output] );

    //set output channel latency
    _chan[right_output]->SetLatency( 1 );
    _chan[left_output]->SetLatency( 1 );
    _chan_cred[right_output]->SetLatency( 1 );
    _chan_cred[left_output]->SetLatency( 1 );

    
    //injection and ejection channel, always 1 latency
    _busrouters[node]->AddInputChannel( _inject[node], _inject_cred[node] );
    _busrouters[node]->AddOutputChannel( _eject[node], _eject_cred[node] );
    _inject[node]->SetLatency( 1 );
    _eject[node]->SetLatency( 1 );
  } 
}

int BusMesh::_LeftChannel( int node, int dim )
{
  // The base channel for a node is 2*_n*node
  int base = 2*_n*node+2*_size;
  // The offset for a left channel is 2*dim + 1
  int off  = 2*dim + 1;

  return ( base + off );
}

int BusMesh::_RightChannel( int node, int dim )
{
  // The base channel for a node is 2*_n*node
  int base = 2*_n*node+2*_size;
  // The offset for a right channel is 2*dim 
  int off  = 2*dim;
  return ( base + off );
}

int BusMesh::_LeftNode( int node, int dim )
{
  int k_to_dim = powi( _k, dim );
  int loc_in_dim = ( node / k_to_dim ) % _k;
  int left_node;
  // if at the left edge of the dimension, wraparound
  if ( loc_in_dim == 0 ) {
    left_node = node + (_k-1)*k_to_dim;
  } else {
    left_node = node - k_to_dim;
  }

  return left_node;
}

int BusMesh::_RightNode( int node, int dim )
{
  int k_to_dim = powi( _k, dim );
  int loc_in_dim = ( node / k_to_dim ) % _k;
  int right_node;
  // if at the right edge of the dimension, wraparound
  if ( loc_in_dim == ( _k-1 ) ) {
    right_node = node - (_k-1)*k_to_dim;
  } else {
    right_node = node + k_to_dim;
  }

  return right_node;
}

int BusMesh::_BusLeftNode( int node )
{
  //int loc_in_dim =  node % _node_per_bus;
  int left_node;
  // if at the left edge of the dimension, wraparound
  if ( node == 0 ) {
    left_node = _nodes - 1;
  } else {
    left_node = node - 1;
  }

  return left_node;
}

int BusMesh::_BusRightNode( int node )
{
//  int loc_in_dim =  node % _node_per_bus;
  int right_node;
  right_node = node + 1;

  return right_node;
}

int BusMesh::_BusLeftChannel( int node )
{
  // The base channel for a node is 2*node
  int base = 2*node;
  // The offset for a left channel is 2*dim + 1
  int off  = 1;

  return ( base + off );
}

int BusMesh::_BusRightChannel( int node )
{
  // The base channel for a node is 2*_n*node
  int base = 2*node;
  // The offset for a right channel is 2*dim
  int off  = 0;
  return ( base + off );
}


int BusMesh::GetN( ) const
{
  return _n;
}

int BusMesh::GetK( ) const
{
  return _k;
}


double BusMesh::Capacity( ) const
{
  return (double)_k / ( _mesh ? 8.0 : 4.0 );
}
