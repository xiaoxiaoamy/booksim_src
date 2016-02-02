// $Id: kncube.hpp 5188 2012-08-30 00:31:31Z dub $

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


#include "network.hpp"

class BusMesh : public Network {

  bool _mesh;

  int _k;
  int _n;
  int _node_per_bus;
  int _router_in_mesh;
  int _router_in_bus;

  void _ComputeSize( const Configuration &config );
  void _BuildNet( const Configuration &config );

  int _LeftChannel( int node, int dim );
  int _RightChannel( int node, int dim );

  int _LeftNode( int node, int dim );
  int _RightNode( int node, int dim );

  int _BusLeftChannel( int node );
  int _BusRightChannel( int node );

  int _BusLeftNode( int node );
  int _BusRightNode( int node );

public:
  BusMesh( const Configuration &config, const string & name, bool mesh );
  static void RegisterRoutingFunctions();

  int GetN( ) const;
  int GetK( ) const;

  double Capacity( ) const;

};

