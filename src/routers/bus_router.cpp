// $Id: iq_router.cpp 5263 2012-09-20 23:40:33Z dub $

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

#include "bus_router.hpp"
#include "booksim.hpp"

#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cassert>
#include <limits>

#include "globals.hpp"
#include "random_utils.hpp"
#include "vc.hpp"
#include "routefunc.hpp"
#include "outputset.hpp"
#include "buffer.hpp"
#include "buffer_state.hpp"
#include "roundrobin_arb.hpp"
#include "allocator.hpp"
#include "switch_monitor.hpp"
#include "buffer_monitor.hpp"

BusRouter::BusRouter( Configuration const & config, Module *parent, 
		    string const & name, int id, int inputs, int outputs, int number)
: Router( config, parent, name, id, inputs, outputs ), _active(false)
{
  _vcs         = config.GetInt( "num_vcs" );
  _node_per_bus = config.GetInt( "node_per_bus");
  _number      = number;

  // Alloc VC's
  _buf.resize(_inputs);
  for ( int i = 0; i < _inputs; ++i ) {
    ostringstream module_name;
    module_name << "buf_" << i;
    _buf[i] = new Buffer(config, _outputs, this, module_name.str( ) );
    module_name.str("");
  }

  // Output queues
  _output_buffer_size = config.GetInt("output_buffer_size");
  _output_buffer.resize(_outputs); 
  _credit_buffer.resize(_inputs); 

  _bufferMonitor = new BufferMonitor(inputs, _classes);
  _switchMonitor = new SwitchMonitor(inputs, outputs, _classes);

#ifdef TRACK_FLOWS
  for(int c = 0; c < _classes; ++c) {
    _stored_flits[c].resize(_inputs, 0);
    _active_packets[c].resize(_inputs, 0);
  }
  _outstanding_classes.resize(_outputs, vector<queue<int> >(_vcs));
#endif
}

BusRouter::~BusRouter( )
{

  if(gPrintActivity) {
    cout << Name() << ".bufferMonitor:" << endl ; 
    cout << *_bufferMonitor << endl ;
    
    cout << Name() << ".switchMonitor:" << endl ; 
    cout << "Inputs=" << _inputs ;
    cout << "Outputs=" << _outputs ;
    cout << *_switchMonitor << endl ;
  }

  for(int i = 0; i < _inputs; ++i)
    delete _buf[i];

  delete _bufferMonitor;
  delete _switchMonitor;
}
  
void BusRouter::AddOutputChannel(FlitChannel * channel, CreditChannel * backchannel)
{
  int min_latency = _crossbar_delay + channel->GetLatency() + backchannel->GetLatency();
  //_next_buf[_output_channels.size()]->SetMinLatency(min_latency);
  Router::AddOutputChannel(channel, backchannel);
}

void BusRouter::ReadInputs( )
{
  bool have_flits = _ReceiveFlits( );
  bool have_credits = _ReceiveCredits( );
  _active = _active || have_flits || have_credits;
}

void BusRouter::_InternalStep( )
{
  if(!_active) {
    return;
  }

  //_InputQueuing( );
  for(map<int, Flit *>::const_iterator iter = _in_queue_flits.begin();
      iter != _in_queue_flits.end();
      ++iter) {

    int const input = iter->first;
    assert((input >= 0) && (input < _inputs));

    Flit * const f = iter->second;
    assert(f);

    int const vc = f->vc;
    assert((vc >= 0) && (vc < _vcs));

    //Buffer * const cur_buf = _buf[input];
    
    //cur_buf->AddFlit(vc, f);
    //string const param_str = GetStr(name);
    //istringstream input_istring(param_str.str());
   // istringstream input_istring(_name.str());
   // string dump;
   // int x, y, z;
   // input_istring >> dump >> x >> dump >> y >> dump >> z;
     
    //2-direction ring bus, [0],[1] is the ring buses, [2] is the inject/eject ports 
    //if(input == 0){
    //  _output_buffer[2].push(f);
    //  if(_number < (_node_per_bus - 1)){
    //      _output_buffer[0].push(f);
    //  }
    //} else {
    //  _output_buffer[1].push(f);
    //}

    //1 ring bus, [0] is the ring bus, [1] is the inject/eject ports
    if(input == 0){
      _output_buffer[1].push(f);
    } else {
      _output_buffer[0].push(f);
    }
  } 

  _bufferMonitor->cycle( );
  _switchMonitor->cycle( );
}

void BusRouter::WriteOutputs( )
{
  _SendFlits( );
  _SendCredits( );
}


//------------------------------------------------------------------------------
// read inputs
//------------------------------------------------------------------------------

bool BusRouter::_ReceiveFlits( )
{
  bool activity = false;
  for(int input = 0; input < _inputs; ++input) { 
    Flit * const f = _input_channels[input]->Receive();
    if(f) {

#ifdef TRACK_FLOWS
      ++_received_flits[f->cl][input];
#endif

      if(f->watch) {
	*gWatchOut << GetSimTime() << " | " << FullName() << " | "
		   << "Buss router " 
		   << "Received flit " << f->id
		   << " from channel at input " << input
		   << "." << endl;
      }
      _in_queue_flits.insert(make_pair(input, f));
      activity = true;
    }
  }
  return activity;
}

bool BusRouter::_ReceiveCredits( )
{
  bool activity = false;
  for(int output = 0; output < _outputs; ++output) {  
    Credit * const c = _output_credits[output]->Receive();
    if(c) {
      _proc_credits.push_back(make_pair(GetSimTime() + _credit_delay, 
					make_pair(c, output)));
      activity = true;
    }
  }
  return activity;
}


//------------------------------------------------------------------------------
// write outputs
//------------------------------------------------------------------------------

void BusRouter::_SendFlits( )
{
  for ( int output = 0; output < _outputs; ++output ) {
    if ( !_output_buffer[output].empty( ) ) {
      Flit * const f = _output_buffer[output].front( );
      assert(f);
      _output_buffer[output].pop( );

#ifdef TRACK_FLOWS
      ++_sent_flits[f->cl][output];
#endif

      if(f->watch)
	*gWatchOut << GetSimTime() << " | " << FullName() << " | "
		    << "Sending flit " << f->id
		    << " to channel at output " << output
		    << "." << endl;
      if(gTrace) {
	cout << "Outport " << output << endl << "Stop Mark" << endl;
      }
      _output_channels[output]->Send( f );
    }
  }
}

void BusRouter::_SendCredits( )
{
  for ( int input = 0; input < _inputs; ++input ) {
    if ( !_credit_buffer[input].empty( ) ) {
      Credit * const c = _credit_buffer[input].front( );
      assert(c);
      _credit_buffer[input].pop( );
      _input_credits[input]->Send( c );
    }
  }
}


////------------------------------------------------------------------------------
//// misc.
////------------------------------------------------------------------------------
//
//void BusRouter::Display( ostream & os ) const
//{
//  for ( int input = 0; input < _inputs; ++input ) {
//    _buf[input]->Display( os );
//  }
//}
//
  int BusRouter::GetUsedCredit(int o) const
  {
    assert((o >= 0) && (o < _outputs));
    BufferState const * const dest_buf = _next_buf[o];
    return dest_buf->Occupancy();
  }
  
  int BusRouter::GetBufferOccupancy(int i) const {
    assert(i >= 0 && i < _inputs);
    return _buf[i]->GetOccupancy();
  }
  
//#ifdef TRACK_BUFFERS
//int BusRouter::GetUsedCreditForClass(int output, int cl) const
//{
//  assert((output >= 0) && (output < _outputs));
//  BufferState const * const dest_buf = _next_buf[output];
//  return dest_buf->OccupancyForClass(cl);
//}
//
//int BusRouter::GetBufferOccupancyForClass(int input, int cl) const
//{
//  assert((input >= 0) && (input < _inputs));
//  return _buf[input]->GetOccupancyForClass(cl);
//}
//#endif
//
  vector<int> BusRouter::UsedCredits() const
  {
    vector<int> result(_outputs*_vcs);
    for(int o = 0; o < _outputs; ++o) {
      for(int v = 0; v < _vcs; ++v) {
        result[o*_vcs+v] = _next_buf[o]->OccupancyFor(v);
      }
    }
    return result;
  }
  
  vector<int> BusRouter::FreeCredits() const
  {
    vector<int> result(_outputs*_vcs);
    for(int o = 0; o < _outputs; ++o) {
      for(int v = 0; v < _vcs; ++v) {
        result[o*_vcs+v] = _next_buf[o]->AvailableFor(v);
      }
    }
    return result;
  }
  
  vector<int> BusRouter::MaxCredits() const
  {
    vector<int> result(_outputs*_vcs);
    for(int o = 0; o < _outputs; ++o) {
      for(int v = 0; v < _vcs; ++v) {
        result[o*_vcs+v] = _next_buf[o]->LimitFor(v);
      }
    }
    return result;
  }
//
//void BusRouter::_UpdateNOQ(int input, int vc, Flit const * f) {
//  assert(!_routing_delay);
//  assert(f);
//  assert(f->vc == vc);
//  assert(f->head);
//  set<OutputSet::sSetElement> sl = f->la_route_set.GetSet();
//  assert(sl.size() == 1);
//  int out_port = sl.begin()->output_port;
//  const FlitChannel * channel = _output_channels[out_port];
//  const Router * router = channel->GetSink();
//  if(router) {
//    int in_channel = channel->GetSinkPort();
//    OutputSet nos;
//    _rf(router, f, in_channel, &nos, false);
//    sl = nos.GetSet();
//    assert(sl.size() == 1);
//    OutputSet::sSetElement const & se = *sl.begin();
//    int next_output_port = se.output_port;
//    assert(next_output_port >= 0);
//    assert(_noq_next_output_port[input][vc] < 0);
//    _noq_next_output_port[input][vc] = next_output_port;
//    int next_vc_count = (se.vc_end - se.vc_start + 1) / router->NumOutputs();
//    int next_vc_start = se.vc_start + next_output_port * next_vc_count;
//    assert(next_vc_start >= 0 && next_vc_start < _vcs);
//    assert(_noq_next_vc_start[input][vc] < 0);
//    _noq_next_vc_start[input][vc] = next_vc_start;
//    int next_vc_end = se.vc_start + (next_output_port + 1) * next_vc_count - 1;
//    assert(next_vc_end >= 0 && next_vc_end < _vcs);
//    assert(_noq_next_vc_end[input][vc] < 0);
//    _noq_next_vc_end[input][vc] = next_vc_end;
//    assert(next_vc_start <= next_vc_end);
//    if(f->watch) {
//      *gWatchOut << GetSimTime() << " | " << FullName() << " | "
//		 << "Computing lookahead routing information for flit " << f->id
//		 << " (NOQ)." << endl;
//    }
//  }
//}
