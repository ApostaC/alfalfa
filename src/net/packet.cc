/* -*-mode:c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/* Copyright 2013-2018 the Alfalfa authors
                       and the Massachusetts Institute of Technology

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

      1. Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.

      2. Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <string>
#include <algorithm>
#include <vector>

#include "packet.hh"

using namespace std;

string Packet::put_header_field( const uint16_t n )
{
  const uint16_t network_order = htole16( n );
  return string( reinterpret_cast<const char *>( &network_order ),
                 sizeof( network_order ) );
}

string Packet::put_header_field( const uint32_t n )
{
  const uint32_t network_order = htole32( n );
  return string( reinterpret_cast<const char *>( &network_order ),
                 sizeof( network_order ) );
}

string Packet::put_header_field( const uint64_t n )
{
  const uint32_t network_order = htole64( n );
  return string( reinterpret_cast<const char *>( &network_order ),
                 sizeof( network_order ) );
}

Packet::Packet( const vector<uint8_t> & whole_frame,
                const uint16_t connection_id,
                const uint32_t source_state,
                const uint32_t target_state,
                const uint32_t frame_no,
                const uint16_t fragment_no,
                const uint16_t time_since_last,
                size_t & next_fragment_start)
  : valid_( true ),
    connection_id_( connection_id ),
    source_state_( source_state ),
    target_state_( target_state ),
    frame_no_( frame_no ),
    fragment_no_( fragment_no ),
    fragments_in_this_frame_( 0 ), /* temp value */
    time_since_last_( time_since_last ),
    send_timestamp_ms_(0), 
    fec_rate_(0),
    red_fragments_in_this_frame_(0),
    control_signal_(0),
    payload_()
{
  assert( not whole_frame.empty() );

  size_t first_byte = MAXIMUM_PAYLOAD * fragment_no;
  assert( first_byte < whole_frame.size() );

  size_t length = min( whole_frame.size() - first_byte, MAXIMUM_PAYLOAD );
  assert( first_byte + length <= whole_frame.size() );

  payload_ = string( reinterpret_cast<const char*>( &whole_frame.at( first_byte ) ), length );

  next_fragment_start = first_byte + length;
}

/* construct incoming Packet */
Packet::Packet( const Chunk & str )
  : valid_( true ),
    connection_id_( str( 0, 2 ).le16() ),
    source_state_( str( 2, 4 ).le32() ),
    target_state_( str( 6, 4 ).le32() ),
    frame_no_( str( 10, 4 ).le32() ),
    fragment_no_( str( 14, 2 ).le16() ),
    fragments_in_this_frame_( str( 16, 2 ).le16() ),
    time_since_last_( str( 18, 4 ).le32() ),
    send_timestamp_ms_( str( 22, 4 ).le32() ),
    fec_rate_( str( 26, 2 ).le16() ),
    red_fragments_in_this_frame_( str( 28, 2 ).le16() ),
    control_signal_( str( 30, 2 ).le16() ),
    svc_layer_no_( str(32, 2).le16() ),
    svc_layer_offset_( str(34, 2).le16() ),
    svc_layer_size_( str(36, 2).le16() ),
    payload_( str( 38 ).to_string() )
{
  if ( fragment_no_ >= fragments_in_this_frame_ ) {
    throw runtime_error( "invalid packet: fragment_no_ >= fragments_in_this_frame" );
  }

  if ( payload_.empty() and control_signal_ == 0) {
    throw runtime_error( "invalid packet: empty payload" );
  }
}

/* construct an empty, invalid packet */
Packet::Packet()
  : valid_( false ),
    connection_id_(),
    source_state_(),
    target_state_(),
    frame_no_(),
    fragment_no_(),
    fragments_in_this_frame_(),
    time_since_last_(),
    send_timestamp_ms_(),
    fec_rate_(),
    red_fragments_in_this_frame_(), 
    control_signal_(), 
    payload_()
{}

/* serialize a Packet */
string Packet::to_string() const
{
  assert( fragments_in_this_frame_ > 0 );

  return put_header_field( connection_id_ )
       + put_header_field( source_state_ )
       + put_header_field( target_state_ )
       + put_header_field( frame_no_ )
       + put_header_field( fragment_no_ )
       + put_header_field( fragments_in_this_frame_ )
       + put_header_field( time_since_last_ )
       + put_header_field( send_timestamp_ms_ )
       + put_header_field( fec_rate_ )
       + put_header_field( red_fragments_in_this_frame_ )
       + put_header_field( control_signal_ )
       + put_header_field( svc_layer_no_ )
       + put_header_field( svc_layer_offset_ )
       + put_header_field( svc_layer_size_ )
       + payload_;
}

void Packet::set_fragments_in_this_frame( const uint16_t x )
{
  fragments_in_this_frame_ = x;
  assert( fragment_no_ < fragments_in_this_frame_ );
}

/* construct outgoing FragmentedFrame */
FragmentedFrame::FragmentedFrame( const uint16_t connection_id,
                                  const uint32_t source_state,
                                  const uint32_t target_state,
                                  const uint32_t frame_no,
                                  const uint32_t time_since_last,
                                  const vector<uint8_t> & whole_frame,
                                  const bool is_key_frame, 
                                  const uint8_t fec_rate)
  : connection_id_( connection_id ),
    source_state_( source_state ),
    target_state_( target_state ),
    frame_no_( frame_no ),
    fragments_in_this_frame_(),
    fragments_(),
    remaining_fragments_( 0 ),
    is_key_frame_ ( is_key_frame ),
    fec_rate_(fec_rate),
    num_fec_fragments_(0)
{
  size_t next_fragment_start = 0;

  for ( uint16_t fragment_no = 0; next_fragment_start < whole_frame.size();
        fragment_no++ ) {
    fragments_.emplace_back( whole_frame, connection_id, source_state_, target_state_,
                             frame_no, fragment_no, 0, next_fragment_start );
  }

  gen_fec_packets(whole_frame);

  fragments_.front().set_time_to_next( time_since_last );

  fragments_in_this_frame_ = fragments_.size();
  remaining_fragments_ = 0;

  for ( Packet & packet : fragments_ ) {
    packet.set_fragments_in_this_frame( fragments_in_this_frame_ );
    packet.set_fec_rate(fec_rate);
    packet.set_red_frags_in_this_frame( num_fec_fragments_ );

    if (is_key_frame_) {
      packet.set_key_frame();
    }
  }
}

  /* construct incoming FragmentedFrame from a Packet */
FragmentedFrame::FragmentedFrame( const uint16_t connection_id,
                                  const Packet & packet )
  : connection_id_( connection_id ),
    source_state_( packet.source_state() ),
    target_state_( packet.target_state() ),
    frame_no_( packet.frame_no() ),
    fragments_in_this_frame_( packet.fragments_in_this_frame() ),
    fragments_( packet.fragments_in_this_frame() ),
    remaining_fragments_( packet.fragments_in_this_frame() ),
    is_key_frame_ ( packet.is_key_frame() ),
    fec_rate_( packet.fec_rate() ),
    num_fec_fragments_ ( packet.red_frags_in_this_frame() )
{
  sanity_check( packet );

  add_packet( packet );
}

void FragmentedFrame::gen_fec_packets(const vector<uint8_t> & whole_frame)
{
  if (fec_rate_ == 0) return;
  double fec_ratio = fec_rate_ / 255.0f;
  uint32_t fec_size = whole_frame.size() * fec_ratio;
  std::vector<uint8_t> fec_vec; 
  auto effective_payload = ((whole_frame.size() - 1) / Packet::MAXIMUM_PAYLOAD + 1) * Packet::MAXIMUM_PAYLOAD;
  fec_vec.resize(effective_payload + fec_size);

  //cerr << "real payload: " << whole_frame.size() << ", effective_payload: " << effective_payload << endl
  //     << " fec ratio: " << fec_ratio << ", fec total " << fec_vec.size() << endl;

  auto original_pkts = fragments_.size();
  size_t next_fragment_start = effective_payload;
  for ( uint16_t fragment_no = original_pkts; next_fragment_start < fec_vec.size();
        fragment_no++ ) {
    fragments_.emplace_back( fec_vec, connection_id_, source_state_, target_state_,
                             frame_no_, fragment_no, 0, next_fragment_start );
  }
  auto after_pkts = fragments_.size();
  num_fec_fragments_ = after_pkts - original_pkts;

}

void FragmentedFrame::sanity_check( const Packet & packet ) const {
  if ( packet.connection_id() != connection_id_ ) {
    cerr << packet.connection_id() << " vs. " << connection_id_ << "\n";
    throw runtime_error( "invalid packet, connection_id mismatch" );
  }

  if ( packet.source_state() != source_state_ ) {
    throw runtime_error( "invalid packet, source_state mismatch" );
  }

  if ( packet.target_state() != target_state_ ) {
    throw runtime_error( "invalid packet, source_state mismatch" );
  }

  if ( packet.fragments_in_this_frame() != fragments_in_this_frame_ ) {
    throw runtime_error( "invalid packet, fragments_in_this_frame mismatch" );
  }

  if ( packet.frame_no() != frame_no_ ) {
    throw runtime_error( "invalid packet, frame_no mismatch" );
  }

  if ( packet.fragment_no() >= fragments_in_this_frame_ ) {
    throw runtime_error( "invalid packet, fragment_no >= fragments_in_this_frame" );
  }
}

/* read a new packet */
void FragmentedFrame::add_packet( const Packet & packet )
{
  sanity_check( packet );

  if ( not fragments_[ packet.fragment_no() ].valid() ) {
    remaining_fragments_--;
    fragments_[ packet.fragment_no() ] = packet;
  }
}

/* send */
void FragmentedFrame::send( UDPSocket & socket )
{
  if ( fragments_.size() != fragments_in_this_frame_ ) {
    throw runtime_error( "attempt to send unfinished FragmentedFrame" );
  }

  assert( complete() );

  for ( const Packet & packet : fragments_ ) {
    socket.send( packet.to_string() );
  }
}

bool FragmentedFrame::complete() const
{
  // consider fec
  return remaining_fragments_ <= num_fec_fragments_;
}

const vector<Packet> & FragmentedFrame::packets() const
{
  if ( (not complete()) or (fragments_.size() != fragments_in_this_frame_) ) {
    throw runtime_error( "attempt to access unfinished FragmentedFrame" );
  }

  return fragments_;
}

string FragmentedFrame::frame() const
{
  string ret;

  if ( not complete() ) {
    throw runtime_error( "attempt to build frame from unfinished FragmentedFrame" );
  }

  unsigned int index = 0;
  for ( const auto & fragment : fragments_ ) {
    ret.append( fragment.payload() );
    index += 1;
    if (index + num_fec_fragments_ == fragments_.size()) {
      break;
    }
  }

  return ret;
}

string FragmentedFrame::partial_frame() const
{
  string ret;

  unsigned int index = 0;
  for ( const auto & fragment : fragments_ ) {
    if ( not fragment.valid() ) {
      break;
    }

    ret.append( fragment.payload() );
    if (index + num_fec_fragments_ == fragments_.size()) {
      break;
    }
  }

  return ret;
}

/* SVCFrame */
SVCFrame::SVCFrame(uint32_t frame_no,
                   std::vector<FragmentedFrame> && layers)
  : frame_no_(frame_no)
{
  /* get packet and rewrite the header for each layer */
  /**
   * header rewritting:
   * frame_no -> svc_layer_no
   * frag_no -> svc_layer_offset
   * num_frags -> svc_layer_size
   *
   * frame_no <- real frame no
   * frag_no <- real frag no
   * num_frags <- real num frags
   */
  for (auto && layer : layers) {
    auto layer_no = layer.frame_no();
    layers_.emplace(layer_no, move(layer));
  }

  for (auto & ent : layers_) {
    auto & layer = ent.second;
    auto & pkts = layer.packets();
    for (auto & pkt : pkts) {
      fragments_.push_back(pkt);
      auto & newpkt = fragments_.back();
      newpkt.set_svc(true);
      newpkt.set_svc_layer_no(newpkt.frame_no());
      newpkt.set_svc_layer_size(newpkt.fragments_in_this_frame());
      newpkt.set_svc_layer_offset(newpkt.fragment_no());
      newpkt.set_frame_no(frame_no_);
    }
  }

  /* rewrite num_frags and frag_no */
  auto total_frags = fragments_.size();
  auto frag_id = 0;
  for (auto & pkt : fragments_) {
    pkt.set_fragments_in_this_frame(total_frags);
    pkt.set_fragment_no(frag_id);
    frag_id++;
  }
}

SVCFrame::SVCFrame(const Packet & pkt)
  : frame_no_(pkt.frame_no())
{
  add_packet(pkt);
}

void SVCFrame::add_packet(const Packet & pkt)
{
  /* restore the header and push it into layers */ 
  /**
   * header rewritting:
   * frame_no <- svc_layer_no
   * frag_no <- svc_layer_offset
   * num_frags <- svc_layer_size
   */
  fragments_.push_back(pkt);
  auto pkt_copy = pkt;
  pkt_copy.set_frame_no(pkt_copy.svc_layer_no());
  pkt_copy.set_fragment_no(pkt_copy.svc_layer_offset());
  pkt_copy.set_fragments_in_this_frame(pkt_copy.svc_layer_size());

  /* add frame into layer */
  if (layers_.count(pkt_copy.svc_layer_no()) == 0) {
    //layers_.emplace(piecewise_construct, 
    //                pkt_copy.svc_layer_no(), 
    //                forward_as_tuple(0, ref(pkt_copy)));
    layers_.insert( make_pair(pkt_copy.svc_layer_no(), 
                              FragmentedFrame(0, pkt_copy)) );
  }
  else {
    layers_.at(pkt_copy.svc_layer_no()).add_packet(pkt_copy);
  }
}

/* AckPacket */

AckPacket::AckPacket( const uint16_t connection_id, const uint32_t frame_no,
                      const uint16_t fragment_no, const uint32_t avg_delay,
                      const uint32_t current_state, deque<uint32_t> complete_states )
  : connection_id_( connection_id ), frame_no_( frame_no ),
    fragment_no_( fragment_no ), avg_delay_( avg_delay ),
    current_state_( current_state ), complete_states_( complete_states ),
    arrive_time_ms_( 0 ), send_time_ms_( 0 ), origin_pkt_size_( 0 )
{}

AckPacket::AckPacket( const Chunk & str )
  : connection_id_( str( 0, 2 ).le16() ),
    frame_no_( str( 2, 4 ).le32() ),
    fragment_no_( str( 6, 2 ).le16() ),
    avg_delay_( str( 8, 4 ).le32() ),
    current_state_( str( 12, 4 ).le32() ),
    complete_states_( str( 28, 4 ).le32() ), //DONT FORGER TO MODIFY HERE WHEN ADDING NEW FIELDS
    arrive_time_ms_( str( 16, 4).le32() ),
    send_time_ms_( str( 20, 4).le32() ),
    origin_pkt_size_( str( 24, 4).le32() )
{
  for ( size_t i = 0; i < complete_states_.size(); i++ ) {
    complete_states_[ i ] = str( 32 + i * 4, 4 ).le32();
  }
}

std::string AckPacket::to_string()
{
  string packet = Packet::put_header_field( connection_id_ )
                + Packet::put_header_field( frame_no_ )
                + Packet::put_header_field( fragment_no_ )
                + Packet::put_header_field( avg_delay_ )
                + Packet::put_header_field( current_state_ ) 
                + Packet::put_header_field( arrive_time_ms_ )
                + Packet::put_header_field( send_time_ms_ )
                + Packet::put_header_field( origin_pkt_size_ );

  packet += Packet::put_header_field( static_cast<uint32_t>( complete_states_.size() ) );

  for ( const auto state : complete_states_ ) {
    packet += Packet::put_header_field( state );
  }

  return packet;
}

void AckPacket::sendto( UDPSocket & socket, const Address & addr )
{
  socket.sendto( addr, to_string() );
}
