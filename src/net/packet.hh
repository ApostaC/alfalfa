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

#ifndef PACKET_HH
#define PACKET_HH

#include <vector>
#include <deque>
#include <cassert>
#include <map>

#include "chunk.hh"
#include "socket.hh"
#include "exception.hh"
#include "optional.hh"
#include "pacer.hh"

class Packet
{
public:
  /* control signals */
  static constexpr uint16_t NORMAL = 0;
  static constexpr uint16_t STOP = 1;

private:
  bool valid_;

  uint16_t connection_id_;
  uint32_t source_state_;
  uint32_t target_state_;
  uint32_t frame_no_;
  uint16_t fragment_no_;
  uint16_t fragments_in_this_frame_;
  uint32_t time_since_last_; /* microseconds */
  uint32_t send_timestamp_ms_; /* milliseconds */

  /* fec related */
  uint16_t fec_rate_; // 0 --> 0, 255 --> 100%
  uint16_t red_fragments_in_this_frame_; // 0 --> 0, 255 --> 100%

  /* stop the transmission */
  uint16_t control_signal_;

  /* svc related */
  bool is_svc_ {false};           // NOTE: only used by the sender! receiver never use this field
  uint16_t svc_layer_no_ {0};     // layer id
  uint16_t svc_layer_offset_ {0}; // index of packets in this layer 
  uint16_t svc_layer_size_ {0}; // number of packets in the current layer

  bool is_retrans_ {false}; // only used by the sender! receiver never use this field
  std::string payload_;

public:
  static constexpr size_t MAXIMUM_PAYLOAD = 1400;

  static std::string put_header_field( const uint16_t n );
  static std::string put_header_field( const uint32_t n );
  static std::string put_header_field( const uint64_t n );

  /* setters */
  void set_send_timestamp_ms(uint32_t send_timestamp_ms) { send_timestamp_ms_ = send_timestamp_ms; }
  void set_stop() { control_signal_ = STOP; }
  void set_retrans() { is_retrans_ = true; }

  void set_svc(bool issvc) { is_svc_ = issvc; }
  void set_svc_layer_no(uint16_t layer_no) { svc_layer_no_ = layer_no; }
  void set_svc_layer_size(uint16_t layer_size) { svc_layer_size_ = layer_size; }
  void set_svc_layer_offset(uint16_t layer_off) { svc_layer_offset_ = layer_off; }

  void set_frame_no(uint32_t frame_no) { frame_no_ = frame_no; } // used by svc rewritting
  void set_fragment_no(uint16_t fragment_no) { fragment_no_ = fragment_no; } // used by svc rewriting 

  /* getters */
  bool valid() const { return valid_; }
  uint16_t connection_id() const { return connection_id_; }
  uint32_t source_state() const { return source_state_; }
  uint32_t target_state() const { return target_state_; }
  uint32_t frame_no() const { return frame_no_; }
  uint16_t fragment_no() const { return fragment_no_; }
  uint16_t fragments_in_this_frame() const { return fragments_in_this_frame_; }
  uint32_t time_since_last() const { return time_since_last_; }
  uint32_t send_timestamp_ms() const { return send_timestamp_ms_; }
  const std::string & payload() const { return payload_; }
  uint16_t control_signal() const { return control_signal_; }
  bool is_retrans() const { return is_retrans_; }

  bool is_svc() const { return is_svc_; }
  uint16_t svc_layer_no() const { return svc_layer_no_; }
  uint16_t svc_layer_size() const { return svc_layer_size_; }
  uint16_t svc_layer_offset() const { return svc_layer_offset_; }

  /* for non-salsify encoders, reuse source_state and target_state as key-frame identifier */
  void set_key_frame() { source_state_ = 0u; target_state_ = ~0u; }
  bool is_key_frame() const { return (source_state_ == 0u) and (target_state_ == ~0u); }

  /* for fec */
  void set_fec_rate(uint8_t fec_rate) { fec_rate_ = fec_rate; }
  uint8_t fec_rate() const { return fec_rate_; }
  void set_red_frags_in_this_frame(uint16_t red_frags) { red_fragments_in_this_frame_ = red_frags; }
  uint16_t red_frags_in_this_frame() const { return red_fragments_in_this_frame_; }

  /* construct outgoing Packet */
  Packet( const std::vector<uint8_t> & whole_frame,
          const uint16_t connection_id,
          const uint32_t source_state,
          const uint32_t target_state,
          const uint32_t frame_no,
          const uint16_t fragment_no,
          const uint16_t time_to_next,
          size_t & next_fragment_start);

  /* construct incoming Packet */
  Packet( const Chunk & str );

  /* construct an empty, invalid packet */
  Packet();

  /* serialize a Packet */
  std::string to_string() const;

  void set_fragments_in_this_frame( const uint16_t x );
  void set_time_to_next( const uint32_t val ) { time_since_last_ = val; }
};

class FragmentedFrame
{
protected:
  uint16_t connection_id_;
  uint32_t source_state_;
  uint32_t target_state_;
  uint32_t frame_no_;
  uint16_t fragments_in_this_frame_;

  std::vector<Packet> fragments_;

  uint32_t remaining_fragments_;

  bool is_key_frame_;
  
  /* for FEC */
  uint8_t fec_rate_ {0};
  uint16_t num_fec_fragments_ {0};

protected:
  void gen_fec_packets(const std::vector<uint8_t> & whole_frame);

public:
  /* construct outgoing FragmentedFrame */
  FragmentedFrame( const uint16_t connection_id,
                   const uint32_t source_state,
                   const uint32_t target_state,
                   const uint32_t frame_no,
                   const uint32_t time_to_next_frame,
                   const std::vector<uint8_t> & whole_frame,
                   const bool is_key_frame = false,
                   const uint8_t fec_rate_ = 0);

  /* construct incoming FragmentedFrame from a Packet */
  FragmentedFrame( const uint16_t connection_id,
                   const Packet & packet );

  void sanity_check( const Packet & packet ) const;

  /* read a new packet */
  void add_packet( const Packet & packet );

  /* send */
  void send( UDPSocket & socket );

  bool complete() const;

  /* getters */
  uint16_t connection_id() const { return connection_id_; }
  uint32_t source_state() const { return source_state_; }
  uint32_t target_state() const { return target_state_; }
  uint32_t frame_no() const { return frame_no_; }
  uint16_t fragments_in_this_frame() const { return fragments_in_this_frame_; }
  std::string frame() const;
  std::string partial_frame() const;
  const std::vector<Packet> & packets() const;
  bool is_key_frame() const { return is_key_frame_; }

  /* setters */
  void set_frame_no(uint32_t frame_no) // only used by svc codec
  { 
    frame_no_ = frame_no; 
    for(auto & pkt : fragments_) {
      pkt.set_frame_no(frame_no);
    }
  } 

  /* fec related */
  uint8_t get_fec_rate() const { return fec_rate_; }
  uint16_t get_num_fec_fragments() const { return num_fec_fragments_; }
  uint16_t get_num_payload_fragments() const { return fragments_in_this_frame() - num_fec_fragments_; }

  /* delete copy-constructor and copy-assign operator */
  FragmentedFrame( const FragmentedFrame & other ) = delete;
  FragmentedFrame & operator=( const FragmentedFrame & other ) = delete;

  /* allow moving */
  FragmentedFrame( FragmentedFrame && other ) noexcept
    : connection_id_( other.connection_id_ ),
      source_state_( other.source_state_ ),
      target_state_( other.target_state_ ),
      frame_no_( other.frame_no_ ),
      fragments_in_this_frame_( other.fragments_in_this_frame_ ),
      fragments_( move( other.fragments_ ) ),
      remaining_fragments_( other.remaining_fragments_ ),
      is_key_frame_( other.is_key_frame_ ),
      fec_rate_( other.fec_rate_),
      num_fec_fragments_( other.num_fec_fragments_ )
  {}
};

/**
 * consists of a list of FragmentedFrames
 * the frame_no() in any fragmentedframe is the layer no
 * ASSUME 0 is the base layer
 */
class SVCFrame 
{
private:
  uint32_t frame_no_ {};
  std::map<int, FragmentedFrame> layers_ {};
  std::vector<Packet> fragments_ {};
  bool is_key_frame_ {false};

public:
  SVCFrame(uint32_t frame_no,
           std::vector<FragmentedFrame> && layers);
  SVCFrame(const Packet & pkt);
  SVCFrame(SVCFrame && other) noexcept
    : frame_no_(other.frame_no_),
      layers_( move(other.layers_) ),
      fragments_( move(other.fragments_) )
      {}

  /* deleted copy-constructor */
  SVCFrame(const SVCFrame &) = delete;
  SVCFrame & operator=(const SVCFrame &) = delete;

  /* getters */
  const FragmentedFrame & get_layer(int index) const { return layers_.at(index); }
  const std::vector<Packet> & fragments() const { return fragments_; }
  uint32_t frame_no() const { return frame_no_; }
  bool is_key_frame() const { return is_key_frame_; }

  /* setters */
  void set_key_frame() { is_key_frame_ = true; }

  /* decode check */
  /**
   * return if the base layer is decodable
   * NOTE: assume layer 0 is the base layer
   */
  bool decodable() const;
  /**
   * return if all the first xxx layers are completed
   */
  bool complete(unsigned expected_layers) const;
  uint32_t decodable_size_bytes() const;

  /* incoming packet */
  void add_packet(const Packet & pkt);

  /**
   * convert the current complete layers into a fragmented frame
   * for compatibility with FrameObserver interface
   */
  Optional<FragmentedFrame> to_fragmented_frame() const;
};

class AckPacket
{
private:
  uint16_t connection_id_;
  uint32_t frame_no_;
  uint16_t fragment_no_;
  uint32_t avg_delay_;

  uint32_t current_state_;
  std::deque<uint32_t> complete_states_;

  uint32_t arrive_time_ms_; // only used by comp framework
  uint32_t send_time_ms_; // only used by comp framework
  uint32_t origin_pkt_size_; // only used by comp framework
public:
  AckPacket( const uint16_t connection_id, const uint32_t frame_no,
             const uint16_t fragment_no, const uint32_t avg_delay,
             const uint32_t current_state, std::deque<uint32_t> complete_states );

  AckPacket( const Chunk & str );

  std::string to_string();

  void sendto( UDPSocket & socket, const Address & addr );

  /* getters */
  uint16_t connection_id() const { return connection_id_; }
  uint32_t frame_no() const { return frame_no_; }
  uint16_t fragment_no() const { return fragment_no_; }
  uint32_t avg_delay() const { return avg_delay_; }
  uint32_t arrive_time_ms() const { return arrive_time_ms_; }
  uint32_t send_time_ms() const { return send_time_ms_; }
  uint32_t origin_pkt_size() const { return origin_pkt_size_; }

  /* setters */
  void set_arrive_time(uint32_t arrive_time_ms) { arrive_time_ms_ = arrive_time_ms; }
  void set_send_time(uint32_t send_time_ms) { send_time_ms_ = send_time_ms; }
  void set_origin_pkt_size(uint32_t origin_size) { origin_pkt_size_ = origin_size; }

  uint32_t current_state() const { return current_state_; }
  std::deque<uint32_t> complete_states() const { return complete_states_; }
};

#endif /* PACKET_HH */
