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

#include "vp8_raster.hh"
#include "decoder.hh"
#include "encoder.hh"

using namespace std;

SafeReferences::SafeReferences( const uint16_t width, const uint16_t height )
  : last( move ( MutableSafeRasterHandle( width, height ) ) ),
    golden( move ( MutableSafeRasterHandle( width, height ) ) ),
    alternative( move ( MutableSafeRasterHandle( width, height ) ) )
{}

SafeReferences::SafeReferences( const References & references )
  : last( move ( load( references.last ) ) ),
    golden( move ( load( references.golden ) ) ),
    alternative( move ( load( references.alternative ) ) )
{}

const SafeRaster & SafeReferences::get( reference_frame reference_id ) const
{
  switch ( reference_id ) {
  case LAST_FRAME: return last.get();
  case GOLDEN_FRAME: return golden.get();
  case ALTREF_FRAME: return alternative.get();
  default: throw LogicError();
  }
}

MutableSafeRasterHandle SafeReferences::load( const VP8Raster & source )
{
  MutableSafeRasterHandle target( source.width(), source.height() );
  target.get().copy_raster( source );
  return target;
}
