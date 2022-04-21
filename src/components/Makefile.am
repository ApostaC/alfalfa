AM_CPPFLAGS = -I$(srcdir)/../util -I$(srcdir)/../decoder -I$(srcdir)/../display -I$(srcdir)/../input -I$(srcdir)/../encoder -I$(srcdir)/../net $(X264_CFLAGS) $(CXX11_FLAGS)
AM_CXXFLAGS = $(PICKY_CXXFLAGS) $(NODEBUG_CXXFLAGS)

noinst_LIBRARIES = libalfalfacomp.a

libalfalfacomp_a_SOURCES = comp_encoder.hh comp_encoder.cc \
			frame_observer.hh frame_observer.cc comp_decoder.hh comp_decoder.cc \
			transmission.hh transmission.cc congestion_control.hh congestion_control.cc
