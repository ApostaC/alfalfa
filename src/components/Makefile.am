AM_CPPFLAGS = -I$(srcdir)/../util -I$(srcdir)/../decoder -I$(srcdir)/../display -I$(srcdir)/../input -I$(srcdir)/../encoder -I$(srcdir)/../net $(X264_CFLAGS) $(CXX11_FLAGS)
AM_CXXFLAGS = $(PICKY_CXXFLAGS) $(NODEBUG_CXXFLAGS)

AM_LDFLAGS = $(STATIC_BUILD_FLAG)
#BASE_LDADD = ../input/libalfalfainput.a ../decoder/libalfalfadecoder.a ../util/libalfalfautil.a $(X264_LIBS) $(JPEG_LIBS) $(SDL_LIBS)
LDADD = ../components/libalfalfacomp.a ../net/libnet.a ../decoder/libalfalfadecoder.a ../encoder/libalfalfaencoder.a ../util/libalfalfautil.a $(X264_LIBS) 

noinst_LIBRARIES = libalfalfacomp.a

libalfalfacomp_a_SOURCES = comp_encoder.hh comp_encoder.cc \
			frame_observer.hh frame_observer.cc comp_decoder.hh comp_decoder.cc \
			transmission.hh transmission.cc congestion_control.hh congestion_control.cc

bin_PROGRAMS = test-sender test-receiver

test_sender_SOURCES = test-sender.cc
#test_sender_LDADD = ../net/libnet.a ../encoder/libalfalfaencoder.a ./libalfalfacomp.a $(BASE_LDADD)
test_sender_LDFLAGS = -pthread

test_receiver_SOURCES = test-receiver.cc
test_receiver_CPPFLAGS = $(AM_CPPFLAGS) $(GLU_CFLAGS) $(GLEW_CFLAGS) $(GLFW3_CFLAGS)
#test_receiver_LDADD = ../net/libnet.a ../decoder/libalfalfadecoder.a ./libalfalfacomp.a $(BASE_LDADD)
test_receiver_LDFLAGS = -pthread
