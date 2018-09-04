# Makefile wrapper for waf

all:
	./waf

run:
	./waf --run 80211p-interference --vis

# free free to change this part to suit your requirements
configure:
	./waf configure --enable-examples --enable-tests

build:
	./waf build

install:
	./waf install

clean:
	./waf clean

distclean:
	./waf distclean
