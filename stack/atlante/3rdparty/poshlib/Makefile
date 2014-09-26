CC=gcc
CXX=g++
CFLAGS=-I.
CXXFLAGS=$(CFLAGS)
OBJS=posh.o
TESTARCHOBJS=tests/arch/archtest.o 
TESTLINKOBJS=tests/linktest/linktest.o  tests/linktest/testdll.o  tests/linktest/testlib.o

archtest:  $(TESTARCHOBJS)
	$(CC) -o archtest $(TESTARCHOBJS)
linktest: $(OBJS) $(TESTLINKOBJS)
	$(CXX) -o linktest $(OBJS) $(TESTLINKOBJS)


%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)
%.o: %.cpp
	$(CC) -c -o $@ $< $(CXXFLAGS)

all: archtest linktest

clean:
	rm -f $(OBJS) $(TESTARCHOBJS) $(TESTLINKOBJS) archtest linktest
