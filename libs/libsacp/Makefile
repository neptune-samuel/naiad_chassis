

all: test 

cflags := -Wall -D__no_framework__

objs += test.o 
objs += sacp_frame.o
objs += portable.o

test: $(objs)
	g++ -o $@ $(objs)

clean:
	rm -rf *.o test 


RELEASE_NAME=libsacp

release:
	tar cvf tmp.tar *
	mkdir -p $(RELEASE_NAME)
	tar xvf tmp.tar -C $(RELEASE_NAME)
	tar czvf $(RELEASE_NAME).tar.gz $(RELEASE_NAME)
	rm -rf tmp.tar $(RELEASE_NAME)

%.o:%.cc 
	g++ $(cflags) -c -o $@ $< 

%.o:%.c
	gcc $(cflags) -c -o $@ $< 
