CROSS = arm-linux-gnueabihf-

all: zpttytest

zpttytest: zptty_test.c
	$(CROSS)gcc -O2 -o zpttytest zptty_test.c 

clean:
	@rm -vf zpttytest *.o *~
