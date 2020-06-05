CC=gcc
CFLASGS=-L
CFLAGS=-Wall

test: test.o readTemp.o
	$(CC) -o test test.o readTemp.o -lpigpio

