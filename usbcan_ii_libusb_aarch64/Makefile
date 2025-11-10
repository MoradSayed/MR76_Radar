all: clean test
test:
	$(CC) -o test test.c -L. -L.. -lpthread  -lusbcan
	$(CC) -o test_zuds test_zuds.c -L. -L.. -lpthread  -lusbcan -lzuds
clean:
	rm -vf test
	rm -vf test_zuds
