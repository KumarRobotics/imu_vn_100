vn200_linux_binary_async: vn200_linux_binary_async.o vn200.o vncp_services.o vndevice.o
	gcc -pthread -o vn200_linux_binary_async vn200_linux_binary_async.o vn200.o vndevice.o vncp_services.o -lrt

vn200_linux_binary_async.o: vn200_linux_binary_async.c
	gcc -c -Wall -I../../include vn200_linux_binary_async.c

vn200.o: ../../src/vn200.c
	gcc -c -Wall -I../../include ../../src/vn200.c

vndevice.o: ../../src/vndevice.c
	gcc -c -Wall -I../../include ../../src/vndevice.c

vncp_services.o: ../../src/arch/linux/vncp_services.c
	gcc -c -Wall -I../../include ../../src/arch/linux/vncp_services.c

clean:
	rm -f *.o vn200_linux_binary_async