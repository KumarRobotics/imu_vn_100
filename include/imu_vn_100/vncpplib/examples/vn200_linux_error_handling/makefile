vn200_linux_error_handling: vn200_linux_error_handling.o vn200.o vncp_services.o vndevice.o
	gcc -pthread -o vn200_linux_error_handling vn200_linux_error_handling.o vn200.o vndevice.o vncp_services.o -lrt

vn200_linux_error_handling.o: vn200_linux_error_handling.c
	gcc -c -Wall -I../../include vn200_linux_error_handling.c

vn200.o: ../../src/vn200.c
	gcc -c -Wall -I../../include ../../src/vn200.c

vndevice.o: ../../src/vndevice.c
	gcc -c -Wall -I../../include ../../src/vndevice.c

vncp_services.o: ../../src/arch/linux/vncp_services.c
	gcc -c -Wall -I../../include ../../src/arch/linux/vncp_services.c

clean:
	rm -f *.o vn200_linux_error_handling