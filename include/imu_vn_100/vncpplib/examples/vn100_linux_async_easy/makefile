vn100_linux_async_easy: vn100_linux_async_easy.o vn100.o vncp_services.o vndevice.o
	gcc -pthread -o vn100_linux_async_easy vn100_linux_async_easy.o vn100.o vndevice.o vncp_services.o -lrt

vn100_linux_async_easy.o: vn100_linux_async_easy.c
	gcc -c -Wall -I../../include vn100_linux_async_easy.c

vn100.o: ../../src/vn100.c
	gcc -c -Wall -I../../include ../../src/vn100.c

vndevice.o: ../../src/vndevice.c
	gcc -c -Wall -I../../include ../../src/vndevice.c

vncp_services.o: ../../src/arch/linux/vncp_services.c
	gcc -c -Wall -I../../include ../../src/arch/linux/vncp_services.c

clean:
	rm -f *.o vn100_linux_async_easy