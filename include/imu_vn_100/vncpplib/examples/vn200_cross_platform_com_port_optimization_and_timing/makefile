vn200_cross_platform_optimization_and_timing: vn200_cross_platform_optimization_and_timing.o vn200.o vncp_services.o vndevice.o
	gcc -pthread -o vn200_cross_platform_optimization_and_timing vn200_cross_platform_optimization_and_timing.o vn200.o vndevice.o vncp_services.o -lrt

vn200_cross_platform_optimization_and_timing.o: vn200_cross_platform_optimization_and_timing.c
	gcc -c -Wall -I../../include vn200_cross_platform_optimization_and_timing.c

vn200.o: ../../src/vn200.c
	gcc -c -Wall -I../../include ../../src/vn200.c

vndevice.o: ../../src/vndevice.c
	gcc -c -Wall -I../../include ../../src/vndevice.c

vncp_services.o: ../../src/arch/linux/vncp_services.c
	gcc -c -Wall -I../../include ../../src/arch/linux/vncp_services.c

clean:
	rm -f *.o vn200_cross_platform_optimization_and_timing