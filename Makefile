all:
	gcc -g -pthread src/main.c src/mavlink_udp.c -o mavlink_udp 
	rm -f *.o

clean:
	rm -f *.o core mavlink_udp
