/*
* Find local ip used as source ip in ip packets.
* Read the /proc/net/route file
*/

#include<stdio.h> //printf
#include<string.h>    //memset
#include<errno.h> //errno
#include<sys/socket.h>
#include<netdb.h>
#include<ifaddrs.h>
#include<stdlib.h>
#include<unistd.h>
#include<netinet/in.h>//INADDR_ANY
#include<arpa/inet.h>

int main(int argc, char *argv[])
{
	FILE *f;
	char line[100], *p, *c;
	printf("32bit format: %x", inet_addr(host));
	f = fopen("/proc/net/route", "r");

	while (fgets(line, 100, f))
	{
		p = strtok(line, " \t");
		c = strtok(NULL, " \t");

		if (p != NULL && c != NULL)
		{
			if (strcmp(c, "00000000") == 0)
			{
				printf("Default interface is : %s \n", p);
				break;
			}
		}
	}

	//which family do we require , AF_INET or AF_INET6
	int fm = AF_INET;
	struct ifaddrs *ifaddr, *ifa;
	int family, s;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1)
	{
		perror("getifaddrs");
		exit(EXIT_FAILURE);
	}

	//Walk through linked list, maintaining head pointer so we can free list later
	for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
	{
		if (ifa->ifa_addr == NULL)
		{
			continue;
		}

		family = ifa->ifa_addr->sa_family;

		if (strcmp(ifa->ifa_name, p) == 0)
		{
			if (family == fm)
			{
				s = getnameinfo(ifa->ifa_addr, (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

				if (s != 0)
				{
					printf("getnameinfo() failed: %s\n", gai_strerror(s));
					exit(EXIT_FAILURE);
				}

				printf("address: %s\n", host);
				
				printf("32bit format: %x", inet_addr(host));
			}
			printf("\n");
		}
	}

	freeifaddrs(ifaddr);

	return 0;
}