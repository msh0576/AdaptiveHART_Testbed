#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define MAXBUF 256
#define PORT 5000
#define GROUP "224.0.1.1"

int get_patch_file(char* ipAddr, char* name);

int main(int argc, char* argv[]) {

if (argc < 2) {
		printf("**USAGE** [filename] [ip] [file_name] [...]");
		return 1;
	}
	
	get_patch_file(argv[1], argv[2]);

}

int get_patch_file(char* ipAddr, char* name)
{
	FILE *fp;
	int state;

	char buff[1023];
	char ihexCmd[1023];
	char exeCmd[1023];

	char *basecmd = "busybox ftpget -v";
	char *user = "-u pi";
	char *pwd = "-p csipi";
	char *host = ipAddr;
	char *filename = name;
	char fullFilename[1023];
	sprintf(fullFilename, "/home/pi/Logs/%s", filename);

	// Get .ihex file using FTP
	sprintf(ihexCmd, "%s %s %s %s %s %s", basecmd, user, pwd, host, filename, fullFilename);

	fp = popen(ihexCmd, "r");
	if (fp == NULL) {
	    perror("erro : ");
	    return -1;
	}

	while(fgets(buff, 1023, fp) != NULL) {
	    printf("%s", buff);
	}

	state = pclose(fp);
	usleep(100);
	printf("success get file from pi.\n");
	printf("\n");
	  
	return 0;
}
