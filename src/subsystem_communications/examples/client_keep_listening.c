/*
	C ECHO client example using sockets
	http://www.binarytides.com/server-client-example-c-sockets-linux/
*/
#include<stdio.h>	//printf
#include<string.h>	//strlen
#include<sys/socket.h>	//socket
#include<arpa/inet.h>	//inet_addr

int main(int argc , char *argv[])
{
	int sock;
	struct sockaddr_in server;
	char message[1000] , server_reply[2000];
	
	//Create socket
	sock = socket(AF_INET , SOCK_STREAM , 0);
	if (sock == -1)
	{
		printf("Could not create socket");
	}
	puts("Socket created");
	
	server.sin_addr.s_addr = inet_addr("127.0.0.1");
	server.sin_family = AF_INET;
	//server.sin_port = htons( 8888 );
	server.sin_port = htons( 80 );

	//Connect to remote server
	if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		perror("connect failed. Error");
		return 1;
	}
	
	puts("Connected\n");
	
	//keep communicating with server
	/*
		MIKE, you might want to look into this while loop more closely.
		This is where the program keeps listening to the server
	*/
	while(1)
	{
		printf("Enter message : ");
		scanf("%s" , message);
		
		//Send some data
		if( send(sock , message , strlen(message) , 0) < 0)
		{
			puts("Send failed");
			return 1;
		}
		
		//Receive a reply from the server
		if( recv(sock , server_reply , 2000 , 0) < 0)
		{
			puts("recv failed");
			break;
		}
	
		// server_reply (char [2000]) is the response (a string) from the server
		/* 
			The message format would be like this:
				<target>:<command>:<input argument1>(:<input argument2>)
			
			The navigation should respond to messages like these from the server:
			- Navigate to a location: 'nav:goto:fab70', 'nav:goto:eb84'
			- Short/small movements: 'nav:move:forward:1', 'nav:move:left:2' (maybe 1 = 30 degrees, 2 = 60 degrees, etc.)
			
			You might need to check the stringmatching.c file from the Project page > Files to see how to use regular expression to parse the message.
		*/

		puts("Server reply :");
		puts(server_reply);

		// Clear the server_reply array
		memset(server_reply, 0, sizeof server_reply);

		printf("x");
	}
	
	close(sock);
	return 0;
}

