/*
 * libjingle
 * Copyright 2012, Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include <gtk/gtk.h>

#include "talk/examples/peerconnection/client/svmp/conductor.h"
#include "talk/examples/peerconnection/client/svmp/flagdefs.h"
//#include "talk/examples/peerconnection/client/linux/main_wnd.h"
#include "talk/examples/peerconnection/client/peer_connection_client.h"


#include <errno.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "talk/base/thread.h"

class CustomSocketServer : public talk_base::PhysicalSocketServer {
 public:
  //CustomSocketServer(talk_base::Thread* thread, GtkMainWnd* wnd)
	CustomSocketServer(talk_base::Thread* thread)
      //: thread_(thread), wnd_(NULL), conductor_(NULL), client_(NULL) {}
	: thread_(thread), conductor_(NULL), client_(NULL) {}
  virtual ~CustomSocketServer() {}

  void set_client(PeerConnectionClient* client) { client_ = client; }
  void set_conductor(Conductor* conductor) { conductor_ = conductor; }

  // Override so that we can also pump the GTK message loop.
  virtual bool Wait(int cms, bool process_io) {
    // Pump GTK events.
    // TODO: We really should move either the socket server or UI to a
    // different thread.  Alternatively we could look at merging the two loops
    // by implementing a dispatcher for the socket server and/or use
    // g_main_context_set_poll_func.
      //while (gtk_events_pending())
      //        gtk_main_iteration();

	 sleep(.6);

    //if (!wnd_->IsWindow() && !conductor_->connection_active() &&
//	  if (!conductor_->connection_active() &&
//        client_ != NULL && !client_->is_connected()) {
//      thread_->Quit();
//    }
    return talk_base::PhysicalSocketServer::Wait(0/*cms == -1 ? 1 : cms*/,
                                                 process_io);
  }

 protected:
  talk_base::Thread* thread_;
  //GtkMainWnd* wnd_;
  //void *wnd_;
  Conductor* conductor_;
  PeerConnectionClient* client_;
};

//
///* strerror_r Wrapper to return a char* instead of int */
//char *Strerror_r(int err, char *buf, size_t len) {
//		strerror_r(err,buf,len);
//		return buf;
//}
//
//
//
//void startListener(struct complete_stream *sinfo) {
//
//	int servfd,newfd,fdmax,
//	    i; //,n;
//        //socklen_t servlen,clilen;
//	fd_set master,read_fds;
//
//	//struct sockaddr_un  cli_addr;//, serv_addr;
//	fdmax=0;
//	//char buf[80];
//	char errbuf[256];
//	printf("daemonListenThread\n");
//
//	//servfd = android_get_control_socket("fbstr_command2");
//	if (servfd < 0 )
//		printf("error with android_get_control_socket: error %s\n",Strerror_r(errno,errbuf,256));
//	else
//		printf("android_get_control_socket success fd %d\n", servfd);
//
//
//	if(listen(servfd, 10) == -1) {
//		//pprintf("Could not listen to raw daemon socket");
//		printf("listen error %s\n",Strerror_r(errno,errbuf,256));
//		return ;
//	}
//
//	FD_ZERO(&master);
//	FD_ZERO(&read_fds);
//	FD_SET(servfd,&master);
//
//	fdmax = max(fdmax,servfd);
//
//	printf("starting select loop\n");
//	for (;;) {
//		read_fds=master;
//
//		if(select(fdmax+1,&read_fds,NULL,NULL,NULL)==-1)
//			printf("select error %s\n",Strerror_r(errno,errbuf,256));
//
//		for(i=0; i<=fdmax; i++){
//			if(FD_ISSET(i,&read_fds)) {
//				printf("fd is set i=%d, servfd %d \n",i,servfd);
//				if(i == servfd) {
//					newfd = accept(servfd,NULL,NULL);
//					if (newfd >0){
//						printf("new connection accepted fd = %d\n",newfd);
//						FD_SET(newfd,&master);
//						fdmax=max(fdmax,newfd);
//					}
//					else
//						printf("accept error: %s\n", Strerror_r(errno,errbuf,256));
//				}else { // existing client
//					int n=recvPacket(i,sinfo);
//					if (n<=0) {
//						printf("closing connection to client \n");
//						close(i);
//						FD_CLR(i, &master);
//					} else {
//						// packet processed correctly
//						printf("new packet processed size : %d\n",n);
//					}
//				}
//			}
//		}
//	}
//
//}



int main(int argc, char* argv[]) {
  //gtk_init(&argc, &argv);
  //g_type_init();
  //g_thread_init(NULL);
  //void* wnd = NULL;

  FlagList::SetFlagsFromCommandLine(&argc, argv, true);
  if (FLAG_help) {
    FlagList::Print(NULL, false);
    return 0;
  }

  // Abort if the user specifies a port that is outside the allowed
  // range [1, 65535].
  if ((FLAG_port < 1) || (FLAG_port > 65535)) {
    printf("Error: %i is not a valid port.\n", FLAG_port);
    return -1;
  }

   printf ("FLAG_server: %s, FLAG_PORT %d\n",FLAG_server,FLAG_port);


  //GtkMainWnd wnd(FLAG_server, FLAG_port, FLAG_autoconnect, FLAG_autocall);
  //wnd.Create();

  talk_base::AutoThread auto_thread;
  talk_base::Thread* thread = talk_base::Thread::Current();
  //CustomSocketServer socket_server(thread, &wnd);
  CustomSocketServer socket_server(thread);
  //thread->Join();
  thread->set_socketserver(&socket_server);

  // Must be constructed after we set the socketserver.
  PeerConnectionClient client;
  talk_base::scoped_refptr<Conductor> conductor(
		  new talk_base::RefCountedObject<Conductor>(&client,FLAG_server, FLAG_port));
     // new talk_base::RefCountedObject<Conductor>(&client, &wnd));
  socket_server.set_client(&client);
  socket_server.set_conductor(conductor);


  //socket_server.Wait();



  thread->Run();


  printf("\ncall started\n\n");
  printf("Press enter to stop...");
  while ((getchar()) != '\n')
	  ;
  // gtk_main();
//  wnd.Destroy();

  thread->set_socketserver(NULL);
  // TODO: Run the Gtk main loop to tear down the connection.
  //while (gtk_events_pending()) {
  //  gtk_main_iteration();
  //}

  return 0;
}

