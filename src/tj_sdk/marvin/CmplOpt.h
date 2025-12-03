#ifndef FX_CMPLOPT_H_
#define FX_CMPLOPT_H_

#define CMPL_LIN
// #define CMPL_WIN

#ifdef CMPL_WIN
// #include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <winsock.h>

#define LOOPHANDLE HANDLE

#define socklen_t int

#endif

#ifdef CMPL_LIN
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define INVALID_HANDLE_VALUE -1
#define LOOPHANDLE int
#endif

#endif
