/* Load through LD_PRELOAD to force listening operations to localhost */
/* cc -Wall -g -fPIC -shared -Wl,-init,init bind.c -o libbind.so -ldl */
/* $ LD_PRELOAD=./libbind.so nc -l 1234 */

#define _GNU_SOURCE

#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

static int ( *_bind )(int sockfd, const struct sockaddr *addr, socklen_t addrlen) = NULL;

__attribute__((constructor))
static void _init(void) {
    if((_bind = dlsym(RTLD_NEXT, "bind")) == NULL)
        _exit(0);
}

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen) {
    struct sockaddr_in *paddr;
    struct sockaddr_in laddr;

    paddr = (struct sockaddr_in *) addr;
    if((sizeof(struct sockaddr_in) == addrlen) &&
            (paddr->sin_family == AF_INET) &&
            (paddr->sin_addr.s_addr == htonl(0))) {
        memcpy(&laddr, paddr, sizeof(struct sockaddr_in));
        laddr.sin_addr.s_addr = htonl(0x7f000001);
        addr = (struct sockaddr *) &laddr;
    }

    return (*_bind)(sockfd, addr, addrlen);
}
