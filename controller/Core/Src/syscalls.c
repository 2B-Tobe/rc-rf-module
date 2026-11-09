/*
 * syscalls.c - the few newlib system calls the C library wants to see even though nothing here uses files,
 * stdio or the heap. They just fail. Without them the link warns that they are "not implemented".
 */
#include <errno.h>
#include <sys/stat.h>
#include <stdint.h>

int _close(int fd)                              { (void)fd; errno = EBADF; return -1; }
int _read(int fd, char *buf, int len)           { (void)fd; (void)buf; (void)len; errno = EBADF; return -1; }
int _write(int fd, const char *buf, int len)    { (void)fd; (void)buf; (void)len; errno = EBADF; return -1; }
int _lseek(int fd, int off, int whence)         { (void)fd; (void)off; (void)whence; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st)             { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                             { (void)fd; return 1; }
int _getpid(void)                               { return 1; }
int _kill(int pid, int sig)                     { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status)                          { (void)status; for (;;) {} }

void *_sbrk(int incr)                           /* no heap is used; keep a tiny one for the library's own needs */
{
  extern uint8_t end;                           /* from the linker script: first byte after .bss */
  static uint8_t *brk;
  if (brk == 0) brk = &end;
  uint8_t *prev = brk;
  brk += incr;
  return prev;
}
