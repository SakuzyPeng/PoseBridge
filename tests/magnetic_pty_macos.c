/* Test-process-only shim: Darwin PTYs have no hardware baud-rate ioctl.
 * Restrict the emulation to /dev/ttysNNN; real USB/serial devices are untouched.
 * No production library is built or linked with this file.
 */
#include <IOKit/serial/ioss.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int pty_ioctl(int fd, unsigned long request, ...) {
    if (IOCPARM_LEN(request) == 0) {
        return ioctl(fd, request);
    }
    va_list args;
    va_start(args, request);
    void *argument = va_arg(args, void *);
    va_end(args);
    /* serialport 4.x uses the original 32-bit request encoding. */
    if (request == IOSSIOSPEED || request == 0x80045402UL) {
        char name[128];
        if (ttyname_r(fd, name, sizeof(name)) == 0 &&
            strncmp(name, "/dev/ttys", 9) == 0 &&
            name[9] >= '0' && name[9] <= '9') {
            return 0;
        }
    }
    return ioctl(fd, request, argument);
}

__attribute__((used)) static struct {
    const void *replacement;
    const void *original;
} interpose __attribute__((section("__DATA,__interpose"))) = {
    (const void *)(uintptr_t)&pty_ioctl, (const void *)(uintptr_t)&ioctl
};
