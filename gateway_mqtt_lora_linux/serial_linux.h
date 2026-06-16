
// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include <termios.h>
#include <unistd.h>

// -----------------------------------------------------------------------------------------------------------------------------------------

#define SERIAL_CONNECT_CHECK_PERIOD 5
#define SERIAL_CONNECT_CHECK_PRINT  30

typedef enum {
    SERIAL_8N1 = 0,
} serial_bits_t;

const char *serial_bits_str(const serial_bits_t bits) {
    switch (bits) {
    case SERIAL_8N1:
        return "8N1";
    default:
        return "unknown";
    }
}

typedef struct {
    const char *port;
    int rate;
    serial_bits_t bits;
} serial_config_t;

// -----------------------------------------------------------------------------------------------------------------------------------------

const serial_config_t *_serial_cfg = NULL;
int serial_fd = -1;

// -----------------------------------------------------------------------------------------------------------------------------------------

bool serial_connect(void) {
    if (_serial_cfg == NULL)
        return false;
    serial_fd = open(_serial_cfg->port, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        fprintf(stderr, "serial: error opening port: %s\n", strerror(errno));
        return false;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        fprintf(stderr, "serial: error getting port attributes: %s\n", strerror(errno));
        goto serial_failed;
    }
    speed_t baud;
    switch (_serial_cfg->rate) {
    case 1200:
        baud = B1200;
        break;
    case 2400:
        baud = B2400;
        break;
    case 4800:
        baud = B4800;
        break;
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
    case 57600:
        baud = B57600;
        break;
    case 115200:
        baud = B115200;
        break;
    default:
        fprintf(stderr, "serial: unsupported baud rate: %d\n", _serial_cfg->rate);
        goto serial_failed;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    if (_serial_cfg->bits != SERIAL_8N1) {
        fprintf(stderr, "serial: unsupported bits: %s\n", serial_bits_str(_serial_cfg->bits));
        goto serial_failed;
    }
    tty.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    tty.c_cflag &= (tcflag_t)~CSIZE;
    tty.c_cflag |= (tcflag_t)CS8;      // 8-bit characters
    tty.c_cflag &= (tcflag_t)~PARENB;  // No parity
    tty.c_cflag &= (tcflag_t)~CSTOPB;  // 1 stop bit
    tty.c_cflag &= (tcflag_t)~CRTSCTS; // No hardware flow control
    tty.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= (tcflag_t)~OPOST; // Raw output
    tty.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= (tcflag_t) ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "serial: error setting port attributes: %s\n", strerror(errno));
        goto serial_failed;
    }
    tcflush(serial_fd, TCIOFLUSH);
    return true;

serial_failed:
    close(serial_fd);
    serial_fd = -1;
    return false;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void serial_disconnect(void) {
    if (serial_fd < 0)
        return;
    close(serial_fd);
    serial_fd = -1;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void serial_flush(void) {
    if (serial_fd < 0)
        return;
    tcflush(serial_fd, TCIOFLUSH);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int serial_write(const unsigned char *buffer, const int length) {
    if (serial_fd < 0)
        return -1;
    usleep(50 * 1000); // yuck
    return (int)write(serial_fd, buffer, (size_t)length);
}

// -----------------------------------------------------------------------------------------------------------------------------------------

int serial_read(unsigned char *buffer, const int length, const unsigned long timeout_ms) {
    if (serial_fd < 0)
        return -1;
    usleep(50 * 1000); // yuck
    struct pollfd pfd = { .fd = serial_fd, .events = POLLIN };
    const int poll_result = poll(&pfd, 1, (int)(timeout_ms > (unsigned long)INT_MAX ? INT_MAX : timeout_ms));
    if (poll_result <= 0)
        return poll_result; // timeout or error
    int bytes_read = 0;
    unsigned char byte;
    while (bytes_read < length) {
        pfd.revents = 0;
        if (poll(&pfd, 1, 100) <= 0)
            break;
        if (read(serial_fd, &byte, 1) != 1)
            break;
        buffer[bytes_read++] = byte;
    }
    return bytes_read;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

bool serial_begin(const serial_config_t *cfg) {
    _serial_cfg = cfg;
    return true;
}

// -----------------------------------------------------------------------------------------------------------------------------------------

void serial_end(void) {
    serial_disconnect();
    _serial_cfg = NULL;
}

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
