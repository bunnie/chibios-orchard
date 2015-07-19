/*
 * Orchard Factory Test
 *
 * The factory test requires that the serial port and SWD pins be wired up.
 * Additionally, it is recommended that a USB power connection be made, so
 * as to check the charger.
 *
 * The Device Under Test (DUT) should be loaded into the test jig before the
 * test is run.  The board will be reflashed.  If OpenOCD is having trouble
 * attaching, it will erase flash.  Therefore, if you start the test prior
 * to loading the device into the jig, it will be completely erased.
 *
 * This test program will flash the DUT and will then run all tests.  If
 * you need to run individual tests, or flash the board separately, arguments
 * may be passed to this program.
 *
 * An example invocation might be:
    sudo ./factory-test \
           --swclk 20 \
           --swdio 21 \
           --button 26 \
           --serial /dev/ttyAMA0 
 */

#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))
enum log_levels {
  LOGF_NONE = 0,
  LOGF_ERROR = 1,
  LOGF_INFO = 2,
  LOGF_DEBUG = 3,
};

enum test_state {
  TEST_FAIL,
  TEST_INPROGRESS,
  TEST_PASS,
};

const char *openocd_default_args[] = {
  "openocd",
  "-c", "interface sysfsgpio",
  "-c", "transport select swd",
  "-c", "source [find target/klx.cfg]",
  "-c", "klx.cpu configure -rtos ChibiOS",
};

struct factory_config {
  const char *openocd_config;
  const char *serial_path;
  const char *image;
  const char **specific_test_names;
  uint32_t    specific_tests;
  int         button_gpio;
  int         swclk_gpio;
  int         swdio_gpio;
  int         green_gpio;
  int         yellow_gpio;
  int         red_gpio;
  int         verbose;
  int         daemon;
  int         do_program;
  int         do_tests;
};

struct factory {
  int                   button_fd;    /* Physical button GPIO handle */
  int                   openocd_sock; /* TCL socket */
  int                   openocd_pid;  /* Process PID */
  int                   serial_fd;    /* TTL UART file */
  int                   green_fd;     /* GPIO fd for green LED */
  int                   yellow_fd;    /* GPIO fd for yellow LED */
  int                   red_fd;       /* GPIO fd for red LED */
  enum test_state       test_state;   /* Currently-displayed state */
  struct termios        serial_old_termios; /* Restore settings on exit */
  struct factory_config cfg;
};

struct factory *g_factory;
static int validate_cpu(struct factory *f);
static int button_pressed(struct factory *f);
static int openocd_stop(struct factory *f);
static int serial_close(struct factory *f);

/*
 * In UNIX, when a child process dies we must call wait()/waitpid() to
 * reap the child, otherwise we'll get zombie processes.  The standard way
 * to do this (and what we do here) is to listen for SIGCHLD which indicates
 * that a child process has stopped or terminated, and then call waidpid().
 */
static void handle_sigchld(int signal) {
  int status;
  pid_t pid;

  while (-1 != (pid = waitpid(-1, &status, WNOHANG)))
    g_factory->openocd_pid = -1;
}

static void handle_sigterm(int signal) {
  openocd_stop(g_factory);
  serial_close(g_factory);
  if (g_factory->button_fd) {
    close(g_factory->button_fd);
    g_factory->button_fd = -1;
  }
  g_factory->cfg.daemon = 0;
  exit(1);
}

static void fdbg(struct factory *f, const char *format, ...) {

  va_list ap;

  if (f->cfg.verbose < LOGF_DEBUG)
    return;

  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
}

static void finfo(struct factory *f, const char *format, ...) {

  va_list ap;

  if (f->cfg.verbose < LOGF_INFO)
    return;

  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
}

static void ferr(struct factory *f, const char *format, ...) {

  va_list ap;

  if (f->cfg.verbose < LOGF_ERROR)
    return;

  va_start(ap, format);
  vfprintf(stderr, format, ap);
  va_end(ap);
}

static int stream_wait_banner(struct factory *f, int fd, const char *banner) {
  int banner_size = strlen(banner);
  uint8_t buf[banner_size];
  int tst;
  int offset = 0;
  int i;

  tcdrain(fd);

  while (1) {
    int ret;
    while (1 == (ret = read(fd, &buf[offset], 1))) {
      
      tst = (offset + 1) % sizeof(buf);
      
      i = 0;
      while (tst != offset) {
        if (banner[i] != buf[tst])
          break;
        tst++;
        i++;
        tst %= sizeof(buf);
      }
      if ((tst == offset) && (banner[i] == buf[tst]))
        return 0;

      offset++;
      offset %= sizeof(buf);
    }
    if ((ret == -1) && (errno != EAGAIN))
      return -1;

    /* This can happen if we lose sync (e.g. board removed during test) */
    if (validate_cpu(f))
      return -1;
    if ((f->button_fd >= 0) && button_pressed(f))
      return -1;
  }
}

static int openocd_run(struct factory *f) {
  const char *openocd_args[ARRAY_SIZE(openocd_default_args) + 7];
  char swdio_str[128];
  char swclk_str[128];
  unsigned int i;

  for (i = 0; i < ARRAY_SIZE(openocd_default_args); i++)
    openocd_args[i] = openocd_default_args[i];

  if (f->cfg.swdio_gpio >= 0) {
    snprintf(swdio_str, sizeof(swdio_str) - 1, "sysfsgpio_swdio_num %d",
        f->cfg.swdio_gpio);
    openocd_args[i++] = "-c";
    openocd_args[i++] = swdio_str;
  }

  if (f->cfg.swclk_gpio >= 0) {
    snprintf(swclk_str, sizeof(swclk_str) - 1, "sysfsgpio_swclk_num %d",
        f->cfg.swclk_gpio);
    openocd_args[i++] = "-c";
    openocd_args[i++] = swclk_str;
  }

  if (f->cfg.openocd_config) {
    openocd_args[i++] = "-f";
    openocd_args[i++] = f->cfg.openocd_config;
  }

  openocd_args[i++] = NULL;

  f->openocd_pid = fork();
  if (f->openocd_pid == -1) {
    perror("Unable to fork");
    return -1;
  }

  if (f->openocd_pid == 0) {
    if (f->cfg.verbose < LOGF_DEBUG) {
      close(STDIN_FILENO);
      close(STDOUT_FILENO);
      close(STDERR_FILENO);
    }
    execvp(openocd_args[0], (char * const*) openocd_args);
    perror("Unable to exec");
    return -1;
  }

  return 0;
}

static int openocd_connect(struct factory *f) {

  const char *address = "127.0.0.1";
  const int port = 6666;
  struct sockaddr_in sockaddr;
  int ret;

  f->openocd_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  
  if (-1 == f->openocd_sock) {
    perror("cannot create socket");
    return -1;
  }
  
  memset(&sockaddr, 0, sizeof(sockaddr));
  
  sockaddr.sin_family = AF_INET;
  sockaddr.sin_port = htons(port);
  ret = inet_pton(AF_INET, address, &sockaddr.sin_addr);
  
  if (ret < 0) {
    perror("Seems like INET support doesn't exist");
    goto err;
  }
  else if (ret == 0) {
    perror("Unable to connect to address");
    goto err;
  }

  if (-1 == connect(f->openocd_sock,
                    (struct sockaddr *)&sockaddr,
                    sizeof(sockaddr))) {
    perror("connect failed");
    goto err;
  }

  return 0;
  
err:
  close(f->openocd_sock);
  f->openocd_sock = -1;
  return -1;
}

static int openocd_send(struct factory *f, const char *msg) {
  int ret;
  int len;
  char token = 0x1a;

  len = strlen(msg);

  ret = write(f->openocd_sock, msg, len);
  if (ret != len) {
    perror("Unable to write");
    return -1;
  }

  ret = write(f->openocd_sock, &token, 1);
  if (ret != 1) {
    perror("Unable to write token");
    return -1;
  }
  return 0;
}

static int openocd_recv(struct factory *f, char *buf, int max) {
  int ret;
  int size;

  size = 0;
  while (size < max) {
    char c;

    ret = read(f->openocd_sock, &c, 1);
    if (ret != 1) {
      perror("Unable to read");
      return -1;
    }

    if (c == 0x1a)
      return size;

    buf[size++] = c;
  }
  return size;
}

static int strbegins(const char *str, const char *begin) {
  return !strncmp(str, begin, strlen(begin));
}

static int openocd_write_image(struct factory *f, const char *elf) {
  char buf[256];

  snprintf(buf, sizeof(buf) - 1, "ocd_flash write_image erase %s", elf);
  openocd_send(f, buf);

  memset(buf, 0, sizeof(buf));
  if (openocd_recv(f, buf, sizeof(buf)) < 0)
    return -1;

  return strstr(buf, "Failed to write memory") != NULL;
}

static uint32_t openocd_readmem(struct factory *f, uint32_t addr) {
  char buf[256];

  snprintf(buf, sizeof(buf) - 1, "ocd_mdw 0x%08x", addr);
  openocd_send(f, buf);

  if (openocd_recv(f, buf, sizeof(buf)) < 0)
    return 0;

  // 0x40048024: 16151502
  // -----------^ 12 characters
  return strtoul(buf + 12, NULL, 16);
}

static int kinetis_check_security(struct factory *f) {
  char buf[256];

  snprintf(buf, sizeof(buf) - 1, "ocd_kinetis mdm check_security");
  openocd_send(f, buf);

  if (openocd_recv(f, buf, sizeof(buf)) < 0)
    return -1;

  if (strbegins(buf, "MDM: Chip is unsecured. Continuing."))
    return 0;
  return 1;
}

static int kinetis_mass_erase(struct factory *f) {
  char buf[256];

  snprintf(buf, sizeof(buf) - 1, "ocd_kinetis mdm mass_erase");
  openocd_send(f, buf);

  if (openocd_recv(f, buf, sizeof(buf)) < 0)
    return -1;

  return 0;
}

static int check_swdid(struct factory *f, uint32_t swdid) {
  char buf[4096];
  uint32_t found_id;
  openocd_send(f, "ocd_transport init");
  openocd_recv(f, buf, sizeof(buf));

  // SWD IDCODE 0x00000000
  // ----------^ 11 characters
  found_id = strtoul(buf + 11, NULL, 0);
  if (swdid == found_id)
    return 0;

  ferr(f, "SWD ID 0x%08x does not match found id 0x%08x\n", swdid, found_id);
  return 1;
}

static int check_dapid(struct factory *f, uint32_t dapid) {
  char buf[4096];
  uint32_t found_id;
  openocd_send(f, "ocd_dap apid");
  openocd_recv(f, buf, sizeof(buf));

  found_id = strtoul(buf, NULL, 0);
  if (dapid == found_id)
    return 0;

  ferr(f, "DAP ID 0x%08x does not match found id 0x%08x\n", dapid, found_id);
  return 1;
}

void print_help(const char *name) {
  printf("Usage:\n");
  printf("    %s\n", name);
  printf(" -c --config     Config file to use with OpenOCD\n");
  printf(" -s --serial     Serial port to use for monitoring\n");
  printf(" -e --elf        ELF image to program\n");
  printf(" -k --swclk      GPIO pin to use for SWD clock\n");
  printf(" -d --swdio      GPIO pin to use for SWD data\n");
  printf(" -b --button     GPIO pin to use for Start button\n");
  printf(" -0 --green      GPIO pin to use for Green LED\n");
  printf(" -1 --yellow     GPIO pin to use for Yellow LED\n");
  printf(" -2 --red        GPIO pin to use for RED LED\n");
  printf(" -p --no-program Skip the programming step\n");
  printf(" -t --no-tests   Skip the testing step\n");
  printf(" -r --run-test   Runs a specific test.  May be specified multiple times.\n");
  printf(" -v --verbose    Increase verbosity.  May be specified multiple times.\n");
}

int parse_args(struct factory_config *cfg, int argc, char **argv) {

  int c;
  int idx = 0;
  static struct option long_options[] = {
    {"config",        required_argument, NULL,  'c'},
    {"serial",        required_argument, NULL,  's'},
    {"elf",           required_argument, NULL,  'e'},
    {"button",        required_argument, NULL,  'b'},
    {"swclk",         required_argument, NULL,  'k'},
    {"swdio",         required_argument, NULL,  'd'},
    {"green",         required_argument, NULL,  '0'},
    {"yellow",        required_argument, NULL,  '1'},
    {"red",           required_argument, NULL,  '2'},
    {"run-test",      required_argument, NULL,  'r'},
    {"verbose",       no_argument,       NULL,  'v'},
    {"no-program",    no_argument,       NULL,  'p'},
    {"no-tests",      no_argument,       NULL,  't'},
    {"help",          no_argument,       NULL,  'h'},
  };

  while ((c = getopt_long(argc, argv, "c:k:d:b:r:e:0:1:2:hvpt", long_options, &idx)) != -1) {
    switch (c) {
    case -1:
      return 0;

    case '0':
      cfg->green_gpio = strtoul(optarg, NULL, 0);
      break;

    case '1':
      cfg->yellow_gpio = strtoul(optarg, NULL, 0);
      break;

    case '2':
      cfg->red_gpio = strtoul(optarg, NULL, 0);
      break;

    case 'e':
      cfg->image = strdup(optarg);
      break;

    case 'c':
      cfg->openocd_config = strdup(optarg);
      break;

    case 'p':
      cfg->do_program = 0;
      break;

    case 't':
      cfg->do_tests = 0;
      break;

    case 'r':
      /* Allocate space for the new test name */
      cfg->specific_test_names = realloc(cfg->specific_test_names,
                                  sizeof(char *) * (cfg->specific_tests + 1));
      cfg->specific_test_names[cfg->specific_tests] = strdup(optarg);
      cfg->specific_tests++;
      break;

    case 's':
      cfg->serial_path = strdup(optarg);
      break;

    case 'b':
      cfg->button_gpio = strtoul(optarg, NULL, 0);
      cfg->daemon = 1;
      break;

    case 'k':
      cfg->swclk_gpio = strtoul(optarg, NULL, 0);
      break;

    case 'd':
      cfg->swdio_gpio = strtoul(optarg, NULL, 0);
      break;

    case 'v':
      cfg->verbose++;
      break;

    case 'h':
      print_help(argv[0]);
      return -1;

    default:
      printf("Unrecognized option: %c\n", c);
      print_help(argv[0]);
      return -1;
    }
  }
  return 0;
}

static int serial_open(struct factory *f) {
  struct termios t;
  int ret;

  if (!f->cfg.serial_path)
    return 0;

  f->serial_fd = open(f->cfg.serial_path, O_RDWR);
  if (-1 == f->serial_fd) {
    perror("Unable to open serial port");
    return -1;
  }

  ret = tcgetattr(f->serial_fd, &t);
  if (-1 == ret) {
    perror("Failed to get attributes");
    goto err;
  }
  f->serial_old_termios = t;

  cfsetispeed(&t, B115200);
  cfsetospeed(&t, B115200);
  cfmakeraw(&t);
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 3; /* This lets us monitor the CPU while waiting for data */

  ret = tcsetattr(f->serial_fd, TCSANOW, &t);
  if (-1 == ret) {
    perror("Failed to set attributes");
    goto err;
  }

  return 0;

err:
  close(f->serial_fd);
  f->serial_fd = -1;
  return -1;
}

static int serial_close(struct factory *f) {

  if (f->serial_fd == -1)
    return 0;

  tcsetattr(f->serial_fd, TCSANOW, &f->serial_old_termios);
  close(f->serial_fd);
  f->serial_fd = -1;

  return 0;
}

static int openocd_stop(struct factory *f) {

  if (f->openocd_pid != -1) {
    int status;
    int tries;
    kill(f->openocd_pid, SIGTERM);

    for (tries = 0; tries < 200; tries++) {
      pid_t pid = waitpid(-1, &status, WNOHANG);
      if (pid == 0) {
        usleep(1000);
        continue;
      }
      f->openocd_pid = -1;
      break;
    }
    if (f->openocd_pid == -1) {
      fdbg(f, "OpenOCD quit after %d tries\n", tries);
    }
    else {
      fdbg(f, "OpenOCD would not quit, sending SIGKILL\n");
      kill(f->openocd_pid, SIGKILL);
      for (tries = 0; tries < 200; tries++) {
        pid_t pid = waitpid(-1, &status, WNOHANG);
        if (pid == 0) {
          usleep(1000);
          continue;
        }
        f->openocd_pid = -1;
        break;
      }
      if (f->openocd_pid == -1) {
        fdbg(f, "OpenOCD quit after SIGKILL, after %d tries\n", tries);
      }
      else {
        fdbg(f, "OpenOCD wouldn't quit\n");
        f->openocd_pid = -1;
      }
    }
  }

  if (f->openocd_sock != -1) {
    (void) shutdown(f->openocd_sock, SHUT_RDWR);
    close(f->openocd_sock);
  }
  f->openocd_sock = -1;

  return 0;
}

static int openocd_reset_halt(struct factory *f) {
  char buf[256];
  int tries = 0;
  int ret;

  do {
    openocd_send(f, "reset halt");
    openocd_recv(f, buf, sizeof(buf));

    openocd_send(f, "ocd_klx.cpu curstate");
    ret = openocd_recv(f, buf, sizeof(buf));
    tries++;
  } while ((ret >= 0) && !strbegins(buf, "halted"));

  fdbg(f, "Halted after %d tries\n", tries);
  return 0;
}

static int openocd_reset(struct factory *f) {
  char buf[256];
  int ret;
  int tries = 0;

  openocd_send(f, "reset run");
  ret = openocd_recv(f, buf, sizeof(buf));
  if (ret < 0)
    return -1;

  /* Wait for the board to exit reset */
  do {
    openocd_send(f, "ocd_klx.cpu curstate");
    memset(buf, 0, sizeof(buf));
    ret = openocd_recv(f, buf, sizeof(buf));
    tries++;
  } while ((ret >= 0) && (strbegins(buf, "reset") || (buf[0] == '\0')));
  return 0;
}

static int validate_cpu(struct factory *f) {
  int ret;

  ret = openocd_readmem(f, 0x40048024);
  if (ret != 0x16151502) {
    ferr(f, "CPU ID was 0x%08x, not 0x16151502\n", ret);
    ret = openocd_readmem(f, 0x40048024);
    if (ret != 0x16151502) {
      ferr(f, "2nd try: CPU ID was 0x%08x, not 0x16151502\n", ret);
      return -1;
    }
  }
  fdbg(f, "Correct CPU ID found: 0x%08x\n", ret);

  return 0;
}

static int cpu_running(struct factory *f) {
  int ret;
  char buf[256];

  memset(buf, 0, sizeof(buf));
  openocd_send(f, "ocd_klx.cpu curstate");
  ret = openocd_recv(f, buf, sizeof(buf));
  if (ret < 0)
    return -1;

  if (!strbegins(buf, "running"))
    return 0;
  return 1;
}

static int orchard_program(struct factory *f) {

  int ret;

  if (openocd_reset_halt(f))
    return -1;

  ret = validate_cpu(f);
  if (ret)
    return ret;

  ret = kinetis_check_security(f);
  if (ret < 0)
    return -1;
  else if (ret > 0) {
    finfo(f, "CPU is locked, doing a mass erase\n");
    kinetis_mass_erase(f);
  }
  else
    finfo(f, "CPU is unlocked\n");

  if (f->cfg.image) {
    finfo(f, "Writing image %s...\n", f->cfg.image);
    ret = openocd_write_image(f, f->cfg.image);
    if (ret)
      return -1;
  }

  return 0;
}

static int writestr(int fd, char *str) {
  return write(fd, str, strlen(str));
}

static int orchard_run_test(struct factory *f, const char *testname) {

  int ret;
  char buf[256];

  snprintf(buf, sizeof(buf) - 1, "test %s 3\r\n", testname);
  ret = writestr(f->serial_fd, buf);
  if (ret <= 0)
    return ret;

  return stream_wait_banner(f, f->serial_fd, "ch> ");
}

static int orchard_run_tests(struct factory *f) {

  int ret;

  ret = writestr(f->serial_fd, "chgcap 4000 14800 3250 440\r\n");
  stream_wait_banner(f, f->serial_fd, "ch> ");
  
  ret = writestr(f->serial_fd, "testall 3\r\n");
  if (ret <= 0)
    return ret;

  return stream_wait_banner(f, f->serial_fd, "ch> ");
}

/* Restart OpenOCD, and run tests as defined in the factory configuration. */
static int run_tests(struct factory *f) {
  int ret;
  int tries = 0;
  int greater_tries = 0;

  ret = serial_open(f);
  if (ret)
    goto cleanup;

  while (1) {
    ret = openocd_run(f);
    if (ret)
      goto cleanup;

    /* Keep trying to connect as long as OpenOCD is still running */
    while ((f->openocd_pid != -1) && (f->openocd_sock == -1)) {
      usleep(50000);
      openocd_connect(f);
      tries++;
    }

    if ((f->openocd_pid == -1) || (f->openocd_sock == -1)) {
      finfo(f, "OpenOCD quit.  Misconfiguration?");
      ret = -1;
      goto cleanup;
    }
    ret = check_swdid(f, 0x0bc11477);
    if (ret)
      goto cleanup;
    finfo(f, "SWD ID matches 0x0bc11477\n");

    ret = check_dapid(f, 0x04770031);
    if (ret)
      goto try_again;
    finfo(f, "DAP ID matches 0x04770031\n");

    if (validate_cpu(f))
      goto try_again;

    break;

try_again:
    openocd_stop(f);
    greater_tries++;
  }

  fdbg(f, "Connected after %d tries\n", greater_tries);
  finfo(f, "SDID: 0x%08x\n", openocd_readmem(f, 0x40048024));
  finfo(f, "FCFG1: 0x%08x\n", openocd_readmem(f, 0x4004804C));
  finfo(f, "FCFG2: 0x%08x\n", openocd_readmem(f, 0x40048050));
  finfo(f, "UIDMH: 0x%08x\n", openocd_readmem(f, 0x40048058));
  finfo(f, "UIDML: 0x%08x\n", openocd_readmem(f, 0x4004805C));
  finfo(f, "UIDL: 0x%08x\n", openocd_readmem(f, 0x40048060));

  if (f->cfg.do_program) {
    finfo(f, "Reprogramming board\n");
    ret = orchard_program(f);
    if (ret)
      goto cleanup;
  }
  else
    finfo(f, "Skipping board programming step\n");

  finfo(f, "Resetting board\n");
  ret = openocd_reset(f);
  if (ret)
    goto cleanup;

  ret = cpu_running(f);
  if (ret == 0) {
    ferr(f, "CPU doesn't appear to be running, trying a complete reset\n");
    kinetis_mass_erase(f);
    ret = -EAGAIN;
    goto cleanup;
  }
  else {
    ferr(f, "Unable to determine if CPU is running\n");
  }

  if (f->serial_fd == -1) {
    finfo(f, "No serial port specified, skipping tests\n");
    goto cleanup;
  }

  ret = stream_wait_banner(f, f->serial_fd, "ch> ");
  if (ret)
    goto cleanup;

  if (f->cfg.specific_tests) {
    int i;
    for (i = 0; i < f->cfg.specific_tests; i++) {
      finfo(f, "Running specific test '%s'\n", f->cfg.specific_test_names[i]);
      ret = orchard_run_test(f, f->cfg.specific_test_names[i]);
      if (ret)
        goto cleanup;
    }
  }

  if (f->cfg.do_tests) {
    finfo(f, "Running tests\n");
    ret = orchard_run_tests(f);
    if (ret)
      goto cleanup;
  }
  else
    finfo(f, "Skipping board testing step\n");

cleanup:
  openocd_stop(f);
  serial_close(f);

  return ret;
}

static int open_write_close(const char *name, const char *valstr)
{
  int ret;
  int fd = open(name, O_WRONLY);
  if (fd < 0)
    return fd;

  ret = write(fd, valstr, strlen(valstr));
  close(fd);

  if (ret < 0)
    return ret;
  return 0;
}

/* RPi-specific config stuff, mostly setting GPIOs.  This will become
 * unnecessary when Device Tree lets us set pullups at boot.
 */
static int config_button_rpi(struct factory *f) {
  /* Couldn't get this code to work, using library instead */
#if 1
  char pyprog[512];

  snprintf(pyprog, sizeof(pyprog)-1, 
        "python -c \""
        "import RPi.GPIO as GPIO; "
        "GPIO.setmode(GPIO.BCM); "
        "GPIO.setup(%d, GPIO.IN, pull_up_down=GPIO.PUD_UP)\"",
        f->cfg.button_gpio);
  return system(pyprog);
#else
static void delay_clocks(int cycles) {
  int i;
  for (i = 0; i < cycles; i++)
    asm("nop");
}

#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define GPIO_PULL_MODE 37
#define GPIO_PULL_MODE_OFF 0
#define GPIO_PULL_MODE_DOWN 1
#define GPIO_PULL_MODE_UP 2
#define GPIO_PULL_CLK0 38
#define GPIO_PULL_CLK1 39
  int m_fd;
  volatile uint32_t *gpio;
  uint32_t shift = f->cfg.button_gpio % 32;

  m_fd = open("/dev/mem", O_RDWR);
  if (m_fd < 0) {
    ferr(f, "Can't open /dev/mem: %s\n", strerror(errno));
    return -1;
  }

  gpio = mmap(NULL, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, GPIO_BASE);

  if (gpio == MAP_FAILED) {
    ferr(f, "Unable to mmap: %s\n", strerror(errno));
    close(m_fd);
    return -1;
  }

  /* Sequence (from BCM2835 manual):
   *  1) Write desired mode to GPIO_PULL_MODE
   *  2) Wait 150 cycles
   *  3) Write control signal to CLK0/1
   *  4) Wait 150 cycles
   *  5) Clear signal from GPIO_PULL_MODE
   *  6) Clear signal from CLK0/1
   */
  gpio[GPIO_PULL_MODE] = (gpio[GPIO_PULL_MODE] & ~3) | GPIO_PULL_MODE_UP;
  delay_clocks(150);

  if (f->cfg.button_gpio < 32)
    gpio[GPIO_PULL_CLK0] = (1 << shift);
  else
    gpio[GPIO_PULL_CLK1] = (1 << shift);
  delay_clocks(150);

  gpio[GPIO_PULL_MODE] &= ~3;
  if (f->cfg.button_gpio < 32)
    gpio[GPIO_PULL_CLK0] = 0;
  else
    gpio[GPIO_PULL_CLK1] = 0;

  munmap((void *)gpio, 4096);
  close(m_fd);

  return 0;
#endif
}

static int open_button(struct factory *f) {

  int ret = -1;
  char str[512];
  int gpio = f->cfg.button_gpio;

  if (gpio < 0)
    return 0;

  snprintf(str, sizeof(str) - 1, "%d", gpio);
  ret = open_write_close("/sys/class/gpio/export", str);
  if (ret && (errno != EBUSY))  {
    ferr(f, "Unable to export GPIO: %s\n", strerror(errno));
    goto cleanup;
  }

  snprintf(str, sizeof(str) - 1, "/sys/class/gpio/gpio%d/direction", gpio);
  ret = open_write_close(str, "in");
  if (ret) {
    ferr(f, "Unable to set GPIO as input: %s\n", strerror(errno));
    goto cleanup;
  }

  ret = config_button_rpi(f);
  if (ret)
    return ret;

  snprintf(str, sizeof(str), "/sys/class/gpio/gpio%d/value", gpio);
  ret = open(str, O_RDWR | O_NONBLOCK | O_SYNC);
  if (ret < 0) {
    ferr(f, "Unable to open GPIO: %s\n", strerror(errno));
    goto cleanup;
  }

  f->button_fd = ret;

  return 0;

cleanup:
  return ret;
}

static int open_leds(struct factory *f) {

  int ret = -1;
  char str[512];
  int gpios[3];
  int *fds[3];
  int led;
  
  gpios[0] = f->cfg.green_gpio;
  fds[0] = &(f->green_fd);
  gpios[1] = f->cfg.yellow_gpio;
  fds[1] = &(f->yellow_fd);
  gpios[2] = f->cfg.red_gpio;
  fds[2] = &(f->red_fd);

  for (led = 0; led < 3; led++) {
    if (gpios[led] < 0)
      continue;

    snprintf(str, sizeof(str) - 1, "%d", gpios[led]);
    ret = open_write_close("/sys/class/gpio/export", str);
    if (ret && (errno != EBUSY))  {
      ferr(f, "Unable to export GPIO: %s\n", strerror(errno));
      return ret;
    }

    snprintf(str, sizeof(str) - 1, "/sys/class/gpio/gpio%d/direction",
        gpios[led]);
    ret = open_write_close(str, "low");
    if (ret) {
      ferr(f, "Unable to set GPIO as output: %s\n", strerror(errno));
      return ret;
    }

    snprintf(str, sizeof(str), "/sys/class/gpio/gpio%d/value", gpios[led]);
    ret = open(str, O_RDWR | O_NONBLOCK | O_SYNC);
    if (ret < 0) {
      ferr(f, "Unable to open GPIO: %s\n", strerror(errno));
      return ret;
    }

    *fds[led] = ret;
  }
  return 0;
}

static int button_pressed(struct factory *f) {

  char bfr;
  int ret;

  if (f->button_fd == -1)
    return 1;

  lseek(f->button_fd, 0, SEEK_SET);
  ret = read(f->button_fd, &bfr, sizeof(bfr));

  if (ret != 1) {
    if ((ret == -1) && (errno == EAGAIN)) {
      if (f->button_fd == -1)
        return -1;
      return button_pressed(f);
    }
    ferr(f, "Unable to read GPIO value: %s\n", strerror(errno));
    return -1;
  }

  return (bfr == '0');
}

static int wait_for_button_press(struct factory *f) {

  int ret;
  if (f->button_fd < 0)
    return 0;

  finfo(f, "Waiting for button press...\n");
  while (1) {
    ret = button_pressed(f);
    if (ret == -1) {
      ferr(f, "Unable to press button\n");
      return -1;
    }
    if (ret == 1)
      break;
  }
  fdbg(f, "Button press detected\n");
  return 0;
}

static int test_setstate(struct factory *f, enum test_state state) {

  const char zero[] = "0";
  const char one[] = "1";

  f->test_state = state;

  if (f->red_fd != -1)
    write(f->red_fd, (state == TEST_FAIL) ? one : zero, 1);
  if (f->yellow_fd != -1)
    write(f->yellow_fd, (state == TEST_INPROGRESS) ? one : zero, 1);
  if (f->green_fd != -1)
    write(f->green_fd, (state == TEST_PASS) ? one : zero, 1);
  return 0;
}

int main(int argc, char **argv) {

  struct factory f;
  int ret;
  g_factory = &f;

  memset(&f, 0, sizeof(f));
  f.openocd_pid = -1;
  f.openocd_sock = -1;
  f.button_fd = -1;
  f.serial_fd = -1;
  f.green_fd = -1;
  f.yellow_fd = -1;
  f.red_fd = -1;
  f.cfg.swclk_gpio = -1;
  f.cfg.swdio_gpio = -1;
  f.cfg.button_gpio = -1;
  f.cfg.green_gpio = -1;
  f.cfg.yellow_gpio = -1;
  f.cfg.red_gpio = -1;
  f.cfg.do_program = 1;
  f.cfg.do_tests = 1;

  signal(SIGCHLD, handle_sigchld);
  signal(SIGTERM, handle_sigterm);
  signal(SIGINT, handle_sigterm);

  if (getuid() != 0) {
    fprintf(stderr, "%s must be run as root\n", argv[0]);
    return 1;
  }

  if (parse_args(&f.cfg, argc, argv))
    return 1;

  if (open_button(&f))
    return 1;

  if (open_leds(&f))
    return 1;

  test_setstate(&f, TEST_PASS);
  do {
    wait_for_button_press(&f);
    test_setstate(&f, TEST_INPROGRESS);
    do {
      ret = run_tests(&f);
    } while (ret == -EAGAIN);

    if (ret)
      test_setstate(&f, TEST_FAIL);
    else
      test_setstate(&f, TEST_PASS);
  } while (f.cfg.daemon);

  openocd_stop(&f);
  serial_close(&f);
  return 0;
}
