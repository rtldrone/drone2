#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>

int main(int argc, char* argv[]) {
  int i2c_fd = open("/dev/i2c-0", O_RDWR);

  if (i2c_fd < 0) {
    std::cout << "error opening" << std::endl;
  }

  if (ioctl(i2c_fd, I2C_SLAVE, 21) < 0) {
    std::cout << "error ioctl" << std::endl;
  }

  char buf[1];
  //if (write(i2c_fd, buf, 1) != 1) {
  //  std::cout << "Write failed" << std::endl;
  //}
  if (read(i2c_fd, buf, 1) != 1) {
    std::cout << " No data" << std::endl;
  }
  std::cout << (int) buf[0] << std::endl;
}