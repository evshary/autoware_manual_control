#ifndef _TERMINAL_READER_HPP_
#define _TERMINAL_READER_HPP_

#include <unistd.h>
#include <termios.h>

class TerminalReader
{
  public:
    TerminalReader()
    {
      tcgetattr(STDIN_FILENO, &origin_settings_);
      // new terminal settings
      new_settings_ = origin_settings_;
      new_settings_.c_lflag &= ~(ICANON | ECHO | ECHOE); // Non-canonical mode
      new_settings_.c_cc[VTIME] = 0;    // No timeout_
      new_settings_.c_cc[VMIN] = 1;     // Return while reading 1 char
      // timeout_ value
      timeout_.tv_sec = 1;
      timeout_.tv_usec = 0;
    }
    void configure_termnial()
    {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &new_settings_);
    }
    void restore_terminal()
    {
      tcsetattr(STDIN_FILENO, TCSADRAIN, &origin_settings_);
    }
    int read_key()
    {
      int ch = 0;

      configure_termnial();
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      select(STDIN_FILENO+1, &fds, NULL, NULL, &timeout_);
      if (FD_ISSET(STDIN_FILENO, &fds)) {
        ch = getchar();
      }
      restore_terminal();

      return ch;
    }
  private:
    struct termios origin_settings_;
    struct termios new_settings_;
    struct timeval timeout_;
};

#endif /*_TERMINAL_READER_HPP_*/
