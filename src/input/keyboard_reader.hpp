#ifndef KEYBOARD_READER_HPP
#define KEYBOARD_READER_HPP

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Non-blocking keyboard reader
class KeyboardReader {
public:
    KeyboardReader() {
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);
        oldf_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf_ | O_NONBLOCK);
    }

    ~KeyboardReader() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
        fcntl(STDIN_FILENO, F_SETFL, oldf_);
    }

    int readKey() {
        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0) return ch;
        return 0;
    }

private:
    struct termios oldt_, newt_;
    int oldf_;
};

#endif // KEYBOARD_READER_HPP